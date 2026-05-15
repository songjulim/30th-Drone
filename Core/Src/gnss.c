#include "gnss.h"

#include "uart_bridge.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

#define GNSS_LINE_BUFFER_SIZE 128U
#define GNSS_RESET_PULSE_MS     5U
#define GNSS_DEFAULT_BAUDRATE   9600U
#define GNSS_DRONE_BAUDRATE     115200U
#define GNSS_UART_TIMEOUT_MS    100U
#define GNSS_CONFIG_SETTLE_MS   150U

#define GNSS_UBX_SYNC_1         0xB5U
#define GNSS_UBX_SYNC_2         0x62U
#define GNSS_UBX_CLASS_CFG      0x06U
#define GNSS_UBX_ID_VALSET      0x8AU
#define GNSS_VALSET_VERSION     0x01U
#define GNSS_VALSET_LAYER_RAM   0x01U
#define GNSS_VALSET_LAYER_BBR   0x02U
#define GNSS_VALSET_MAX_PAYLOAD 96U
#define GNSS_UBX_FRAME_OVERHEAD 8U

typedef struct
{
  uint32_t key;
  uint32_t value;
  uint8_t value_size;
} gnss_ubx_config_entry_t;

static uint8_t gnss_rx_byte = 0U;
static char gnss_line_buffer[GNSS_LINE_BUFFER_SIZE];
static char gnss_ready_line[GNSS_LINE_BUFFER_SIZE];
static volatile uint16_t gnss_line_length = 0U;
static volatile uint16_t gnss_ready_length = 0U;
static volatile uint8_t gnss_collecting = 0U;
static volatile uint8_t gnss_sentence_ready = 0U;
static volatile uint8_t gnss_bridge_mode_active = 0U;

static const gnss_ubx_config_entry_t gnss_drone_config[] = {
    {0x40520001UL, GNSS_DRONE_BAUDRATE, 4U}, /* CFG-UART1-BAUDRATE */
    {0x10740001UL, 1U, 1U},                  /* CFG-UART1OUTPROT-UBX */
    {0x10740002UL, 0U, 1U},                  /* CFG-UART1OUTPROT-NMEA */
    {0x20110021UL, 6U, 1U},                  /* CFG-NAVSPG-DYNMODEL: airborne <1g */
    {0x30210001UL, 200U, 2U},                /* CFG-RATE-MEAS: 5 Hz */
    {0x30210002UL, 1U, 2U},                  /* CFG-RATE-NAV */
    {0x20910007UL, 1U, 1U},                  /* CFG-MSGOUT-UBX_NAV_PVT-UART1 */
    {0x209100BBUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GGA-UART1 */
    {0x209100CAUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GLL-UART1 */
    {0x209100C0UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GSA-UART1 */
    {0x209100C5UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GSV-UART1 */
    {0x209100ACUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_RMC-UART1 */
    {0x209100B1UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_VTG-UART1 */
};

static void gnss_append_u16(uint8_t *buffer, uint16_t *index, uint16_t value)
{
  buffer[*index] = (uint8_t)(value & 0xFFU);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 8U) & 0xFFU);
  (*index)++;
}

static void gnss_append_u32(uint8_t *buffer, uint16_t *index, uint32_t value)
{
  buffer[*index] = (uint8_t)(value & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 8U) & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 16U) & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 24U) & 0xFFUL);
  (*index)++;
}

static void gnss_append_value(uint8_t *buffer, uint16_t *index, uint32_t value, uint8_t value_size)
{
  uint8_t i;

  for (i = 0U; i < value_size; i++)
  {
    buffer[*index] = (uint8_t)((value >> (8U * i)) & 0xFFUL);
    (*index)++;
  }
}

static void gnss_append_ubx_checksum(uint8_t *frame, uint16_t frame_length_without_checksum)
{
  uint8_t checksum_a = 0U;
  uint8_t checksum_b = 0U;
  uint16_t i;

  for (i = 2U; i < frame_length_without_checksum; i++)
  {
    checksum_a = (uint8_t)(checksum_a + frame[i]);
    checksum_b = (uint8_t)(checksum_b + checksum_a);
  }

  frame[frame_length_without_checksum] = checksum_a;
  frame[frame_length_without_checksum + 1U] = checksum_b;
}

static HAL_StatusTypeDef gnss_set_uart_baudrate(uint32_t baudrate)
{
  HAL_StatusTypeDef status;

  (void)HAL_UART_Abort(&huart2);
  huart2.Init.BaudRate = baudrate;

  status = HAL_UART_Init(&huart2);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_UARTEx_DisableFifoMode(&huart2);
}

static HAL_StatusTypeDef gnss_send_drone_config(void)
{
  uint8_t frame[GNSS_VALSET_MAX_PAYLOAD + GNSS_UBX_FRAME_OVERHEAD];
  uint16_t index = 0U;
  uint16_t length_index;
  uint16_t payload_length;
  uint16_t i;

  frame[index++] = GNSS_UBX_SYNC_1;
  frame[index++] = GNSS_UBX_SYNC_2;
  frame[index++] = GNSS_UBX_CLASS_CFG;
  frame[index++] = GNSS_UBX_ID_VALSET;
  length_index = index;
  index += 2U;

  frame[index++] = GNSS_VALSET_VERSION;
  frame[index++] = (uint8_t)(GNSS_VALSET_LAYER_RAM | GNSS_VALSET_LAYER_BBR);
  frame[index++] = 0U;
  frame[index++] = 0U;

  for (i = 0U; i < (uint16_t)(sizeof(gnss_drone_config) / sizeof(gnss_drone_config[0])); i++)
  {
    gnss_append_u32(frame, &index, gnss_drone_config[i].key);
    gnss_append_value(frame, &index, gnss_drone_config[i].value, gnss_drone_config[i].value_size);
  }

  payload_length = (uint16_t)(index - GNSS_UBX_FRAME_OVERHEAD + 2U);
  gnss_append_u16(frame, &length_index, payload_length);
  gnss_append_ubx_checksum(frame, index);
  index += 2U;

  return HAL_UART_Transmit(&huart2, frame, index, GNSS_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef gnss_configure_for_drone(void)
{
  HAL_StatusTypeDef status;

  (void)gnss_set_uart_baudrate(GNSS_DEFAULT_BAUDRATE);
  (void)gnss_send_drone_config();
  HAL_Delay(GNSS_CONFIG_SETTLE_MS);

  status = gnss_set_uart_baudrate(GNSS_DRONE_BAUDRATE);
  if (status != HAL_OK)
  {
    return status;
  }

  status = gnss_send_drone_config();
  HAL_Delay(GNSS_CONFIG_SETTLE_MS);

  return status;
}

static uint32_t gnss_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  return primask;
}

static void gnss_exit_critical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static HAL_StatusTypeDef gnss_start_receive_it(void)
{
  return HAL_UART_Receive_IT(&huart2, &gnss_rx_byte, 1U);
}

static void gnss_reset_line_state(void)
{
  gnss_line_length = 0U;
  gnss_collecting = 0U;
}

HAL_StatusTypeDef gnss_init(void)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_SET);
  gnss_reset_line_state();
  gnss_sentence_ready = 0U;
  gnss_ready_length = 0U;
  gnss_bridge_mode_active = 0U;

  status = gnss_configure_for_drone();
  if (status != HAL_OK)
  {
    return status;
  }

  return gnss_start_receive_it();
}

void gnss_reset(void)
{
  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(GNSS_RESET_PULSE_MS);
  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_SET);
}

bool gnss_read_line(char *buffer, size_t buffer_size)
{
  uint16_t ready_length;
  uint32_t primask;

  if ((buffer == NULL) || (buffer_size == 0U))
  {
    return false;
  }

  if (gnss_bridge_mode_active != 0U)
  {
    return false;
  }

  primask = gnss_enter_critical();

  if (gnss_sentence_ready == 0U)
  {
    gnss_exit_critical(primask);
    return false;
  }

  ready_length = gnss_ready_length;
  if (ready_length >= buffer_size)
  {
    ready_length = (uint16_t)(buffer_size - 1U);
  }

  memcpy(buffer, gnss_ready_line, ready_length);
  buffer[ready_length] = '\0';
  gnss_sentence_ready = 0U;

  gnss_exit_critical(primask);

  return true;
}

void gnss_set_bridge_mode(uint8_t active)
{
  gnss_bridge_mode_active = active;
  gnss_reset_line_state();
  gnss_sentence_ready = 0U;
  gnss_ready_length = 0U;

  (void)HAL_UART_AbortReceive(&huart2);
  (void)gnss_start_receive_it();
}

void gnss_handle_uart_error(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  gnss_reset_line_state();
  gnss_sentence_ready = 0U;
  (void)HAL_UART_AbortReceive(huart);
  (void)gnss_start_receive_it();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  if (gnss_bridge_mode_active != 0U)
  {
    (void)uart_bridge_enqueue_gnss_byte(gnss_rx_byte);
    (void)gnss_start_receive_it();
    return;
  }

  if (gnss_rx_byte == '$')
  {
    gnss_line_buffer[0] = '$';
    gnss_line_length = 1U;
    gnss_collecting = 1U;
  }
  else if (gnss_collecting != 0U)
  {
    if (gnss_rx_byte == '\r')
    {
      (void)gnss_start_receive_it();
      return;
    }

    if (gnss_rx_byte == '\n')
    {
      if (gnss_line_length > 0U)
      {
        primask = gnss_enter_critical();
        memcpy(gnss_ready_line, gnss_line_buffer, gnss_line_length);
        gnss_ready_line[gnss_line_length] = '\0';
        gnss_ready_length = gnss_line_length;
        gnss_sentence_ready = 1U;
        gnss_exit_critical(primask);
      }

      gnss_reset_line_state();
      (void)gnss_start_receive_it();
      return;
    }

    if (gnss_line_length < (GNSS_LINE_BUFFER_SIZE - 1U))
    {
      gnss_line_buffer[gnss_line_length] = (char)gnss_rx_byte;
      gnss_line_length++;
    }
    else
    {
      gnss_reset_line_state();
    }
  }

  (void)gnss_start_receive_it();
}
