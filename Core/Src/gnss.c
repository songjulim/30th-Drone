#include "gnss.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

#define GNSS_LINE_BUFFER_SIZE 128U
#define GNSS_RESET_PULSE_MS     5U

static uint8_t gnss_rx_byte = 0U;
static char gnss_line_buffer[GNSS_LINE_BUFFER_SIZE];
static char gnss_ready_line[GNSS_LINE_BUFFER_SIZE];
static volatile uint16_t gnss_line_length = 0U;
static volatile uint16_t gnss_ready_length = 0U;
static volatile uint8_t gnss_collecting = 0U;
static volatile uint8_t gnss_sentence_ready = 0U;

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
  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_SET);
  gnss_reset_line_state();
  gnss_sentence_ready = 0U;
  gnss_ready_length = 0U;

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

void gnss_handle_uart_error(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  gnss_reset_line_state();
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
