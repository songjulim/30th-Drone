#include "uart_bridge.h"

#include "debug.h"

extern UART_HandleTypeDef huart2;

#define UART_BRIDGE_PC_TO_GNSS_BUFFER_SIZE 512U
#define UART_BRIDGE_GNSS_TO_PC_BUFFER_SIZE 512U
#define UART_BRIDGE_PC_TO_GNSS_CHUNK_SIZE   32U
#define UART_BRIDGE_GNSS_TO_PC_CHUNK_SIZE  128U

typedef struct
{
  uint8_t data[UART_BRIDGE_PC_TO_GNSS_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t count;
} uart_bridge_pc_to_gnss_fifo_t;

typedef struct
{
  uint8_t data[UART_BRIDGE_GNSS_TO_PC_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t count;
} uart_bridge_gnss_to_pc_fifo_t;

static uart_bridge_pc_to_gnss_fifo_t uart_bridge_pc_to_gnss_fifo;
static uart_bridge_gnss_to_pc_fifo_t uart_bridge_gnss_to_pc_fifo;
static volatile uint8_t uart_bridge_active = 0U;

static uint32_t uart_bridge_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void uart_bridge_exit_critical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void uart_bridge_reset_fifos(void)
{
  uint32_t primask = uart_bridge_enter_critical();

  uart_bridge_pc_to_gnss_fifo.head = 0U;
  uart_bridge_pc_to_gnss_fifo.tail = 0U;
  uart_bridge_pc_to_gnss_fifo.count = 0U;
  uart_bridge_gnss_to_pc_fifo.head = 0U;
  uart_bridge_gnss_to_pc_fifo.tail = 0U;
  uart_bridge_gnss_to_pc_fifo.count = 0U;

  uart_bridge_exit_critical(primask);
}

void uart_bridge_init(void)
{
  uart_bridge_active = 0U;
  uart_bridge_reset_fifos();
}

void uart_bridge_set_active(bool active)
{
  uart_bridge_active = active ? 1U : 0U;
  uart_bridge_reset_fifos();
}

bool uart_bridge_is_active(void)
{
  return (uart_bridge_active != 0U);
}

HAL_StatusTypeDef uart_bridge_enqueue_pc_data(const uint8_t *data, uint16_t length)
{
  uint32_t primask;
  uint16_t index;
  uint16_t i;

  if ((data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  primask = uart_bridge_enter_critical();

  if ((uint32_t)uart_bridge_pc_to_gnss_fifo.count + (uint32_t)length > UART_BRIDGE_PC_TO_GNSS_BUFFER_SIZE)
  {
    uart_bridge_exit_critical(primask);
    return HAL_BUSY;
  }

  index = uart_bridge_pc_to_gnss_fifo.head;

  for (i = 0U; i < length; i++)
  {
    uart_bridge_pc_to_gnss_fifo.data[index] = data[i];
    index = (uint16_t)((index + 1U) % UART_BRIDGE_PC_TO_GNSS_BUFFER_SIZE);
  }

  uart_bridge_pc_to_gnss_fifo.head = index;
  uart_bridge_pc_to_gnss_fifo.count = (uint16_t)(uart_bridge_pc_to_gnss_fifo.count + length);

  uart_bridge_exit_critical(primask);

  return HAL_OK;
}

HAL_StatusTypeDef uart_bridge_enqueue_gnss_byte(uint8_t data)
{
  uint32_t primask = uart_bridge_enter_critical();
  uint16_t index;

  if (uart_bridge_gnss_to_pc_fifo.count >= UART_BRIDGE_GNSS_TO_PC_BUFFER_SIZE)
  {
    uart_bridge_exit_critical(primask);
    return HAL_BUSY;
  }

  index = uart_bridge_gnss_to_pc_fifo.head;
  uart_bridge_gnss_to_pc_fifo.data[index] = data;
  uart_bridge_gnss_to_pc_fifo.head = (uint16_t)((index + 1U) % UART_BRIDGE_GNSS_TO_PC_BUFFER_SIZE);
  uart_bridge_gnss_to_pc_fifo.count++;

  uart_bridge_exit_critical(primask);

  return HAL_OK;
}

static uint16_t uart_bridge_copy_pc_chunk(uint8_t *buffer, uint16_t max_length)
{
  uint32_t primask;
  uint16_t copied = 0U;

  primask = uart_bridge_enter_critical();

  while ((copied < max_length) && (uart_bridge_pc_to_gnss_fifo.count > 0U))
  {
    buffer[copied] = uart_bridge_pc_to_gnss_fifo.data[uart_bridge_pc_to_gnss_fifo.tail];
    uart_bridge_pc_to_gnss_fifo.tail = (uint16_t)((uart_bridge_pc_to_gnss_fifo.tail + 1U) % UART_BRIDGE_PC_TO_GNSS_BUFFER_SIZE);
    uart_bridge_pc_to_gnss_fifo.count--;
    copied++;
  }

  uart_bridge_exit_critical(primask);

  return copied;
}

static uint16_t uart_bridge_copy_gnss_chunk(uint8_t *buffer, uint16_t max_length)
{
  uint32_t primask;
  uint16_t copied = 0U;

  primask = uart_bridge_enter_critical();

  while ((copied < max_length) && (uart_bridge_gnss_to_pc_fifo.count > 0U))
  {
    buffer[copied] = uart_bridge_gnss_to_pc_fifo.data[uart_bridge_gnss_to_pc_fifo.tail];
    uart_bridge_gnss_to_pc_fifo.tail = (uint16_t)((uart_bridge_gnss_to_pc_fifo.tail + 1U) % UART_BRIDGE_GNSS_TO_PC_BUFFER_SIZE);
    uart_bridge_gnss_to_pc_fifo.count--;
    copied++;
  }

  uart_bridge_exit_critical(primask);

  return copied;
}

void uart_bridge_process(void)
{
  uint8_t pc_to_gnss_chunk[UART_BRIDGE_PC_TO_GNSS_CHUNK_SIZE];
  uint8_t gnss_to_pc_chunk[UART_BRIDGE_GNSS_TO_PC_CHUNK_SIZE];
  uint16_t chunk_length;

  if (uart_bridge_active == 0U)
  {
    return;
  }

  chunk_length = uart_bridge_copy_pc_chunk(pc_to_gnss_chunk, UART_BRIDGE_PC_TO_GNSS_CHUNK_SIZE);
  if (chunk_length > 0U)
  {
    if (HAL_UART_Transmit(&huart2, pc_to_gnss_chunk, chunk_length, 10U) != HAL_OK)
    {
      (void)uart_bridge_enqueue_pc_data(pc_to_gnss_chunk, chunk_length);
    }
  }

  chunk_length = uart_bridge_copy_gnss_chunk(gnss_to_pc_chunk, UART_BRIDGE_GNSS_TO_PC_CHUNK_SIZE);
  if (chunk_length > 0U)
  {
    if (debug_uart1_write_raw(gnss_to_pc_chunk, chunk_length) != HAL_OK)
    {
      uint16_t index;

      for (index = 0U; index < chunk_length; index++)
      {
        if (uart_bridge_enqueue_gnss_byte(gnss_to_pc_chunk[index]) != HAL_OK)
        {
          break;
        }
      }
    }
  }
}
