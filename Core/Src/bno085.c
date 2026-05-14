#include "bno085.h"
#include "debug.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define BNO085_CS_LOW()    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define BNO085_CS_HIGH()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define BNO085_WAKE_LOW()  HAL_GPIO_WritePin(SPI1_WAKE_GPIO_Port, SPI1_WAKE_Pin, GPIO_PIN_RESET)
#define BNO085_WAKE_HIGH() HAL_GPIO_WritePin(SPI1_WAKE_GPIO_Port, SPI1_WAKE_Pin, GPIO_PIN_SET)
#define BNO085_RST_LOW()   HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_RESET)
#define BNO085_RST_HIGH()  HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_SET)

bool bno085_communication_test(void)
{
  uint8_t tx_buffer[4] = {0};
  uint8_t rx_buffer[4] = {0};
  uint32_t start_tick;
  char hex_str[64];
  int offset;

  uart1_printf("[BNO085] === HW Diagnostic Start ===\r\n");
  HAL_Delay(10);

  /* ---- STEP 1: GPIO 핀 상태 확인 ---- */
  uart1_printf("[BNO085] GPIO: CS=%d RST=%d WAKE=%d INT=%d\r\n",
    HAL_GPIO_ReadPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin),
    HAL_GPIO_ReadPin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin),
    HAL_GPIO_ReadPin(SPI1_WAKE_GPIO_Port, SPI1_WAKE_Pin),
    HAL_GPIO_ReadPin(SPI1_INT_GPIO_Port, SPI1_INT_Pin));
  HAL_Delay(10);

  /* ---- STEP 2: SPI 루프백 테스트 (MOSI→MISO 확인) ---- */
  /* 이 테스트는 센서와 무관하게 SPI 주변장치 자체가 동작하는지 확인합니다.
     센서의 CS를 해제(HIGH)하여 버스에서 분리한 상태로 진행합니다. */
  BNO085_CS_HIGH();
  tx_buffer[0] = 0xAB; tx_buffer[1] = 0xCD; tx_buffer[2] = 0xEF; tx_buffer[3] = 0x12;
  memset(rx_buffer, 0, 4);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 4U, 100U);
  uart1_printf("[BNO085] Loopback TX: AB CD EF 12\r\n");
  HAL_Delay(5);
  uart1_printf("[BNO085] Loopback RX: %02X %02X %02X %02X\r\n",
    rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
  HAL_Delay(10);

  /* ---- STEP 3: 센서 리셋 (긴 부팅 대기) ---- */
  /* PS0(WAKE)=HIGH 상태에서 리셋을 풀어야 SPI 모드로 부팅됨 */
  BNO085_CS_HIGH();
  BNO085_WAKE_HIGH();  /* PS0 = HIGH */
  HAL_Delay(2);

  BNO085_RST_LOW();
  HAL_Delay(30);       /* 리셋 펄스를 길게 유지 */
  BNO085_RST_HIGH();

  uart1_printf("[BNO085] Reset released, waiting 300ms for boot...\r\n");
  HAL_Delay(300);      /* BNO085 풀 부팅 대기 (최대 300ms 소요될 수 있음) */

  /* ---- STEP 4: INT 핀 대기 ---- */
  uart1_printf("[BNO085] INT pin now = %d\r\n",
    HAL_GPIO_ReadPin(SPI1_INT_GPIO_Port, SPI1_INT_Pin));
  HAL_Delay(5);

  start_tick = HAL_GetTick();
  while (HAL_GPIO_ReadPin(SPI1_INT_GPIO_Port, SPI1_INT_Pin) != GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start_tick) > 1000U)
    {
      uart1_printf("[BNO085] INT timeout after 1s\r\n");
      return false;
    }
    HAL_Delay(1);
  }
  uart1_printf("[BNO085] INT LOW after %lu ms\r\n", HAL_GetTick() - start_tick);
  HAL_Delay(5);

  /* ---- STEP 5: 3회 연속 SHTP 헤더 읽기 시도 ---- */
  for (int attempt = 0; attempt < 3; attempt++)
  {
    memset(tx_buffer, 0, 4);
    memset(rx_buffer, 0, 4);

    BNO085_WAKE_LOW();
    HAL_Delay(1);
    BNO085_CS_LOW();
    HAL_Delay(1);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 4U, 100U);

    BNO085_CS_HIGH();
    BNO085_WAKE_HIGH();

    uart1_printf("[BNO085] Read#%d st=%d: %02X %02X %02X %02X\r\n",
      attempt + 1, status,
      rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
    HAL_Delay(10);

    /* 유의미한 데이터를 받았으면 성공 */
    if ((rx_buffer[0] | rx_buffer[1] | rx_buffer[2] | rx_buffer[3]) != 0x00)
    {
      uint16_t pkt_len = ((uint16_t)rx_buffer[1] << 8 | rx_buffer[0]) & 0x7FFFU;
      uart1_printf("[BNO085] PASSED! Len=%d Ch=%d Seq=%d\r\n",
        pkt_len, rx_buffer[2], rx_buffer[3]);
      return true;
    }

    /* INT가 다시 LOW가 되길 대기 */
    HAL_Delay(50);
  }

  uart1_printf("[BNO085] FAILED: All 3 reads returned zeros\r\n");
  return false;
}
