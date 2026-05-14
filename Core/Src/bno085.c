/*
 * bno085.c - BNO085 9-DOF Sensor Hub Driver (SPI / SHTP)
 *
 * Implements Rotation Vector (0x05) output for drone heading.
 * Uses SHTP (Sensor Hub Transport Protocol) over SPI1.
 *
 * Pin mapping (from main.h):
 *   PA4  = CS      (SPI1_CS)
 *   PA5  = SCK     (SPI1_SCK)
 *   PA6  = MISO    (SPI1_MISO)
 *   PA7  = MOSI    (SPI1_MOSI)
 *   PC5  = NRST    (SPI1_RESET)
 *   PB0  = WAKE    (SPI1_WAKE / PS0)
 *   PB1  = H_INTN  (SPI1_INT, active LOW)
 */

#include "bno085.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* ======================== Pin Control ======================== */
#define BNO_CS_LOW()    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define BNO_CS_HIGH()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define BNO_WAKE_LOW()  HAL_GPIO_WritePin(SPI1_WAKE_GPIO_Port, SPI1_WAKE_Pin, GPIO_PIN_RESET)
#define BNO_WAKE_HIGH() HAL_GPIO_WritePin(SPI1_WAKE_GPIO_Port, SPI1_WAKE_Pin, GPIO_PIN_SET)
#define BNO_RST_LOW()   HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_RESET)
#define BNO_RST_HIGH()  HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_SET)
#define BNO_INT_READ()  HAL_GPIO_ReadPin(SPI1_INT_GPIO_Port, SPI1_INT_Pin)

/* ======================== SHTP Constants ======================== */
#define SHTP_HDR_SIZE       4U
#define SHTP_MAX_PAYLOAD    300U
#define SHTP_MAX_XFER       (SHTP_HDR_SIZE + SHTP_MAX_PAYLOAD)

#define SHTP_CH_COMMAND     0U
#define SHTP_CH_EXECUTABLE  1U
#define SHTP_CH_CONTROL     2U
#define SHTP_CH_INPUT       3U

/* ======================== SH-2 Report IDs ======================== */
#define SH2_SET_FEATURE_CMD     0xFDU
#define SH2_ROTATION_VECTOR     0x05U
#define SH2_CMD_REQUEST         0xF2U  /* Command Request report */
#define SH2_CMD_RESPONSE        0xF1U  /* Command Response report */
#define SH2_CMD_ME_CALIBRATE    0x07U  /* ME Calibration command */
#define SH2_CMD_DCD_SAVE        0x06U  /* Save DCD command */

/* ======================== Conversion ======================== */
#define Q14_SCALE   (1.0f / 16384.0f)
#define Q12_SCALE   (1.0f / 4096.0f)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG  (180.0f / M_PI)

/* ======================== Internal State ======================== */
static uint8_t shtp_tx[SHTP_MAX_XFER];
static uint8_t shtp_rx[SHTP_MAX_XFER];
static uint8_t tx_seq[6]; /* per-channel TX sequence counters */

static float euler_yaw, euler_pitch, euler_roll;
static float heading_accuracy;
static bool data_valid = false;
static uint8_t mag_cal_status = 0;      /* 0~3 from rotation vector status bits */
static uint8_t cmd_seq_num = 0;         /* Command sequence counter */
static volatile uint8_t cal_cmd_status = 0xFF; /* 0=success from ME Cal response */

/* Forward declarations */
static bool bno085_wait_int(uint32_t timeout_ms);

/* ================================================================
 *                     Low-Level SHTP I/O
 * ================================================================ */

/*
 * Read one SHTP packet from the sensor (non-blocking).
 * Returns payload length on success, 0 if no data or error.
 * channel_out receives the SHTP channel number.
 * Payload data is stored at shtp_rx[SHTP_HDR_SIZE ..].
 */
static uint16_t shtp_read(uint8_t *channel_out)
{
  uint16_t pkt_len, payload_len, xfer_len;

  /* INT must be LOW for data to be available */
  if (BNO_INT_READ() != GPIO_PIN_RESET)
    return 0U;

  memset(shtp_tx, 0, SHTP_HDR_SIZE);
  memset(shtp_rx, 0, SHTP_HDR_SIZE);

  BNO_CS_LOW();
  HAL_Delay(1);

  /* 1) Read 4-byte SHTP header */
  if (HAL_SPI_TransmitReceive(&hspi1, shtp_tx, shtp_rx,
                               SHTP_HDR_SIZE, 100U) != HAL_OK)
  {
    BNO_CS_HIGH();
    return 0U;
  }

  pkt_len = ((uint16_t)shtp_rx[1] << 8 | shtp_rx[0]) & 0x7FFFU;

  if (pkt_len <= SHTP_HDR_SIZE || pkt_len == 0x7FFFU)
  {
    BNO_CS_HIGH();
    return 0U;
  }

  if (channel_out)
    *channel_out = shtp_rx[2];

  payload_len = pkt_len - SHTP_HDR_SIZE;
  xfer_len = (payload_len > SHTP_MAX_PAYLOAD) ? SHTP_MAX_PAYLOAD : payload_len;

  /* 2) Read payload (keeping CS LOW for the entire transaction) */
  memset(shtp_tx, 0, xfer_len);
  if (HAL_SPI_TransmitReceive(&hspi1, shtp_tx,
                               shtp_rx + SHTP_HDR_SIZE,
                               xfer_len, 200U) != HAL_OK)
  {
    BNO_CS_HIGH();
    return 0U;
  }

  /* 3) Drain remaining bytes if packet exceeds our buffer */
  if (payload_len > SHTP_MAX_PAYLOAD)
  {
    uint16_t remain = payload_len - SHTP_MAX_PAYLOAD;
    uint8_t dummy_tx[32] = {0};
    uint8_t dummy_rx[32];
    while (remain > 0U)
    {
      uint16_t chunk = (remain > 32U) ? 32U : remain;
      HAL_SPI_TransmitReceive(&hspi1, dummy_tx, dummy_rx, chunk, 100U);
      remain -= chunk;
    }
  }

  BNO_CS_HIGH();
  return xfer_len;
}

/*
 * Write an SHTP packet on the specified channel.
 * payload points to the SH-2 command bytes (without SHTP header).
 */
static bool shtp_write(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
  uint16_t pkt_len = SHTP_HDR_SIZE + payload_len;

  if (pkt_len > SHTP_MAX_XFER)
    return false;

  /* Build SHTP header */
  shtp_tx[0] = (uint8_t)(pkt_len & 0xFFU);
  shtp_tx[1] = (uint8_t)((pkt_len >> 8) & 0x7FU);
  shtp_tx[2] = channel;
  shtp_tx[3] = tx_seq[channel]++;

  /* Copy payload */
  memcpy(shtp_tx + SHTP_HDR_SIZE, payload, payload_len);

  /* Wake sensor and wait for INT */
  BNO_WAKE_LOW();
  if (!bno085_wait_int(100U))
  {
    BNO_WAKE_HIGH();
    return false;
  }

  /* SPI transfer */
  memset(shtp_rx, 0, pkt_len);
  BNO_CS_LOW();
  HAL_Delay(1);
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, shtp_tx, shtp_rx,
                                                  pkt_len, 200U);
  BNO_CS_HIGH();
  BNO_WAKE_HIGH();

  return (st == HAL_OK);
}

/* Wait for INT pin to go LOW with timeout */
static bool bno085_wait_int(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (BNO_INT_READ() != GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
      return false;
  }
  return true;
}

/* Drain all pending packets (call after reset) */
static void bno085_drain_packets(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t ch;

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (shtp_read(&ch) == 0U)
    {
      HAL_Delay(5);
      /* If INT is still HIGH (no more data), we're done */
      if (BNO_INT_READ() != GPIO_PIN_RESET)
        break;
    }
  }
}

/* ================================================================
 *                     SH-2 Commands
 * ================================================================ */

/*
 * Enable Rotation Vector report at the specified interval.
 * interval_us: report period in microseconds (e.g., 20000 = 50 Hz)
 */
static bool sh2_enable_rotation_vector(uint32_t interval_us)
{
  uint8_t cmd[17];

  cmd[0]  = SH2_SET_FEATURE_CMD;    /* 0xFD */
  cmd[1]  = SH2_ROTATION_VECTOR;    /* 0x05 */
  cmd[2]  = 0x00;                   /* Feature flags */
  cmd[3]  = 0x00;                   /* Change sensitivity LSB */
  cmd[4]  = 0x00;                   /* Change sensitivity MSB */
  cmd[5]  = (uint8_t)(interval_us & 0xFFU);
  cmd[6]  = (uint8_t)((interval_us >> 8) & 0xFFU);
  cmd[7]  = (uint8_t)((interval_us >> 16) & 0xFFU);
  cmd[8]  = (uint8_t)((interval_us >> 24) & 0xFFU);
  cmd[9]  = 0x00;  /* Batch interval (4 bytes) */
  cmd[10] = 0x00;
  cmd[11] = 0x00;
  cmd[12] = 0x00;
  cmd[13] = 0x00;  /* Sensor-specific config (4 bytes) */
  cmd[14] = 0x00;
  cmd[15] = 0x00;
  cmd[16] = 0x00;

  return shtp_write(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

/* ================================================================
 *                     Quaternion → Euler
 * ================================================================ */

static void quaternion_to_euler(float qi, float qj, float qk, float qr)
{
  /* ZYX convention (yaw-pitch-roll) */
  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  euler_roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

  float sinp = 2.0f * (qr * qj - qk * qi);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  euler_pitch = asinf(sinp) * RAD_TO_DEG;

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  euler_yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

  /* Normalize yaw to 0~360 */
  if (euler_yaw < 0.0f)
    euler_yaw += 360.0f;
}

/* ================================================================
 *                     Parse Sensor Reports
 * ================================================================ */

/*
 * Parse a Rotation Vector report from the payload buffer.
 * payload points to the report data (after SHTP header).
 * Returns true if parsed successfully.
 */
static bool parse_rotation_vector(const uint8_t *payload, uint16_t len)
{
  /* Scan payload for Report ID 0x05 */
  for (uint16_t i = 0; i + 13 < len; i++)
  {
    if (payload[i] == SH2_ROTATION_VECTOR)
    {
      /* Offset from Report ID byte:
       *   +0  Report ID (0x05)
       *   +1  Sequence
       *   +2  Status
       *   +3  Delay
       *   +4  Delay MSB (sometimes used, sometimes rolled into +3)
       *   +4  Quat i LSB
       *   +5  Quat i MSB
       *   +6  Quat j LSB
       *   +7  Quat j MSB
       *   +8  Quat k LSB
       *   +9  Quat k MSB
       *   +10 Quat real LSB
       *   +11 Quat real MSB
       *   +12 Accuracy LSB
       *   +13 Accuracy MSB
       */
      const uint8_t *d = &payload[i];

      /* Status bits [1:0] = calibration accuracy (0~3) */
      mag_cal_status = d[2] & 0x03U;

      int16_t qi_raw = (int16_t)((uint16_t)d[5]  << 8 | d[4]);
      int16_t qj_raw = (int16_t)((uint16_t)d[7]  << 8 | d[6]);
      int16_t qk_raw = (int16_t)((uint16_t)d[9]  << 8 | d[8]);
      int16_t qr_raw = (int16_t)((uint16_t)d[11] << 8 | d[10]);
      int16_t ac_raw = (int16_t)((uint16_t)d[13] << 8 | d[12]);

      float qi = (float)qi_raw * Q14_SCALE;
      float qj = (float)qj_raw * Q14_SCALE;
      float qk = (float)qk_raw * Q14_SCALE;
      float qr = (float)qr_raw * Q14_SCALE;

      heading_accuracy = (float)ac_raw * Q12_SCALE * RAD_TO_DEG;

      quaternion_to_euler(qi, qj, qk, qr);
      data_valid = true;
      return true;
    }
  }
  return false;
}

/* ================================================================
 *                     Public API
 * ================================================================ */

bool bno085_init(void)
{
  uart1_printf("[BNO085] Initializing...\r\n");
  HAL_Delay(10);

  memset(tx_seq, 0, sizeof(tx_seq));
  data_valid = false;

  /* 1) Initial pin state */
  BNO_CS_HIGH();
  BNO_WAKE_HIGH();  /* PS0 = HIGH for SPI mode */
  HAL_Delay(2);

  /* 2) Hardware reset */
  BNO_RST_LOW();
  HAL_Delay(30);
  BNO_RST_HIGH();
  HAL_Delay(300);   /* BNO085 needs up to 300ms to fully boot */

  /* 3) Wait for first INT assertion */
  if (!bno085_wait_int(500U))
  {
    uart1_printf("[BNO085] Init FAIL: INT timeout\r\n");
    return false;
  }

  /* 4) Drain boot packets (Advertisement, Init response) */
  bno085_drain_packets(200U);
  uart1_printf("[BNO085] Boot packets drained\r\n");
  HAL_Delay(10);

  /* 5) Enable Rotation Vector at 400 Hz (2,500 µs interval) */
  if (!sh2_enable_rotation_vector(2500U))
  {
    uart1_printf("[BNO085] Init FAIL: Set Feature cmd\r\n");
    return false;
  }

  uart1_printf("[BNO085] Rotation Vector enabled (400Hz)\r\n");
  HAL_Delay(10);

  /* 6) Wait for first report to confirm operation */
  for (int retry = 0; retry < 20; retry++)
  {
    HAL_Delay(25);
    if (bno085_process())
    {
      int yaw_i = (int)euler_yaw;
      int yaw_d = ((int)(euler_yaw * 10.0f)) % 10;
      if (yaw_d < 0) yaw_d = -yaw_d;
      uart1_printf("[BNO085] Init OK! Yaw=%d.%d\r\n", yaw_i, yaw_d);
      return true;
    }
  }

  uart1_printf("[BNO085] Init WARN: No report yet (may start later)\r\n");
  return true;  /* Sensor configured, data will arrive eventually */
}

void bno085_communication_test(void)
{
  (void)bno085_init();
}

bool bno085_process(void)
{
  uint8_t channel = 0;
  uint16_t payload_len;

  /* Non-blocking: return immediately if no data pending */
  payload_len = shtp_read(&channel);
  if (payload_len == 0U)
    return false;

  const uint8_t *payload = shtp_rx + SHTP_HDR_SIZE;

  /* Sensor reports arrive on channel 3 (Input) */
  if (channel == SHTP_CH_INPUT)
  {
    return parse_rotation_vector(payload, payload_len);
  }

  /* Command responses arrive on channel 2 (Control) */
  if (channel == SHTP_CH_CONTROL && payload_len >= 6U)
  {
    if (payload[0] == SH2_CMD_RESPONSE)
    {
      uint8_t cmd = payload[2];
      if (cmd == SH2_CMD_ME_CALIBRATE)
        cal_cmd_status = payload[5]; /* R0: 0=success */
    }
  }

  return false;
}

bno085_euler_t bno085_get_euler(void)
{
  bno085_euler_t e;
  e.yaw = euler_yaw;
  e.pitch = euler_pitch;
  e.roll = euler_roll;
  e.accuracy_deg = heading_accuracy;
  e.valid = data_valid;
  return e;
}

/* ================================================================
 *              Calibration: SH-2 Commands & Public API
 * ================================================================ */

/*
 * Send ME Calibration command (Report ID 0xF2, Command 0x07).
 * accel/gyro/mag: 1=enable dynamic calibration, 0=disable
 */
static bool sh2_send_me_calibrate(uint8_t accel, uint8_t gyro, uint8_t mag)
{
  uint8_t cmd[12];

  cmd[0]  = SH2_CMD_REQUEST;       /* 0xF2 Command Request */
  cmd[1]  = cmd_seq_num++;         /* Sequence number */
  cmd[2]  = SH2_CMD_ME_CALIBRATE;  /* 0x07 ME Calibration */
  cmd[3]  = accel;                 /* P0: Accel cal enable */
  cmd[4]  = gyro;                  /* P1: Gyro cal enable */
  cmd[5]  = mag;                   /* P2: Mag cal enable */
  cmd[6]  = 0x00;                  /* P3: Subcommand 0x00 = Configure */
  cmd[7]  = 0x00;                  /* P4: Planar accel cal */
  cmd[8]  = 0x00;                  /* P5: Reserved */
  cmd[9]  = 0x00;                  /* P6: Reserved */
  cmd[10] = 0x00;                  /* P7: Reserved */
  cmd[11] = 0x00;                  /* P8: Reserved */

  return shtp_write(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

/*
 * Send Save DCD command (Report ID 0xF2, Command 0x06).
 * Saves dynamic calibration data to sensor's internal flash.
 */
static bool sh2_save_dcd(void)
{
  uint8_t cmd[12];

  cmd[0]  = SH2_CMD_REQUEST;       /* 0xF2 Command Request */
  cmd[1]  = cmd_seq_num++;         /* Sequence number */
  cmd[2]  = SH2_CMD_DCD_SAVE;      /* 0x06 Save DCD */
  cmd[3]  = 0x00;                  /* P0~P8: all reserved */
  cmd[4]  = 0x00;
  cmd[5]  = 0x00;
  cmd[6]  = 0x00;
  cmd[7]  = 0x00;
  cmd[8]  = 0x00;
  cmd[9]  = 0x00;
  cmd[10] = 0x00;
  cmd[11] = 0x00;

  return shtp_write(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

void bno085_calibrate_begin(void)
{
  /* Enable dynamic calibration for all three sensors */
  cal_cmd_status = 0xFF;
  sh2_send_me_calibrate(1, 1, 1);
  uart1_printf("[BNO085] Calibration started (rotate in figure-8)\r\n");
}

void bno085_calibrate_end(void)
{
  /* Disable all dynamic calibration to lock values */
  sh2_send_me_calibrate(0, 0, 0);
  uart1_printf("[BNO085] Calibration stopped\r\n");
}

bool bno085_calibrate_save(void)
{
  if (!sh2_save_dcd())
  {
    uart1_printf("[BNO085] DCD save FAILED (SPI error)\r\n");
    return false;
  }
  uart1_printf("[BNO085] DCD saved to flash!\r\n");
  return true;
}

bno085_cal_accuracy_t bno085_get_cal_accuracy(void)
{
  return (bno085_cal_accuracy_t)mag_cal_status;
}
