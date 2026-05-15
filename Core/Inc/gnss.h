#ifndef __GNSS_H__
#define __GNSS_H__

#include "main.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct
{
  uint32_t i_tow_ms;
  int32_t longitude_deg_1e7;
  int32_t latitude_deg_1e7;
  int32_t height_mm;
  int32_t height_msl_mm;
  uint32_t horizontal_accuracy_mm;
  uint32_t vertical_accuracy_mm;
  int32_t north_velocity_mm_s;
  int32_t east_velocity_mm_s;
  int32_t down_velocity_mm_s;
  int32_t ground_speed_mm_s;
  int32_t heading_motion_deg_1e5;
  uint32_t speed_accuracy_mm_s;
  uint32_t heading_accuracy_deg_1e5;
  uint16_t position_dop_0p01;
  uint8_t fix_type;
  uint8_t fix_ok;
  uint8_t satellites_used;
} gnss_pvt_t;

HAL_StatusTypeDef gnss_init(void);
void gnss_reset(void);
bool gnss_read_line(char *buffer, size_t buffer_size);
bool gnss_read_pvt(gnss_pvt_t *pvt);
void gnss_set_bridge_mode(uint8_t active);
void gnss_handle_uart_error(UART_HandleTypeDef *huart);

#endif
