#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"

void debug_init(void);
void debug_process(void);
void debug_set_bridge_mode(uint8_t active);
int uart1_printf(const char *format, ...);
HAL_StatusTypeDef debug_uart1_write_raw(const uint8_t *data, uint16_t length);

extern volatile float uart1_rx_float_value;
extern volatile float uart6_rx_float_value;

extern volatile uint8_t debug_uart_update_flag;
extern volatile uint8_t debug_uart6_update_flag;
extern volatile uint8_t debug_oled_update_flag;
extern volatile uint8_t debug_oled_tick_divider;
extern volatile uint32_t main_flag;

#endif
