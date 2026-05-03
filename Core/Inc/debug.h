#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"

void debug_init(void);
void debug_process(void);
int uart1_printf(const char *format, ...);

extern volatile float uart1_rx_float_value;

extern volatile uint8_t debug_uart_update_flag;
extern volatile uint8_t debug_oled_update_flag;
extern volatile uint8_t debug_oled_tick_divider;
extern volatile uint32_t main_flag;

#endif
