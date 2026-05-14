#ifndef __UART_BRIDGE_H__
#define __UART_BRIDGE_H__

#include "main.h"

#include <stdbool.h>

void uart_bridge_init(void);
void uart_bridge_set_active(bool active);
bool uart_bridge_is_active(void);
HAL_StatusTypeDef uart_bridge_enqueue_pc_data(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef uart_bridge_enqueue_gnss_byte(uint8_t data);
void uart_bridge_process(void);

#endif
