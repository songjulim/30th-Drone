#ifndef __GNSS_H__
#define __GNSS_H__

#include "main.h"

#include <stdbool.h>
#include <stddef.h>

HAL_StatusTypeDef gnss_init(void);
void gnss_reset(void);
bool gnss_read_line(char *buffer, size_t buffer_size);
void gnss_handle_uart_error(UART_HandleTypeDef *huart);

#endif
