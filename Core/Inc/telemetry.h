#ifndef TELEMETRY_H
#define TELEMETRY_H


#include "stdio.h"
#include "lora.h"
#include "n6_rcv.h"
#include "gps_send.h"
#include "main.h"
#include <stdint.h>

void telemetry (void);
void telemetry_init(void);

void GPS_INIT(void);
void GPS_SEND(void);

uint8_t RX_WAIT(void);

void fifo_tel_set(void); //payload 세팅
void rx_tel_set(void);  // 실제 텔레메트리 전송용 세팅
void rx_tel_read(void);

void gps_update_from_gnss(void);
void gps_packet(void);

extern volatile uint8_t lora_rx_flag;  // rx 인터럽트 들오면 켜지는 플래그



#endif
