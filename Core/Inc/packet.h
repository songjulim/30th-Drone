#ifndef PACKET_H
#define PACKET_H

#include "main.h"
#include <stdint.h>

/*
 * 공통 패킷 정의
 * LoRa 최종 패킷 형식:
 *
 * [id][seq][comp][len][payload...]
 */

#define ID_GROUND       0xFF
#define ID_DRONE        0xFE

#define MSG_KEYBOARD    0xF1
#define MSG_WAYPOINT    0xF2
#define MSG_GPS         0xF3

#define PACKET_BUF_SIZE 64

extern uint8_t tx_buf[PACKET_BUF_SIZE];     //gps_send에서 써야해서 extern
extern uint8_t idx;

void packet_clear(void);

void put_uint8(uint8_t val);
void put_uint16(uint16_t val);
void put_int32(int32_t val);

uint8_t* get_tx_buf(void);
uint8_t get_tx_len(void);

#endif
