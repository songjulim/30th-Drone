#include "packet.h"
#include <string.h>

/*
 * packet.c
 *
 * 역할:
 * - LoRa로 보낼 데이터를 tx_buf에 바이트 단위로 직렬화
 * - 구조체를 그대로 보내지 않고 직접 쪼개서 넣음
 * - padding/alignment 문제 방지
 * - little-endian 기준
 */

uint8_t tx_buf[PACKET_BUF_SIZE];
uint8_t idx = 0;


/* 패킷 버퍼 초기화 */
void packet_clear(void)
{
    idx = 0;
    memset(tx_buf, 0, sizeof(tx_buf));
}


/* 1바이트 넣기 */
void put_uint8(uint8_t val)
{
    if(idx >= PACKET_BUF_SIZE)
    {
        return;
    }

    tx_buf[idx++] = val;
}


/* 2바이트 uint16_t 넣기, little-endian */
void put_uint16(uint16_t val)
{
    if(idx + 2 > PACKET_BUF_SIZE)
    {
        return;
    }

    tx_buf[idx++] = (uint8_t)(val & 0xFF);
    tx_buf[idx++] = (uint8_t)((val >> 8) & 0xFF);
}


/* 4바이트 int32_t 넣기, little-endian */
void put_int32(int32_t val)
{
    if(idx + 4 > PACKET_BUF_SIZE) // put_int32()는 4바이트를 넣는 함수여서 , 넣을 데이터가 많아지면 즉, idx가 커지다가 남은 공간이 4개보다 작으면 , 최소한 tx_buf에 빈 칸 4개가 남아 있어야 하니깐 더 못들가서 return으로 빠져나옴
    {
        return;
    }

    tx_buf[idx++] = (uint8_t)(val & 0xFF);
    tx_buf[idx++] = (uint8_t)((val >> 8) & 0xFF);
    tx_buf[idx++] = (uint8_t)((val >> 16) & 0xFF);
    tx_buf[idx++] = (uint8_t)((val >> 24) & 0xFF);
}


/* 송신 버퍼 주소 반환 */
uint8_t* get_tx_buf(void)
{
    return tx_buf;
}


/* 현재까지 패킹된 길이 반환 */
uint8_t get_tx_len(void)
{
    return idx;
}
