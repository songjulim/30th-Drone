#include "gps_send.h"
#include "lora.h"
#include "packet.h"
#include <stdint.h>

/////////////////////////////////////////////////// 지상국쪽 코드에서 packet.cpp에서 데이터 보내는쪽 부분

/*
 * STM -> 지상국 GPS 송신 패킷
 *
 * 형식:
 * [id][seq][comp][len][lat 4byte][lon 4byte]
 *
 * id   = ID_DRONE
 * seq  = GPS 패킷 번호
 * comp = MSG_GPS
 * len  = GPS 좌표 개수, 지금은 1
 * lat  = 위도 * 1e7
 * lon  = 경도 * 1e7
 */

#define GPS_SEND_PERIOD_MS 1500  // gps 보대는 주기

static uint8_t gps_seq = 0;
static uint32_t last_gps_send_tick = 0;

/*
 * 임의 GPS 초기값
 * 37.1234567, 127.1234567
 */
static int32_t fake_lat = 373207900;
static int32_t fake_lon = 1271258800;


/* 임의 GPS 위치 업데이트 */
static void gps_fake_update(void)
{
    /*
     * e7 단위
     * 100 = 0.0000100도
     * 150 = 0.0000150도
     */
    fake_lat += 100;
    fake_lon += 150;
}



static void gps_packet_make(void)   ////// gps데이터 압축 하는 함수
{
    packet_clear();
    /*
     * [id][seq][comp][len][lat][lon]
     */
    put_uint8(ID_DRONE); // 드론에서 보내는 신호인 flag
    put_uint8(gps_seq++); //몇번째 보내는 패킷인지 (자동으로 +1)
    put_uint8(MSG_GPS); // 보내는 데이터가 gps임을 보내는 데이터
    put_uint8(1);    // GPS 좌표 1개 들어있음의 의미로 len은 1

    put_int32(fake_lat);
    put_int32(fake_lon);
}


/* GPS 송신 초기화 */
void GPS_Send_Init(void)
{
    gps_seq = 0;
    last_gps_send_tick = HAL_GetTick();

    fake_lat = 373207900;
    fake_lon = 1271258800;

    packet_clear();
}


/* main while문에서 계속 호출해서 주기적으로 보내는 최종 함수  */
void GPS_Send_Task(void)
{
    uint32_t now = HAL_GetTick();       //  HAL_GetTick() : 보드에 전원이 켜진 후 몇 밀리초(ms)가 지났는지 알려주는 타이머 --> 이 함수가 호출되면 전원 켜지고 몇초 지났는지 시간값 반환

    // 현재 시간에서 마지막 신호 보냈을때의 시간값을 빼서  그 시간이 내가 정한 GPS_SEND_PERIOD_MS보다 클때 만 tx되게
    if(now - last_gps_send_tick < GPS_SEND_PERIOD_MS)
        return;

    last_gps_send_tick = now;

    gps_fake_update();   // gps 임의로 업데이트 ---> 나중에는 여기에다가 실제 gps 값 넣는함수 넣으면 됨 ********************
    gps_packet_make();   // 받은걸 압축하는과정

    /*
     * lora.c에 있어야 하는 함수:
     * void lora_send_packet(uint8_t *payload, uint8_t len);
     */
    lora_log("GPS TX : seq=%d, len=%d, lat=%ld, lon=%ld\r\n",gps_seq - 1,idx,fake_lat,fake_lon);   /////////////////////////보내지는지 확인용

    /*실제로 이제 보내는거*/
    payload_write(tx_buf, idx);    //보낼 데이터를 lora 레지스터 payload에 넣기
    real_tx();   //실제로 보내는 작업
}






