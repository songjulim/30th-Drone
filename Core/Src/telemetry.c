#include "telemetry.h"
#include "lora.h"
#include "packet.h"
#include "n6_rcv.h"
#include "gnss.h"
#include <stdint.h>



#define GPS_SEND_PERIOD 1500  // gps 보대는 주기 ---> 이거 안씀
#define RX_WAIT_PERIOD 780  // RX 기다리는 주기   --> 124byte 보내는 기준으로 sf->7 기준 airtime은 이론상 650ms정도여서 여유있게 750ms으로 둠
#define TELEMETRY_RSSI_DEBUG_ENABLE 0  // CTRL 숫자 테스트 중에는 RSSI UART 출력이 섞이지 않게 0으로 둠


static uint8_t gps_seq = 0;
static uint32_t last_gps_send_tick = 0;
static uint32_t last_rx_wait_tick = 0;
static uint8_t rx_tel_flag = 0;
int tel_rcv = 0;
int tel_r_byte = 0;
static uint8_t tx_retry = 0;
static uint8_t rx_retry = 0;

static uint8_t tx_fail = 0;

// telemetry() 안에서 while(1)로 갇히지 않도록 만든 간단한 상태머신
// 각 상태는 한 번 호출될 때 필요한 확인만 하고 바로 return해서,
// main loop의 가상 드론 제어 함수가 계속 실행될 수 있게 한다.

typedef enum    // enum은 열거형으로 어떤 이름에 상수로서의 의미를 부여하는거다 --> 0부터 쓰면 자동으로 뒤에는 값 안 배정해주면 1증가한 1,2 값이 됨
{
    TEL_STATE_GPS_SEND = 0,
    TEL_STATE_TX_WAIT,
    TEL_STATE_RX_WAIT
} TelemetryState; // tx,rx 관련 루프에 안걸리게 하기위한 플래그들 모음

static TelemetryState tel_state = TEL_STATE_GPS_SEND; // 위의 열거형의 값 0,1,2만 저장 가능한 자료형인 변수 선언(가독성 위햇 이렇게 선언)
static uint8_t telemetry_initialized = 0;
static uint8_t rx_window_active = 0;

static int32_t gps_lat = 0;
static int32_t gps_lon = 0;
static gnss_pvt_t lora_gnss_pvt;
static uint8_t lora_gnss_has_fix = 0;

void telemetry_init(void)
{
	fifo_tel_set(); // 송수를 위한 stanby 모드 변경+ rx,tx주고 받을때 어떤 메모리 쓸지 세팅
    rx_tel_set(); //rx위한 초기 세팅
    GPS_INIT(); //tx 데이터 조정을 위한

    ////////////////////////////////////////////////////////////////////////////////나중에 tx 인터럽트신호 0x40쪽 이용해서 하기

    tel_state = TEL_STATE_GPS_SEND; // 맨 처음 상태는 GPS send 상태
    rx_window_active = 0;
    telemetry_initialized = 1; // 바로 아래에서 이게 텔레메트리 초기 세팅 했으면 아래 조건 다시 안들갈라고 1로
}


void telemetry(void)
{
    if(!telemetry_initialized) // 텔레메트리 초기 세팅안됬으면 다시 해주기(한번해주면 다시는 안들어옴)
    {
        telemetry_init();
    }

	//lora_log("===========================================\r\n");

    N6_RCV_Process();
    N6_RCV_TimeoutCheck();

    // [non-blocking 변경]
    // 예전 telemetry()는 아래 기존 코드처럼 while(1) 안에서 GPS_SEND(), RX_WAIT()를 계속 돌렸다.
    // 그러면 main.c의 while(1)로 돌아가지 않아서, 나중에 드론 제어 함수가 실행되지 못한다.
    // 이제는 현재 상태만 한 번 확인하고 바로 return해서 main loop가 계속 돌도록 한다.

    if(tel_state == TEL_STATE_GPS_SEND) // 1. GPS SEND 타이밍이면 ----> gps를 보내는 설ㅈ정
    {
        GPS_SEND();  // 일단 주기적으로 tx로 gps 보냄  ===> 실제로 보낼때 까지    //   real_tx 안에서 tx 보내는데 최대 1.5초까지 기다려줌
        tel_state = TEL_STATE_TX_WAIT;
        return;
    }

    if(tel_state == TEL_STATE_TX_WAIT) // 2. GPS 보내고 TX_WAIT 모드 --> tx가 제대로 이루어졌는지 검토하는 단계(재시도 할지 or 전송 완료해서 rx로 바꿀지)
    {
        int tx_status = lora_tx_process(); // tx상태에 대해서 반환받음 ==> 1.tx완료 , 2.tx실패, 3. tx중   --> 이 3개중 한개 상태 반환

        if(tx_status == LORA_TX_STATUS_BUSY) // tx처리중이면 바로 나감
        {
            return;
        }

        if(tx_status == LORA_TX_STATUS_FAIL) // tx가 내가 정한 어떤 시간 이상으로 넘어갈때까지 아직 성공이 안됬으면 실패
        {
            if(tx_retry == 0)
            {
                tx_retry = 1;
                tel_state = TEL_STATE_GPS_SEND; // 다시 gps send로 변경
                return;
            }

            tx_retry = 0;
        }

        else if(tx_status == LORA_TX_STATUS_DONE) // tx 완료시
        {
            tx_retry = 0; // 재시도 x
        }

        tel_state = TEL_STATE_RX_WAIT; //tx 끝났으니 rx 모드 변경
        return;
    }

    if(tel_state == TEL_STATE_RX_WAIT) // 다시 돌아와서 rx모드일떄는 일로
    {
        if(RX_WAIT())
        {
            tel_state = TEL_STATE_GPS_SEND; // 그리고 rx 끝났으니 다시 gps 보내는 모드로
        }

        return;
    }

    return;

#if 0
    // [기존 blocking 코드 백업]
    // 아래 코드는 telemetry() 안에서 while(1)을 돌던 원래 방식이다.
    // 현재 테스트에서는 main loop로 계속 돌아가야 하므로 비활성화했다.

	fifo_tel_set(); // 송수신 위한 stanby 모드 변경 + rx,tx주고 받을때 어떤 메모리 쓸지 세팅
    rx_tel_set(); //rx위한 초기 세팅
    GPS_INIT(); //tx 데이터 조정을 위한


    ////////////////////////////////////////////////////////////////////////////////나중에 tx 인터럽트할떄 0x40쪽 이용해서 하기

	while(1)
	{
		//lora_log("===========================================\r\n");


	    N6_RCV_Process();
	    N6_RCV_TimeoutCheck();

	    GPS_SEND();  // 일단 주기적으로 tx로 gps 보냄  ===> 실제로 보낼때 까지    //   real_tx 안에서 tx 보내는데 최대 1.5초까지 기다려줌

	    N6_RCV_Process();
	    N6_RCV_TimeoutCheck();

	    RX_WAIT();

		//lora_log("===========================================\r\n");
	}

#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rx 쪽

uint8_t RX_WAIT(void)
{
    // [non-blocking 변경]
    // 기존 RX_WAIT()는 아래 while(1)에서 RX_WAIT_PERIOD 동안 계속 머물렀다.
    // 이제 첫 호출에서는 RX 모드만 켜고 바로 빠져나가고,
    // 다음 호출부터는 수신 flag/timeout만 확인한 뒤 바로 return한다.

    if(!rx_window_active) // ==> rx_window_active는 초기에 0 / 아래에서 초기 세팅으로 rx 관련 다 set 해줌
    {
        //lora_log("<RX_START>\r\n");
        lora_write(0x01, stanby); // rx 전환을 위해서 stanby로 모드 리셋
        lora_rx_flag = 0;
        rx_retry = 0; // 1.5초 지나기전에 rx끝나면 다시 rx_tel_read 안들어가게 방지하는 플래그
        lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어
        lora_write(0x0D, 0x00); // 새로운 데이터 들어왔을때 다시 0x00부터 읽을 수 있게 fifo 포인터 맞춤
        lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작

        last_rx_wait_tick = HAL_GetTick(); // rx 비동기 기준 시점 저장
        rx_window_active = 1; // 이거 켜서 다시 여기 조건 안들오게
        return 0;
    }

    N6_RCV_Process();
    N6_RCV_TimeoutCheck();

    if(HAL_GetTick() - last_rx_wait_tick < RX_WAIT_PERIOD) // rx 모드 스타트하고 나서 내가 정한 0.78초동안 해당되는동안
    {
        if(TELEMETRY_RSSI_DEBUG_ENABLE)
        {
            uart_send_noise_rssi(); // 매트랩으로 보내기
        }

        if(lora_rx_flag && (rx_retry==0)) //rx 인터럽트로 인한 rx_flag가 켜지고 rx_retry인 rx 재시도 가 0이면
        {
            lora_rx_flag = 0;
            rx_retry = 1;
            rx_tel_read(); // 실제로 레지스터에서 읽어옴
        }

        return 0;
    }

    // RX시간이 끝났을 때만 LoRa를 정리하고 다음 상태로 넘어가게 한다.
    lora_write(0x01, stanby);
    lora_write(0x12, 0xFF);
    lora_rx_flag = 0;
    rx_window_active = 0;
    return 1;

#if 0
    // [기존 blocking 코드 백업]
    // 아래 코드는 RX_WAIT_PERIOD 동안 while로 머물던 원래 방식이다.
    // 현재 테스트에서는 RX window를 여러 main loop 호출에 나눠서 처리한다.

	//lora_log("<RX_START>\r\n");
	lora_write(0x01, stanby); // rx 전환을 위해서 stanby로 모드 리셋
	lora_rx_flag = 0;
	rx_retry =0; // 1.5초 지나기전에 rx끝나면 다시 rx_tel_read 안들가게 방지용 플래그
	lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어lora_write(0x12, 0xFF);
	lora_write(0x0D, 0x00); // 새로운 데이터 들오니깐 다시 0x00부터 써지니깐 fifo 포인터를 맨처음으로 옮겨서 여기서 부터 읽을라고
	lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작

	last_rx_wait_tick = HAL_GetTick(); // rx 비교 기준 시점을 저장
	/*
	while(1)
	{
		uint32_t now2 = HAL_GetTick(); //  HAL_GetTick() : 보드에 전원이 켜진 후 몇 밀리초(ms)가 지났는지 알려주는 타이머 --> 이 함수가 호출되면 전원 켜지고 몇초 지났는지 시간값 반환
		if(now2 - last_rx_wait_tick < RX_WAIT_PERIOD) // ********************* 여기서 1.5초동안 기다리니깐 이 시간이 rx대기 시간이자, tx의 주기가 된다
		{

			uart_send_noise_rssi();

			if(lora_rx_flag && (rx_retry==0))   // 이러면 rx_retry때문에 rx한주기당 딱 한번만 받을수있음
		    {
		    	lora_rx_flag = 0; //////////////// ====> 만약 연속으로 계속 들오면 어케 처리???????????????????????????????????????????????????????????????????????????????????
		    	rx_retry =1;
		    	rx_tel_read();
		    	//break;
		    }

		    continue;
		}
		//lora_log("WAIT_TIME : %d s \r\n", (now2 - last_rx_wait_tick)/100 );
		//lora_log("RX_nothing....\r\n\n");
		break; // rx로 지정된 시간동안 들온게 없으면, 혹은 전송 끝나면
	}*/
	while(1)
	{
	    uint32_t now2 = HAL_GetTick();

	    N6_RCV_Process();
	    N6_RCV_TimeoutCheck();

	    if(now2 - last_rx_wait_tick < RX_WAIT_PERIOD)
	    {
	        uart_send_noise_rssi(); // 매트랩으로 보내는거

	        if(lora_rx_flag && (rx_retry==0))
	        {
	            lora_rx_flag = 0;
	            rx_retry = 1;
	            rx_tel_read();
	        }

	        continue;
	    }

	    break;
	}
	// 안전빵으로 한번 더 /  혹시 다른 함수 추가해서 구조 바뀔때 이거 까먹어서 오류 안나게
	lora_write(0x01, stanby);
	lora_write(0x12, 0xFF);
	lora_rx_flag = 0;
#endif
}

void rx_tel_read(void) ////////////////////////////////////////////////// 테라텀에서 확인하는 함수
{
	tel_r_byte = 0;
	rx_tel_flag = lora_read(0x12);


	if(rx_tel_flag&0x20) //PayloadCrcError발생시 탈출
	{
		//lora_log("PayloadCrcError\r\n\n");
		lora_write(0x01,stanby);
		lora_write(0x12, 0xFF);
		return;
	}


	if(rx_tel_flag & 0x40)  // 무언가 수신이 됬다면(RXDONE 비트 뜨면)
	{
		lora_write(0x01,stanby); //수신 끝나서 stanby로

		rssi_graph(); // 매트랩으로 보는거

		tel_r_byte = lora_read(0x13); // RegRxNbBytes로 수신되니 데이터 길이(byte수) 있음
		lora_write(0x0D, lora_read(0x10));//RegFifoAddrPtr(0x0D)로 수신데이터 읽기 위해서 포인터를  RegFifoRxCurrentAddr(0x10)이용해서 → 가장 마지막으로 수신된 패킷의 FIFO 시작 주소로 이동
		//lora_log("receive byte : %d\r\n",tel_r_byte);
		//lora_log("rx_success!!!\r\n\n receive data : ");
	}

    if(tel_r_byte > 0)
    {
        uint8_t tel_rx_buf[256];
        if(tel_r_byte > sizeof(tel_rx_buf))
        {
            tel_r_byte = sizeof(tel_rx_buf);
        }

        if(lora_read_payload_dma(tel_rx_buf, (uint8_t)tel_r_byte) == HAL_OK)
        {
            for(int i = 0; i < tel_r_byte; i++) // 받은 한 패킷길이만큼 읽기
            {
                tel_rcv = tel_rx_buf[i]; // RegFifo burst DMA로 한번에 읽어서 SPI 점유 시간을 줄임
                //lora_log(" %02X",tel_rcv);
            }
        }
    }


	 lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////GPS 보내는거 관련

/* GPS 송신 초기화 */
void GPS_INIT(void)
{
    gps_seq = 0;
    //last_gps_send_tick = HAL_GetTick(); // tx할때 용              ===> 빼도 되지않나?

    gps_lat = 0;
    gps_lon = 0;
    lora_gnss_has_fix = 0;

    packet_clear();
}


/* main while문에서 계속 호출해서 주기적으로 보내는 최종 함수  */
void GPS_SEND(void)
{
    // [non-blocking 변경]
    // GPS_SEND()도 while로 재시도하지 않고, 이번 loop에서는 GPS payload 생성 + TX 시작 요청만 한다.
    // TX 완료/timeout/재시도 판단은 telemetry()의 TEL_STATE_TX_WAIT에서 처리한다.
	lora_write(0x01, stanby);
	last_gps_send_tick = HAL_GetTick(); // 다음 전송 기준 시간 저장

    gps_update_from_gnss();   // 실제 GNSS 값 업데이트
    if(!lora_gnss_has_fix)
    {
        packet_clear(); // GNSS fix 없으면 가짜 좌표를 만들지 않고 이번 GPS TX는 넘김
        tx_fail = 1;
        return;
    }

    gps_packet();   // 받은거 패킷화하는과정

    //lora_log("<GPS_DATA>\r\n");
    //lora_log("GPS TX : seq=%d, len=%d, lat=%ld, lon=%ld\r\n",gps_seq - 1,idx,gps_lat,gps_lon);   /////////////////////////보내지는지 확인

    payload_write(tx_buf, idx);    //보낼 데이터를 lora 레지스터 payload에 넣기
    tx_fail = real_tx();   // 이제는 TX 시작만 하고 바로 return함
    return;

#if 0
    // [기존 blocking/재시도 코드 백업]
    // 아래 코드는 함수 안에서 while(1)을 돌며 실패 시 재시도하던 원래 방식이다.
    // 현재 테스트에서는 main loop가 막히지 않도록 telemetry 상태머신이 재시도를 담당한다.
	lora_write(0x01, stanby);
	last_gps_send_tick = HAL_GetTick(); // 이 다음이 바로 또 tx 시작하는 시점이니깐 이때 끝나는 시점의 시간을 tx 비교 기준 시점dmfh 저장

	while(1)
	{
		/*uint32_t now1 = HAL_GetTick();       //  HAL_GetTick() : 보드에 전원이 켜진 후 몇 밀리초(ms)가 지났는지 알려주는 타이머 --> 이 함수가 호출되면 전원 켜지고 몇초 지났는지 시간값 반환

    	// 현재 시간에서 마지막 신호 보냈을때의 시간값을 빼서  그 시간이 내가 정한 GPS_SEND_PERIOD_MS보다 클때 만 tx되게 ==> 그니깐 강제로 1.5초 정도 여기에 루프를 가줘서 시간만큼 기다린다음 아래로 내려가서 한번 딱 보내는거 (tx 전송 주기 맞추는 용도)
    	if(now1 - last_gps_send_tick < GPS_SEND_PERIOD)
        	continue;
        */
    	gps_update_from_gnss();   // 실제 GNSS 값 업데이트
    	gps_packet();   // 받은걸 압축하는과정

    	//lora_log("<GPS_DATA>\r\n");
    	//lora_log("GPS TX : seq=%d, len=%d, lat=%ld, lon=%ld\r\n",gps_seq - 1,idx,gps_lat,gps_lon);   /////////////////////////보내지는지 확인용

    	/*실제로 이제 보내는거*/
    	payload_write(tx_buf, idx);    //보낼 데이터를 lora 레지스터 payload에 넣기
    	tx_fail = real_tx();   //실제로 보내는 작업    -------------> 이 real_tx 안에서 txdone될떄까지 1.5초정도 기다리는게 있어서 tx전송 시간은 최대 1.5초가 될수있음

    	if(tx_fail ==1) // real_tx에서 tx 실패시 1 반환 / tx_fail이 1  ==> 이안에서 몇포동안 보내기 실패하면 tx_tfail=1 반환하는거 되있음  =====================> 나중에 이거가지고 실패시 어케할지 세팅
    	{

    		if(tx_retry == 1) // 실패시 한번 더 전송 재시도했는데 그래도 안되면 그냥 tx 관두기
    		{
    			tx_retry = 0;
    			//lora_log("tx_fail ..... \r\n");
    			return;
    		}

    		tx_retry +=1;
    		//lora_log("tx_retrying\r\n");
    		continue; // 다시 gps 업데이트하고 보내기 시도
    	}

    	else // tx 성공하면
    	{
    		tx_retry = 0;
    		return;
    	}
	}
#endif
}


/* 실제 GNSS 위치 업데이트 */
void gps_update_from_gnss(void)
{
    if(gnss_read_pvt(&lora_gnss_pvt))
    {
        gps_lat = lora_gnss_pvt.latitude_deg_1e7;
        gps_lon = lora_gnss_pvt.longitude_deg_1e7;
        lora_gnss_has_fix = lora_gnss_pvt.fix_ok;
    }

}


void gps_packet(void)   ////// gps데이터 압축 하는 함수
{
    /*
     * 사람 검출 이벤트 중복 방지용 static 변수
     *
     * static이라서 gps_packet() 함수가 끝나도 값이 유지됨.
     * 즉, 이전 GPS 송신 때의 N6 상태를 기억해두고
     * 현재 N6 상태와 비교할 수 있음.
     */
    static uint8_t prev_detected = 0;
    static uint8_t prev_count = 0;

    // n6에서 빼온 검출여부랑 사람수를 여기에 또 복사해옴
    uint8_t now_detected = N6_RCV_GetDetected(); // == n6_detected
    uint8_t now_count = N6_RCV_GetPersonCount(); // == n6_person_count

    /*
     * 실제 GPS 패킷에 실어 보낼 값
     *
     * 기본값은 0,0으로 둔다.
     * 즉, 특별한 새 사람 발견 이벤트가 없으면
     * GPS 패킷 끝에는 항상 00 00이 붙는다.
     */
    uint8_t send_detected = 0;
    uint8_t send_count = 0;

    packet_clear();

    /*
     * [id][seq][comp][len][lat][lon]
     */
    put_uint8(ID_DRONE);     // 드론에서 보내는 신호인 flag
    put_uint8(gps_seq++);    // 몇 번째 보내는 패킷인지
    put_uint8(MSG_GPS);      // GPS 메시지
    put_uint8(1);            // GPS 좌표 1개

    put_int32(gps_lat);
    put_int32(gps_lon);

    /*
     * ------------------------------------------------------------
     * 사람 검출 이벤트 처리
     * ------------------------------------------------------------
     *
     * 기존 방식:
     *   N6_GetDetectedFlag(), N6_GetPersonCount()를 매번 그대로 전송
     *
     * 문제:
     *   사람이 계속 잡히면 GPS마다 01 01이 계속 나가서
     *   GUI에서 마커가 계속 찍힐 수 있음.
     *
     * 수정 방식:
     *   이전 N6 상태와 현재 N6 상태가 달라졌을 때만 이벤트로 판단.
     *
     * 예:
     *   0명 -> 1명 : 01 01 한 번 전송
     *   1명 -> 1명 : 00 00 전송
     *   1명 -> 2명 : 01 02 한 번 전송
     *   1명 -> 0명 : 00 00 전송
     * ------------------------------------------------------------
     */
    if ((now_detected != prev_detected) || (now_count != prev_count)) // 지금 검출여부랑 이전검출 여부가 다르거나 or 지금 사람수랑 이전 사람수라 다를떄만 처리 (n6쪽에서 똑같은 정보 계속 보내면 정보 미어터져서 상태 바뀔떄만 전송하기 위함)
    {
        if (now_detected == 1 && now_count > 0) // 지금 사람이 감지됬고, 사람이 1명이상 감지되면(이것도 오류 방지 , 검출안됬는데 사람은 뜬 오류 or 검출됬는데 사람은 0명인 오류)
        {
            send_detected = 1;       // 감지 on
            send_count = now_count; //  지금 사람수를 보낼 변수에 담음
        }

        prev_detected = now_detected;  // 그리고 비교를 위해서 지금 검출여부와 사람수를 이전 검출여부,사람수 변수에 넣음
        prev_count = now_count;
    }

    // 그리고 실제로 보냄
    put_uint8(send_detected);
    put_uint8(send_count);
}
/* 기존 코드
void gps_packet(void)   ////// gps데이터 압축 하는 함수
{
    packet_clear();

    //[id][seq][comp][len][lat][lon]

    put_uint8(ID_DRONE); // 드론에서 보내는 신호인 flag
    put_uint8(gps_seq++); //몇번째 보내는 패킷인지 (자동으로 +1)
    put_uint8(MSG_GPS); // 보내는 데이터가 gps임을 보내는 데이터
    put_uint8(1);    // GPS 좌표 1개 들어있음의 의미로 len은 1

    put_int32(gps_lat);
    put_int32(gps_lon);

    put_uint8(N6_GetDetectedFlag()); // 사람검출 플래그
    put_uint8(N6_GetPersonCount()); // 검출 사람 수 플래그
}*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fifo_tel_set(void) //payload 세팅
{
    lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, fifo는 stanby에서만 접근 가능

    lora_write_burst(0x0E,0X80,0X00,no_more); //송수신데이터 저장 메모리 위치 설정
    //lora_write(0x0E,0X80);  //RegFifoTxBaseAddr : 송신 데이터(Tx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x80)
    //lora_write(0x0F,0X00);  //RegFifoRxBaseAddr : 수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
}


void rx_tel_set(void)  // 실제 텔레메트리 전송용 세팅
{
	lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어
	lora_write(0x0D, 0x00); // RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정
	lora_write(0x40, 0x00); // D0을 rxdone의 인터럽트로 쓰겠다
	//lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


