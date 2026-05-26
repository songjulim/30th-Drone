#include <lora.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define LORA_UART_DEBUG_ENABLE 0  // 실제 드론 주행 통합 버전에서는 LoRa 디버그 UART6 출력 비활성화

//////////////////////////////////////////////////////////////////////////////////////////////// 2. 전역 변수 + 프로토타입

char data[200]="";  				// sprintf로 시리얼로 보내기 위한 문자열을 저장하는 배열 --> 또한 뒤에서 HAL_UART_Transmit가 원라는 시작주소의 자료형 타입이 uint8_t여서 char말고 uint8로 선언
uint8_t rxflag = 0;
int rcv = 0;
int r_byte = 0;

static int snd = 0;
static int cnt __attribute__((unused)) = 0;
uint8_t txflag = 0;

#define LORA_SPI_DMA_TIMEOUT_MS 5
#define LORA_SPI_DMA_BUF_SIZE 260
#define LORA_TX_IRQ_POLL_PERIOD_MS 5

#if LORA_UART_DEBUG_ENABLE
static uint32_t last_rssi_tick = 0;
#endif
static uint8_t rx_buf[256];
static uint8_t lora_spi_tx_buf[LORA_SPI_DMA_BUF_SIZE];
static uint8_t lora_spi_rx_buf[LORA_SPI_DMA_BUF_SIZE];
static volatile uint8_t lora_spi_dma_busy = 0;
static volatile uint8_t lora_spi_dma_done = 0;
static volatile uint8_t lora_spi_dma_error = 0;
static volatile HAL_StatusTypeDef lora_spi_last_status = HAL_OK;
static volatile uint32_t lora_spi_error_count = 0;
static volatile uint32_t lora_spi_timeout_count = 0;
static uint8_t lora_async_rx_value = 0;
static uint8_t lora_async_read_active = 0;

// TX_DONE을 기다리는 while문 때문에 main loop가 멈추는 문제를 보기 위해 추가한 상태값
// real_tx()는 이제 TX 시작만 하고 바로 return, lora_tx_process()가 매 loop마다 완료 여부만 확인한다.
static uint8_t lora_tx_active = 0;
static uint32_t lora_tx_start_tick = 0;
static uint32_t lora_tx_irq_last_poll_tick = 0;
static uint8_t lora_tx_irq_read_pending = 0;

#if LORA_UART_DEBUG_ENABLE
static void uart_print_hex(uint8_t *buf, uint8_t len);
static int16_t lora_get_current_rssi(void);
static int16_t lora_get_packet_rssi(void);
static int16_t lora_get_packet_snr_x100(void);
#endif
static HAL_StatusTypeDef lora_spi_dma_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
static HAL_StatusTypeDef lora_spi_dma_start(const uint8_t *tx_data, uint16_t len);
static HAL_StatusTypeDef lora_read_async_start(uint8_t adr);
static uint8_t lora_read_async_is_done(void);
static uint8_t lora_read_async_get(void);
static HAL_StatusTypeDef lora_write_fifo_dma(const uint8_t *buf, uint8_t len);
static HAL_StatusTypeDef lora_read_fifo_dma(uint8_t *buf, uint8_t len);


////////////////////////////////////////////////////////////////////////////////////////////////

// --- 함수 구현부 (main.c에서 잘라내기해서 붙여넣기) ---

void lora_log(const char *fmt, ...) {          //printf처럼 그냥 문자열 프린트하고싶을떄
#if LORA_UART_DEBUG_ENABLE
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
#else
    (void)fmt; // 실제 주행 통합에서는 UART6 로그 송신을 하지 않음
#endif
}

static HAL_StatusTypeDef lora_spi_dma_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    uint32_t start;
    HAL_StatusTypeDef status;

    if((len == 0) || (len > LORA_SPI_DMA_BUF_SIZE))
    {
        lora_spi_last_status = HAL_ERROR;
        lora_spi_error_count++;
        return HAL_ERROR;
    }

    if(lora_spi_dma_busy)
    {
        lora_spi_last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    if(tx_data != lora_spi_tx_buf)
    {
        memcpy(lora_spi_tx_buf, tx_data, len);
    }
    memset(lora_spi_rx_buf, 0, len);

    lora_spi_dma_busy = 1;
    lora_spi_dma_done = 0;
    lora_spi_dma_error = 0;

    HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);  //CS LOW
    status = HAL_SPI_TransmitReceive_DMA(&hspi4, lora_spi_tx_buf, lora_spi_rx_buf, len);
    if(status != HAL_OK)
    {
        HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
        lora_spi_dma_busy = 0;
        lora_spi_last_status = status;
        lora_spi_error_count++;
        return status;
    }

    start = HAL_GetTick();
    while(!lora_spi_dma_done)
    {
        if(lora_spi_dma_error)
        {
            HAL_SPI_Abort(&hspi4);
            HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
            lora_spi_dma_busy = 0;
            lora_spi_last_status = HAL_ERROR;
            lora_spi_error_count++;
            return HAL_ERROR;
        }

        if(HAL_GetTick() - start > LORA_SPI_DMA_TIMEOUT_MS)
        {
            HAL_SPI_Abort(&hspi4);
            HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
            lora_spi_dma_busy = 0;
            lora_spi_last_status = HAL_TIMEOUT;
            lora_spi_timeout_count++;
            return HAL_TIMEOUT;
        }
    }

    if((rx_data != NULL) && (rx_data != lora_spi_rx_buf))
    {
        memcpy(rx_data, lora_spi_rx_buf, len);
    }

    lora_spi_last_status = HAL_OK;
    return HAL_OK;
}

static HAL_StatusTypeDef lora_spi_dma_start(const uint8_t *tx_data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if((len == 0) || (len > LORA_SPI_DMA_BUF_SIZE))
    {
        lora_spi_last_status = HAL_ERROR;
        lora_spi_error_count++;
        return HAL_ERROR;
    }

    if(lora_spi_dma_busy)
    {
        lora_spi_last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    memcpy(lora_spi_tx_buf, tx_data, len);
    memset(lora_spi_rx_buf, 0, len);

    lora_spi_dma_busy = 1;
    lora_spi_dma_done = 0;
    lora_spi_dma_error = 0;

    HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);  //CS LOW
    status = HAL_SPI_TransmitReceive_DMA(&hspi4, lora_spi_tx_buf, lora_spi_rx_buf, len);
    if(status != HAL_OK)
    {
        HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
        lora_spi_dma_busy = 0;
        lora_spi_last_status = status;
        lora_spi_error_count++;
        return status;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef lora_read_async_start(uint8_t adr)
{
    uint8_t tx_buffer[2] = {(read_bit | adr), 0x00};
    HAL_StatusTypeDef status;

    if(lora_async_read_active)
    {
        return HAL_BUSY;
    }

    status = lora_spi_dma_start(tx_buffer, 2);
    if(status == HAL_OK)
    {
        lora_async_read_active = 1;
    }

    return status;
}

static uint8_t lora_read_async_is_done(void)
{
    if(!lora_async_read_active)
    {
        return 0;
    }

    if(lora_spi_dma_error)
    {
        lora_spi_last_status = HAL_ERROR;
        lora_spi_error_count++;
        lora_async_read_active = 0;
        return 1;
    }

    if(!lora_spi_dma_done)
    {
        return 0;
    }

    lora_async_rx_value = lora_spi_rx_buf[1];
    lora_spi_last_status = HAL_OK;
    lora_async_read_active = 0;
    return 1;
}

static uint8_t lora_read_async_get(void)
{
    return lora_async_rx_value;
}

static HAL_StatusTypeDef lora_write_fifo_dma(const uint8_t *buf, uint8_t len)
{
    if((len + 1U) > LORA_SPI_DMA_BUF_SIZE)
    {
        return HAL_ERROR;
    }

    lora_spi_tx_buf[0] = write_bit | 0x00;
    memcpy(&lora_spi_tx_buf[1], buf, len);

    return lora_spi_dma_transfer(lora_spi_tx_buf, NULL, len + 1U);
}

static HAL_StatusTypeDef lora_read_fifo_dma(uint8_t *buf, uint8_t len)
{
    if((len + 1U) > LORA_SPI_DMA_BUF_SIZE)
    {
        return HAL_ERROR;
    }

    memset(lora_spi_tx_buf, 0, len + 1U);
    lora_spi_tx_buf[0] = read_bit | 0x00;

    if(lora_spi_dma_transfer(lora_spi_tx_buf, lora_spi_rx_buf, len + 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    memcpy(buf, &lora_spi_rx_buf[1], len);
    return HAL_OK;
}

HAL_StatusTypeDef lora_read_payload_dma(uint8_t *buf, uint8_t len)
{
    return lora_read_fifo_dma(buf, len);
}

void LORA_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI4)
    {
        HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
        lora_spi_last_status = HAL_OK;
        lora_spi_dma_done = 1;
        lora_spi_dma_busy = 0;
    }
}

void LORA_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI4)
    {
        HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
        lora_spi_dma_busy = 0;
        lora_spi_last_status = HAL_ERROR;
        lora_spi_error_count++;
        lora_spi_dma_error = 1;
    }
}

void LORA_DIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SPI4_INT_Pin)
    {
        lora_rx_flag = 1;
    }
}


void lora_write(uint8_t adr,uint8_t data)
{
	uint8_t tx_buffer[2] = {(write_bit|adr) , data};
	lora_spi_dma_transfer(tx_buffer, NULL, 2);
}

void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3) //레지스터 길이가 2byte보다 길어서 한번에 BURST access방식으로 레지스터에 넣어줌(주소가 이어진 레지스터의 상황에서만 가능)
{
	uint8_t tx_buffer[4] = {(write_bit|adr) , data1 , data2 , data3};
	if(data3 != no_more)  lora_spi_dma_transfer(tx_buffer, NULL, 4);
	else if(data3==no_more)  lora_spi_dma_transfer(tx_buffer, NULL, 3); // 3개까지 burst하고싶지않을때 no_more 인자 주면됨
}

uint8_t lora_read(uint8_t adr) //레지스터 접근해서 읽을때
{
	uint8_t rx_buffer[2] = {(read_bit|adr) , 0x00}; //차피 레지스터로 mosi로 1bit보내면 miso로 1bit 나오니깐 결국 이 배열엔는 보낸자리에 새 데이터가 차는거니깐 배열한개로 tx로 보내면서 rx로 보낸자리ㅣ를 채우는 방식으로 가능
	lora_spi_dma_transfer(rx_buffer, rx_buffer, 2);
	//HAL_SPI_TransmitReceive함수가 spi.tranfer처럼 보내고 바로 반환값을 3번쨰 인자에 반환하는함수임
	// 이 함수 들가면 2,3번째 인자로 주소값을 줘야하므로 바로 값을 못넣고 변수나,문자열에 넣어서 주소를 넘겨줘야한다.
	//4번째 인자는 몇 바이트 보내고 통신종료할지--> 이거는 레지스터마다 다르니깐 바꿔주기ㅣ******
	//이떄 들어가는 tx,rx의 인자는 tx는 뭐 알아서 하면되는데 rx는 무조건 배열로 해야한다
	//왜냐하면 rx는 2번째 byte타이밍에서 받아오는데 그럴려면 변수로 하면 변수에는 첫 byte에오는 더미데이터를 받아서 두번째 byte타이밍에 오는 값을 그 변수가 못받음 -->받으려면 배열선언해서 두번째 배열요소에 그 응답이 저장되니깐 그걸 읽으면된다
	//sprintf(data, "read data: 0x%02X\r\n",rx_buffer[1]); //시리얼로 보낼 문자열을 data라는 문자열에 담음
	//HAL_UART_Transmit(&huart6, (uint8_t*)data, strlen(data), 100); // uart로 보내기

	return rx_buffer[1];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// lora 기본 세팅 관련

void lora_setup(void)
{
	lora_write(0x09, 0x8F); //출력 관련 세팅 : PA_BOOST 핀(1)--> 안테나쪽으로 출력연결+고출력, MAX POWER : 000 , OUTPUTPOWER : 1111(17dBm)  --> 1000 1111 = 0x8F
	lora_write(0x0B, 0x31); // 과전류 방지 전류 보호회로 세팅(OCP) --> 0011 0001 --> 140mA로 제한

	lora_write(0x11,0x00); //인터럽트 마스크 안해줘서 다 허용(tx 인터럽트 mask 안해주는겸)
	lora_write(0x12, 0xFF); // 안전빵으로 인터럽트 flag 한번씩 다 clear
}


void lora_freq(void) //주파수 922.1MHz로 설정  ( 922100000 / 61.03515625 = 15107686 = E68666)
{
    //주의 : 주파수(LoRa Frequency)를 변경하려면, 칩은 반드시 Sleep 모드나 Standby 모드 상태여야 한다."
    lora_write(0x01,sleep);
    HAL_Delay(10);
    lora_write_burst(0x06,0XE6,0X86,0X66); // 주소, RegFrfMsb, RegFrfMid, RegFrfLsb (burst access로 함)
    //lora_read(0x06);
    //lora_read(0x07);
    //lora_read(0x08);
}

void packet_set(void) //LORA링크 성격 설정 +  속도·신뢰성·수신 동작 방식을 설정 - 이 레지스터들 쓸때 무조건 SLEEP이나 STDBY상태여야함
{
    //sprintf(data, "BW(대역폭):125kHz, CR(부호화율):4/5, Explicit Header mode\r\n  SF->9, 통신방식 : 패킷, CRC mode : ON, \r\n\n"); //시리얼로 보낼 문자열을 data라는 문자열에 담음
    //HAL_UART_Transmit(&huart6, (uint8_t*)data, strlen(data), 100); // uart로 보내기

    lora_write(0x1D,0X72);  //RegModemConfig1(0x1D) -> 0111 0010 = 0X72 (BW(대역폭):125kHz, CR(부호화율):4/5, Explicit Header mode)
    //lora_read(0x1D);

    lora_write(0x1E,0X74);  //LoRa의 속도·신뢰성·수신 동작 방식을 정하는 핵심 레지스터  - 이 레지스터 쓸때 무조건 SLEEP이나 STDBY상태여야함
                            //RegModemConfig2(0x1E)_ -> 0111 0100 = 0X94 (SF->7(통신거리,속도),  통신방식 : 패킷으로(분석용x), CRC모드 쓰기, RX TIMEOUT : 0(이건 일단은 0으로 나중에 최적화부분 비트여서))
    //lora_read(0x1E);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  rx관련
void rx_set(void)
{
	lora_write(0x01,stanby); //sleep,stanby모드여야한다*************
	HAL_Delay(10);
	lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어

	lora_write(0x0F,0X00); //수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
	lora_write(0x0D, 0x00); ////RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정
	lora_write(0x40, 0x00); // D0을 rxdone의 인터럽트로 쓰겠다
	lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작
}


void rx_read(void) ////////////////////////////////////////////////// 테라텀에서 확인하는 함수
{

	rxflag = lora_read(0x12); //rxdone 계속 보기
	//lora_log(".\r\n");

	if(rxflag&0x20) //PayloadCrcError발생시 탈출
	{
		lora_log("PayloadCrcError\r\n");
		lora_write(0x01,stanby);
		lora_write(0x12, 0xFF);
		return;
	}


	if(rxflag & 0x40)  // 무언가 수신이 됬다면
	{
		lora_write(0x01,stanby); //수신 끝나서 stanby로
		r_byte = lora_read(0x13); // RegRxNbBytes로 수신되니 데이터 길이(byte수) 있음
		lora_write(0x0D, lora_read(0x10));//RegFifoAddrPtr(0x0D)로 수신데이터 읽기 위해서 포인터를  RegFifoRxCurrentAddr(0x10)이용해서 → 가장 마지막으로 수신된 패킷의 FIFO 시작 주소로 이동
		lora_log("receive byte : %d\r\n",r_byte);
		lora_log("rx_success!!!\r\n receive data : ");
	}

	 for(int i = 0; i < r_byte; i++) //받은 한 패킷길이만큼 읽기
	 {
		 rcv=lora_read(0x00); // RegFifo : 포인터로 가르킨 주소 0x00에서 데이터를 읽음 -> 포인터가 알아서 1씩 증가함
		 lora_log(" %c",rcv);
	 }


	 lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 송수신 관련 set

// 송수신 위한 stanby 모드 변경 + rx,tx주고 받을때 어떤 메모리 쓸지 세팅
void fifo_set(void) //payload 전송전 세팅
{
    lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, fifo는 stanby에서만 접근 가능


    lora_write_burst(0x0E,0X80,0X00,no_more); //송수신데이터 저장 메모리 위치 설정
    //lora_write(0x0E,0X80);  //RegFifoTxBaseAddr : 송신 데이터(Tx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x80)
    //lora_write(0x0F,0X00);  //RegFifoRxBaseAddr : 수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// tx 관련
void payload_write(const uint8_t *buf, uint8_t len) // 실제 보낼 데이터 넣는 설정
{
	 //lora_log("PAYLOAD WRITE: len=%d, data=", len);

	    for(uint8_t i = 0; i < len; i++)
	    {
	        //lora_log("%02X ", buf[i]);
	    }

	    //lora_log("\r\n\n");


	lora_write(0x22,len);  //RegPayloadLength : 보낼 데이터 크기 설정 + header에 싣는 payload 길이

    // 1. 포인터 초기화 (가장 중요!)
    lora_write(0x0D, 0x80); //RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정

    // 2. FOR문으로 데이터 밀어넣기

    lora_write_fifo_dma(buf, len);   // RegFifo : 포인터로 가르킨 주소 0x80에 보낼 데이터를 씀 -> 포인터가 알아서 1씩 증가함

     lora_write(0x0D, 0x80); //위에서 값 쓰면서 포인터 증가해서 읽을라면 다시 처음 쓴 위치로 돌려야 순서대로 읽을수이씨음

    return; // flight path: skip FIFO readback verification to avoid doubling SPI traffic.
    if(lora_read_fifo_dma(rx_buf, len) == HAL_OK)
    {
        for(int i = 0; i < len; i++)
        {
           snd = rx_buf[i]; // 0x00에 접근하면 자동 포인터 증가해서 읽기위해서 이렇게 짬
        }
    }
}


int real_tx(void)  ////////////////// 실제로 보내는곳
{
    //lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, 이제 접근해서 보내야하니 무조건 stanby로
	//lora_log("<TX_START>\r\n");

    // 이미 TX가 진행 중이면 새 TX를 시작하지 않는다.
    // LoRa는 TX/RX를 동시에 못 하므로 main loop 상태머신에서 순서를 보장해야 한다.
    if(lora_tx_active)
    {
        return 1;
    }

    lora_write(0x12, 0xFF); //안전빵으로 들어올 때 인터럽트 플래그 한번 클리어 해줌 + txdone에 1써서 다시 인터럽트 플래그 클리어
    lora_write(0x0D, 0x80); // 포인터 위치 다시 돌림
    lora_write(0x01,tx); // 실제로 보냄 **********************************

    // [non-blocking 변경]
    // 기존에는 아래 while문에서 TxDone flag가 뜰 때까지 최대 TXDONE_WAIT_PERIOD 동안 기다렸다.
    // 실제 드론 주행 코드와 합치면 그 시간 동안 main loop가 막히므로,
    // 여기서는 TX 모드 진입만 하고 바로 return한다.
    // TxDone/timeout 확인은 lora_tx_process()가 main loop에서 조금씩 처리한다.
    lora_tx_start_tick = HAL_GetTick();
    lora_tx_irq_last_poll_tick = 0;
    lora_tx_active = 1;
    lora_tx_irq_read_pending = 0;
    return 0; // TX 시작 성공. 완료 여부는 lora_tx_process()에서 확인

#if 0
    // [기존 blocking 코드 백업]
    // 아래 코드는 TxDone을 while로 기다리던 원래 방식이다.
    // 현재 테스트에서는 main loop가 막히지 않도록 비활성화했다.
	uint32_t start = HAL_GetTick();

    while((lora_read(0x12) & 0x08) == 0)  // tx 보내고 보낼때까지 아래의 TXDONE_WAIT_PERIOD초 동안 기다림  이때까지 txdone 비트 1 안되면 전송 실패로 간주(txdone 인터럽트 1초동안안켜지면 tx 실패 0x08 비트 위치에 txdone 있음)
      {
          if(HAL_GetTick() - start > TXDONE_WAIT_PERIOD )
          {
              //lora_log("TX TIMEOUT\r\n\n");
              lora_write(0x01, stanby);
              lora_write(0x12, 0xFF);
              return 1;
          }
      }

    txflag = lora_read(0x12);

        if(txflag & 0x08)    //txdone 인터럽트 켜지면 전송 성공
        {
            //lora_log("TX DONE: irq=0x%02X\r\n\n", txflag);
        }
        else
        {
            //lora_log("TX FAIL: irq=0x%02X\r\n\n", txflag);
            lora_write(0x01, stanby);
            lora_write(0x12, 0xFF);
            return 1;
        }

      lora_write(0x01, stanby);
      lora_write(0x12, 0xFF); // tx보냈으니깐 txdone 인터럽트 비트 다시 초기화
      return 0; //문제 없으면 0 반환
#endif
}

int lora_tx_process(void)
{
    if(!lora_tx_active) // lora_tx_active는 real_tx에서 tx처리 다했을떄 1로 켜짐 , 송신할게 없으면 tx 보낼거 없다는 시그널 반환
    {
        return LORA_TX_STATUS_IDLE;
    }

    if(!lora_tx_irq_read_pending)
    {
        uint32_t now = HAL_GetTick();

        if((now - lora_tx_irq_last_poll_tick) < LORA_TX_IRQ_POLL_PERIOD_MS)
        {
            return LORA_TX_STATUS_BUSY;
        }

        HAL_StatusTypeDef status = lora_read_async_start(0x12); //LoRa의 IRQ flag 레지스터 확인

        if(status == HAL_BUSY)
        {
            return LORA_TX_STATUS_BUSY;
        }

        if(status != HAL_OK)
        {
            lora_write(0x01, stanby);
            lora_write(0x12, 0xFF);
            lora_tx_active = 0;
            return LORA_TX_STATUS_FAIL;
        }

        lora_tx_irq_read_pending = 1;
        lora_tx_irq_last_poll_tick = now;
        return LORA_TX_STATUS_BUSY;
    }

    if(!lora_read_async_is_done())
    {
        if(HAL_GetTick() - lora_tx_start_tick > TXDONE_WAIT_PERIOD )
        {
            HAL_SPI_Abort(&hspi4);
            HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET); //CS HIGH
            lora_spi_dma_busy = 0;
            lora_async_read_active = 0;
            lora_tx_irq_read_pending = 0;
            lora_write(0x01, stanby);
            lora_write(0x12, 0xFF);
            lora_tx_active = 0;
            lora_spi_last_status = HAL_TIMEOUT;
            lora_spi_timeout_count++;
            return LORA_TX_STATUS_FAIL;
        }

        return LORA_TX_STATUS_BUSY;
    }

    if(lora_spi_last_status != HAL_OK)
    {
        lora_tx_irq_read_pending = 0;
        lora_write(0x01, stanby);
        lora_write(0x12, 0xFF);
        lora_tx_active = 0;
        return LORA_TX_STATUS_FAIL;
    }

    lora_tx_irq_read_pending = 0;
    txflag = lora_read_async_get();

    // ------------------> 총 3가지로 1. tx완료, 2.tx 실패 , 3.tx중   이 3개중 한개의 상태를 반환

    if(txflag & 0x08)    //txdone 인터럽트 켜지면 전송 성공
    {
        //lora_log("TX DONE: irq=0x%02X\r\n\n", txflag);
        lora_write(0x01, stanby);
        lora_write(0x12, 0xFF); // tx보냈으니깐 txdone 인터럽트 비트 다시 초기화
        lora_tx_active = 0; // 송신 끝남 표시
        return LORA_TX_STATUS_DONE; // tx 완료 반환
    }

    if(HAL_GetTick() - lora_tx_start_tick > TXDONE_WAIT_PERIOD ) // lora_tx_start_tick는 real_tx에서 송신처리 다했을때의 시간 ==> 그 시간과 지금이 내가 설정한 tx시간보다 크면 실패
    {
        //lora_log("TX TIMEOUT\r\n\n");
        lora_write(0x01, stanby);
        lora_write(0x12, 0xFF);
        lora_tx_active = 0;
        return LORA_TX_STATUS_FAIL;
    }

    // 아직 TX 중이면 기다리지 않고 바로 빠져나간다.
    return LORA_TX_STATUS_BUSY; // 아직 tx보내고 있어서 txdone은 안됬고 , 리미트 tx시간 보다 아직 덜 넘은 아직 tx보내고 있는중이면 작동중이라는 플래그 줌
}

uint8_t lora_tx_is_busy(void)
{
    return lora_tx_active;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  매트랩 코드보는용 함수들


void rssi_graph(void) ///////////////////////////////////////////////////////// 매트랩 수신 보는용 함수
{
    uint8_t irq = lora_read(0x12);   // RegIrqFlags
#if LORA_UART_DEBUG_ENABLE
    uint32_t now = HAL_GetTick();
    int16_t pkt_rssi = lora_get_packet_rssi();
    int16_t snr_x100 = lora_get_packet_snr_x100();
#endif

    /* CRC 에러 */
    if(irq & 0x20)
    {
#if LORA_UART_DEBUG_ENABLE
        char line[64];
        snprintf(line, sizeof(line), "P,%lu,%d,%d,0,\r\n", now, pkt_rssi, snr_x100);
        HAL_UART_Transmit(&huart6, (uint8_t*)line, strlen(line), 100);
#endif

        lora_write(0x01, stanby);
        lora_write(0x12, 0xFF);
        r_byte = 0;
        return;
    }

    /* RX done */
    if(irq & 0x40)
    {
        lora_write(0x01, stanby);

        r_byte = lora_read(0x13);                    // RegRxNbBytes
        lora_write(0x0D, lora_read(0x10));           // FIFO pointer = current RX start

        if(r_byte > sizeof(rx_buf)) r_byte = sizeof(rx_buf);

        for(int i = 0; i < r_byte; i++)
        {
            rx_buf[i] = lora_read(0x00);            // RegFifo
        }

#if LORA_UART_DEBUG_ENABLE
        char head[64];
        snprintf(head, sizeof(head), "P,%lu,%d,%d,1,", now, pkt_rssi, snr_x100);
        HAL_UART_Transmit(&huart6, (uint8_t*)head, strlen(head), 100);

        uart_print_hex(rx_buf, r_byte);

        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 100);
#endif
    }

    lora_write(0x12, 0xFF);
    r_byte = 0;
}

void uart_send_noise_rssi(void)
{
#if LORA_UART_DEBUG_ENABLE
    uint32_t now = HAL_GetTick();

    if(now - last_rssi_tick >= 50)   // 매트랩에 찍기위한 값들을 보내는 주기
    {
        last_rssi_tick = now;

        char line[64];
        int16_t rssi = lora_get_current_rssi();

        snprintf(line, sizeof(line), "R,%lu,%d\r\n", now, rssi);
        HAL_UART_Transmit(&huart6, (uint8_t*)line, strlen(line), 100);
    }
#endif
}

#if LORA_UART_DEBUG_ENABLE
static void uart_print_hex(uint8_t *buf, uint8_t len)
{
    char hx[4];

    for(uint8_t i = 0; i < len; i++)
    {
        snprintf(hx, sizeof(hx), "%02X", buf[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)hx, strlen(hx), 100);
    }
}

static int16_t lora_get_current_rssi(void)
{
    /* HF port(779MHz 이상) 기준 */
    return (int16_t)lora_read(0x1B) - 157;
}
#endif

#if LORA_UART_DEBUG_ENABLE
static int16_t lora_get_packet_rssi(void)
{
    return (int16_t)lora_read(0x1A) - 157;
}

static int16_t lora_get_packet_snr_x100(void)
{
    int8_t snr_raw = (int8_t)lora_read(0x19);
    return (int16_t)snr_raw * 25;   // 0.25dB * 100
}
#endif
