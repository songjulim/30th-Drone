#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"  // 이걸 넣어야 GPIO_PIN_RESET 같은 거 인식함
#include "stdio.h"
#include "string.h"

// 1. 매크로 정의 (main.c에 있던 거 다 가져옴)
#define write_bit 0x80
#define read_bit 0x00
#define sleep 0x80 // 레지스터 설정중에서 LORA모드 변경 + SLEEP 상태(SPI만됨, RF정지)에서 설정할수있는 모드,레지스터가있음,저전력모드. 즉, 시동을 끄고 좀 설정하는 느낌
                   // 전원을 처음켜면 FSK/OSK모드여서 이걸 LORA모드로 사용하기 위해서는 무조건 SLEEP상태에서만 가능(stanby불가능) 그래서 전원 꼽고 이거 한번 켜주고 레지스터 설정해야 lora모드의 레지스터의 주소로 값이 들어감
#define stanby 0x81 // RF 송수신은 아직 안 하지만, 클럭 켜지고, 전력좀 쓰고. 즉, 시동키고 대기하는 느낌
#define tx 0x83
#define no_more 0xFF // burst accecss에서 2개이상으로 하고싶지는 않을때

#define TXDONE_WAIT_PERIOD 780  // TX완료 될때까지 기다리는 시간

// real_tx()가 TxDone을 while로 기다리지 않고,
// main loop에서 lora_tx_process()로 조금씩 상태를 확인하기 위한 값
#define LORA_TX_STATUS_BUSY 0
#define LORA_TX_STATUS_DONE 1
#define LORA_TX_STATUS_FAIL 2
#define LORA_TX_STATUS_IDLE 3


// [중요] main.c에 있는 핸들러들을 '빌려온다'고 선언해야 함
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart1;
extern volatile uint8_t lora_rx_flag;  // rx 인터럽트 들어오면 켜지는 플래그



////////////////////////////////////////////////////////////////////////////////////////////////////// 3. 함수 프로토타입 (남들이 쓸 함수만)
void lora_log(const char *fmt, ...);
void lora_setup(void);
void lora_write(uint8_t adr,uint8_t data);
void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3);
uint8_t lora_read(uint8_t adr);
void lora_freq(void);
void packet_set(void);
void rx_set(void);
void rx_read(void);
void rssi_graph(void);
void uart_send_noise_rssi(void);

void fifo_set(void);
void payload_write(const uint8_t *buf, uint8_t len);
HAL_StatusTypeDef lora_read_payload_dma(uint8_t *buf, uint8_t len);
int real_tx(void);
int lora_tx_process(void);
uint8_t lora_tx_is_busy(void);
void LORA_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void LORA_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void LORA_DIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif /* INC_LORA_H_ */


