# 1차 통합 정리: Drone + LoRa + N6 + GNSS

이 브랜치는 기존 STM32H753 드론/GNSS 코드에 LoRa telemetry와 N6 수신 코드를 1차로 통합한 작업본이다.

## 연결 핀

- N6 TX -> STM32 `PD9 / USART3_RX`
- LoRa `SCK` -> `PE2 / SPI4_SCK`
- LoRa `CS` -> `PE3 / GPIO_Output`
- LoRa `INT/DIO0` -> `PE4 / EXTI`
- LoRa `MISO` -> `PE5 / SPI4_MISO`
- LoRa `MOSI` -> `PE6 / SPI4_MOSI`
- LoRa `RESET` -> `PC13 / GPIO_Output`
- LoRa `EN`은 코드에서 제어하지 않고 3.3V에 직접 연결하는 전제로 둔다.

## LoRa 동작 구조

- LoRa는 `SPI4`를 사용한다.
- SPI4 RX/TX는 DMA로 설정되어 있다.
- SPI4 DMA 완료 callback에서 LoRa CS를 HIGH로 올린다.
- 기존의 긴 `TX_DONE` 대기 loop는 제거하고, `lora_tx_process()`에서 상태를 나눠 확인한다.
- telemetry는 `telemetry()` 상태머신으로 동작하며, main loop 또는 주행 loop에서 계속 짧게 호출된다.

주의할 점:

- 현재 LoRa SPI 함수는 DMA를 사용하지만, 일부 `lora_write()`, `lora_read()` 계열은 DMA 완료를 짧게 기다리는 wrapper 구조가 남아 있다.
- 정상 상황에서는 SPI4 단독 사용이고 payload도 작아서 영향이 작을 가능성이 높지만, DMA busy/error 상황에서는 짧은 지연이 발생할 수 있다.
- 완전한 비동기 구조로 가려면 LoRa register read/write까지 telemetry 상태머신 단계로 더 세분화해야 한다.

## N6 수신 구조

- N6는 `USART3_RX(PD9)` 1바이트 interrupt 방식으로 수신한다.
- `HAL_UART_RxCpltCallback()`에서 `USART3`이면 `N6_RCV_RxCpltCallback()`으로 넘긴다.
- N6 문자열 형식은 `@DET,detected,count` 기준으로 파싱한다.
- 일정 시간 새 N6 데이터가 없으면 검출 상태를 0으로 초기화한다.

## GNSS와 LoRa GPS 송신

- GNSS 모듈은 기존 드론 코드의 `USART2` 기반 GNSS 처리 흐름을 사용한다.
- LoRa GPS 좌표는 `telemetry.c`에서 `gnss_read_pvt()`로 실제 GNSS PVT 값을 읽어 사용한다.
- 예전 테스트용 fake GPS 증가 로직은 제거했다.
- `gnss_read_pvt()`는 호출하면 내부 ready flag를 클리어하는 소비형 함수라서, main loop의 디버그 출력에서는 더 이상 호출하지 않는다.
- 즉 GNSS PVT 소비 주체는 LoRa telemetry 쪽으로 정리했다.

## 드론 주행 코드와의 관계

- 기존 센서/모터 주행 루프는 유지하고, 주행 중 `motor_rate_pid_update()` 이후 `telemetry()`를 한 단계씩 호출한다.
- LoRa TX/RX 대기 자체가 main loop를 오래 붙잡지 않도록 상태머신으로 분리했다.
- IMU 쪽 SPI는 `SPI2`, LoRa는 `SPI4`라서 SPI 버스는 분리되어 있다.

## 확인한 빌드

- STM32CubeIDE 일반 프로젝트 형식으로 변환했다.
- `Debug` 빌드 기준 `30th_Drone_LoRa_Integrated.elf` 링크까지 성공했다.

## 다음에 보면 좋은 부분

- 실제 하드웨어에서 N6 `@DET,...` 수신 확인
- 실제 GNSS fix 후 LoRa payload 좌표 확인
- LoRa TX_DONE/RX_DONE 동작 확인
- 비행 테스트 전, LoRa SPI register 접근까지 완전 async로 더 쪼갤지 판단
