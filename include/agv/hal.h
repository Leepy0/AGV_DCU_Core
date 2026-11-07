/*
 * hal.h - 하드웨어 경계. hal/f2808 이 레지스터로, hal/host 가 시험용 흉내로 구현한다.
 * 버스 주소·핀 배치·레지스터 값은 보드에 고정된 사실이고 사이트와 무관하다.
 */
#ifndef AGV_HAL_H
#define AGV_HAL_H

#include "agv/types.h"

/* 외부 I/O 버스 주소 */
enum {
    BUS_AMP = 0, BUS_DISP = 1, BUS_SEG = 2, BUS_LAMP = 3,
    BUS_IN_A = 4, BUS_IN_B = 5, BUS_IN_C = 6, BUS_IN_D = 7,
    BUS_EXT_OUT_A = 8, BUS_EXT_OUT_B = 9, BUS_EXT_IN_A = 10, BUS_EXT_IN_B = 11
};

enum { HAL_SCI_A = 0, HAL_SCI_B = 1 };

/* SCI 프레임 형식: SCICCR 값 */
#define HAL_SCI_8N1 0x07u
#define HAL_SCI_8N2 0x87u

/* 부팅. 인터럽트는 hal_start 전까지 꺼져 있다 */
void     hal_sys_init(void);
void     hal_start(void);              /* 타이머 시작, 인터럽트 허용, 워치독 시작 */
void     hal_wdog_kick(void);
void     hal_delay_us(uint16_t us);

uint16_t hal_irq_save(void);
void     hal_irq_restore(uint16_t state);

/* 버스. 읽기는 2 ms ISR 전용, 쓰기는 어느 문맥이든 된다 */
uint16_t hal_bus_read(uint16_t addr);
void     hal_bus_write(uint16_t addr, uint16_t data);
void     hal_bus_enable(void);         /* 출력 래치 허용. 첫 출력 뒤 한 번 */

void     hal_dac_write(uint16_t channel, uint16_t value);
void     hal_adc_start(void);
uint16_t hal_adc_battery(void);        /* ADCRESULT0 원값 */
void     hal_sound(uint16_t on);
void     hal_led_toggle(void);

/* SCI. 링 버퍼·FIFO 는 HAL 이 소유한다. rx_cb 없는 포트는 hal_sci_rx 로 꺼낸다 */
void     hal_sci_init(uint16_t port, uint16_t ccr, uint16_t brr);
int      hal_sci_send(uint16_t port, const uint16_t *buf, uint16_t n);   /* 통째로 넣으면 1 */
int      hal_sci_rx(uint16_t port, uint16_t *byte);                      /* 꺼내면 1 */
uint16_t hal_sci_take_errors(uint16_t port);                             /* 오버런·수신 오류 복구 횟수 */

/* Microwire EEPROM. 쓰기는 시작·끝을 나눠 칩의 쓰기 시간 동안 CPU 를 막지 않는다 */
#define HAL_EE_WRITE_US 5000u
uint16_t hal_ee_read(uint16_t addr);
void     hal_ee_write_begin(uint16_t addr, uint16_t value);
void     hal_ee_write_end(void);

/* HAL 이 ISR 에서 부르는 코어 함수 */
void     sys_tick_2ms(void);
void     sys_encoder(uint16_t wheel, uint32_t period);
void     sys_scia_rx(uint16_t byte);

#endif /* AGV_HAL_H */
