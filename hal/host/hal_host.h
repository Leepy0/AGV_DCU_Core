/* hal_host.h - 시험에서 들여다보는 가짜 하드웨어 상태 */
#ifndef HAL_HOST_H
#define HAL_HOST_H
#include "agv/hal.h"

#define HOST_SCI_BUF 512

typedef struct {
    uint16_t ccr, brr, inits;
    uint16_t tx[HOST_SCI_BUF], tx_len, tx_full;
    uint16_t rx[HOST_SCI_BUF], rx_len, rx_pos;
    uint32_t rx_at[HOST_SCI_BUF];  /* 바이트가 도착하는 host.cycles. 0 = 바로. hal_sci_rx 는 도착한 것만 낸다 */
    uint32_t rx_stamp;             /* 마지막으로 꺼낸 바이트의 rx_at */
} host_sci_t;

typedef struct {
    uint16_t bus_in[16], bus_out[16];
    uint32_t bus_writes[16];
    uint16_t key;                  /* 눌린 키 값 */
    uint16_t bus_enabled, started;
    uint32_t wdog_kicks, adc_starts, led;
    uint16_t dac[2], adc, sound;
    host_sci_t sci[2];
    uint16_t ee[256];
    uint32_t ee_writes;
    int      ee_bad_addr;          /* 이 주소는 써도 값이 안 들어간다 */
    int      ee_128;               /* 1 = 128 워드 칩: 주소 7비트를 무시한다 */
    uint32_t cycles;               /* hal_cycles. 시험이 올린다 */
    void   (*on_send)(uint16_t port, const uint16_t *buf, uint16_t n);   /* 모의 센서: 요청을 보면 rx 에 응답을 넣는다 */
} host_hw_t;

extern host_hw_t host;

void host_reset(void);
#endif
