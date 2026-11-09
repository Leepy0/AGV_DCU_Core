/* hal_host.h - 시험에서 들여다보는 가짜 하드웨어 상태 */
#ifndef HAL_HOST_H
#define HAL_HOST_H
#include "agv/hal.h"

#define HOST_SCI_BUF 512

typedef struct {
    uint16_t ccr, brr, inits;
    uint16_t tx[HOST_SCI_BUF], tx_len, tx_full;
    uint16_t rx[HOST_SCI_BUF], rx_len, rx_pos;
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
} host_hw_t;

extern host_hw_t host;

void host_reset(void);
#endif
