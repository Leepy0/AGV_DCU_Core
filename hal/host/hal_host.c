/* hal_host.c - PC 시험용 HAL. 버스·DAC·SCI·EEPROM 을 배열로 흉내 낸다 */
#include "hal_host.h"
#include <string.h>

host_hw_t host;

/* 시험에서 EEPROM 쓰기 시간 흉내: 시작만 하고 끝날 때 값이 들어간다 */
static uint16_t s_pending_addr, s_pending_value, s_pending;

void host_reset(void)
{
    memset(&host, 0, sizeof(host));
    memset(host.ee, 0xFF, sizeof(host.ee));
    memset(host.bus_in, 0xFF, sizeof(host.bus_in));
    host.bus_in[BUS_IN_A] = 0xFF;
    host.bus_in[BUS_IN_B] = 0xF6;     /* E-STOP(b0) 고활성이라 0, 확장 점퍼 끔 */
    host.bus_in[BUS_IN_C] = 0xF8;     /* 키 없음 */
    host.bus_in[BUS_IN_A] &= (uint16_t)~0x30u;   /* 범퍼 고활성이라 0 */
    host.ee_bad_addr = -1;
    s_pending = 0;
}

void hal_sys_init(void)           { }
uint32_t hal_cycles(void)         { return host.cycles; }
void hal_start(void)              { host.started = 1; }
void hal_wdog_kick(void)          { host.wdog_kicks++; }
void hal_delay_us(uint16_t us)    { (void)us; }
uint16_t hal_irq_save(void)       { return 0; }
void hal_irq_restore(uint16_t s)  { (void)s; }

/* 키는 세그먼트 바이트 b5·b6 로 고른 행의 3비트가 입력 C 하위에 나온다.
 * 실차 확인: b5 는 MODE·SHIFT·UP 행, b6 는 RESET·SET·DOWN 행 */
uint16_t hal_bus_read(uint16_t addr)
{
    if (addr == BUS_IN_C) {
        uint16_t row = (host.bus_out[BUS_SEG] & 0x20u) ? (host.key & 7u) : ((host.key >> 3) & 7u);
        return (uint16_t)((host.bus_in[addr] & 0xF8u) | row);
    }
    return host.bus_in[addr];
}
void     hal_bus_write(uint16_t addr, uint16_t d) { host.bus_out[addr] = d; host.bus_writes[addr]++; }
void     hal_bus_enable(void)                     { host.bus_enabled = 1; }

void     hal_dac_write(uint16_t ch, uint16_t v)   { host.dac[ch] = v; }
void     hal_adc_start(void)                      { host.adc_starts++; }
uint16_t hal_adc_battery(void)                    { return host.adc; }
void     hal_sound(uint16_t on)                   { host.sound = on; }
void     hal_led_toggle(void)                     { host.led++; }

void hal_sci_init(uint16_t port, uint16_t ccr, uint16_t brr)
{
    host.sci[port].ccr = ccr;
    host.sci[port].brr = brr;
    host.sci[port].inits++;
    host.sci[port].rx_len = host.sci[port].rx_pos = 0;
}

int hal_sci_send(uint16_t port, const uint16_t *buf, uint16_t n)
{
    uint16_t i;
    if (host.sci[port].tx_full) { return 0; }
    for (i = 0; i < n; i++) {
        if (host.sci[port].tx_len < HOST_SCI_BUF) { host.sci[port].tx[host.sci[port].tx_len++] = buf[i]; }
    }
    if (host.on_send != 0) { host.on_send(port, buf, n); }
    return 1;
}

int hal_sci_rx(uint16_t port, uint16_t *byte)
{
    host_sci_t *s = &host.sci[port];
    if ((s->rx_pos >= s->rx_len) || ((int32_t)(s->rx_at[s->rx_pos] - host.cycles) > 0)) { return 0; }
    s->rx_stamp = s->rx_at[s->rx_pos];
    *byte = s->rx[s->rx_pos++];
    return 1;
}

uint32_t hal_sci_rx_stamp(uint16_t port) { return host.sci[port].rx_stamp; }

uint16_t hal_sci_take_errors(uint16_t port) { (void)port; return 0; }

static uint16_t ee_addr(uint16_t addr) { return (uint16_t)(addr & (host.ee_128 ? 0x7Fu : 0xFFu)); }

uint16_t hal_ee_read(uint16_t addr) { return host.ee[ee_addr(addr)]; }

void hal_ee_write_begin(uint16_t addr, uint16_t value)
{
    s_pending_addr = ee_addr(addr);
    s_pending_value = value;
    s_pending = 1;
    host.ee_writes++;
}

void hal_ee_write_end(void)
{
    if (s_pending && ((int)s_pending_addr != host.ee_bad_addr)) { host.ee[s_pending_addr] = s_pending_value; }
    s_pending = 0;
}
