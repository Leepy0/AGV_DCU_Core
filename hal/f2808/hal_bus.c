/* hal_bus.c - 멀티플렉스 외부 버스. 2 ms ISR 과 메인이 같은 핀을 쓰므로 쓰기는 인터럽트를 막는다 */
#include "agv/hal.h"
#include "regs.h"

#define DATA_MASK 0x00FFUL
#define ADDR_LOW  (7UL << 8)

static void set_addr(uint16_t addr)
{
    uint32_t on = ((uint32_t)(addr & 7u) << 8) | ((addr & 8u) ? PIN_A3 : 0UL);
    GPACLEAR = (ADDR_LOW | PIN_A3) & ~on;
    GPASET = on;
}

static uint16_t cycle(uint16_t addr, uint32_t rw)
{
    set_addr(addr);
    if (rw) { GPASET = PIN_RW; }
    else    { GPACLEAR = PIN_RW; }
    hal_delay_us(1);
    GPACLEAR = PIN_STB;
    hal_delay_us(1);
    GPASET = PIN_STB;
    return (uint16_t)(GPADAT_L & 0x00FFu);   /* 읽기 데이터는 STB 를 올린 뒤에 샘플한다 */
}

uint16_t hal_bus_read(uint16_t addr)
{
    EALLOW;
    GPADIR_L &= 0xFF00u;
    EDIS;
    return cycle(addr, PIN_RW);
}

void hal_bus_write(uint16_t addr, uint16_t data)
{
    uint16_t irq = hal_irq_save();

    EALLOW;
    GPADIR_L |= 0x00FFu;
    EDIS;
    GPACLEAR = DATA_MASK & ~(uint32_t)data;
    GPASET = DATA_MASK & (uint32_t)data;
    (void)cycle(addr, 0UL);
    hal_irq_restore(irq);
}

void hal_bus_enable(void) { GPACLEAR = PIN_OE; }
