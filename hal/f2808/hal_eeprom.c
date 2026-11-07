/* hal_eeprom.c - Microwire 비트뱅 (x16, 주소 8비트). 쓰기 완료 대기는 호출자 몫이다.
 * GPADAT 읽기-수정-쓰기 대신 SET/CLEAR 를 써서 2 ms 버스 ISR 과 핀을 겹쳐 쓰지 않는다 */
#include "agv/hal.h"
#include "regs.h"

#define OP_EWEN   0x04C0u
#define OP_EWDS   0x0400u
#define OP_WRITE  0x0500u
#define OP_READ   0x0600u
#define OP_BITS   11u

static void sk(uint16_t high)
{
    if (high) { GPASET = PIN_EE_SK; }
    else      { GPACLEAR = PIN_EE_SK; }
    hal_delay_us(1);
}

static void cs(uint16_t high)
{
    if (high) { GPASET = PIN_EE_CS; }
    else      { GPACLEAR = PIN_EE_CS; }
    hal_delay_us(1);
}

static void shift_out(uint16_t data, uint16_t bits)
{
    uint16_t i;

    for (i = 0; i < bits; i++) {
        if (data & (1u << (bits - 1u - i))) { GPBSET = PINB_EE_DI; }
        else                                { GPBCLEAR = PINB_EE_DI; }
        sk(0);
        sk(1);
    }
    GPBCLEAR = PINB_EE_DI;
    sk(0);
}

static void op_begin(void)
{
    cs(1);
    sk(1);
    sk(0);
}

static void op_end(void)
{
    sk(1);
    sk(0);
    cs(0);
}

uint16_t hal_ee_read(uint16_t addr)
{
    uint16_t i, value = 0;

    op_begin();
    shift_out((uint16_t)(OP_READ | (addr & 0xFFu)), OP_BITS);
    hal_delay_us(1);
    for (i = 0; i < 16u; i++) {
        sk(1);
        value = (uint16_t)(value << 1);
        if (GPBDAT & PINB_EE_DO) { value |= 1u; }
        sk(0);
    }
    op_end();
    sk(1);
    sk(0);
    return value;
}

void hal_ee_write_begin(uint16_t addr, uint16_t value)
{
    op_begin();
    shift_out(OP_EWEN, OP_BITS);
    op_end();
    op_begin();
    shift_out((uint16_t)(OP_WRITE | (addr & 0xFFu)), OP_BITS);
    shift_out(value, 16u);
    cs(0);
}

void hal_ee_write_end(void)
{
    op_begin();
    shift_out(OP_EWDS, OP_BITS);
    op_end();
}
