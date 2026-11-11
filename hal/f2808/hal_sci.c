/* hal_sci.c - SCI-A·B. 원본은 FIFO 를 끄고 한 바이트씩 받았다. 여기서는 16단 FIFO 를 켠다.
 * 제어를 ISR 밖으로 옮겼어도 EEPROM·버스 쓰기 구간의 지연을 FIFO 가 흡수한다 */
#include "agv/hal.h"
#include "regs.h"

#define RING_SIZE 128u

#define FFTX_INIT    0xE040u   /* SCIRST, SCIFFENA, TXFIFOXRESET, TXFFINTCLR. 인터럽트 꺼짐 */
#define FFTX_IENA    0x0020u
#define FFTX_INTCLR  0x0040u
#define FFTX_STAT    0x1F00u
#define FFRX_INIT    0x6061u   /* OVRCLR, RXFIFORESET, INTCLR, IENA, RXFFIL=1 */
#define FFRX_OVF     0x8000u
#define FFRX_STAT    0x1F00u
#define FFRX_INTCLR  0x0040u
#define FFRX_RESET   0x2000u
#define FFRX_OVRCLR  0x4000u
#define RXBUF_ERR    0xC000u   /* 프레임·패리티 오류 표시 */
#define RXST_ERROR   0x0080u
#define CTL1_RUN     0x0023u
#define CTL1_RESET   0x0003u
#define TX_FIFO_DEPTH 16u

typedef struct {
    uint16_t base;
    uint16_t tx[RING_SIZE];
    volatile uint16_t tx_head, tx_tail;
    uint16_t rx[RING_SIZE];
    volatile uint16_t rx_head, rx_tail;
    volatile uint16_t errors;
    volatile uint32_t rx_at;   /* 마지막으로 링에 넣은 바이트를 받은 hal_cycles */
} sci_t;

static sci_t s_sci[2];

void hal_sci_init(uint16_t port, uint16_t ccr, uint16_t brr)
{
    sci_t *s = &s_sci[port];
    uint16_t base = (port == HAL_SCI_A) ? 0x7050u : 0x7750u;
    uint16_t irq = hal_irq_save();

    s->base = base;
    s->tx_head = s->tx_tail = 0;
    s->rx_head = s->rx_tail = 0;
    SCI_CCR(base) = ccr;
    SCI_CTL1(base) = CTL1_RESET;
    SCI_CTL2(base) = (uint16_t)((SCI_CTL2(base) & 0xFFFEu) | 0x0002u);
    SCI_HBAUD(base) = (uint16_t)(brr >> 8);
    SCI_LBAUD(base) = (uint16_t)(brr & 0xFFu);
    SCI_FFTX(base) = FFTX_INIT;
    SCI_FFRX(base) = FFRX_INIT;
    SCI_FFCT(base) = 0;
    SCI_CTL1(base) = CTL1_RUN;
    hal_irq_restore(irq);
}

int hal_sci_send(uint16_t port, const uint16_t *buf, uint16_t n)
{
    sci_t *s = &s_sci[port];
    uint16_t head = s->tx_head;
    uint16_t used = (uint16_t)((head - s->tx_tail) & (RING_SIZE - 1u));
    uint16_t i, irq;

    if ((uint16_t)(used + n) >= RING_SIZE) { return 0; }
    for (i = 0; i < n; i++) {
        s->tx[head] = buf[i];
        head = (uint16_t)((head + 1u) & (RING_SIZE - 1u));
    }
    s->tx_head = head;
    irq = hal_irq_save();
    SCI_FFTX(s->base) |= (uint16_t)(FFTX_IENA | FFTX_INTCLR);
    hal_irq_restore(irq);
    return 1;
}

int hal_sci_rx(uint16_t port, uint16_t *byte)
{
    sci_t *s = &s_sci[port];
    uint16_t tail = s->rx_tail;

    if (tail == s->rx_head) { return 0; }
    *byte = s->rx[tail];
    s->rx_tail = (uint16_t)((tail + 1u) & (RING_SIZE - 1u));
    return 1;
}

uint32_t hal_sci_rx_stamp(uint16_t port) { return s_sci[port].rx_at; }   /* 32비트 읽기 한 번(MOVL)이라 ISR 과 갈리지 않는다 */

uint16_t hal_sci_take_errors(uint16_t port)
{
    uint16_t irq = hal_irq_save();
    uint16_t n = s_sci[port].errors;

    s_sci[port].errors = 0;
    hal_irq_restore(irq);
    return n;
}

/* 오버런은 파서가 틀린 체크섬으로 걸러내므로 FIFO 만 살린다 */
static void rx_recover(sci_t *s)
{
    uint16_t base = s->base;

    if (SCI_FFRX(base) & FFRX_OVF) {
        s->errors++;
        SCI_FFRX(base) |= FFRX_OVRCLR;
        SCI_FFRX(base) &= (uint16_t)~FFRX_RESET;
        SCI_FFRX(base) |= FFRX_RESET;
    }
    if (SCI_RXST(base) & RXST_ERROR) {
        s->errors++;
        SCI_CTL1(base) = CTL1_RESET;
        SCI_CTL1(base) = CTL1_RUN;
    }
}

static void tx_isr(sci_t *s)
{
    uint16_t base = s->base;
    uint16_t room = (uint16_t)(TX_FIFO_DEPTH - ((SCI_FFTX(base) & FFTX_STAT) >> 8));

    while ((room > 0u) && (s->tx_tail != s->tx_head)) {
        SCI_TXBUF(base) = s->tx[s->tx_tail];
        s->tx_tail = (uint16_t)((s->tx_tail + 1u) & (RING_SIZE - 1u));
        room--;
    }
    if (s->tx_tail == s->tx_head) { SCI_FFTX(base) &= (uint16_t)~FFTX_IENA; }
    SCI_FFTX(base) |= FFTX_INTCLR;
    PIEACK = 0x0100u;
}

__interrupt void hal_scia_rx_isr(void)
{
    sci_t *s = &s_sci[HAL_SCI_A];
    uint16_t base = s->base;

    while (SCI_FFRX(base) & FFRX_STAT) {
        uint16_t w = SCI_RXBUF(base);
        if ((w & RXBUF_ERR) == 0u) { sys_scia_rx(w & 0xFFu); }
    }
    rx_recover(s);
    SCI_FFRX(base) |= FFRX_INTCLR;
    PIEACK = 0x0100u;
}

__interrupt void hal_scib_rx_isr(void)
{
    sci_t *s = &s_sci[HAL_SCI_B];
    uint16_t base = s->base;

    while (SCI_FFRX(base) & FFRX_STAT) {
        uint16_t w = SCI_RXBUF(base);
        uint16_t next = (uint16_t)((s->rx_head + 1u) & (RING_SIZE - 1u));
        if (((w & RXBUF_ERR) == 0u) && (next != s->rx_tail)) {
            s->rx[s->rx_head] = w & 0xFFu;
            s->rx_head = next;
            s->rx_at = hal_cycles();
        }
    }
    rx_recover(s);
    SCI_FFRX(base) |= FFRX_INTCLR;
    PIEACK = 0x0100u;
}

__interrupt void hal_scia_tx_isr(void) { tx_isr(&s_sci[HAL_SCI_A]); }
__interrupt void hal_scib_tx_isr(void) { tx_isr(&s_sci[HAL_SCI_B]); }
