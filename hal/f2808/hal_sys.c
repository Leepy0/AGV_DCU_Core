/* hal_sys.c - 클럭, 플래시 대기 상태, PIE, 타이머, GPIO, 워치독, DAC, ADC, eCAP */
#include "agv/hal.h"
#include "regs.h"

#define WDCR_OFF      0x0068u   /* WDCHK=101, WDDIS=1 */
#define WDCR_ON       0x002Du   /* WDCHK=101, WDPS=/16: 약 105 ms */
#define PLL_MUL_10    0x000Au   /* 20 MHz × 10 / 2 = 100 MHz */
#define TIMER0_PRD    200000UL  /* 2 ms */

/* F2808 의 값. 부품 판별 분기는 뺐다 */
#define PCLKCR0_ALL   0xCFDCu
#define PCLKCR1_ALL   0xCF3Fu

/* 핀 기능 */
#define GPAMUX1_VAL   0xA0000000UL   /* GPIO14·15 = SCI-B */
#define GPAMUX2_VAL   0x05050455UL   /* SPI-A, eQEP1B(미사용), eCAP1·2, SCI-A */
#define GPADIR_VAL    0xCC503F00UL
#define GPBDIR_VAL    0x00000005UL
#define GPAPUD_VAL    0x0000FFFFUL
#define GPBPUD_VAL    0x0000FFFFUL

#define ECAP1         0x6A00u
#define ECAP2         0x6A20u
#define ECCTL1_VAL    0x01AAu   /* 전부 상승 에지, 매 이벤트 리셋(차분), 캡처 허용 */
#define ECCTL2_VAL    0x0018u   /* 연속, CEVT1 랩, 카운터 동작, REARM */
#define ECEINT_CEVT1  0x0002u
#define ECCLR_INT     0x0001u

__interrupt void hal_timer0_isr(void);
__interrupt void hal_ecap1_isr(void);
__interrupt void hal_ecap2_isr(void);
__interrupt void hal_scia_rx_isr(void);
__interrupt void hal_scia_tx_isr(void);
__interrupt void hal_scib_rx_isr(void);
__interrupt void hal_scib_tx_isr(void);

extern unsigned int RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;

#pragma CODE_SECTION(flash_init, "ramfuncs")
static void flash_init(void)
{
    EALLOW;
    FOPT |= 0x0001u;                                      /* 파이프라인 */
    FBANKWAIT = (uint16_t)((FBANKWAIT & 0xF0F0u) | 0x0303u);
    FOTPWAIT  = (uint16_t)((FOTPWAIT & 0xFFE0u) | 0x0005u);
    FSTDBYWAIT  |= 0x01FFu;
    FACTIVEWAIT |= 0x01FFu;
    EDIS;
    asm(" RPT #7 || NOP");                                /* 파이프라인 비움 */
}

static void pll_init(void)
{
    if (PLLSTS & 0x0008u) { asm(" ESTOP0"); }             /* 발진 없음 */
    if (PLLSTS & 0x0002u) { PLLSTS &= 0xFFFDu; }
    if ((PLLCR & 0x000Fu) != PLL_MUL_10) {
        EALLOW;
        PLLSTS |= 0x0040u;                                /* 클럭 감시 끔 */
        PLLCR = (uint16_t)((PLLCR & 0xFFF0u) | PLL_MUL_10);
        EDIS;
        while ((PLLSTS & 0x0001u) == 0u) { }
        EALLOW;
        PLLSTS &= 0xFFBFu;
        EDIS;
    }
}

static __interrupt void isr_unexpected(void) { for (;;) { } }   /* 워치독이 리셋한다 */

static void pie_init(void)
{
    uint16_t i;

    asm(" SETC INTM");
    PIECTRL &= 0xFFFEu;
    for (i = 1; i <= 12; i++) {
        PIEIER(i) = 0;
        PIEIFR(i) = 0;
    }
    IER = 0;
    IFR = 0;
    EALLOW;
    for (i = 0; i < PIE_VECT_COUNT; i++) {
        R32(PIE_VECT + 2u * i) = (uint32_t)&isr_unexpected;
    }
    PIE_VEC(1, 7) = (uint32_t)&hal_timer0_isr;
    PIE_VEC(4, 1) = (uint32_t)&hal_ecap1_isr;
    PIE_VEC(4, 2) = (uint32_t)&hal_ecap2_isr;
    PIE_VEC(9, 1) = (uint32_t)&hal_scia_rx_isr;
    PIE_VEC(9, 2) = (uint32_t)&hal_scia_tx_isr;
    PIE_VEC(9, 3) = (uint32_t)&hal_scib_rx_isr;
    PIE_VEC(9, 4) = (uint32_t)&hal_scib_tx_isr;
    EDIS;
    PIECTRL |= 0x0001u;
    PIEACK = 0xFFFFu;
}

static void gpio_init(void)
{
    EALLOW;
    GPAQSEL1 = 0;
    GPAQSEL2 = 0;
    GPBQSEL1 = 0;
    GPAMUX1 = GPAMUX1_VAL;
    GPAMUX2 = GPAMUX2_VAL;
    GPBMUX1 = 0;
    GPAPUD = GPAPUD_VAL;
    GPBPUD = GPBPUD_VAL;
    GPASET = PIN_OE | PIN_STB | PIN_LDAC | PIN_SOUND;     /* 방향을 바꾸기 전에 비활성 레벨 */
    GPADIR = GPADIR_VAL;
    GPBDIR = GPBDIR_VAL;
    EDIS;
}

static void adc_init(void)
{
    ADCTRL3 |= 0x00C0u;                 /* 밴드갭·기준 */
    hal_delay_us(500);
    ADCTRL3 |= 0x0020u;                 /* ADC 전원 */
    hal_delay_us(20);
    ADCTRL1 = 0x2280u;                  /* SUSMOD=2, ACQ_PS=2, CPS=1 */
    ADCTRL3 = (uint16_t)(ADCTRL3 | 0x0009u);   /* ADCCLKPS=4, 동시 샘플 */
    ADCMAXCONV = 0x0011u;
    ADCCHSELSEQ1 = 0x3210u;
    ADCCHSELSEQ3 = 0x7654u;
}

static void dac_init(void)
{
    SPICCR &= 0xFF7Fu;
    SPICTL = 0x0006u;                   /* 마스터, 송신 */
    SPIBRR = 0x0004u;                   /* 5 MHz */
    SPICCR = 0x004Du;                   /* 하강 에지, 14비트 */
    SPICCR |= 0x0080u;
}

static void ecap_init(uint16_t base)
{
    ECAP_EINT(base) = 0;
    ECAP_CLR(base) = 0xFFFFu;
    ECAP_CTL2(base) = 0;
    ECAP_CTL1(base) = ECCTL1_VAL;
    ECAP_CTL2(base) = ECCTL2_VAL;
    ECAP_EINT(base) = ECEINT_CEVT1;
}

void hal_sys_init(void)
{
    uint16_t i;
    uint16_t n = (uint16_t)(&RamfuncsLoadEnd - &RamfuncsLoadStart);
    const volatile unsigned int *from = &RamfuncsLoadStart;
    volatile unsigned int *to = &RamfuncsRunStart;

    EALLOW;
    WDCR = WDCR_OFF;
    EDIS;
    pll_init();
    EALLOW;
    HISPCP = 0x0001u;                   /* HSPCLK 50 MHz */
    LOSPCP = 0x0002u;                   /* LSPCLK 25 MHz */
    XCLK = (uint16_t)((XCLK & 0xFFFCu) | 0x0002u);
    PCLKCR0 = PCLKCR0_ALL;
    PCLKCR1 = PCLKCR1_ALL;
    EDIS;
    for (i = 0; i < n; i++) { to[i] = from[i]; }
    flash_init();
    pie_init();
    gpio_init();
    adc_init();
    dac_init();
    ecap_init(ECAP1);
    ecap_init(ECAP2);

    T0_TCR = 0x0010u;                   /* 멈춤 */
    T0_PRD = TIMER0_PRD;
    T0_TPR = 0;
    T0_TPRH = 0;
    T0_TCR = 0x4C30u;                   /* TIE, SOFT, FREE, TRB, TSS */

    T1_TCR = 0x0010u;                   /* 자유 카운터: 멈추고 주기 최대, 분주 없음 */
    T1_PRD = 0xFFFFFFFFUL;
    T1_TPR = 0;
    T1_TPRH = 0;
    T1_TCR = 0x0C20u;                   /* SOFT, FREE, TRB. 인터럽트 없이 바로 돈다 */
}

/* Timer1 은 내려 센다. 뒤집어 올라가는 값으로 낸다 */
uint32_t hal_cycles(void) { return ~T1_TIM; }

void hal_start(void)
{
    PIEIER(1) |= 0x0040u;
    PIEIER(4) |= 0x0003u;
    PIEIER(9) |= 0x000Fu;
    IER |= 0x0109u;                     /* INT1, INT4, INT9 */
    T0_TCR &= 0xFFEFu;
    asm(" CLRC INTM");
    asm(" CLRC DBGM");
    EALLOW;
    WDCR = WDCR_ON;
    EDIS;
}

void hal_wdog_kick(void)
{
    EALLOW;
    WDKEY = 0x0055u;
    WDKEY = 0x00AAu;
    EDIS;
}

void hal_delay_us(uint16_t us)
{
    while (us > 0u) {
        asm(" RPT #100 || NOP");
        us--;
    }
}

uint16_t hal_irq_save(void)              { return __disable_interrupts(); }
void     hal_irq_restore(uint16_t state) { __restore_interrupts(state); }

void hal_dac_write(uint16_t channel, uint16_t value)
{
    uint16_t word = (uint16_t)((value & 0x0FFFu) | ((channel & 1u) ? 0x3000u : 0x2000u));
    volatile uint16_t dummy;

    while (SPISTS & 0x0020u) { }        /* BUFFULL */
    SPITXBUF = (uint16_t)(word << 2);
    while ((SPISTS & 0x0040u) == 0u) { }
    GPACLEAR = PIN_LDAC;
    dummy = SPIRXBUF;
    GPASET = PIN_LDAC;
    (void)dummy;
}

void     hal_adc_start(void)   { ADCTRL2 |= 0x2000u; }   /* 시퀀서는 리셋하지 않는다 (원본 동작) */
uint16_t hal_adc_battery(void) { return ADCRESULT0; }

/* 저활성 */
void hal_sound(uint16_t on)
{
    if (on) { GPACLEAR = PIN_SOUND; }
    else    { GPASET = PIN_SOUND; }
}

void hal_led_toggle(void) { GPATOGGLE = PIN_LED; }

/* ---- ISR ---- */

__interrupt void hal_timer0_isr(void)
{
    GPBSET = PINB_TIME;
    sys_tick_2ms();
    GPBCLEAR = PINB_TIME;
    PIEACK = 0x0001u;
}

static void ecap_isr(uint16_t base, uint16_t wheel)
{
    sys_encoder(wheel, ECAP_CAP1(base));
    ECAP_CLR(base) = ECEINT_CEVT1;
    ECAP_CTL2(base) |= 0x0008u;         /* REARM */
    ECAP_CLR(base) = ECCLR_INT;
    PIEACK = 0x0008u;
}

__interrupt void hal_ecap1_isr(void) { ecap_isr(ECAP1, 0); }
__interrupt void hal_ecap2_isr(void) { ecap_isr(ECAP2, 1); }
