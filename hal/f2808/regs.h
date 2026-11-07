/* regs.h - F2808 레지스터 주소와 값. TI 헤더 없이 빌드하려고 쓰는 것만 자체 정의한다 */
#ifndef HAL_REGS_H
#define HAL_REGS_H

#include "agv/types.h"

#define R16(a)  (*(volatile uint16_t *)(a))
#define R32(a)  (*(volatile uint32_t *)(a))

#define EALLOW  asm(" EALLOW")
#define EDIS    asm(" EDIS")

extern cregister volatile unsigned int IER;
extern cregister volatile unsigned int IFR;

/* 시스템 제어 */
#define XCLK        R16(0x7010)
#define PLLSTS      R16(0x7011)
#define HISPCP      R16(0x701A)
#define LOSPCP      R16(0x701B)
#define PCLKCR0     R16(0x701C)
#define PCLKCR1     R16(0x701D)
#define PLLCR       R16(0x7021)
#define WDKEY       R16(0x7025)
#define WDCR        R16(0x7029)

/* 플래시 대기 상태 */
#define FOPT        R16(0x0A80)
#define FSTDBYWAIT  R16(0x0A84)
#define FACTIVEWAIT R16(0x0A85)
#define FBANKWAIT   R16(0x0A86)
#define FOTPWAIT    R16(0x0A87)

/* PIE */
#define PIECTRL     R16(0x0CE0)
#define PIEACK      R16(0x0CE1)
#define PIEIER(n)   R16(0x0CE2 + 2u * ((n) - 1u))
#define PIEIFR(n)   R16(0x0CE3 + 2u * ((n) - 1u))
#define PIE_VECT    0x0D00u
#define PIE_VECT_COUNT 128u
/* 그룹 x 의 y 번째 벡터 주소 */
#define PIE_VEC(x, y) R32(PIE_VECT + 0x40u + (((x) - 1u) * 8u + ((y) - 1u)) * 2u)

/* CPU Timer0 */
#define T0_PRD      R32(0x0C02)
#define T0_TCR      R16(0x0C04)
#define T0_TPR      R16(0x0C06)
#define T0_TPRH     R16(0x0C07)

/* GPIO 제어 */
#define GPAMUX1     R32(0x6F86)
#define GPAMUX2     R32(0x6F88)
#define GPADIR      R32(0x6F8A)
#define GPAPUD      R32(0x6F8C)
#define GPAQSEL1    R32(0x6F82)
#define GPAQSEL2    R32(0x6F84)
#define GPBQSEL1    R32(0x6F92)
#define GPBMUX1     R32(0x6F96)
#define GPBDIR      R32(0x6F9A)
#define GPBPUD      R32(0x6F9C)
#define GPADIR_L    R16(0x6F8A)

/* GPIO 데이터 */
#define GPADAT_L    R16(0x6FC0)
#define GPADAT      R32(0x6FC0)
#define GPASET      R32(0x6FC2)
#define GPACLEAR    R32(0x6FC4)
#define GPATOGGLE   R32(0x6FC6)
#define GPBDAT      R32(0x6FC8)
#define GPBSET      R32(0x6FCA)
#define GPBCLEAR    R32(0x6FCC)

/* 핀 */
#define PIN_RW      (1UL << 11)
#define PIN_OE      (1UL << 12)
#define PIN_STB     (1UL << 13)
#define PIN_A3      (1UL << 20)
#define PIN_LDAC    (1UL << 22)
#define PIN_LED     (1UL << 26)
#define PIN_SOUND   (1UL << 27)
#define PIN_EE_SK   (1UL << 30)
#define PIN_EE_CS   (1UL << 31)
#define PINB_EE_DI  (1UL << 0)    /* GPIO32 */
#define PINB_EE_DO  (1UL << 1)    /* GPIO33 */
#define PINB_TIME   (1UL << 2)    /* GPIO34: 2 ms ISR 시간 측정 */

/* SPI-A */
#define SPICCR      R16(0x7040)
#define SPICTL      R16(0x7041)
#define SPISTS      R16(0x7042)
#define SPIBRR      R16(0x7044)
#define SPIRXBUF    R16(0x7047)
#define SPITXBUF    R16(0x7048)

/* ADC */
#define ADCTRL1     R16(0x7100)
#define ADCTRL2     R16(0x7101)
#define ADCMAXCONV  R16(0x7102)
#define ADCCHSELSEQ1 R16(0x7103)
#define ADCCHSELSEQ3 R16(0x7105)
#define ADCRESULT0  R16(0x7108)
#define ADCTRL3     R16(0x7118)

/* eCAP. base = 0x6A00, 0x6A20 */
#define ECAP_CAP1(b)   R32((b) + 0x04u)
#define ECAP_CTL1(b)   R16((b) + 0x14u)
#define ECAP_CTL2(b)   R16((b) + 0x15u)
#define ECAP_EINT(b)   R16((b) + 0x16u)
#define ECAP_CLR(b)    R16((b) + 0x18u)

/* SCI. base = 0x7050, 0x7750 */
#define SCI_CCR(b)     R16((b) + 0x0u)
#define SCI_CTL1(b)    R16((b) + 0x1u)
#define SCI_HBAUD(b)   R16((b) + 0x2u)
#define SCI_LBAUD(b)   R16((b) + 0x3u)
#define SCI_CTL2(b)    R16((b) + 0x4u)
#define SCI_RXST(b)    R16((b) + 0x5u)
#define SCI_RXBUF(b)   R16((b) + 0x7u)
#define SCI_TXBUF(b)   R16((b) + 0x9u)
#define SCI_FFTX(b)    R16((b) + 0xAu)
#define SCI_FFRX(b)    R16((b) + 0xBu)
#define SCI_FFCT(b)    R16((b) + 0xCu)

#endif /* HAL_REGS_H */
