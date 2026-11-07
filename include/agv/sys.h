/* sys.h - 메인 스케줄러. docs/01_아키텍처.md 2절 */
#ifndef AGV_SYS_H
#define AGV_SYS_H
#include "agv/types.h"

#ifndef SYS_VERSION
#define SYS_VERSION 2611     /* YYMM. 빌드가 -DSYS_VERSION 으로 덮어쓴다 */
#endif

void     sys_boot(void);     /* 인터럽트 허용 전까지 */
void     sys_scan(void);     /* 메인 루프 한 번 */
uint16_t sys_overrun(void);
uint16_t sys_ticks(void);    /* 2 ms 카운터 */
int      sys_power_on(void); /* gPowerOnPulse */
#endif
