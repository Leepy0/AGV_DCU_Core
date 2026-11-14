/* link.h - SCI-A 링크 선택. 프레임 위 계층은 링크를 모른다. link_send 는 drive.h */
#ifndef AGV_LINK_H
#define AGV_LINK_H
#include "agv/types.h"

enum { LINK_ZIGBEE = 0, LINK_WIFI = 1 };

#define LINK_BRR_9600    0x0144u
#define LINK_BRR_38400   0x0051u
#define LINK_BRR_115200  0x001Au

/* 표시 모드와 P_INTERFACE_MODE·P_COMM_LINK 로 SCI 값을 정한다 */
void link_config(int16_t ui_mode, int16_t iface, int16_t kind, uint16_t *ccr, uint16_t *brr);
void link_select(void);                      /* 현재 파라미터로 SCI-A 를 다시 잡는다 */
void link_guide_init(void);                  /* SCI-B: 가이드센서 115200 8N1 고정 */
#endif
