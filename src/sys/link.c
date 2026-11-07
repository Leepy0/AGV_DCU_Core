/* link.c - SCI-A 보레이트 선택. WiFi 전용 처리(F 프레임, 생존 신호)는 fwup·mgmt 와 함께 넣는다 */
#include "agv/link.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/hal.h"
#include "agv/param.h"

void link_config(int16_t ui_mode, int16_t iface, int16_t kind, uint16_t *ccr, uint16_t *brr)
{
    *ccr = HAL_SCI_8N1;
    *brr = LINK_BRR_115200;
    if ((kind == LINK_WIFI) || (ui_mode != UI_AUTO)) { return; }
    if ((iface >= 2) && (iface <= 7)) { *brr = LINK_BRR_9600; }
    else if (iface == 8)              { *ccr = HAL_SCI_8N2; }
    else                              { *brr = LINK_BRR_38400; }
}

void link_select(void)
{
    uint16_t ccr, brr;
    link_config(agv_drive.ui_mode, param_get(P_INTERFACE_MODE), param_get(P_COMM_LINK), &ccr, &brr);
    hal_sci_init(HAL_SCI_A, ccr, brr);
}

void link_guide_init(void) { hal_sci_init(HAL_SCI_B, HAL_SCI_8N1, LINK_BRR_115200); }

int link_send(const uint16_t *buf, uint16_t n) { return hal_sci_send(HAL_SCI_A, buf, n); }
