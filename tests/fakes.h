/* fakes.h - 호스트 시험용 가짜 상위 계층 */
#ifndef FAKES_H
#define FAKES_H
#include "agv/drive.h"
#include "agv/param.h"

extern uint32_t fake_in, fake_out, fake_seq_off_count;
extern uint16_t fake_key;           /* ui_key() */
extern int      fake_restart_count, fake_link_full, fake_tx_count;
extern uint16_t fake_tx[256][20];
extern int      fake_speed_arg, fake_forback_arg, fake_turn_count;
extern int      fake_log_count;     /* log_event 횟수. 마지막 code·a 를 남긴다 */
extern uint16_t fake_log_code;
extern int16_t  fake_log_a;

/* EEPROM 흉내. 전원 차단은 fake_ee_cut 번째 쓰기부터 안 써지는 것으로 본다 */
extern uint16_t fake_ee[128];
extern int      fake_ee_writes, fake_ee_cut;
extern const param_store_t fake_store;

void fake_reset(void);
void fake_params(void);     /* 빈 EEPROM 에서 부팅해 기본값 1 세트를 쓴다 */
#endif
