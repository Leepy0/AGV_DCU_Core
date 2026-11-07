/*
 * run.h - 런 조정. 시작·정지·비상, 오류 처리, 방향 전환, 대기 시작, 시퀀스 디스패치.
 * run_coordinate 는 motor_run 앞에서 돈다. E-STOP·범퍼·STOP 이 같은 틱의 motor_run 에 닿게 하려는 순서다.
 */
#ifndef AGV_RUN_H
#define AGV_RUN_H
#include "agv/types.h"

#define RUN_READY_TIMEOUT 200       /* 가이드 준비 시간 초과 2 s */

void     run_reset(void);
void     run_boot_restore(void);    /* 부팅: KEEP 값을 되살린다 */
void     run_coordinate(void);      /* 10 ms, motor_run 앞 */
void     run_seq(void);             /* 10 ms, motor_run 뒤. 시퀀스 스텝과 ACT */
void     run_after_motor(void);     /* 10 ms, site_manual 뒤. error_check 후반 */
void     run_encoder(uint16_t wheel, uint32_t period);   /* eCAP ISR */

uint32_t run_pulses(uint16_t wheel);       /* 좌 Total_length, 우 누적 펄스 */
int      run_is_idle(void);                /* gFlag_AGV_None_Action */
int      run_auto_started(void);           /* Flag_Auto_Start */
int32_t  run_rail_cut(void);               /* 마지막 정상 응답 뒤 이동 펄스 */
void     run_rail_cut_clear(void);

void     run_seq_init(void);               /* Auto_Loading_Init */
void     run_reset_request(void);          /* Flag_Reset_control */
void     run_wait_start(void);             /* Flag_Wait_Start = ON */
void     run_restart_now(void);            /* Flag_Run_Restart = ON */
void     run_dir_change(int16_t code);     /* Dir_Change_Flag: 1 전진, 2 후진 */
void     run_relay(int16_t backward, int16_t save);    /* ForBack_Relay_Change */
void     run_command_stop(void);           /* 'C' 2·3 공통: 즉시 정지와 대기 해제 */

void     run_report_start(void);           /* DA_ACS_Start */
void     run_report_stop(void);            /* DA_ACS_Stop */
void     run_standby(int16_t id, int16_t reset);
void     run_standby_clear(int16_t id);
int16_t  run_standby_num(void);
void     run_standby_auto_off(void);
#endif
