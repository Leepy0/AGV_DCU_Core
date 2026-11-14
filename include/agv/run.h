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

int16_t  run_seek_rpm(void);              /* 정렬 회전 앞 교차 테이프 찾기 속도 rpm. SPN~min(LOS, 12 m/min) */
void     run_move_tape_start(int16_t kind);   /* 시퀀스 AGV_MOVE( 한계, TAPE… ): kind 3 바로 정지, 4 GD 정지. tagact_move_tape 가 부른다 */
int      run_move_tape_on(void);          /* 1 = 교차선까지 이동 중(아직 못 찾음)이거나 교차선 뒤 가장자리를 기다린다 */
int      run_tape_turn_held(void);        /* 1 = 인정한 교차선의 회전을 맡고 있다(회전이 거부됐거나 GD 정지가 끊겨 다시 걸 때까지) */
void     run_tape_rx(uint16_t s, uint16_t e);     /* 'h' 응답마다(guide_poll): 교차선 인정. s·e = 추적기가 고른 덩어리 칸, 없으면 0 */

/* 마지막 교차선 정지 기록(CCS: run_tape_last). 분류 3·4 정지·교차선 회전 앞 정지·AGV_MOVE( 한계, TAPE… ) */
typedef struct {
    int16_t  kind;              /* 3 바로 정지, 4 GD 정지, -1 교차선 회전 */
    int16_t  ticks;             /* 인정한 틱부터 섰을 때까지 10 ms 틱 */
    float    run_mm;            /* 인정한 자리부터 선 자리까지 간 거리(정지 거리를 세는 바퀴) */
    float    width_mm;          /* 잰 교차선 폭(앞·뒤 가장자리 사이). -1 = 못 봄·분류 3 */
    uint16_t n;                 /* 기록 수 */
    float    over_mm;           /* 지나침: 4·회전은 목표 뒤 거리(TAPE_BACK_MM 60 넘으면 258 로 섰다), 3 은 정지를 건 뒤 간 거리 */
} run_tape_last_t;
extern run_tape_last_t run_tape_last;
uint32_t run_pulses(uint16_t wheel);       /* 좌 Total_length, 우 누적 펄스 */
int      run_is_idle(void);                /* gFlag_AGV_None_Action */
int      run_auto_started(void);           /* Flag_Auto_Start */
int16_t  run_wait_left(void);              /* 재출발 대기 남은 초. 대기 아니면 -1 */
int32_t  run_rail_cut(void);               /* 마지막 정상 응답 뒤 이동 펄스 */
void     run_rail_cut_clear(void);

void     run_seq_init(void);               /* Auto_Loading_Init. 교차선 요청·센서 정지 요청도 지운다(run_tape_req_clear, tagact_fs_clear) */
void     run_tape_req_clear(void);         /* 교차선 요청·GD 구간을 지운다. 다시 출발은 지우지 않는다(수동 전환은 ui.c 가 부른다) */
void     run_reset_request(void);          /* Flag_Reset_control */
void     run_wait_start(void);             /* Flag_Wait_Start = ON */
void     run_wait_cancel(void);            /* Flag_Wait_Start = OFF */
void     run_forback_input(uint16_t id);   /* 수동 방향 전환 입력. 기본 IN_IF_5 */
void     run_remote_pause(uint16_t on);    /* 리모컨 입력의 일시정지 사용. 기본 끔 */
void     run_restart_now(void);            /* Flag_Run_Restart = ON */
void     run_dir_change(int16_t code);     /* Dir_Change_Flag: 1 전진, 2 후진 */
void     run_relay(int16_t backward, int16_t save);    /* ForBack_Relay_Change */
void     run_command_stop(void);           /* 'C' 2·3 공통: 즉시 정지와 대기 해제 */

void     run_report_start(void);           /* DA_ACS_Start */
void     run_report_stop(void);            /* DA_ACS_Stop */
void     run_standby(int16_t id, int16_t reset);   /* reset: RUN_STANDBY_* 비트 */
#define  RUN_STANDBY_RESET  1   /* 먼저 스테이션 큐를 지운다(레거시 DA_ACS_Standby 의 RESET) */
#define  RUN_STANDBY_AUTO   2   /* 번호 > 0 이면 자동 재전송을 늘 켠다(회전 중에도). 태그 표 대기 보고 */
#define  RUN_STANDBY_NOSTOP 4   /* 정지는 걸지 않는다(같은 태그 규칙의 정지 분류 2·3·4 가 선다). 보고·자동 재전송은 같다 */
void     run_standby_clear(int16_t id);
int16_t  run_standby_num(void);
void     run_standby_auto_off(void);
#endif
