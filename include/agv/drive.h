/*
 * drive.h - 주행 제어 공유 상태
 *
 * motor·stop·steer 와 아직 없는 run·guide·ui 모듈이 함께 읽고 쓰는 값을 한곳에 둔다.
 * 레거시 전역을 그대로 옮긴 경계이며, 괄호 안은 대응하는 레거시 이름이다.
 * volatile 표시가 없는 필드는 10 ms 문맥에서만 접근한다.
 */
#ifndef AGV_DRIVE_H
#define AGV_DRIVE_H

#include "agv/types.h"

typedef enum { RUN_READY = 300, RUN_JOG, RUN_DRIVE, RUN_TURN } run_mode_t;           /* Flag_control */
typedef enum {                                                                        /* Traveling_step */
    DRIVE_CHANGE_SPEED = 400, DRIVE_CRUISE, DRIVE_OBS_STOP,
    DRIVE_COMM,             /* 403 안 씀(레거시 COMM_AUTO, 들어가는 길이 없다). 번호 자리만 남긴다 */
    DRIVE_RISK_STOP, DRIVE_RISK_RESUME
} drive_step_t;
typedef enum { UI_AUTO = 101, UI_JOG, UI_PARAM, UI_REMOTE,
               UI_IO_IN = 120, UI_IO_OUT } ui_mode_t;                                 /* current_mode */

/* 앰프 방향 (motorMotionDirection, Turn_Direction 의 값) */
enum { MOTION_FWD = 0, MOTION_BACK = 1, MOTION_CCW = 2, MOTION_CW = 3 };
enum { TAPE_STEP_SETTLE = 3, TAPE_STEP_FIX = 4 };   /* 정렬 회전: 중앙에서 선 뒤 정착, 보정 회전 */
enum { MOVE_FOUND = 4, MOVE_DONE, MOVE_FAIL };       /* move_on 의 뒤 값 */

/* 오류 코드. ACS 로 전송되므로 값 고정 */
#define ERR_AMP_L_OVERLOAD  200
#define ERR_AMP_R_OVERLOAD  201
#define ERR_ENC_L_STALL     202   /* 지령은 있는데 왼쪽 엔코더만 멈춤 */
#define ERR_ENC_R_STALL     203
#define ERR_ESTOP           204
#define ERR_BUMPER          205
#define ERR_PARAM_LIMIT     250
#define ERR_PARAM_CHECKSUM  251
#define ERR_GUIDE_COMM      255
#define ERR_GUIDE_RANGE     257
#define ERR_GUIDE_SENSOR    258
#define ERR_BAT_OVER        260
#define ERR_BRANCH_TIMEOUT  265
#define ERR_STOP_APPROACH   277
#define ERR_BAT_LOW         280
#define ERR_OBSTACLE        281
#define ERR_CROSS_WAIT      285
#define ERR_CROSS_NO_ACK    286
#define ERR_PAUSE           290

#define DRIVE_ALIGN_RAMP 30   /* 정렬 뒤 조향 상한을 0.3 s 에 걸쳐 푼다(steer.c: (60 - k)/60, 30 에서 평소의 절반 = |v| × 1) */
#define DRIVE_ALIGN_HOLD 31   /* start_align 정렬 전 값 */
#ifndef DRIVE_ALIGN_DEV_LO
#define DRIVE_ALIGN_DEV_LO 3  /* 출발 정렬 속도 상한: |위치| 이 단위 이하면 LLOW */
#endif
#ifndef DRIVE_ALIGN_DEV_HI
#define DRIVE_ALIGN_DEV_HI 10 /* 이 단위 이상이면 SPN(위험 감속 문턱과 같다). 사이는 곧게 잇는다 */
#endif
#ifndef DRIVE_ALIGN_DEV_HYS
#define DRIVE_ALIGN_DEV_HYS 2 /* 속도 상한을 내릴 때 편차 히스테리시스(단위) */
#endif
#ifndef DRIVE_ALIGN_MAX_MM
#define DRIVE_ALIGN_MAX_MM 500   /* 출발 뒤 선을 본 채 이만큼(두 바퀴 평균) 가면 정렬 확인이 없어도 출발 정렬을 끝낸다 */
#endif
#ifndef AGV_START_ALIGN
#define AGV_START_ALIGN 1     /* 출발 정렬 단계(모든 빌드). 0 = 예전 출발 */
#endif

typedef struct {
    /* 입력 */
    int16_t ui_mode;                 /* current_mode */
    int16_t stop_sw;                 /* 1 = 눌림 (STOP_SW == nON) */
    int16_t tag_l, tag_r;            /* 1 = 감지 (TAG_L/R_SENSOR == nON) */
    int16_t estop;                   /* Flag_Estop */
    int16_t obs_f, obs_b;            /* 1 = 감지 (OBS_F/B_SENSOR == nON) */
    int16_t obs_f_ext, obs_b_ext;    /* 1 = 감지 (Ext_OBS_F/B == nON) */
    int16_t jog_bits;                /* JOG 입력 4비트 */
    int16_t jog_left_only, jog_right_only, jog_stopping;   /* Flag_*_key_OnlyOn, Flag_jog_stopping */
    int16_t jog_rpm;                 /* 조그 처리 결과. 조그 모드에서 속도 지령으로 쓴다 */
    volatile int32_t period[2];      /* Period. eCAP ISR 이 쓴다 */

    /* 오류 */
    int16_t system_err, stop_err, action_err;

    /* 런 상태 */
    int16_t control;                 /* run_mode_t. Flag_control */
    int16_t step;                    /* drive_step_t. Traveling_step */
    int16_t flag_ready, flag_run, flag_jog_on, flag_turn;
    int16_t flag_stopping;           /* Flag_stopping */
    int16_t stopping_process;        /* DA_Stopping_Process */
    int16_t wait_start, run_restart; /* Flag_Wait_Start, Flag_Run_Restart */
    int16_t backward;                /* AGV_Backward */
    int16_t motion;                  /* motorMotionDirection */
    int16_t amp_dir;                 /* 앰프에 낼 방향 (Driving_Forward_Backward 의 인자) */
    int16_t amp_run;                 /* AMP_x_RUNZ */
    int16_t amp_reset;               /* AMP_x_RESETZ 한 틱 펄스 */
    int16_t sound_on;
    int16_t speed_cmd;               /* gDriving_speed */
    int16_t cur_speed;               /* nCurrent_Speed */
    int16_t vin_out;                 /* 램프 출력 */
    int16_t scv_accel;               /* S커브 가속도 상태 */
    int16_t last_input;              /* da_last_velocity_input */
    int16_t restart_cnt;             /* gReStart_Cnt */
    int16_t drive_start_cnt;         /* Cnt_Driving_strat */
    /* 출발 정렬 단계(START·자동 출발·태그 정지 뒤 출발, run.c start_run). DRIVE_ALIGN_HOLD 이상 = 정렬 전: 조향 상한 |v_cmd| × 1,
     * DRIVE_ALIGN_HOLD 면 속도 상한 = 편차에 따라 LLOW~SPN(선 없음 SPN, motor.c), 위험 감속에 새로 들어가지 않는다.
     * +1 은 튜닝(조향 상한만). 끝 = 정렬 확인 또는 선을 본 채 DRIVE_ALIGN_MAX_MM. 1~DRIVE_ALIGN_RAMP = 끝난 뒤 조향 상한을 ×2 로 푸는 틱.
     * 0 = 평소. AGV_START_ALIGN 0 이면 쓰지 않는다(늘 0, run.c·steer.c·motor.c 코드 없음) */
    int16_t start_align;
    /* 시퀀스 이동(AGV_MOVE, tagact_move_mm·tagact_move_tape). 0 없음, 1 요청(run.c 가 출발), 2 출발함(주행 첫 틱에 정지 거리를 건다),
     * 3 이동 중(거리 이동은 STOP_REASON_MOVE), MOVE_FOUND 교차선 인정 뒤 서는 중. 서면 stop.c finish 가 도착(거리 정지·교차선 인정 뒤)이면
     * MOVE_DONE, 다른 원인이면 MOVE_FAIL 로 두고 시퀀스 스텝이 읽고 0 으로. 시퀀스 중단(run_seq_init) 때 0 */
    int16_t move_on;
    int32_t move_pulses;             /* 거리 이동 목표 펄스(32767 이하), 교차선 이동 한계 펄스 */
    int16_t move_speed;              /* 최고 속도 rpm(SPN~LLOW) */
    int16_t immediate_stop;          /* DA_Rfid_Retry.ImmediateStop */

    /* 정지·회전 거리. 엔코더 ISR 이 감소시킨다 */
    volatile int16_t stop_pulses;    /* gCnt_stop_lenght */
    volatile int16_t turn_pulses;    /* Cnt_Turn_Angle */
    volatile int16_t cross_pulses;   /* 교차로 대기 정지까지 남은 거리. cross_*_flag 가 켜져 있을 때만 뜻이 있다 */
    volatile uint32_t travel_pulses; /* 정지 거리를 세는 바퀴의 누적 펄스. 돈다. 차로만 쓴다 */
    int16_t turn_motion;             /* Turn_Direction: 0 없음, MOTION_CCW, MOTION_CW */
    int16_t turn_90;                 /* 지금 회전의 90도 두 바퀴 펄스 = 휠 간격 값 × 회전 보정비. tagact_turn 이 넣는다 */
    int16_t turn_obs_use;            /* Turn_OBS_Use */
    int16_t turn_rpm;                /* 이번 회전 속도 rpm(태그 지정). 0 = 자동 P.turn_speed. tagact_turn 이 넣고 회전 끝에 0 */
    int16_t tape_align;              /* DA_Tape_Turn_Angle_Align */
    int16_t tape_slow;               /* 1 = 정렬 회전을 처음부터 최저 속도로 돈다 */
    int16_t tape_origin, tape_origin_ok;   /* 회전 시작 때의 가이드 위치. 슬립은 이 값과의 차로 잰다 */
    int16_t tape_step;               /* DA_Tape_Turn_Step */
    int16_t tape_max;                /* DA_Tape_Turn_Max */
    int16_t tape_total_pulse;        /* DA_Tape_Turn_Total_Pulse */
    int16_t tape_lost_cnt;           /* DA_Tape_Turn_Origin_Lost_Cnt */
    float   tape_slip_ratio;         /* DA_Tape_Turn_Slip_Ratio. 출발 라인으로 잰 실제/펄스 비 */
    float   tape_target_cnt;         /* DA_Tape_Turn_Target_Cnt. 슬립 보정 목표. 탐색 범위의 기준 */
    float   tape_end_cnt;            /* 라인 중앙 예상 펄스. 목표를 새 라인 위치로 고친 값. 감속 계획과 TN 표시 */
    float   tape_sum_mt, tape_sum_tt;   /* 슬립비 최소제곱 누적: 잰 펄스 × 돈 펄스, 돈 펄스² */
    int16_t tape_plan_min;           /* 이번 정렬 회전의 감속 계획 최솟값 rpm. 0 = 아직 없음. 계획을 다시 올리지 않는다 */
    int16_t tape_stop_id;            /* DA_Guide_Stop_Action. 교차선에서 서고 난 뒤 시작할 시퀀스. 0 = 없음 */
    int16_t tape_stop_kind;          /* 교차선 찾아 서기: 0 없음, 3 본 자리에서 바로, 4 GD 더 가서(TA_STOP_KIND_*). 태그 해석기가 넣고 run.c 가 처리한 뒤 둘 다 0 으로 */
    int16_t tape_turn_dir;           /* DA_Tape_Turn_Dir. 교차 테이프를 찾으면 회전한다 */
    int16_t tape_turn_deg;           /* DA_Tape_Turn_Degree */
    uint16_t tape_turn_next;         /* 회전 인자 둘째 칸: fb | auto<<4 | obs<<8 */
    volatile int32_t tape_turned;    /* 정렬 회전에서 지금까지 돈 펄스. 엔코더 ISR 이 센다. 중앙에서 선 뒤(정착·보정)는 넣지 않는다 */
    /* 정렬 회전 정지 뒤 보정. tape_step TAPE_STEP_SETTLE·TAPE_STEP_FIX 동안 tape_align 은 1 로 남아 회전이 안 끝난 것으로 보인다 */
    int16_t tape_side;               /* 새 라인이 중앙에 오기 전 위치 부호(+1/-1). 0 = 모름. 같은 부호면 덜 왔다 */
    int16_t tape_wait;               /* 정착 틱 */
    int16_t tape_fix_rev;            /* 1 = 보정 회전이 원래 회전 방향의 반대(지나쳤다) */
    volatile int32_t tape_fix_turned;   /* 이번 보정에서 돈 두 바퀴 펄스. 엔코더 ISR 이 센다 */
    /* 마지막 정렬 회전 기록(CCS). 다음 정렬 회전 시작 때 지운다 */
    int16_t tape_stop_pos;           /* 중앙에서 선 뒤 정착한 편차(단위) */
    int16_t tape_fix_cnt;            /* 보정 횟수 */
    int16_t tape_final_pos;          /* 끝낼 때 편차(단위) */
    int16_t tape_fix_end;            /* -1 정지 뒤 확인 안 함(탐색 범위 끝 등), 0 중앙, 1 보정 횟수 상한, 2 보정 각 상한, 3 선 잃음, 4 방향 모름 */

    /* 일시정지 조건 */
    int16_t cross_in_flag, cross_in_on;     /* Flag_Cross_In_Flag/On */
    int16_t cross_out_flag, cross_out_on;   /* OUT 확인 대기·유지. OUT 태그에서는 서지 않는다 (acs.h) */
    int16_t cross_ack_error;                /* DA_ACS_Cross_In/Out_Ack_Error 의 OR */
    int16_t pause_stop;              /* Flag_Pause_Stop */
    int16_t ext_sen_on;              /* Ext_Sen_OnOff */
    int16_t obs_use;                 /* Flag_OBS_Sen_Use */
    int16_t obs_on;                  /* nOBS_On_Flag */
    int16_t guide_realign_cnt;       /* gGuideRealign_Cnt */
    int16_t risk_stop;               /* nGuide_Risk_Stop_Flag */

    /* 가이드 (guide 모듈이 채운다) */
    int16_t guide_pos;               /* Guide_ReadData */
    int16_t guide_all_on, guide_all_off;
    int16_t guide_wide;              /* Wide_Guide_On_Value */
    uint16_t guide_led;              /* GuideSensor_LED */
    uint32_t guide_led32;            /* GuideSensor_LED_32 하위 32비트 */
    uint16_t guide_span;             /* guide_track 이 고른 덩어리 폭. 'h' 소스의 커브 판정 입력 */
    int16_t guide_track;             /* guide_track_t. 'a' 소스에서는 항상 0 */
    int16_t guide_vert;              /* wide 'a' 응답의 세로 센서(1 켜짐). 'a' 응답 때만 고친다. 아직 쓰는 기능 없음 */
    int16_t guide_kf_init;           /* gGuideKF_Init. 0 이면 다음 호출에서 필터 초기화 */

    /* 분기 */
    int16_t branch_ready;            /* Flag_Branch_Ready */
    int16_t branch_start;            /* Flag_Branch_start */
    int16_t branch_step;             /* Flag_Branch_Step */
    int16_t branch_return_cnt;       /* Cnt_Branch_Return */
    volatile int16_t cnt_branch_l, cnt_branch_r;   /* 엔코더 ISR 이 센다 */
    volatile int32_t straight_cnt;   /* gStraight_Cnt */
    int16_t p_cmd;
    int16_t p_test;                  /* 식별 시험의 옆 계단 지령(칸). 분기가 지우지 않는다 */

    /* 조향 출력 */
    int16_t add_speed_l, add_speed_r;
    int16_t steer_hold;              /* 1 = 조향 보정 0·적분 지움(교차선 뒤 GD 구간, run.c) */
    int16_t offset_kp;               /* offset_p_kp */
    int16_t p_err, p_err_1;
    float   p_kp, p_ki, p_pi_out;
    float   kf_pos, kf_rate, kf_p00, kf_p01, kf_p11;

    /* 속도 루프 */
    int16_t v_cmd;                   /* v_cmd_p */
    int16_t rpm[2];                  /* w_rpm */
    int16_t rpm_avg[2];              /* w_rpm_flt_add: 6샘플 평균, PI 피드백 */
    int16_t rpm_flt[2];              /* w_rpm_flt */
    int16_t rpm_fit_sum;             /* w_rpm_fit_sum */
    int16_t w_cmd[2], w_err[2], w_err_1[2];
    float   iref_it[2], iref_fb[2], w_pi_out[2];
    int16_t tq_cmd[2];               /* DAC 로 내보낼 토크 지령 */

    /* 상위 계층 훅. NULL 이면 건너뛴다 */
    void (*drive_pre)(void);         /* 주행 단계 앞: 시작 처리, 고저속 전환 */
} agv_drive_t;

extern agv_drive_t agv_drive;

/* 상위 모듈이 구현한다 */
void run_request_restart(uint16_t sec);                  /* 재출발 예약. sec 초 뒤. 0 = 바로 */
int  link_send(const uint16_t *buf, uint16_t n);         /* 통째로 큐에 넣으면 1, 여유 없으면 0 */

#endif /* AGV_DRIVE_H */
