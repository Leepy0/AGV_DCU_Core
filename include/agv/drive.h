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
    DRIVE_CHANGE_SPEED = 400, DRIVE_CRUISE, DRIVE_OBS_STOP, DRIVE_COMM, DRIVE_RISK_STOP, DRIVE_RISK_RESUME
} drive_step_t;
typedef enum { UI_AUTO = 101, UI_JOG, UI_PARAM, UI_REMOTE,
               UI_IO_IN = 120, UI_IO_OUT } ui_mode_t;                                 /* current_mode */

/* 앰프 방향 (motorMotionDirection, Turn_Direction 의 값) */
enum { MOTION_FWD = 0, MOTION_BACK = 1, MOTION_CCW = 2, MOTION_CW = 3 };

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
    int16_t immediate_stop;          /* DA_Rfid_Retry.ImmediateStop */

    /* 정지·회전 거리. 엔코더 ISR 이 감소시킨다 */
    volatile int16_t stop_pulses;    /* gCnt_stop_lenght */
    volatile int16_t turn_pulses;    /* Cnt_Turn_Angle */
    int16_t turn_motion;             /* Turn_Direction: 0 없음, MOTION_CCW, MOTION_CW */
    int16_t turn_obs_use;            /* Turn_OBS_Use */
    int16_t tape_align;              /* DA_Tape_Turn_Angle_Align */
    int16_t tape_slow;               /* 1 = 정렬 회전을 처음부터 최저 속도로 돈다 */
    int16_t tape_origin, tape_origin_ok;   /* 회전 시작 때의 가이드 위치. 슬립은 이 값과의 차로 잰다 */
    int16_t tape_step;               /* DA_Tape_Turn_Step */
    int16_t tape_max;                /* DA_Tape_Turn_Max */
    int16_t tape_total_pulse;        /* DA_Tape_Turn_Total_Pulse */
    int16_t tape_lost_cnt;           /* DA_Tape_Turn_Origin_Lost_Cnt */
    float   tape_slip_ratio;         /* DA_Tape_Turn_Slip_Ratio */
    float   tape_target_cnt;         /* DA_Tape_Turn_Target_Cnt */
    int16_t tape_stop_id;            /* DA_Guide_Stop_Action. 0 = 없음 */
    int16_t tape_turn_dir;           /* DA_Tape_Turn_Dir. 교차 테이프를 찾으면 회전한다 */
    int16_t tape_turn_deg;           /* DA_Tape_Turn_Degree */
    uint16_t tape_turn_next;         /* 회전 인자 둘째 칸: fb | auto<<4 | obs<<8 */
    volatile int32_t tape_turned;    /* 정렬 회전에서 지금까지 돈 펄스. 엔코더 ISR 이 센다 */

    /* 일시정지 조건 */
    int16_t cross_in_flag, cross_in_on;     /* Flag_Cross_In_Flag/On */
    int16_t cross_out_flag, cross_out_on;   /* Flag_Cross_Out_Flag/On */
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
    int16_t guide_kf_init;           /* gGuideKF_Init. 0 이면 다음 호출에서 필터 초기화 */

    /* 분기 */
    int16_t branch_ready;            /* Flag_Branch_Ready */
    int16_t branch_start;            /* Flag_Branch_start */
    int16_t branch_step;             /* Flag_Branch_Step */
    int16_t branch_return_cnt;       /* Cnt_Branch_Return */
    volatile int16_t cnt_branch_l, cnt_branch_r;   /* 엔코더 ISR 이 센다 */
    volatile int32_t straight_cnt;   /* gStraight_Cnt */
    int16_t p_cmd;

    /* 조향 출력 */
    int16_t add_speed_l, add_speed_r;
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
