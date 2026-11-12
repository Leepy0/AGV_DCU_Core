/*
 * stop.h - MotorStop 상태기계. 모든 정지 지령을 10 ms 제어기 하나가 맡는다.
 *
 * 요청 쪽은 사유만 남기고 기다리지 않는다. 태그 목표는 stop_pulses 를 넣은 뒤에 요청한다.
 * 목표 0 은 "도착"이지 목표 없는 급정지가 아니다.
 * rpm 은 감속기 앞 모터축 값이고 틱은 10 ms 다.
 */
#ifndef AGV_STOP_H
#define AGV_STOP_H

#include "agv/types.h"

#define STOP_MIN_RPM             350    /* 바닥 rpm 시험 기준 속도(예전 정지 바닥 값) */
/* 정지 바닥. 엔코더는 펄스 간격 캡처라 펄스가 오면 속도가 정확하다. 150 rpm 은 바퀴당 펄스 간격 33 ms(3틱)라
 * 멈춤 판단(예측 간격 × 2 = 67 ms)과 정착(0.15 s)이 10 ms 주기로 가린다. 약 1.6 m/min(94.5 rpm 당 1 m/min). 실제 멈춤은 학습이 올린다 */
#define STOP_FLOOR_RPM           150
#define STALL_INTERVAL_FACTOR    2      /* 예측 펄스 간격의 이 배 넘게 펄스가 없으면 멈춤 */
#define STOP_STALL_MIN_TICKS     2      /* 멈춤 판단 최소 틱. 틱 경계 떨림 */
#define STOP_PRED_LAG            0.2f   /* 바퀴 예측 1차 지연(10 ms 마다 지령 쪽으로 이 몫, 시정수 약 50 ms) */
#define STOP_COMMAND_MAX_RPM     3500
#define STOP_LEARN_STEP_RPM      25
#define STOP_LEARN_MAX_RPM       900    /* 학습 상한. 하한은 정지 바닥 */
#define STOP_DISTANCE_TOL_MM     3.0f
#define STOP_ANGLE_TOL_DEG       0.3f
#define STOP_QUIET_TICKS         15     /* 두 바퀴 모두 조용해야 끝난다 */
#define STOP_GRACE_TICKS         20     /* 바닥 속도에서 학습 전 머무는 시간 */
#define STOP_LEARN_INTERVAL      15
#define STOP_LEARN_TIMEOUT       200
#define STOP_NO_PROGRESS_TICKS   300
#define STOP_MOVE_CONFIRM        2
#define STOP_FEEDBACK_DELAY      5
/* 저크(가속도 변화) 고정 STOP_JERK_NUM / STOP_JERK_DEN = 2.4 rpm/틱²: DCC 60 에 25틱(0.25 s). 가감속 한계는 상한일 뿐이라
 * 속도 변화 Δv 가 작으면 최고 가감속도가 √(2.4 × Δv) 쯤에서 풀린다. 예전(위험 감속 SCV_JERK_OLD)은 한계/STOP_JERK_DIV */
#define STOP_JERK_NUM            12
#define STOP_JERK_DEN            5
#define STOP_JERK_DIV            8
#define STOP_TAIL_MARGIN        2      /* 위치 정지: 끝 꼬리 앞 바닥 속도로 가는 펄스(거리 제동이 꼬리 전에 바닥에 들게) */
#define STOP_TAIL_RPM_TICKS      1122   /* 위치 정지 끝 꼬리: 바닥 150 에서 0 까지 저크 S커브로 가는 거리(rpm·틱, 19틱, 2.2 펄스). test_stop 이 확인 */
#define STOP_RAPID_DCC_DIV       1
#define STOP_RAPID_DCC_MAX       60
#define STOP_ENCODER_PPR         12
#define STOP_OBSTACLE_CLEAR_TICKS 30    /* OPT_PAUSE_RESUME_SMOOTH: 감속 중 장애물이 이만큼 비어야 이어 간다 */

typedef enum {
    STOP_REASON_NONE = 0,
    STOP_REASON_TAG_POSITION,
    STOP_REASON_TAG_DOCKING,
    STOP_REASON_BEFORE_TURN,
    STOP_REASON_TURN_ANGLE,
    STOP_REASON_TRAFFIC_POSITION,       /* 교차로 대기 목표 D.cross_pulses. 서면 STOP_REASON_TRAFFIC 유지로 넘어간다 */
    STOP_REASON_MOVE,                   /* 시퀀스 MOVE_MM: 서 있다 출발해 D.stop_pulses 에 선다. 제동점까지 D.speed_cmd 로 순항 */
    STOP_REASON_COMMAND = 10,
    STOP_REASON_SWITCH,
    STOP_REASON_OBSTACLE,
    STOP_REASON_TRAFFIC,
    STOP_REASON_PAUSE,
    STOP_REASON_EMERGENCY = 20
} stop_reason_t;

typedef enum {
    STOP_PHASE_IDLE = 0, STOP_PHASE_CRUISE, STOP_PHASE_DECELERATE, STOP_PHASE_APPROACH,
    STOP_PHASE_LEARN_FLOOR, STOP_PHASE_WAIT_QUIET, STOP_PHASE_COMPLETE, STOP_PHASE_FAILED
} stop_phase_t;

typedef struct {
    int16_t reason, phase;
    int16_t command_rpm, acceleration, maximum_rpm, floor_rpm, candidate_floor_rpm;
    float   pred_rpm;           /* 예측 바퀴 rpm: 지령의 1차 지연. 멈춤 판단의 펄스 간격 */
    int16_t accel_limit, decel_limit;
    int16_t elapsed_ticks, floor_hold_ticks, learn_ticks, candidate_hold_ticks;
    int16_t evidence_left, evidence_right;
    int16_t no_progress_ticks, zero_command_ticks;
    int16_t braking_start_rpm;
    int32_t braking_start_pulses;
    float   required_decel;
    int16_t braking_rate_limited;
    int32_t previous_pulses;
} stop_ctrl_t;

typedef struct {
    int32_t remaining_pulses;
    int16_t tolerance_pulses;
    float   pulses_per_rpm_tick;
    int16_t idle_left, idle_right;
    int16_t fresh_left, fresh_right;
} stop_feedback_t;

/* 마지막 위치 정지 기록. 위치 정지(회전 각 제외)가 두 바퀴 조용(COMPLETE)이 되면 쓴다.
 * 교차로 대기는 뒤이은 유지(STOP_REASON_TRAFFIC)가 조용해질 때 쓴다. 명령·스위치·비상이 끊으면 쓰지 않는다.
 * 넘침 = 목표 지점부터 선 자리까지 진행 방향 거리. 남은 거리 카운터는 0 에서 멈추니 바퀴 누적 펄스로 잰다.
 * CCS Expressions 에 stop_last 를 넣고 본다 */
typedef struct {
    int16_t  reason;            /* 1~6(6 = MOVE_MM) */
    int16_t  entry_rpm;         /* 시작 때 지령 */
    float    target_mm;         /* 시작 때 남은 거리 */
    float    overshoot_mm;      /* + 지나침, - 못 미침 */
    float    required_decel;    /* 계획 감속도 rpm/s. dcc × 100 과 견준다. 0 = 거리 제동 없음 */
    float    max_overshoot_mm;  /* 부팅 뒤 최대 */
    int16_t  rate_limited;      /* 1 = 계획 감속도가 dcc 를 넘어 묶였다 */
    int16_t  stop_ticks;        /* 시작부터 끝까지 10 ms 틱. 일시정지 포함 */
    uint16_t n;                 /* 기록 수 */
} stop_last_t;

extern stop_last_t stop_last;

/* 제어기 본체. 하드웨어 접근 없음 */
void    stop_ctrl_reset(stop_ctrl_t *c);
void    stop_ctrl_begin(stop_ctrl_t *c, int16_t reason, int16_t entry_rpm, int16_t maximum_rpm,
                        int16_t acc, int16_t dcc, int32_t remaining, float pulse_scale);
int16_t stop_ctrl_update(stop_ctrl_t *c, const stop_feedback_t *fb);
int16_t stop_is_position(int16_t reason);
int16_t stop_is_pause(int16_t reason);
int16_t stop_clamp(int16_t value, int16_t lower, int16_t upper);

/* 런타임 */
void    stop_reset(void);                     /* 제어기·요청·일시정지 기억 전부 지움 */
void    stop_clear(void);                     /* 제어기·일시정지 기억만 지움 */
void    stop_clear_request(void);
void    stop_invalidate_paused(void);
void    stop_request_tag(void);
void    stop_request_rapid(int16_t reason);
int16_t stop_run(void);                       /* 반환 0 = 정지 제어가 쉬는 중 */
void    stop_begin_runtime(int16_t reason);
int16_t stop_blocking_reason(void);
int16_t stop_alignment_speed(int16_t target_rpm);
int16_t stop_turn_dcc(void);                   /* 회전 감속. 일반·정렬 회전이 같은 값을 쓴다 */
int16_t stop_turn_speed(int16_t target_rpm);   /* 정렬 회전 속도 경사 */
void    stop_now(void);                       /* RFID 재시도 보상으로 이미 지나친 경우 즉시 0 지령 */

int16_t stop_is_braking(void);
int16_t stop_phase(void);
int16_t stop_reason(void);
int16_t stop_floor_rpm(void);
int16_t stop_min_rpm(void);        /* 정지 바닥 하한: STOP_FLOOR_RPM */
const stop_ctrl_t *stop_ctrl(void);

#endif /* AGV_STOP_H */
