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

#define STOP_MIN_RPM             350
#define STOP_COMMAND_MAX_RPM     3500
#define STOP_LEARN_STEP_RPM      25
#define STOP_LEARN_MAX_RPM       900
#define STOP_DISTANCE_TOL_MM     3.0f
#define STOP_ANGLE_TOL_DEG       0.3f
#define STOP_STALL_TICKS         10
#define STOP_QUIET_TICKS         15     /* 두 바퀴 모두 조용해야 끝난다 */
#define STOP_GRACE_TICKS         20     /* 바닥 속도에서 학습 전 머무는 시간 */
#define STOP_LEARN_INTERVAL      15
#define STOP_LEARN_TIMEOUT       200
#define STOP_NO_PROGRESS_TICKS   300
#define STOP_MOVE_CONFIRM        2
#define STOP_FEEDBACK_DELAY      5
#define STOP_JERK_DIV            8
#define STOP_TURN_DCC_DIV        4
#define STOP_ENTRY_BLEND_PCT     10     /* 위치 제동 초반의 진입 혼합 구간. 0 이면 등감속 */
#define STOP_TURN_DCC_MAX        10
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
const stop_ctrl_t *stop_ctrl(void);

#endif /* AGV_STOP_H */
