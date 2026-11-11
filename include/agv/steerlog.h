/*
 * steerlog.h - 조향 기록기. 곡선 계단 조향·교차선 급감속의 원인을 보려고 주행 중 10 ms 시계열과 사건을 남긴다.
 * 개발 시험이라 AGV_DEV_TESTS 0 이미지에는 없다(코드·데이터·훅 전부).
 *
 * 기록: RUN_DRIVE 이고 vin_out 이 0 이 아닌 동안 10 ms 마다 표본 하나(sys.c, curvelog_tick 다음).
 * 차가 서면(vin_out 0 또는 RUN_DRIVE 벗어남) 얼고, 다음 출발 때 표본·사건·설정을 지우고 새로 시작한다.
 * 링은 마지막 STEERLOG_N 표본(약 6 s)을 남긴다. 표본 k(출발 뒤 0 부터)는 s[k % STEERLOG_N],
 * total - n <= k < total 일 때만 남아 있다. 가장 오래된 표본은 n < STEERLOG_N 이면 s[0], 아니면 s[w].
 * 서 있는 동안 CCS 에서 agv_steerlog 를 읽는다. 비보존 RAM(L0 데이터 구역, cmd 의 steerlog_ram)이라 전원·리셋에 지워진다.
 *
 * 표본 한 칸 6워드. 풀기:
 *   pf : b0~7 guide_pos(8비트 부호, ±127 에서 자름), b8~10 step - 400(drive_step_t, 7 = 그 밖),
 *        b11~12 guide_track, b13 guide_all_on, b14 guide_all_off, b15 곡선 판정(steer.c curve_detected 와 같은 식)
 *   p2 : p_pi_out(조향 출력 rpm, steer.c 처럼 소수 버림) × 2 + risk_stop.  p = p2 >> 1, risk = p2 & 1
 *   io : b0~7 적분 몫 i_out / 16(8비트 부호, rpm 16 단위), b8~15 offset_kp(8비트 부호, ±127 에서 자름)
 *   v_cmd, rpm[0], rpm[1] : 속도 지령, 실측 rpm 좌·우(D.rpm)
 *   w_cmd 는 담지 않고 푼다: 좌 = max(0, v_cmd + p + offset_kp), 우 = max(0, v_cmd - p - offset_kp)
 *   (motor.c motor_speed_pi 의 RUN_DRIVE 식. 정지 처리가 속도 루프를 건너뛴 틱만 다를 수 있다)
 */
#ifndef AGV_STEERLOG_H
#define AGV_STEERLOG_H

#include "agv/tune.h"
#include "agv/types.h"

#if AGV_DEV_TESTS

#define STEERLOG_N   600     /* 6.00 s. 표본 6워드. 상태(s_sl)·헤더와 같이 cmd RAML0D(0xF80 워드)에 들어가야 한다. 616 은 실기 링크에서 34워드 넘쳤다 */
#define STEERLOG_EV  16

enum {                       /* 사건 종류. 모두 '진입' 순간 한 건 */
    STEERLOG_EV_RISK = 1,    /* 위험 감속 진입(risk_stop 0 → 1, motor.c motor_drive_step) */
    STEERLOG_EV_LIMIT,       /* 조향 출력이 상한(|v_cmd| × 2, 1500 이하)에 닿음 */
    STEERLOG_EV_JUMP,        /* 응답 사이 guide_pos 가 6단위 이상 뜀(전부 켜짐·꺼짐 응답 제외) */
    STEERLOG_EV_ALL_ON,      /* guide_all_on 진입 */
    STEERLOG_EV_LOS,         /* auto_speed_change 가 속도를 LOS 로 낮춤 */
    STEERLOG_EV_DRS,         /* auto_speed_change 가 속도를 DRS 로 올림 */
    STEERLOG_EV_LOST         /* 선 잃음(guide_all_off) 진입 */
};

typedef struct {
    uint16_t pf;
    int16_t  p2;
    int16_t  io;
    int16_t  v_cmd;
    int16_t  rpm[2];
} steerlog_smp_t;

typedef struct {
    int16_t  kind;           /* STEERLOG_EV_* */
    int16_t  trk;            /* b0~1 이번 응답 guide_track, b4~5 직전 응답 guide_track */
    int32_t  dist_mm;        /* 출발 뒤 주행 거리 mm(두 바퀴 평균) */
    int16_t  prev_pos, pos;  /* 직전·이번 응답 guide_pos */
    uint16_t prev_lo, prev_hi;   /* 직전 응답 LED 비트 0~15, 16~31(wide 는 guide_led32, 일반은 guide_led) */
    uint16_t lo, hi;         /* 이번 응답 LED 비트 */
    int16_t  vin_out;        /* rpm */
    int16_t  p_pi_out;       /* rpm */
    int16_t  tape_w;         /* guide_tape_width 칸. 0 = 모름 */
    uint16_t tick;           /* 그 틱의 표본 번호(total 하위 16비트). 링에서 그때를 찾는다 */
} steerlog_ev_t;

typedef struct {             /* 출발 때 설정 */
    int32_t  straight_len;   /* P.straight_len 펄스 */
    int16_t  auto_drive_mode, auto_slow_mode;
    int16_t  tag_pulse2, tag_pulse10;   /* P.tag_pulse[2](자동 감속 곡선 거리), P.tag_pulse[10] */
    int16_t  los, drs, acc, dcc;        /* P.low_speed, P.target_speed rpm, P.acc, P.dcc */
    int16_t  obs_use;        /* D.obs_use. 0 이면 auto_speed_change 가 안 돈다 */
    int16_t  speed_high;     /* param P_SPEED_HIGH(m/min). 자동 감속은 이 값 범위에서만 */
} steerlog_cfg_t;

typedef struct {
    uint16_t w;              /* 다음에 쓸 칸 */
    uint16_t n;              /* 든 표본 수(최대 STEERLOG_N) */
    uint32_t total;          /* 출발 뒤 표본 수(10 ms) */
    uint16_t ev_n;           /* 사건 총 횟수. ev 에는 처음 STEERLOG_EV 건만 */
    uint16_t runs;           /* 전원 투입 뒤 출발 횟수 */
    steerlog_cfg_t cfg;
    steerlog_ev_t  ev[STEERLOG_EV];
    steerlog_smp_t s[STEERLOG_N];
} steerlog_t;

extern steerlog_t agv_steerlog;

void  steerlog_reset(void);          /* 전원 투입: 전부 비운다 */
void  steerlog_tick(void);           /* 10 ms. sys.c 가 curvelog_tick 다음에 부른다 */
void  steerlog_event(int16_t kind);  /* 다른 모듈의 사건(run.c auto_speed_change). 기록 중일 때만 남긴다 */
float steer_i_out(void);             /* steer.c 적분 몫(읽기 전용) */

#endif /* AGV_DEV_TESTS */
#endif /* AGV_STEERLOG_H */
