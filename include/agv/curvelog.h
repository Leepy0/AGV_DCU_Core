/*
 * curvelog.h - 곡선 구간 기록기. 테이프 곡률(센서 점)과 차체 곡률(오도메트리)을 곡선마다 남긴다.
 *
 * 일반 자동 주행(UI_AUTO, 주행 중, 회전 아님)에서만 기록한다. 튜닝 모드는 기록하지 않는다.
 * 묶음 CURVELOG_GROUPS 개 × 곡선 CURVELOG_CURVES 개. 파라미터 모드에서 자동 모드로 돌아올 때 LG·OPT 가
 * 지금 묶음의 값과 다르면 다음 묶음으로 넘어가고 그 묶음을 비운다(마지막 다음은 1). 같으면 묶음을 잇고,
 * 다르지만 지금 묶음에 곡선이 없으면 번호를 두고 값만 고친다. 묶음마다 그 LG·OPT 를 적는다.
 * 전원 투입 때 전부 비운다(보존 영역 아님). CCS 에서 agv_curvelog 를 읽는다.
 *
 * 좌표: 곡선 시작점이 원점, 시작 때 진행 방향이 x 축, 진행 방향 왼쪽이 y. 후진이면 진행 방향(센서 쪽)이 앞이다.
 * 센서 점 = 회전 중심 + 진행 방향 GD + 왼쪽 편차(조향이 쓰는 위치 부호, 1단위 GUIDE_UNIT_MM).
 */
#ifndef AGV_CURVELOG_H
#define AGV_CURVELOG_H

#include "agv/types.h"

#define CURVELOG_GROUPS 4
#define CURVELOG_CURVES 8

typedef struct {
    int16_t seq;            /* 묶음 안 순번, 1부터 */
    int16_t len_mm;         /* 회전 중심 이동 길이 mm */
    int16_t v_x10;          /* 평균 속도 m/min × 10 */
    int16_t r_tape_mm;      /* 테이프 반경 mm (센서 점 원 맞춤). 0 = 측정 없음·20 m 넘음 */
    int16_t r_body_mm;      /* 차체 반경 mm (길이 / 총 각변화). 0 = 20 m 넘음 */
    int16_t r_theory_mm;    /* 이론 차체 반경 mm = sqrt(테이프² - GD²). 0 = 테이프 반경 없음 */
    int16_t dev_max_x10;    /* 편차 절대값 최대 mm × 10 */
    int16_t dev_rms_x10;    /* 편차 RMS mm × 10 */
    int16_t exit_over_x10;  /* 출구 넘침 mm × 10: 끝 뒤 1 s 안, 구간 평균 편차와 반대쪽 최대 */
    int16_t entry_lag_mm;   /* 진입 지연 mm: 차체 곡률이 문턱을 넘은 거리 - 센서 점 곡률이 넘은 거리. 0 = 센서 점이 못 넘음 */
    int16_t lost;           /* 선 잃음 횟수 */
    int16_t dir;            /* 1 왼쪽, -1 오른쪽 (진행 방향 기준) */
    /* 곡선 안 첫 선 잃음 진단. lost 0 이면 전부 0 */
    int16_t  lost_at_mm;    /* 곡선 시작부터 회전 중심 이동 거리 mm */
    int16_t  lost_prev_pos; /* 직전 정상 응답의 위치(단위) */
    uint16_t lost_prev_lo, lost_prev_hi;    /* 직전 정상 응답의 'h' LED 비트 0~15, 16~31 */
    uint16_t lost_led_lo, lost_led_hi;      /* 선 잃음 응답의 LED 비트. 0 이 아니면 켜진 LED 가 있었는데 받지 않았다 */
    int16_t  lost_trk;      /* b0~1 직전 정상 응답의 추적 상태(0 NORMAL, 1 EDGE, 2 HOLD).
                               b2 선 잃음 동안 다시 잡기 지속 확인(GUIDE_REACQ_N)으로 LOST 를 낸 응답이 있었다 */
} curve_rec_t;

typedef struct {
    int16_t lg, opt;        /* 이 묶음을 시작할 때의 LG·OPT */
    int16_t n;              /* 이 묶음에서 본 곡선 수. CURVELOG_CURVES 를 넘으면 앞 것만 남는다 */
    curve_rec_t c[CURVELOG_CURVES];
} curve_group_t;

typedef struct {
    int16_t group;          /* 지금 묶음 1~CURVELOG_GROUPS */
    curve_group_t g[CURVELOG_GROUPS];
} curvelog_t;

extern curvelog_t agv_curvelog;

/* 10 ms 한 틱 입력. 시험은 이것으로 합성 궤적을 넣는다 */
typedef struct {
    float   dl_mm, dr_mm;   /* 이번 틱 좌우 바퀴 이동 거리(크기) */
    int16_t backward;       /* 1 = 후진 */
    int16_t pos;            /* 가이드 위치(단위) */
    int16_t lost;           /* 1 = 선 없음 */
    uint32_t led;           /* 이번 'h' 응답 LED 비트(일반 센서는 하위 16) */
    int16_t track;          /* guide_track_t */
    int16_t reacq;          /* 1 = 다시 잡기 지속 확인 중(guide_reacq_pending) */
    int16_t active;         /* 0 = 기록 안 함(진행 중 구간은 버린다) */
    float   wb_mm, gd_mm;   /* 휠 간격(회전 보정비 적용), 센서 거리 */
} curvelog_in_t;

void curvelog_reset(void);                  /* 전원 투입: 전부 비우고 묶음 1 */
void curvelog_next_group(void);             /* 다음 묶음으로 넘어가 비운다 */
void curvelog_feed(const curvelog_in_t *in);
void curvelog_tick(void);                   /* 10 ms. 주행 상태를 읽어 curvelog_feed 를 부른다 */

#endif /* AGV_CURVELOG_H */
