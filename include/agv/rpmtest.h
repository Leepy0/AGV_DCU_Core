/* rpmtest.h - 바닥 rpm 시험. 튜닝 모드 t3.2(RPM)에서 START. 제작(t1)은 4회만, 점검(t2.4)은 1·4회만 한다.
 * 바퀴를 띄우지 않는다. 식별 시험 주행로(교차선 X, Y)에서 한다. 0회(선 찾기, L 측정, 회전 두 번)와
 * 선 찾기·회전·재출발은 ident.c 가 하고, 회 안의 측정은 여기서 한다. 회마다 방향이 바뀐다.
 *  1회 DAC 올림 계단(개루프): 200, 400, 600, 800, 1200, 1600, 2000. 단계마다 1.5 s
 *  2회 DAC 내림 계단(개루프): 2000, 1200, 600, 0. 단계마다 1 s. 내린 뒤 0.4 s 감속 곡선
 *  3회 출발·정지 문턱(개루프): DAC 0 → 600 을 3 s, 600 → 0 을 3 s 뒤 0.3 s 대기. 첫·마지막 펄스 때 DAC
 *  4회 저속 폐루프: 목표 300, 250, 200, 150, 100 rpm. 단계마다 1.5 s. 속도 PI·조향 그대로, 바닥 속도 제한 없음
 *  5회 350 rpm 아래 정지(폐루프): 350 rpm 정속에서 세 방법으로 두 번씩 선다. 방법 1·2·3·1·2·3, 0.4 m 간격
 *      1 지령 0 계단, 2 30 mm 램프, 3 60 mm 램프. 램프 지령 = 350 × sqrt(남은 거리 / 램프 거리)
 * 개루프 회(1~3)는 먼저 최저 속도 폐루프로 200 mm 간다(접근, 단계 1). 선에서 벗어나 폐루프가 정한 방향으로
 * 굴러가는 중에 DAC 고정을 건다. 3회는 접근 뒤 DAC 0 으로 선 다음 오름을 시작한다.
 * 개루프 회는 공통 DAC 에 가이드 편차 비례 좌우 차(RPMT_STEER_K, 공통의 ±10%)를 더해 낸다.
 * 회가 끝나면 지령을 실제 속도에서 잇고(속도 PI 적분에 DAC 를 넣어 충격 없이) 다음 회 자리로 간다(ident.h 회 사이 규칙).
 * 개루프 1~3회는 들 거리를 미리 몰라 선에서 출발한다. 4·5회는 rpmt_need_mm 만큼 있으면 직선 위에서 출발한다.
 * 매 틱 안전: rpm 이 1번 최고 속도의 2/3 를 넘거나 앞 선까지 남은 거리(위치 모델) < 감속 거리 + 300 mm 이면
 *  그 단계를 끊고(cut) 회를 끝낸다. 측정 중 교차선도(위치 모델상 선 300 mm 안일 때만. 아니면 실패) 그 단계를 끊고(cut 4) 회를 끝내며, 선 찾기가 선을 본 것처럼
 *  GD 더 가서 돈다. 개루프 중 한 바퀴 멈춤(한쪽 < 20 rpm, 다른 쪽 > 100 rpm 0.3 s)이나 가이드 5칸 이상·라인 잃음은
 *  그 단계를 끊고(cut 8·16·32) 충격 없이 풀어 보통 정지한 뒤 다음 회 자리로 간다.
 *  STOP·E-STOP·오류·패널 키·가이드 편차 10칸 이상(폐루프 회)·접근 10 s 초과는 실패.
 * 실패 코드 = 회 × 10 + 단계. 단계 8 은 회 사이 이동·선 찾기, 9 는 회전. 0회는 08·09.
 * 결과는 agv_rpmt 에 남는다(test_keep) */
#ifndef AGV_RPMTEST_H
#define AGV_RPMTEST_H
#include "agv/types.h"

#define RPMT_PASSES   5
#define RPMT_UP       7
#define RPMT_DN       4
#define RPMT_LOW      5
#define RPMT_STOPS    6
#define RPMT_DN_CURVE 20           /* 20 ms 간격 0.4 s */
#define RPMT_TRACE    30           /* 10 ms 간격 0.3 s */
#define RPMT_STEER_K  20           /* 개루프 조향: 가이드 1칸당 DAC 좌우 차 */
/* 끊은 까닭(cut). 8 이상은 차가 비뚤어져 회를 끊고 보통 정지한 뒤 다음 자리로 간다 */
#define RPMT_CUT_DIST     1        /* 앞 선까지 남은 거리 */
#define RPMT_CUT_OVER     2        /* 과속 */
#define RPMT_CUT_LINE     4        /* 측정 중 교차선 */
#define RPMT_CUT_STALL_L  8        /* 개루프: 좌 바퀴 < 20 rpm, 우 > 100 rpm 이 0.3 s 넘음 */
#define RPMT_CUT_STALL_R  16       /* 개루프: 우 바퀴가 그렇다 */
#define RPMT_CUT_GUIDE    32       /* 개루프: 가이드 5칸 이상이나 라인 잃음 */

typedef struct {             /* 계단 한 단계 (1·2회) */
    int16_t dac;             /* 공통 DAC */
    int16_t rpm[2];          /* 끝 0.5 s 평균. 좌, 우 */
    int16_t slope;           /* 끝 0.5 s 좌우 평균 기울기 rpm/s */
    int16_t ripple[2];       /* 끝 0.5 s 최대 - 최소 */
    int16_t steady;          /* 1 = |slope| < 200 rpm/s */
    int16_t settle_ms;       /* 단계 시작부터 끝 평균 ± 띠(앞 단계와 차의 10%, 최소 20 rpm) 안에 든 뒤 안 나간 시각.
                                -1 = 끝까지 못 듦 */
    int16_t dist_mm;         /* 이 단계에서 간 거리 */
    int16_t ms;              /* 단계 실제 길이. 끊기면 짧다. 0 = 못 함 */
    int16_t cut;             /* RPMT_CUT_*. 0 = 끝까지 함 */
    int16_t dw[2];           /* 단계 마지막 틱에 낸 좌, 우 DAC(조향 차 포함) */
} rpmt_step_t;

typedef struct {             /* 3회 */
    int16_t move_dac[2];     /* 오름 중 첫 펄스 때 그 바퀴 DAC. -1 = 안 움직임 */
    int16_t move_rpm[2];     /* 그때 측정 rpm */
    int16_t stop_dac[2];     /* 내림·대기 중 마지막 펄스 때 그 바퀴 DAC. -1 = 펄스 없음 */
    int16_t stop_rpm[2];     /* 그때 측정 rpm */
    int16_t peak_rpm[2];     /* 회 중 최대 */
    int16_t dist_mm;
    int16_t cut;             /* RPMT_CUT_*. 접근 뒤 3 s 안에 안 서도 1 */
} rpmt_brk_t;

typedef struct {             /* 4회 한 단계. 끝 1 s 로 잰다 */
    int16_t target;
    int16_t mean[2];
    int16_t lo[2], hi[2];
    int16_t ripple[2];       /* 표준편차 */
    int16_t stall[2];        /* 100 ms 넘게 펄스가 없던 횟수 */
    int16_t upd[2];          /* 펄스가 온(측정이 바뀐) 틱 수. 0 = 측정이 멈췄다 */
    int16_t tq[2];           /* 평균 토크 지령(DAC) */
    int16_t dist_mm;
    int16_t cut;
} rpmt_low_t;

typedef struct {             /* 5회 정지 한 번 */
    float   dist_mm;         /* 트리거부터 정지(마지막 펄스)까지 엔코더 거리 */
    float   under_mm;        /* 램프 끝 전에 섰으면 램프 거리 - 간 거리. 0 = 끝까지 감. 방법 1 은 0 */
    float   peak_dec;        /* 정지 전 0.2 s, 10 ms rpm 차 최대 × 100. rpm/s */
    int16_t method;          /* 1 계단 0, 2 30 mm 램프, 3 60 mm 램프 */
    int16_t v0;              /* 트리거 직전 0.1 s 좌우 평균 rpm */
    int16_t t_ms;            /* 트리거부터 마지막 펄스까지. -1 = 3 s 안에 안 섬 */
    int16_t cut;             /* 1 = 안 섬, 2 = 남은 거리로 끊음(이 정지는 못 함), 4 = 교차선(비트) */
    int16_t trace[RPMT_TRACE];   /* 마지막 펄스까지 0.3 s, 10 ms 마다 좌우 평균 rpm. [29] = 마지막 펄스 틱 */
} rpmt_stop_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 회 × 10 + 단계 */
    int16_t pass, phase;     /* 회 0~5, 단계 1~7 (8 이동·선 찾기, 9 회전) */
    int16_t cfg_high;        /* 시작 때 1번 */
    int16_t lim_rpm;         /* 과속 문턱 = 1번 rpm × 2/3 */
    int16_t steer_k;         /* RPMT_STEER_K */
    int16_t amp;             /* 앰프 추정. 'S'(83) 속도 지령, 'T'(84) 토크 지령, 0 모름 */
    int16_t low_rpm;         /* 4회에서 멈춤 없이(stall 0, 측정 갱신, 평균이 목표 ±25%) 돈 가장 낮은 목표. 0 = 없음 */
    int16_t brk_dac;         /* 3회 move_dac 좌우 큰 값. -1 = 안 움직임 */
    float   track_mm;        /* 0회에 잰 L */
    rpmt_step_t up[RPMT_UP];
    rpmt_step_t dn[RPMT_DN];
    int16_t dn_curve[RPMT_DN - 1][RPMT_DN_CURVE];   /* 2회 단계 2~4 시작 뒤 20 ms 마다 좌우 평균 rpm */
    rpmt_brk_t  brk;
    rpmt_low_t  low[RPMT_LOW];
    rpmt_stop_t stp[RPMT_STOPS];
} rpmt_result_t;

extern rpmt_result_t agv_rpmt;

/* ident.c 가 부른다. 회 시작(선 위, 재출발 뒤)과 측정 틱. 0 진행, 1 회 끝, 2 회 끝(비뚤어짐, 세운다), -1 실패 */
void rpmt_pass_begin(void);
int  rpmt_pass_run(void);
void rpmt_pass_cut(int16_t why);   /* 측정 중 교차선: 지금 단계(또는 3·5회 기록)를 why 로 끊고 회를 끝낸다 */
float rpmt_need_mm(uint16_t pass); /* 그 회에 필요한 앞 거리 mm. 0 = 개루프라 미리 모른다(선에서 출발) */
#endif
