/* gain.h - 연속 시험(t1) 조향 이득 찾기(단계 7)의 판정과 후보 순서. 하드웨어와 무관한 순수 계산.
 * 판정: 정속 창의 가이드 위치(10 ms 표본, 1/8 단위, 창 평균을 뺀다)로
 *   깨끗 = RMS ≤ GAIN_RMS_MM, 그리고 사행 없음.
 *   사행 = 부호가 번갈아 바뀌는 반주기가 GAIN_RUN_MIN 번 이상 이어진다. 반주기 하나 = 부호 바뀜과 다음 부호 바뀜 사이,
 *          길이 GAIN_HALF_MIN~GAIN_HALF_MAX 틱, 봉우리 GAIN_PEAK8/8 단위 이상. 부호 바뀜은 ±GAIN_HYS8/8 단위 이력으로 센다.
 *   창이 GAIN_WIN_MIN 틱보다 짧으면 판정하지 않는다(verdict 0).
 * 후보: K_P 60, 50, 40, 30, 25, 20. K_I = max(1, 반올림(K_P × 5 / 60)). 60 부터 내려가며 처음 깨끗한 값에서 멈춘다.
 * 모두 탈락이면 20·2 를 추천하고 ng 를 켠다 */
#ifndef AGV_GAIN_H
#define AGV_GAIN_H
#include "agv/types.h"

#define GAIN_CANDS      6
#define GAIN_WIN_MIN    300       /* 정속 창 최소 3 s */
#define GAIN_WIN_MAX    400       /* 정속 창 최대 4 s (표본 400개 = 기존 편차 창 버퍼 크기) */
#define GAIN_SCALE      8         /* 표본 = 가이드 단위 × 8 */
#define GAIN_RMS_MM     6.0f
#define GAIN_HYS8       8         /* ±1 단위 */
#define GAIN_PEAK8      16        /* 봉우리 2 단위 이상 */
#define GAIN_HALF_MIN   20        /* 반주기 0.2 s */
#define GAIN_HALF_MAX   100       /* 1.0 s */
#define GAIN_RUN_MIN    4         /* 이어진 반주기 */
#define GAIN_V_MPM      20        /* 찾기 속도 상한 m/min. 속도 = min(이것, DRS × 2/3) */

enum { GAIN_V_NONE = 0, GAIN_V_CLEAN, GAIN_V_RMS, GAIN_V_WOBBLE, GAIN_V_BOTH };   /* 판정 */

typedef struct {             /* 후보 한 개의 기록(결과 구조체에 남긴다) */
    int16_t rms10;           /* 정속 편차 RMS mm × 10 */
    int16_t peak10;          /* 반주기 봉우리 평균, 단위 × 10 */
    int16_t flips;           /* 부호 바뀜 수 */
    int16_t verdict;         /* GAIN_V_* */
} gain_log_t;

/* g[0..n-1] = 가이드 위치 × GAIN_SCALE. 결과는 r 에 */
void gain_judge(const int16_t *g, int16_t n, gain_log_t *r);

typedef struct { int16_t idx, done, ng; } gain_seq_t;   /* idx = 지금(마지막) 후보 */
void    gain_seq_begin(gain_seq_t *s);
int16_t gain_kp(const gain_seq_t *s);
int16_t gain_ki(const gain_seq_t *s);
void    gain_seq_next(gain_seq_t *s, int clean);        /* 판정을 알린다: 깨끗이면 끝, 아니면 다음 후보(없으면 NG 로 끝) */

#endif /* AGV_GAIN_H */
