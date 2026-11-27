/* gain.h - 연속 시험(t1) 조향 이득 찾기(단계 7)의 판정과 후보 순서. 하드웨어와 무관한 순수 계산.
 * 표본: 정속 창 6~8 s(GAIN_TICKS_MIN~GAIN_TICKS_MAX 틱) 동안 GAIN_DECIM 틱(20 ms)마다 가이드 위치 하나(1/8 단위).
 *   표본 수는 GAIN_N_MIN~GAIN_N_MAX(300~400). 버퍼는 편차 창 버퍼 그대로(400칸)다. 아래 반주기 길이는 표본 수다.
 * 판정: 창 평균을 뺀 표본으로
 *   깨끗 = RMS ≤ GAIN_RMS_MM, 그리고 사행 없음.
 *   사행 = 부호가 번갈아 바뀌는 반주기가 GAIN_RUN_MIN 번 이상 이어진다. 반주기 하나 = 부호 바뀜과 다음 부호 바뀜 사이,
 *          길이 GAIN_HALF_MIN~GAIN_HALF_MAX 표본(0.2~2.0 s), 봉우리 GAIN_PEAK8/8 단위 이상. 부호 바뀜은 ±GAIN_HYS8/8 단위 이력으로 센다.
 *   표본이 GAIN_N_MIN 개보다 적으면(창 6 s 미만) 판정하지 않는다(verdict 0).
 * 후보(K_P·K_I 쌍): 60·5, 60·2, 60·1, 50·1, 40·1. K_I 를 먼저 내리고 K_P 는 그 뒤에 내린다. 앞에서부터 달려
 *   처음 깨끗한 값을 찾으면 여유를 위해 한 칸 낮은 후보를 한 번 더 달린다. 그것도 깨끗하면 그 값, 탈락이면 처음 깨끗한 값을 추천한다.
 *   처음 깨끗한 값이 마지막(40·1)이면 그것으로 끝(NG 아님). 모두 탈락이면 40·1 을 추천하고 ng 를 켠다 */
#ifndef AGV_GAIN_H
#define AGV_GAIN_H
#include "agv/types.h"

#define GAIN_CANDS      5
#define GAIN_DECIM      2         /* 표본 간격 = 2틱(20 ms) */
#define GAIN_TICKS_MIN  600       /* 정속 창 최소 6 s (틱) */
#define GAIN_TICKS_MAX  800       /* 정속 창 최대 8 s (틱) */
#define GAIN_N_MIN      (GAIN_TICKS_MIN / GAIN_DECIM)   /* 300 표본 */
#define GAIN_N_MAX      (GAIN_TICKS_MAX / GAIN_DECIM)   /* 400 표본 = 기존 편차 창 버퍼 크기 */
#define GAIN_SCALE      8         /* 표본 = 가이드 단위 × 8 */
#define GAIN_RMS_MM     6.0f
#define GAIN_HYS8       8         /* ±1 단위 */
#define GAIN_PEAK8      14        /* 봉우리 1.75 단위 이상 */
#define GAIN_HALF_MIN   10        /* 반주기 0.2 s (표본) */
#define GAIN_HALF_MAX   100       /* 2.0 s (표본) */
#define GAIN_RUN_MIN    4         /* 이어진 반주기 */
#define GAIN_V_MPM      20        /* 찾기 속도 상한 m/min. 속도 = min(이것, DRS × 2/3) */

enum { GAIN_V_NONE = 0, GAIN_V_CLEAN, GAIN_V_RMS, GAIN_V_WOBBLE, GAIN_V_BOTH };   /* 판정 */

typedef struct {             /* 후보 한 개의 기록(결과 구조체에 남긴다) */
    int16_t rms10;           /* 정속 편차 RMS mm × 10 */
    int16_t peak10;          /* 반주기 봉우리 평균, 단위 × 10 */
    int16_t flips;           /* 부호 바뀜 수 */
    int16_t verdict;         /* GAIN_V_* */
} gain_log_t;

/* g[0..n-1] = 가이드 위치 × GAIN_SCALE(20 ms 표본, n ≤ GAIN_N_MAX). 결과는 r 에 */
void gain_judge(const int16_t *g, int16_t n, gain_log_t *r);

/* idx = 지금(마지막) 후보, 끝나면 추천 후보. first = 처음 깨끗했던 후보(-1 = 아직) */
typedef struct { int16_t idx, done, ng, first; } gain_seq_t;
void    gain_seq_begin(gain_seq_t *s);
int16_t gain_kp(const gain_seq_t *s);
int16_t gain_ki(const gain_seq_t *s);
void    gain_seq_next(gain_seq_t *s, int clean);        /* 판정을 알린다. 다음 후보로 가거나 끝낸다(머리 주석의 순서) */
int16_t gain_seq_miss(gain_seq_t *s);                   /* 후보가 재시도까지 판정되지 않았다. 한 칸 낮춘 후보면 탈락과 같이 끝내고 1,
                                                           아니면 0(t1 실패) */

#endif /* AGV_GAIN_H */
