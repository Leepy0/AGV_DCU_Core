/* tune.h - 튜닝 모드 */
#ifndef AGV_TUNE_H
#define AGV_TUNE_H
#include "agv/types.h"
#include "agv/guide.h"

enum { TUNE_OFF = 0, TUNE_WAIT, TUNE_GO_1, TUNE_PAUSE, TUNE_TURN, TUNE_RESTART, TUNE_GO_2, TUNE_DONE };

/* 결과 쪽 */
enum { TUNE_PAGE_WHEEL_BASE = 0,   /* 계산한 휠 간격 (24번) */
       TUNE_PAGE_DISTANCE,         /* 1 m 지점의 실측 오차 mm (+ 더 감) → 휠 지름 (22번) */
       TUNE_PAGE_RETURN,           /* 출발점으로 돌아온 오차 mm. 기록만 한다 */
       TUNE_PAGE_GUIDE,            /* 'a' = k × 'h' 의 k × 100. 음수면 'h' 부호가 반대 */
       TUNE_PAGE_STEER_RMS,        /* 직진 중 'a' 편차 RMS × 10 */
       TUNE_PAGE_STEER_MAX,        /* 직진 중 'a' 편차 최대 */
       TUNE_PAGE_FLOOR,            /* 정지 때 학습한 바닥 속도 rpm */
       TUNE_PAGE_LED,              /* 직진 중 최대 점등 LED 수 */
       TUNE_PAGE_LR,               /* 우/좌 펄스 비 - 1, 천분율 */
       TUNE_PAGE_COUNT };
#define TUNE_NO_DATA (-32767)

void     tune_reset(void);
void     tune_arm(void);          /* 전원 투입 때 켠다. 끝내려면 전원을 다시 켠다 */
void     tune_load(void);         /* 부팅 때 저장한 가이드 부호·배율을 되살린다 */
int      tune_active(void);
int      tune_failed(void);       /* 1 = 회전 실패로 TUNE 로 돌아옴 */
uint16_t tune_state(void);
uint16_t tune_page(void);
int16_t  tune_result(void);       /* 계산한 휠 간격 값 */
int16_t  tune_value(uint16_t page);
void     tune_run(void);          /* 10 ms */
int      tune_key(uint16_t key);  /* 0 무시, 1 처리, 2 저장함 */

/* guide_poll 이 부른다 */
int      tune_want_h(void);
void     tune_on_a(int16_t status, int16_t pos);
void     tune_on_h(const guide_rx_t *rx);
#endif
