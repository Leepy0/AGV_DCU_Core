/* tune.h - 튜닝 모드 */
#ifndef AGV_TUNE_H
#define AGV_TUNE_H
#include "agv/types.h"
#include "agv/guide.h"

enum { TUNE_OFF = 0, TUNE_WAIT, TUNE_GO_1, TUNE_TURN, TUNE_RESTART, TUNE_GO_2, TUNE_STOPPING, TUNE_DONE,
       TUNE_IDENT, TUNE_IDENT_DONE, TUNE_RPM, TUNE_RPM_DONE, TUNE_TOP, TUNE_TOP_DONE };

/* 대기 중 SHIFT 로 고르는 시험. TUNE → IDEN → RPM → TOP → TUNE */
enum { TUNE_MODE_TUNE = 0, TUNE_MODE_IDENT, TUNE_MODE_RPM, TUNE_MODE_TOP, TUNE_MODE_COUNT };
#define TUNE_RPM_PAGES 3          /* 최대 rpm·최고 속도 결과 쪽: 0 OK/F, 1 좌 rpm, 2 우 rpm */

/* 결과 쪽. UP·DOWN 으로 넘긴다 */
enum { TUNE_PAGE_WHEEL_BASE = 0,   /* WW 계산한 휠 간격 (24번). 수정 가능 */
       TUNE_PAGE_DIAMETER,         /* DI 교차선 간격으로 고친 휠 지름 (22번). 수정 가능 */
       TUNE_PAGE_GAP,              /* GA 선과 회전 중심 사이 간격 mm 입력. 저장값 = 입력 + OV → GD */
       TUNE_PAGE_OVERRUN,          /* OV 선을 본 뒤 멈출 때까지 미끄러진 거리 mm */
       TUNE_PAGE_GUIDE_DIST,       /* GD 지금 가이드센서 거리 (50번). 수정 가능 */
       TUNE_PAGE_GUIDE,            /* G  'a' = k × 'h' 의 k × 100. 음수면 'h' 부호가 반대 */
       TUNE_PAGE_STEER_RMS,        /* SE 직진 중 'a' 편차 RMS × 10 */
       TUNE_PAGE_STEER_MAX,        /* SP 직진 중 'a' 편차 최대 */
       TUNE_PAGE_FLOOR,            /* FL 정지 때 쓴 바닥 속도 rpm */
       TUNE_PAGE_LED,              /* LE 직진 중 일반 라인 최대 점등 LED 수 (교차선 제외) */
       TUNE_PAGE_LR,               /* LR 우/좌 펄스 비 - 1, 천분율 */
       TUNE_PAGE_COUNT };
#define TUNE_NO_DATA (-32767)

void     tune_reset(void);
void     tune_arm(void);          /* 전원 투입 때 켠다. 끝내려면 전원을 다시 켠다 */
void     tune_load(void);         /* 부팅 때 저장한 가이드 부호·배율을 되살린다 */
int      tune_active(void);
int      tune_failed(void);       /* 1 = 실패로 TUNE 로 돌아옴 */
int      tune_ident_mode(void);   /* 1 = START 가 식별 시험을 한다. 대기 중 SHIFT 로 바꾼다 */
int      tune_rpm_mode(void);     /* 1 = START 가 최대 rpm 시험(바퀴 띄움)을 한다 */
int      tune_top_mode(void);     /* 1 = START 가 바닥 최고 속도 시험을 한다 */
int      tune_editing(void);      /* 값을 고치는 중 */
int16_t  tune_edit_place(void);   /* 1 일, 2 십, 3 백의 자리 */
uint16_t tune_state(void);
uint16_t tune_page(void);
uint16_t tune_lines(void);        /* 이번 구간에서 본 교차선 수 */
uint32_t tune_line_at(uint16_t i); /* i 번째 교차선을 본 때의 주행 펄스 */
void     tune_cross_clear(void);  /* 교차선 수를 0 으로. 식별 시험이 선 찾기 전에 부른다 */
int16_t  tune_result(void);       /* 계산한 휠 간격 값 */
int16_t  tune_value(uint16_t page);
void     tune_run(void);          /* 10 ms */
int      tune_key(uint16_t key);  /* 0 무시, 1 처리, 2 저장함 */

/* guide_poll 이 부른다 */
int      tune_want_h(void);
void     tune_on_a(int16_t status, int16_t pos);
void     tune_on_h(const guide_rx_t *rx);
#endif
