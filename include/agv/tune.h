/* tune.h - 튜닝 모드. 전원을 켤 때 MODE + SET. 끝내려면 전원을 다시 켠다.
 * 메뉴는 파라미터처럼 대·중·소 단계다. 키:
 *  MODE 같은 단계 다음 칸(결과 쪽은 다음 쪽), SET 아래 단계로·수정 시작·확정, RESET 한 단계 위·수정 취소
 *  (결과 쪽에서는 채택 빼기. 추천은 처음부터 채택이고 저장 쪽 SV n 에서 SET 하면 쓰기가 끝날 때까지 BUSY 뒤 RBOT),
 *  START 고른 시험, UP·DOWN 결과 넘기기·값 증감, SHIFT 고치는 자리
 *  t1 제작   t1 아래 입력 5칸(지름 22번, GD 50번, 최고 속도 상한, 교차선 간격 L, ACC 상한 AU). START = 연속 시험 전체
 *  t2 점검   t2.1 빠른 점검, t2.2 바퀴 교체, t2.3 가이드 센서 교체, t2.4 모터·앰프 교체. 2.1·2.2·2.4 는 SET 으로 L 입력
 *  t3 개발   t3.1 IDEN, t3.2 RPM, t3.3 TOP, t3.4 ALL, t3.5 TEMP, t3.6 HS(SET 으로 속도 입력). 결과는 RAM 에만.
 *            시험 추가는 tune.c s_dev 표 한 줄
 * FND(tune_text): 켠 뒤 1 s(또는 첫 키까지) EEPROM 칩 크기 EE128·EE256(storage.h ee_probe),
 *  메뉴 't' + 번호(점으로 나눔) + 두 글자 이름, 입력 이름 + 값, 연속 시험 진행 'B'(t1)·'C'(t2) + 단계 + 회 2자리
 *  + 순서, 실패 'F' + 코드, 결과 쪽, 저장 뒤 RBOT */
#ifndef AGV_TUNE_H
#define AGV_TUNE_H
#include "agv/types.h"
#include "agv/guide.h"

/* t3 시험의 진행 상태 다음 값이 끝 상태다 */
enum { TUNE_OFF = 0, TUNE_WAIT, TUNE_BLD, TUNE_BLD_DONE, TUNE_SAVED,
       TUNE_IDENT, TUNE_IDENT_DONE, TUNE_RPM, TUNE_RPM_DONE, TUNE_TOP, TUNE_TOP_DONE, TUNE_ALL, TUNE_ALL_DONE,
       TUNE_TEMP, TUNE_TEMP_DONE, TUNE_HS, TUNE_HS_DONE };

/* t3.n */
enum { TUNE_DEV_IDENT = 1, TUNE_DEV_RPM, TUNE_DEV_TOP, TUNE_DEV_ALL, TUNE_DEV_TEMP, TUNE_DEV_HS };
#define TUNE_RPM_PAGES 3          /* 최고 속도·통합 결과 쪽: 0 OK/F, 1 좌 rpm, 2 우 rpm */
#define TUNE_RPMT_PAGES 4         /* 바닥 rpm 결과 쪽: 0 OK/F, 1 L 저속 최저 rpm, 2 B 출발 DAC, 3 A 앰프 추정 S·T */
#define TUNE_HS_PAGES 5           /* 고속 스윕 결과 쪽: 0 OK(+가장 좋은 후보)/F, 1~4 후보 평균 편차 RMS */

/* EEPROM (128 워드 칩 기준). 0~50 파라미터·표시, 51~109 KEEP(77~81 은 공차 파라미터), 115~118 체크섬 밖 파라미터, 120~124 OUT 큐와 겹치지 않는다.
 * 입력 L·ACC 상한은 확정 즉시, 기준값은 t1 결과 SAVE 때만 쓴다. 회전 보정비는 KEEP 54(storage.h), 기구 단계 끝에 바로 쓴다 */
#define TUNE_EE_TRACK  126        /* 교차선 간격 L mm. 0 = 지름 보정 안 함 */
#define TUNE_EE_CAP    127        /* 위 바이트 ACC 상한(ACC 단위). 0 = ACC 지금 값. 아래 바이트는 미사용(옛 최고 속도 상한, 지금은 DRS) */
#define TUNE_EE_CHECK  110        /* 기준값 검사 = TUNE_BASE_MARK ^ 기준값 6칸 합. 틀리면 기준값 없음 */
#define TUNE_BASE_MARK 0xB501u    /* 아래 바이트 = 판 */
enum { TUNE_BASE_TOP = 0,         /* 111 최고 rpm 좌우 평균 */
       TUNE_BASE_DEC,             /* 112 타력 감속도 rpm/s */
       TUNE_BASE_FLOOR,           /* 113 정지 바닥 속도 rpm */
       TUNE_BASE_LR,              /* 114 우/좌 펄스 비 - 1, 천분율 */
       TUNE_BASE_BAT,             /* 119 전압 dV (48번) */
       TUNE_BASE_AMP_SE,          /* 125 앰프('S'·'T'·0) << 8 | 직진 편차 RMS × 10 (0~255) */
       TUNE_BASE_N };
extern const uint16_t tune_base_addr[TUNE_BASE_N];
/* 점검 판정. 기준 대비 % (LR 은 지금 값) 가 넘으면 NG */
#define TUNE_NG_TOP_PCT   (-5)
#define TUNE_NG_DEC_PCT   (-15)
#define TUNE_NG_FLOOR_PCT 20
#define TUNE_NG_LR        10      /* |우/좌 - 1| 천분율 */
#define TUNE_NG_SE_PCT    50      /* 직진 편차 RMS */
#define TUNE_NO_DATA (-32767)

void     tune_reset(void);
void     tune_arm(void);          /* 전원 투입 때 켠다 */
void     tune_load(void);         /* 부팅: 가이드 부호·배율, 입력 L·상한을 되살린다 */
int      tune_active(void);
int      tune_ident_mode(void);   /* t3.1 을 골랐다 */
int      tune_rpm_mode(void);     /* t3.2 */
int      tune_top_mode(void);     /* t3.3 */
int      tune_all_mode(void);     /* t3.4 */
int      tune_temp_mode(void);    /* t3.5 */
int      tune_hs_mode(void);      /* t3.6 */
int16_t  tune_hs_mpm(void);       /* t3.6 입력 속도. 0 = 1번 */
int      tune_editing(void);      /* 값을 고치는 중 */
uint16_t tune_state(void);
uint16_t tune_page(void);
uint16_t tune_menu(void);         /* 지금 칸: 대 × 1000 + 중 × 10 + 소. 예 t2.1.1 = 2011, t3 = 3000, t1 15번째 칸 = 1150 */
uint16_t tune_lines(void);        /* 이번 구간에서 본 교차선 수 */
uint32_t tune_line_at(uint16_t i); /* i 번째 교차선을 본 때의 주행 펄스 */
void     tune_cross_clear(void);  /* 교차선 수를 0 으로. 식별 시험이 선 찾기 전에 부른다 */
int      tune_base_valid(void);
int16_t  tune_base(uint16_t i);   /* 기준값 칸. TUNE_BASE_* */
void     tune_run(void);          /* 10 ms */
int      tune_key(uint16_t key);  /* 0 무시, 1 처리, 2 저장함 */
uint16_t tune_text(char *c);      /* 메뉴·입력·연속 시험 표시 5글자('0'~'9', 'A'~'Z', '-', ' '). 반환 = 점 자리 비트 */
uint16_t tune_flick(void);        /* 깜박일 자리 비트. 0 = 없음 */
/* 결과 쪽 값 표시 5글자: 이름 두 글자 + 세 자리, wide 1 = 이름 첫 글자 + 네 자리. 넘치면 OVF·OVER. 없음(TUNE_NO_DATA)은 --- */
void     tune_value_text(char *c, char n0, char n1, int16_t v, int wide);

/* 제자리 회전 보정비 = 실제 회전에 든 펄스 ÷ 휠 간격(24번)으로 계산한 펄스. 잰 값 × 1000 을 KEEP_TURN_SCALE 에 둔다.
 * 이 범위 밖(빈 EEPROM 포함)이면 보정 없음(1). 일반·정렬 회전의 목표 펄스와 TN 표시에 건다(tagact_turn) */
#define TUNE_TURN_LOW  750
#define TUNE_TURN_UP   1300
float    tune_turn_scale(void);

/* ident.c 가 부른다 */
void     tune_turn_scale_set(int16_t x1000);   /* 바로 저장. 범위 밖이면 범위 끝으로 */
int      tune_guide_save(void);   /* 'h' 부호·배율 저장. 측정 부족이면 0 */
int16_t  tune_guide_k100(void);   /* 'a' = k × 'h' 의 k × 100. TUNE_NO_DATA = 측정 부족 */

/* guide_poll 이 부른다 */
int      tune_want_h(void);
void     tune_on_a(int16_t status, int16_t pos);
void     tune_on_h(const guide_rx_t *rx);
#endif
