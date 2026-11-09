/* ui.h - 키, 7세그, 램프·부저. 표시 한 자리와 키 스캔은 2 ms ISR, 나머지는 10 ms */
#ifndef AGV_UI_H
#define AGV_UI_H
#include "agv/types.h"

/* 키 값 */
#define KEY_MODE   0x01
#define KEY_SHIFT  0x02
#define KEY_UP     0x04
#define KEY_RESET  0x08
#define KEY_SET    0x10
#define KEY_DOWN   0x20

/* 파라미터 편집 화면 단계. 1~5 묶음 선택(MODE 다음 묶음), 11~15 번호 선택(UP·DOWN)·편집.
 * 묶음 표시 P1SPD 속도, P2RUN 주행·안전, P3CAR 차량·튜닝, P4COM 통신·표시(램프 포함), P5INF 정보·개발.
 * 5번 묶음 목록에서 SHIFT 2 s = 숨김 항목(4·43·38·39번)을 이번 전원 동안 보인다 */
enum { UI_GROUP_SPEED = 1, UI_GROUP_RUN, UI_GROUP_CAR, UI_GROUP_COMM, UI_GROUP_INFO, UI_GROUP_COUNT = UI_GROUP_INFO };
#define UI_PARA_LAMP_FIRST 91
#define UI_PARA_LAMP_LAST  93
#define UI_LAMP_EXT_COUNT  3

/* 자동 표시 항목 (P_DISPLAY_MODE) */
enum { DISP_SET_SPEED = 1, DISP_CUR_SPEED, DISP_TOTAL_TIME, DISP_TAG_COUNT, DISP_BAT_V, DISP_BAT_RATE,
       DISP_STATION, DISP_STOCK, DISP_EXT_IO, DISP_TAG_INFO };

void     ui_reset(void);
void     ui_isr(uint16_t key_input);    /* 2 ms: 키 스캔, 깜박임, 7세그 한 자리 */
void     ui_run(void);                  /* 10 ms: 키 처리, 램프, 부저 */
void     ui_iotest_wait(void);          /* IO 시험 진입 대기: TEST 표시 */
void     ui_iotest_enter(void);
void     ui_iotest_run(void);           /* 10 ms: IO 시험 키·표시·출력 */
void     ui_digits(uint16_t *out);      /* 시험용: 지금 표시 색인 5자리 */
uint16_t ui_param_label(uint16_t idx, char *c3);   /* 파라미터 화면 이름 3글자. 반환 = 값 자릿수(0 = ON/OFF) */
void     ui_param_text(uint16_t idx, int16_t v, char *c);   /* 파라미터 화면과 같은 5글자('0'~'9', 'A'~'Z', '-', ' ', '_') */
int16_t  ui_para_number(void);
uint16_t ui_key(void);                 /* 지금 눌린 키 값 */
int16_t  ui_lamp_ext(uint16_t i);
void     ui_lamp_ext_set(uint16_t i, int16_t value);
#endif
