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

/* 파라미터 편집 화면 단계. 1~3 그룹 선택, 11~13 번호 선택·편집 */
enum { UI_GROUP_USE = 1, UI_GROUP_OTHER = 2, UI_GROUP_LAMP = 3 };
#define UI_PARA_LAMP_FIRST 91
#define UI_PARA_LAMP_LAST  93
#define UI_LAMP_EXT_COUNT  3

/* 자동 표시 항목 (P_DISPLAY_MODE) */
enum { DISP_SET_SPEED = 1, DISP_CUR_SPEED, DISP_TOTAL_TIME, DISP_TAG_COUNT, DISP_BAT_V, DISP_BAT_RATE,
       DISP_STATION, DISP_STOCK, DISP_EXT_IO, DISP_TAG_INFO };

void     ui_reset(void);
void     ui_isr(uint16_t key_input);    /* 2 ms: 키 스캔, 깜박임, 7세그 한 자리 */
void     ui_run(void);                  /* 10 ms: 키 처리, 램프, 부저 */
void     ui_digits(uint16_t *out);      /* 시험용: 지금 표시 색인 5자리 */
int16_t  ui_para_number(void);
uint16_t ui_key(void);                 /* 지금 눌린 키 값 */
int16_t  ui_lamp_ext(uint16_t i);
void     ui_lamp_ext_set(uint16_t i, int16_t value);
#endif
