/*
 * manual.h - 수동 조작 표 (옛 DA_Manual_Act 의 호기별 if / else if / else 사슬)와 프로파일 적재 조건.
 * 표 매크로(MANUAL_BEGIN·MAN·MAN_ELSE·PROFILE_LOAD_IF …)는 agv/seq_dsl.h. 설계와 예: docs/03_시퀀스_DSL.md 5.4절
 *
 * 한 줄 = 사슬의 가지 하나. 사슬은 MAN 줄 여럿과 끝의 MAN_ELSE 한 줄이다(C 의 if / else if / else 하나).
 *  - 위에서부터 호기·모드·조건이 맞는 첫 MAN 줄의 출력만 쓴다. 맞는 줄이 없으면 MAN_ELSE 줄의 출력을 쓴다.
 *  - 쓰지 않은 출력은 그대로 둔다(암묵적으로 끄지 않는다).
 */
#ifndef AGV_MANUAL_H
#define AGV_MANUAL_H

#include "agv/seq.h"

/* 줄 하나 10워드(C28x) */
typedef struct {
    uint16_t id;          /* 호기 범위: 하위 8비트 처음, 상위 8비트 끝 */
    uint16_t kind;        /* 하위 4비트 MAN_K_*, MAN_F_ELSE, 상위 8비트 키(0 = 키 조건 없음) */
    uint32_t on, off;     /* 입력 조건 (seq_mask_true) */
    uint32_t set, clr;    /* 출력: clr 끄고 set 켬. MAN_K_FB_SW·MAN_K_OBS_B 는 set 이 입력 ID */
} man_row_t;

enum { MAN_K_IDLE = 0, MAN_K_JOG, MAN_K_FB_SW, MAN_K_OBS_B };
#define MAN_F_ELSE  0x10u

/* 표와 프로파일 표를 넘긴다(적재 조건은 프로파일 표의 PROFILE_LOAD_IF 칸) */
void    manual_bind(const man_row_t *rows, uint16_t n, const profile_row_t *prof, uint16_t nprof);
void    manual_tick(void);      /* 10 ms. site_manual 이 부른다 */
int16_t manual_load(void);      /* seq_site_t.load_fn 에 넣는다. 1 적재, 0 공차, -1 이 프로파일은 적재 조건 없음 */

#endif /* AGV_MANUAL_H */
