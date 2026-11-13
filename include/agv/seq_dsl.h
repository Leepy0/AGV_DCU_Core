/*
 * seq_dsl.h - 사이트 표 작성 매크로. 시퀀스·ACT·태그 표를 C 파일에 한 줄 한 스텝으로 쓴다.
 * 설계와 예: docs/03_시퀀스_DSL.md
 *
 * 규칙
 *  - 시간 인자는 전부 1/100초. 300 = 3초.
 *  - ON·OFF·IN·L 같은 짧은 이름을 쓰므로 표 파일은 legacy_compat.h 와 함께 포함하지 않는다.
 *  - 컴파일 옵션 --c99 필요 (가변 인자 매크로).
 */
#ifndef AGV_SEQ_DSL_H
#define AGV_SEQ_DSL_H

#include "agv/seq.h"
#include "agv/acs.h"
#include "agv/tagact.h"
#include "agv/io.h"
#include "agv/param.h"
#include "agv/ui.h"

/* ---- 내부 도구 ---- */
#define DSL_CAT_(a, b)   a##b
#define DSL_CAT(a, b)    DSL_CAT_(a, b)
#define DSL_NARG_(a, b, c, d, e, f, g, h, i, j, k, l, m, n_, o, p, n, ...) n
#define DSL_NARG(...)    DSL_NARG_(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define DSL_UNPAREN(...) __VA_ARGS__

/* ID 가 0~31 또는 IO_NONE 이 아니면 컴파일 오류 (앰프 신호를 조건에 쓰는 경우 등) */
#define DSL_ID_OK(x)     (0 * sizeof(char[(((x) < 32) || ((x) == IO_NONE)) ? 1 : -1]))
#define DSL_BIT(x)       ((((x) == IO_NONE) ? 0UL : (1UL << ((x) & 31))) + DSL_ID_OK(x))

/* 논리 0 이 활성인 입력(기본 active-low 배선에서 핀 1 = 활성). 같은 핀을 차종마다 반대 극성으로 쓸 때
 * 사이트 io_map.h 가 이름 정의에 표시한다: #define SENH_1 IN_ACTIVE_HIGH( IN_EXT_1 ).
 * ON()·OFF() 만 받고 반대쪽 절반으로 보낸다. ANY_ON·ANY2_ON·출력·LOAD_SENSORS 에 쓰면 컴파일 오류다.
 * C 코드의 io_in( ) 에는 쓰지 않는다(논리 ID 를 쓴다) */
#define DSL_ACTIVE_HIGH  0x100
#define IN_ACTIVE_HIGH(x)  ((x) + DSL_ACTIVE_HIGH + DSL_ID_OK(x))
#define DSL_HI(x)        (((x) & DSL_ACTIVE_HIGH) != 0)
#define DSL_RAW(x)       ((x) & ~DSL_ACTIVE_HIGH)
#define DSL_LBIT(x)      (DSL_HI(x) ? 0UL : DSL_BIT(DSL_RAW(x)))   /* 논리 1 활성(보통) */
#define DSL_HBIT(x)      (DSL_HI(x) ? DSL_BIT(DSL_RAW(x)) : 0UL)   /* 논리 0 활성(IN_ACTIVE_HIGH) */

/* F 는 신호 하나를 비트로 바꾸는 매크로(DSL_BIT, DSL_LBIT, DSL_HBIT) */
#define DSL_MASK1(F, a)                 F(a)
#define DSL_MASK2(F, a, b)              (F(a) | F(b))
#define DSL_MASK3(F, a, b, c)           (DSL_MASK2(F, a, b) | F(c))
#define DSL_MASK4(F, a, b, c, d)        (DSL_MASK3(F, a, b, c) | F(d))
#define DSL_MASK5(F, a, b, c, d, e)     (DSL_MASK4(F, a, b, c, d) | F(e))
#define DSL_MASK6(F, a, b, c, d, e, f)  (DSL_MASK5(F, a, b, c, d, e) | F(f))
#define DSL_MASK7(F, a, b, c, d, e, f, g)     (DSL_MASK6(F, a, b, c, d, e, f) | F(g))
#define DSL_MASK8(F, a, b, c, d, e, f, g, h)  (DSL_MASK7(F, a, b, c, d, e, f, g) | F(h))
#define DSL_MASK9(F, a, b, c, d, e, f, g, h, i)                    (DSL_MASK8(F, a, b, c, d, e, f, g, h) | F(i))
#define DSL_MASK10(F, a, b, c, d, e, f, g, h, i, j)                (DSL_MASK9(F, a, b, c, d, e, f, g, h, i) | F(j))
#define DSL_MASK11(F, a, b, c, d, e, f, g, h, i, j, k)             (DSL_MASK10(F, a, b, c, d, e, f, g, h, i, j) | F(k))
#define DSL_MASK12(F, a, b, c, d, e, f, g, h, i, j, k, l)          (DSL_MASK11(F, a, b, c, d, e, f, g, h, i, j, k) | F(l))
#define DSL_MASK13(F, a, b, c, d, e, f, g, h, i, j, k, l, m)       (DSL_MASK12(F, a, b, c, d, e, f, g, h, i, j, k, l) | F(m))
#define DSL_MASK14(F, a, b, c, d, e, f, g, h, i, j, k, l, m, n)    (DSL_MASK13(F, a, b, c, d, e, f, g, h, i, j, k, l, m) | F(n))
#define DSL_MASK15(F, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o) (DSL_MASK14(F, a, b, c, d, e, f, g, h, i, j, k, l, m, n) | F(o))
#define DSL_MASK16(F, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p) (DSL_MASK15(F, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o) | F(p))
#define DSL_FMASK(F, ...)  DSL_CAT(DSL_MASK, DSL_NARG(__VA_ARGS__))(F, __VA_ARGS__)
#define DSL_MASK(...)    DSL_FMASK(DSL_BIT, __VA_ARGS__)

/* 조건은 64비트 상수 하나: 하위 32비트 must_on, 상위 32비트 must_off */
typedef unsigned long long dsl_cond_t;
#define DSL_COND_ON(c)   ((uint32_t)(c))
#define DSL_COND_OFF(c)  ((uint32_t)((c) >> 32))
/* 양쪽 절반에 같이 선 비트 = "그중 하나라도 ON". ANY_ON 만 그렇게 쓴다(아래 IN 이 겹침을 막는다) */

#define DSL_STEP(kind, target, a0, a1, to, on, off, out)                                     \
    { (uint16_t)(kind), (uint16_t)(target), (uint16_t)(a0), (uint16_t)(a1), (uint16_t)(to), 0u, \
      (uint32_t)(on), (uint32_t)(off), (uint32_t)(out) },
#define DSL_STEP_C(kind, target, a0, a1, to, c, out)                                         \
    DSL_STEP(kind, target, a0, a1, to, DSL_COND_ON(c), DSL_COND_OFF(c), out)
/* 확장 조건 스텝. x 는 X_* 세 값(코드, a, b)으로 펼쳐져 xk, xa, xb 로 들어온다. a, b 는 arg0, arg1 */
#define DSL_STEP_X(kind, target, to, xk, xa, xb, c, any2)                                     \
    { (uint16_t)(kind), (uint16_t)(target), (uint16_t)(xa), (uint16_t)(xb), (uint16_t)(to), (uint16_t)(xk), \
      DSL_COND_ON(c), DSL_COND_OFF(c), (uint32_t)(any2) },

/* ---- 조건 ----
 * ON( a ) = a 가 활성(자재 감지, 신호 들어옴), OFF( a ) = 비활성. 핀 레벨은 io_map.h 의 이름이 정한다.
 * 한 조건 안에서 같은 핀을 반대 극성으로 겹쳐 쓰면(ON(SENH_1) 과 ON(SEN_1), ANY_ON(SEN_1, …) 과 ON(SENH_1) 등) 양쪽 절반에 같은
 * 비트가 서서 ANY_ON 으로 읽히거나 한쪽이 묻힌다. 컴파일 오류로 막는다. 같은 핀을 같은 뜻(같은 절반)으로 두 번 쓰는 것은 된다 */
#define DSL_NOX(m)       (0 * sizeof(char[((m) == 0) ? 1 : -1]))
#define DSL_ONOFF(lo, hi, ...)  ((((dsl_cond_t)DSL_FMASK(lo, __VA_ARGS__)) | (((dsl_cond_t)DSL_FMASK(hi, __VA_ARGS__)) << 32)) \
                                 + DSL_NOX(DSL_FMASK(DSL_LBIT, __VA_ARGS__) & DSL_FMASK(DSL_HBIT, __VA_ARGS__)))
#define ON(...)          DSL_ONOFF(DSL_LBIT, DSL_HBIT, __VA_ARGS__)
#define OFF(...)         DSL_ONOFF(DSL_HBIT, DSL_LBIT, __VA_ARGS__)
/* 나열한 신호 중 하나라도 켜짐. 같은 마스크를 양쪽 절반에 넣는다. 예: IN( ANY_ON(S1, S2), OFF(S5) ) */
#define ANY_ON(...)      (((dsl_cond_t)DSL_MASK(__VA_ARGS__)) | (((dsl_cond_t)DSL_MASK(__VA_ARGS__)) << 32))
/* IN 인자마다 앞 인자들의 합 p 와 견준다: p 의 한 절반과 이 인자의 다른 절반이 겹치면 오류(ANY_ON 은 두 절반이라 어느 겹침이든 오류) */
#define DSL_XK(p, c)     ((dsl_cond_t)(p) & (((dsl_cond_t)(c) >> 32) | ((dsl_cond_t)(c) << 32)))
#define DSL_IN1(a)       (a)
#define DSL_IN2(a, b)    (((a) | (b)) + DSL_NOX(DSL_XK(a, b)))
#define DSL_IN3(a, b, c)             (((a) | (b) | (c)) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c)))
#define DSL_IN4(a, b, c, d)          (((a) | (b) | (c) | (d)) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) | DSL_XK((a) | (b) | (c), d)))
#define DSL_IN5(a, b, c, d, e)       (((a) | (b) | (c) | (d) | (e)) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) \
                                      | DSL_XK((a) | (b) | (c), d) | DSL_XK((a) | (b) | (c) | (d), e)))
#define DSL_IN6(a, b, c, d, e, f)    (((a) | (b) | (c) | (d) | (e) | (f)) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) \
                                      | DSL_XK((a) | (b) | (c), d) | DSL_XK((a) | (b) | (c) | (d), e) | DSL_XK((a) | (b) | (c) | (d) | (e), f)))
#define IN(...)          DSL_CAT(DSL_IN, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define NO_IN            ((dsl_cond_t)0)
/* 두 번째 "하나라도 ON" 묶음: (a||b) && (c||d). WHEN_ANY2·WAIT_ANY2 의 둘째 인자 */
#define ANY2_ON(...)     DSL_MASK(__VA_ARGS__)

/* ---- 확장 조건 (입력이 아닌 조건). X_* 는 세 값 "코드, a, b" 로 펼쳐진다 ----
 * 시퀀스 WHEN_IF·WAIT_IF 와 태그 규칙 IF_X 가 같이 쓴다 */
#define DSL_SLOT(k)      (((uint16_t)((k) & 0xFF) << 8) + (0 * sizeof(char[((k) < 256) ? 1 : -1])))   /* 16비트 int: 부호 없이 민다 */
#define DSL_NZ(n)        (0 * sizeof(char[((n) != 0) ? 1 : -1]))
#define X_ALWAYS                     XC_NONE, 0, 0
#define X_FORWARD                    XC_FORWARD, 0, 0
#define X_BACKWARD                   XC_BACKWARD, 0, 0
#define X_FORBACK_MODE               XC_FORBACK_MODE, 0, 0
#define X_LOW_BATTERY                XC_LOW_BATTERY, 0, 0
#define X_CROSS_IN                   XC_CROSS, 0, 0                      /* IN 대기 또는 유지 중 */
#define X_CROSS_OUT                  (XC_CROSS | DSL_SLOT(1)), 0, 0      /* OUT 확인 대기 또는 유지 중 (OUT 태그에서는 서지 않는다) */
#define X_CROSS_IN_ON                (XC_CROSS | DSL_SLOT(2)), 0, 0      /* IN 유지(허가 대기 정지) 중 */
#define X_CROSS_OUT_ON               (XC_CROSS | DSL_SLOT(3)), 0, 0
#define X_ERR_EQ(n)                  XC_ERR_EQ, (n), 0
#define X_PARAM_EQ(p, v)             (XC_PARAM_EQ | DSL_SLOT(p)), (v), 0
#define X_PARAM_GE(p, v)             (XC_PARAM_GE | DSL_SLOT(p)), (v), 0
#define X_PARAM_LE(p, v)             (XC_PARAM_LE | DSL_SLOT(p)), (v), 0
#define X_ID(lo, hi)                 XC_ID, (lo), (hi)
#define X_KEY(k)                     XC_KEY, KEY_##k, 0
#define X_STATION(k, lo, hi)         (XC_STATION | DSL_SLOT(k)), (lo), (hi)
#define X_STATION_RUNNING(k, lo, hi) (XC_STATION_RUNNING | DSL_SLOT(k)), (lo), (hi)
#define X_STATION_MOD(k, n, m)       (XC_STATION_MOD | DSL_SLOT(k)), ((n) + DSL_NZ(n)), (m)
#define X_STATION_DIV(k, n, m)       (XC_STATION_DIV | DSL_SLOT(k)), ((n) + DSL_NZ(n)), (m)
#define X_TAG_MINUS_EQ_STATION(k, n) (XC_TAG_MINUS_EQ_ST | DSL_SLOT(k)), (n), 0   /* 태그 표 전용 */
#define X_TAG_EQ_STATION(k)          (XC_TAG_EQ_ST | DSL_SLOT(k)), 0, 0          /* 태그 표 전용 */
#define X_NOT(x)                     DSL_XNOT_(x)
#define DSL_XNOT_(k, a, b)           ((k) | XC_NOT), a, b

/* 타임아웃 시간을 스텝 진입부터가 아니라 조건이 거짓인 틱만 누적해서 잰다 (레거시 else { cnt_2++ }) */
#define UNMATCHED(t)     ((t) | SEQ_TO_UNMATCHED)

/* ---- 표 틀 ---- */
#define SEQ_LABELS(...)  enum { SEQ_LABEL_ZERO_ = SEQ_NO_LABEL, __VA_ARGS__ };
#define SEQ_BEGIN(name)  static const seq_step_t name##_steps[] = {
#define SEQ_END          };
#define ACT_BEGIN(name)  SEQ_BEGIN(name)
#define ACT_END          SEQ_END

/* 시퀀스 대응표: 프로파일 + 요청 ID → 표 */
#define SEQ_MAP_BEGIN(name)      const seq_def_t name[] = {
#define SEQ_MAP(profile, id, seq)  { (uint16_t)(profile), (uint16_t)(id), seq##_steps, \
                                     (uint16_t)(sizeof(seq##_steps) / sizeof(seq_step_t)) },
#define SEQ_MAP_END              };
#define ACT_MAP_BEGIN(name)      SEQ_MAP_BEGIN(name)
#define ACT_MAP(profile, id, act)  SEQ_MAP(profile, id, act)
#define ACT_MAP_END              SEQ_MAP_END

#define PROFILE_TABLE_BEGIN(name)  const profile_row_t name[] = {
#define PROFILE(profile, type, from, to)  { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to), 0UL },
/* 적재 센서가 있는 프로파일. mask = LOAD_SENSORS(SEN_1, ...) 하나라도 켜지면 적재 */
#define PROFILE_LOAD(profile, type, from, to, mask)  { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to), (uint32_t)(mask) },
#define LOAD_SENSORS(...)          DSL_MASK(__VA_ARGS__)
#define PROFILE_TABLE_END          };

/* ---- 라벨·분기 ---- */
#define L(label)         DSL_STEP(SK_LABEL, label, 0, 0, 0, 0, 0, 0)
#define GOTO(label)      DSL_STEP(SK_GOTO,  label, 0, 0, 0, 0, 0, 0)

/* SELECT_BEGIN·SELECT_END 는 읽기 위한 표시이며 스텝을 만들지 않는다. 가지 수 제한 없음 */
#define SELECT_BEGIN
#define WHEN(c, label)               DSL_STEP_C(SK_WHEN, label, 0, 0, 0, c, 0)
#define WHEN_STATE(id, val, label)   DSL_STEP(SK_WHEN_STATE, label, id, val, 0, 0, 0, 0)
#define WHEN_IF(x, label)            DSL_STEP_X(SK_WHEN, label, 0, x, NO_IN, 0)
#define WHEN_IF_IN(x, c, label)      DSL_STEP_X(SK_WHEN, label, 0, x, c, 0)
#define WHEN_ANY2(c, any2, label)    DSL_STEP_C(SK_WHEN, label, 0, 0, 0, c, any2)
/* 호기별 분기. 프로파일을 나누지 않고 한 표에서 호기 번호로 가른다 */
#define WHEN_ID(lo, hi, label)       WHEN_IF(X_ID(lo, hi), label)
#define OTHERWISE(label)             DSL_STEP(SK_WHEN, label, 0, 0, 0, 0, 0, 0)
#define SELECT_END
#define SELECT_END_TO(t, label)      DSL_STEP(SK_WHEN_TIMEOUT, label, 0, 0, t, 0, 0, 0)

/* ---- 출력 ---- */
#define OUT_ON(...)      DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, DSL_MASK(__VA_ARGS__), 0, 0)
#define OUT_OFF(...)     DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, 0, DSL_MASK(__VA_ARGS__), 0)
/* 켜는 시간, 끄는 시간, 횟수 */
#define OUT_PULSE(o, on_t, off_t, n) \
    DSL_STEP(SK_OUT_PULSE, SEQ_NO_LABEL, on_t, off_t, n, 0, 0, DSL_BIT(o))
/* 접점 통신: 출력을 period 간격으로 점멸하다가 조건이 참이면 끄고 다음 스텝 */
#define OUT_FLICKER_UNTIL_IN(o, period, c) \
    DSL_STEP_C(SK_OUT_FLICKER, SEQ_NO_LABEL, period, 0, 0, c, DSL_BIT(o))
#define OUT_FLICKER_UNTIL_IN_TO(o, period, c, t, label) \
    DSL_STEP_C(SK_OUT_FLICKER, label, period, 0, t, c, DSL_BIT(o))
/* 출력 켬, 응답 켜짐 대기, 출력 끔, 응답 꺼짐 대기 */
#define HANDSHAKE(o, in_ack, t, label) \
    DSL_STEP(SK_HANDSHAKE, label, 0, 0, t, DSL_BIT(in_ack), 0, DSL_BIT(o))

/* ---- 벨트: 사이트 io_map.h 의 BELTn_CW, BELTn_CCW 출력 마스크를 쓴다 ----
 * 방향별로 출력 개수가 달라도 된다. 없는 방향은 마스크 0 이며, 켜면 컴파일 오류 */
#define OUTS(...)        DSL_MASK(__VA_ARGS__)
#define NO_OUT           0UL
#define DSL_BELT_OK(m)   (0 * sizeof(char[(m) ? 1 : -1]))
/* BELT_ON( n, 방향 ): 켜고 다음 스텝. BELT_ON( n, 방향, UNTIL( 조건 ) ): 켜고 그 벨트의 정지 조건을 걸고 바로 다음 스텝.
 * 코어가 시퀀스가 살아 있는 동안 매 틱 조건을 보고 참이면 그 벨트 출력 둘을 끄고 조건을 지운다(켤 때 이미 참이면 그 틱에 끈다).
 * 조건을 지우는 때: 벨트를 껐을 때, 그 벨트 출력을 바꾸는 OUT·BELT 스텝(BELT_OFF·BELT_ON 등), 시퀀스 끝·중단·초기화(출력은 그대로).
 * 시퀀스 전용(ACT 금지), 동시에 SEQ_BELTS(3) 개. 방향은 첫 매크로에서 붙인다(사이트 CW·CCW 매크로가 펼쳐지지 않게) */
#define BELT_ON(n, ...)  DSL_BELTON_(BELT##n, DSL_BELTD_##__VA_ARGS__)
#define DSL_BELTD_CW     _CW
#define DSL_BELTD_CCW    _CCW
/* 벨트 마스크(BELTn_*)가 펼쳐질 때 DSL_CAT·DSL_FMASK 를 쓰므로 고르는 데는 따로 붙이는 매크로를 쓴다(중첩 펼침 막힘 방지) */
#define DSL_BPASTE_(a, b)    a##b
#define DSL_BPASTE(a, b)     DSL_BPASTE_(a, b)
#define DSL_BELTON_(b, ...)  DSL_BPASTE(DSL_BELTON, DSL_NARG(__VA_ARGS__))(b, __VA_ARGS__)
#define DSL_BELTON1(b, d)                                                           \
    DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0,                                         \
             b##d + DSL_BELT_OK(b##d), (b##_CW | b##_CCW) & ~(uint32_t)(b##d), 0)
#define DSL_BELTON3(b, d, k, c)                                                     \
    DSL_STEP(SK_BELT_UNTIL, SEQ_NO_LABEL, (uint16_t)DSL_COND_OFF(c), (uint16_t)(DSL_COND_OFF(c) >> 16), DSL_UNTIL_OK_##k, \
             b##d + DSL_BELT_OK(b##d), (b##_CW | b##_CCW) & ~(uint32_t)(b##d), DSL_COND_ON(c))
/* 셋째 칸은 UNTIL( 조건 ) 만 받는다(다른 것은 컴파일 오류) */
#define UNTIL(c)         DSL_UNTIL_MARK, (c)
#define DSL_UNTIL_OK_DSL_UNTIL_MARK  0
#define BELT_OFF(n)      DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, 0, BELT##n##_CW | BELT##n##_CCW, 0)
/* WAIT_BELTS( 1, 2, 3 ): 적은 벨트마다 출력 둘(CW·CCW)이 모두 꺼질 때까지 대기(입력이 아니라 출력을 본다. WAIT_OUT 과 같은 스텝) */
#define DSL_BELTM(n)     (BELT##n##_CW | BELT##n##_CCW)
#define DSL_WB1(a)       DSL_BELTM(a)
#define DSL_WB2(a, b)    (DSL_BELTM(a) | DSL_BELTM(b))
#define DSL_WB3(a, b, c) (DSL_BELTM(a) | DSL_BELTM(b) | DSL_BELTM(c))
#define WAIT_BELTS(...)  DSL_STEP(SK_WAIT_OUT, SEQ_NO_LABEL, 0, 0, 0, 0, DSL_BPASTE(DSL_WB, DSL_NARG(__VA_ARGS__))(__VA_ARGS__), 0)

/* ---- 대기 ---- */
#define WAIT(c)                     DSL_STEP_C(SK_WAIT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
#define WAIT_TO(c, t, label)        DSL_STEP_C(SK_WAIT, label, 0, 0, t, c, 0)
#define WAIT_STABLE(c, s)           DSL_STEP_C(SK_WAIT_STABLE, SEQ_NO_LABEL, s, 0, 0, c, 0)
#define WAIT_STABLE_TO(c, s, t, label)  DSL_STEP_C(SK_WAIT_STABLE, label, s, 0, t, c, 0)
#define WAIT_OUT(c)                 DSL_STEP_C(SK_WAIT_OUT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
#define DELAY(t)                    DSL_STEP(SK_DELAY, SEQ_NO_LABEL, t, 0, 0, 0, 0, 0)
#define WAIT_IF(x)                  DSL_STEP_X(SK_WAIT, SEQ_NO_LABEL, 0, x, NO_IN, 0)
#define WAIT_IF_TO(x, t, label)     DSL_STEP_X(SK_WAIT, label, t, x, NO_IN, 0)
#define WAIT_IF_IN(x, c)            DSL_STEP_X(SK_WAIT, SEQ_NO_LABEL, 0, x, c, 0)
#define WAIT_IF_IN_TO(x, c, t, label)    DSL_STEP_X(SK_WAIT, label, t, x, c, 0)
#define WAIT_ANY2(c, any2)               DSL_STEP_C(SK_WAIT, SEQ_NO_LABEL, 0, 0, 0, c, any2)
#define WAIT_ANY2_TO(c, any2, t, label)  DSL_STEP_C(SK_WAIT, label, 0, 0, t, c, any2)
/* 파라미터 시간: 값 x 단위. 단위 TICK(10 ms) DSEC(0.1초) SEC MIN. 값은 스캔마다 다시 읽는다 */
#define DSL_UNIT_TICK               1
#define DSL_UNIT_DSEC               10
#define DSL_UNIT_SEC                100
#define DSL_UNIT_MIN                6000
#define DSL_PARAM_OK(p)             (0 * sizeof(char[(((p) > 0) && ((p) < P_COUNT)) ? 1 : -1]))
#define DELAY_PARAM(p, unit)        DSL_STEP(SK_DELAY_PARAM, SEQ_NO_LABEL, (p) + DSL_PARAM_OK(p), DSL_UNIT_##unit, 0, 0, 0, 0)
#define WAIT_TO_PARAM(c, p, unit, label) \
    DSL_STEP_C(SK_WAIT_PARAM, label, (p) + DSL_PARAM_OK(p), DSL_UNIT_##unit, 0, c, 0)

/* ---- ACS ---- */
/* 보고 종류: 레거시 DA_ACS_<이름> 과 같은 이름. 값은 프레임 Info 글자 */
#define DSL_INFO_LOADING    'L'
#define DSL_INFO_UNLOADING  'U'
#define DSL_INFO_NEXT       'N'
#define DSL_INFO_ARRIVE     'A'
#define DSL_INFO_FINISH     'F'
#define DSL_INFO_PROCESSING 'P'
#define DSL_INFO_STANDBY    'B'
#define DSL_INFO_CALL       'C'
/* 번호 자리의 스테이션 큐 번호: ACS_SEND( NEXT, STATION(1) ) = 레거시 DA_ACS_Next( ACS_Station_Call_Num[1] ). k 0~23.
 * 상수식이라 정적 표에 쓴다(ACS( 종류, STATION(k) ) 의 출처 자리에서는 이름으로 붙는다) */
#define DSL_STATION_MARK       0x20000L
#define STATION(k)             (DSL_STATION_MARK + (k) + (0 * sizeof(char[(((k) >= 0) && ((k) < ACS_STATION_MAX)) ? 1 : -1])))
#define DSL_IS_STATION(n)      ((n) >= DSL_STATION_MARK)
#define DSL_SEND_OK(n)         (0 * sizeof(char[(((n) >= 0) && (((n) < 0x10000L) || DSL_IS_STATION(n))) ? 1 : -1]))
#define DSL_SEND_SRC(n)        (DSL_IS_STATION(n) ? ((((n) & 0xFF) == 0) ? SEQ_NUM_STATION0 : SEQ_NUM_STATION((n) & 0xFF)) : SEQ_NUM_CONST)
#define ACS(info, src)         DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_##src, 0, 0, 0, 0)
/* 번호 상수 0~65535 또는 STATION(k). STATION(k) 는 ACS( 종류, STATION(k) ) 와 같은 스텝(k 0 은 STATION0) */
#define ACS_SEND(info, num)    DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, DSL_SEND_SRC(num) + DSL_SEND_OK(num), \
                                        DSL_IS_STATION(num) ? 0 : (num), 0, 0, 0)
#define ACS_WAIT_ACK(t, label) DSL_STEP(SK_ACS_WAIT_ACK, label, 0, 0, t, 0, 0, 0)
#define ACS_STATION(idx, val, cnt)  DSL_STEP(SK_ACS_STATION, SEQ_NO_LABEL, idx, val, cnt, 0, 0, 0)
#define STATION_RESET          DSL_STEP(SK_STATION_RESET, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)

/* ---- 주행 관련 (ACT 에서 금지, seq_bind 가 부팅 때 거부) ----
 * 방식 옵션은 인자 맨 뒤에 적는다. 안 적으면 일반 동작이다. 정해 두지 않은 옵션·조합은 컴파일 오류다.
 * 태그 표(tag_dsl.h)와 뜻이 같은 동작은 같은 이름이다(AGV_SPEED·AGV_SPEED_MMIN·AGV_FORBACK·AGV_TURN) */
#define AGV_SPEED(level)       DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_##level, 0, 0, 0, 0, 0)
/* m/min 4~255. 0~3 은 단계 번호와 겹쳐 받지 않는다(단계는 AGV_SPEED) */
#define DSL_MMIN_OK(v)         (0 * sizeof(char[(((v) >= 4) && ((v) <= 255)) ? 1 : -1]))
#define AGV_SPEED_MMIN(v)      DSL_STEP(SK_SPEED, SEQ_NO_LABEL, (v) + DSL_MMIN_OK(v), 0, 0, 0, 0, 0)
#define AGV_FORBACK(dir)       DSL_STEP(SK_FORBACK, SEQ_NO_LABEL, DIR_##dir, 0, 0, 0, 0, 0)
/* AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] ). 옵션을 쓰려면 속도 칸(0 = 자동)을 적는다.
 *   옵션 없음: 회전하고 끝날 때까지 기다린다. NOWAIT: 회전을 걸고 다음 스캔에 다음 스텝(회전 중에 뒤 스텝이 돈다).
 *   ALIGN: 선 자리에서 정렬 회전(목표각 근처에서 주행로를 찾아 맞춤)을 하고 정착·보정까지 끝나 서면 다음 스텝.
 *     정렬이 오류(못 찾음 258 등)로 끝나면 다음 스텝으로 가지 않고 시퀀스가 오류 규칙대로 끝난다. NOWAIT 와 함께 못 쓴다.
 *   OBS: 회전 중 장애물 감지(셋째 칸 b8 = 1. 주행 중 장애물 센서 사용 OBS_ON/OBS_OFF 와 다른 것). NOWAIT 와 함께 쓸 수 있다.
 *   TAPE 는 시퀀스에서 컴파일 오류(시퀀스 중 교차선 회전은 하지 않는다).
 * 속도 m/min 0~31, 0 이나 생략 = 자동(TNS·ETN). 지정하면 적재·공차와 상관없이 그 회전 한 번만 그 속도(회전 최저 속도~TNS 한계로
 * 묶는다. 태그 회전과 같다, tagact_turn_next).
 * 셋째 칸: 회전 뒤 방향 b0~3 | 자동 출발 b4~7 | 장애물 b8 | 속도 b11~15(태그 둘째 칸과 같은 배치) */
#define DSL_SOPT_K_NOWAIT      1
#define DSL_SOPT_K_ALIGN       2
#define DSL_SOPT_K_OBS         0
#define DSL_SOPT_O_NOWAIT      0
#define DSL_SOPT_O_ALIGN       0
#define DSL_SOPT_O_OBS         1
#define DSL_SKIND(k)           (((k) == 0) ? SK_TURN : (((k) == 1) ? SK_TURN_NOWAIT : SK_TURN_ALIGN))
/* 회전 뒤 방향·자동 출발은 0~15(넘치면 옆 칸으로 샌다) */
#define DSL_STURN_OK(fb, au, v, k1, k2, o) \
    (0 * sizeof(char[(((v) >= 0) && ((v) < 32) && (((k1) == 0) || ((k2) == 0)) && ((o) <= 1) \
                      && ((fb) >= 0) && ((fb) < 16) && ((au) >= 0) && ((au) < 16)) ? 1 : -1]))
#define DSL_STURN_W(d, deg, fb, au, v, k1, k2, o) \
    DSL_STEP(DSL_SKIND((k1) + (k2)), SEQ_NO_LABEL, d, deg, \
             ((fb) | ((au) << 4) | ((o) << 8) | ((uint16_t)(v) << 11)) + DSL_STURN_OK(fb, au, v, k1, k2, o), 0, 0, 0)
#define DSL_STURN3(d, deg, fb, au)             DSL_STURN_W(d, deg, fb, au, 0, 0, 0, 0)
#define DSL_STURN4(d, deg, fb, au, v)          DSL_STURN_W(d, deg, fb, au, v, 0, 0, 0)
#define DSL_STURN5(d, deg, fb, au, v, a)       DSL_STURN_W(d, deg, fb, au, v, DSL_SOPT_K_##a, 0, DSL_SOPT_O_##a)
#define DSL_STURN6(d, deg, fb, au, v, a, b)    DSL_STURN_W(d, deg, fb, au, v, DSL_SOPT_K_##a, DSL_SOPT_K_##b, \
                                                           DSL_SOPT_O_##a + DSL_SOPT_O_##b)
/* 방향은 이 매크로에서 바로 붙인다(사이트 legacy_const.h 의 CW·CCW 매크로가 펼쳐지지 않게) */
#define AGV_TURN(dir, ...)     DSL_CAT(DSL_STURN, DSL_NARG(__VA_ARGS__))(TURN_##dir, __VA_ARGS__)
/* 표의 거리는 모두 cm 다(태그 AGV_STOP( cm )·SEQ_START( cm, x ), 파라미터 SL 과 같다). 스텝에는 mm 로 넣는다(× 10).
 * AGV_MOVE( cm ): 그 자리에서 출발해 cm 앞(지금 전후진 방향)에 위치 정지하고, 설 때까지 기다린 뒤 다음 스텝. 1~3276 cm(밖이면 컴파일 오류).
 * 속도는 거리에 맞춰 SPN~LLOW 안에서 자동(tagact_move_mm). 이동 중 태그는 무시, 장애물은 일시정지 뒤 이어 감.
 * AGV_MOVE( 한계 cm, TAPE ): 그 자리에서 출발해 다음 교차선에서 서고(태그 교차선 정지와 같은 인정·서는 방식·찾는 속도),
 * 설 때까지 기다린 뒤 다음 스텝. 출발 때 선 위면 그 선은 건너뛴다. 한계 1~3276 cm 안에 못 찾으면 서고 오류 258, 시퀀스 끝.
 * AGV_MOVE( 한계 cm, TAPE_GD ): 같고, 교차선을 본 뒤 GD 더 가서 회전 중심이 교차선 위에 서게 한다(교차선 회전 앞 정지와 같은 루틴) */
#define DSL_CM_OK(cm)          (0 * sizeof(char[(((cm) >= 1) && ((cm) <= 3276)) ? 1 : -1]))
#define DSL_CM2MM(cm)          ((cm) * 10 + DSL_CM_OK(cm))
#define DSL_MOVE1(cm)          DSL_STEP(SK_MOVE_MM, SEQ_NO_LABEL, DSL_CM2MM(cm), 0, 0, 0, 0, 0)
#define DSL_MOVE2(cm, opt)     DSL_MOVE_##opt(cm)
#define DSL_MOVE_TAPE(cm)      DSL_STEP(SK_MOVE_TAPE, SEQ_NO_LABEL, DSL_CM2MM(cm), 0, 0, 0, 0, 0)
#define DSL_MOVE_TAPE_GD(cm)   DSL_STEP(SK_MOVE_TAPE, SEQ_NO_LABEL, DSL_CM2MM(cm), 4, 0, 0, 0, 0)
#define AGV_MOVE(...)          DSL_CAT(DSL_MOVE, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define ERROR_HOLD(code, c, label)  DSL_STEP_C(SK_ERROR_HOLD, label, code, 0, 0, c, 0)
#define END_RESTART            DSL_STEP(SK_END_RESTART, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)

/* ---- 공통 ---- */
#define OBS_ON                 DSL_STEP(SK_OBS, SEQ_NO_LABEL, 1, 0, 0, 0, 0, 0)
#define OBS_OFF                DSL_STEP(SK_OBS, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)
#define SET_STATE(id, val)     DSL_STEP(SK_SET_STATE, SEQ_NO_LABEL, id, val, 0, 0, 0, 0)
#define WAIT_STATE(id, val)    DSL_STEP(SK_WAIT_STATE, SEQ_NO_LABEL, id, val, 0, 0, 0, 0)
#define ACT_START(id)          DSL_STEP(SK_ACT_START, SEQ_NO_LABEL, id, 0, 0, 0, 0, 0)
#define WAIT_ACT(id)           DSL_STEP(SK_WAIT_ACT, SEQ_NO_LABEL, id, 0, 0, 0, 0, 0)
#define CALL(idx)              DSL_STEP(SK_CALL, SEQ_NO_LABEL, idx, 0, 0, 0, 0, 0)
#define END_STOP               DSL_STEP(SK_END_STOP, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)
#define END_ERROR(code)        DSL_STEP(SK_END_ERROR, SEQ_NO_LABEL, code, 0, 0, 0, 0, 0)

/* ---- 태그 표 ---- */
#define TAG_TABLE_BEGIN(name)  const tag_rule_t name[] = {
#define TAG_TABLE_END          };
#define ANY_PROFILE            0
/* 위에서 아래로 첫 번째로 맞는 규칙 하나만 실행. 액션은 최대 4칸 */
#define TAG(tag, profile, ...)  { (uint16_t)(tag), (uint16_t)(profile), TAG_COND_NONE, 0, 0, { __VA_ARGS__ } },
/* cond 는 IF_* 하나. 확장 조건 둘과 입력 조건 하나를 AND 한다 */
#define TAG_IF(tag, profile, cond, ...)  DSL_TAG_(tag, profile, cond, (__VA_ARGS__))
#define DSL_TAG_(t, p, k1, a1, b1, k2, a2, b2, on, off, acts)                                         \
    { (uint16_t)(t), (uint16_t)(p), (uint16_t)(k1), (uint16_t)(a1), (uint16_t)(b1), { DSL_UNPAREN acts }, \
      (uint16_t)(k2), (uint16_t)(a2), (uint16_t)(b2), (uint32_t)(on), (uint32_t)(off) },

/* 압축 표 (생성기용, tagact.h tag_packed_t). 번호 0 칸(없음)은 BEGIN 이 넣는다.
 * TAGP(태그, 프로파일, 확장 조건 1 번호, 확장 조건 2 번호, 입력 번호, 동작 번호) */
#define TAG_X_BEGIN(name)      static const tag_x_t name[] = { { X_ALWAYS },
#define TAG_X(x)               { x },
#define TAG_IN_BEGIN(name)     static const tag_in_t name[] = { { 0UL, 0UL },
#define TAG_IN(c)              { DSL_COND_ON(c), DSL_COND_OFF(c) },
#define TAG_ACT_BEGIN(name)    static const tag_act_t name[] = { { { 0 } },
#define TAG_ACTS(...)          { { __VA_ARGS__ } },
#define TAGP_BEGIN(name)       static const tag_pack_t name[] = {
#define TAGP(tag, profile, x1, x2, in, act)     { (uint16_t)((tag) | ((uint16_t)(profile) << 10)), (uint16_t)((x1) | ((uint16_t)(x2) << 8)), (uint16_t)((act) | ((uint16_t)(in) << 8)) },
#define TAG_POOL_END           };
#define TAG_PACKED(name, rules, x, in, act)     const tag_packed_t name = { rules, x, in, act, (uint16_t)(sizeof(rules) / sizeof(rules[0])) };

#define IF_X(x)                x, X_ALWAYS, 0UL, 0UL
#define IF_X2(x1, x2)          x1, x2, 0UL, 0UL
#define IF_IN(c)               X_ALWAYS, X_ALWAYS, DSL_COND_ON(c), DSL_COND_OFF(c)
#define IF_X_IN(x, c)          x, X_ALWAYS, DSL_COND_ON(c), DSL_COND_OFF(c)
#define IF_X2_IN(x1, x2, c)    x1, x2, DSL_COND_ON(c), DSL_COND_OFF(c)

#define IF_FORBACK_MODE        IF_X(X_FORBACK_MODE)
#define IF_LOW_BATTERY         IF_X(X_LOW_BATTERY)
#define IF_FORWARD             IF_X(X_FORWARD)
#define IF_BACKWARD            IF_X(X_BACKWARD)
#define IF_STATION(idx, lo, hi)          IF_X(X_STATION(idx, lo, hi))
#define IF_STATION_RUNNING(idx, lo, hi)  IF_X(X_STATION_RUNNING(idx, lo, hi))
#define IF_ID(lo, hi)                    IF_X(X_ID(lo, hi))
#define IF_STATION_MOD(k, n, m)          IF_X(X_STATION_MOD(k, n, m))
#define IF_STATION_DIV(k, n, m)          IF_X(X_STATION_DIV(k, n, m))
#define IF_TAG_MINUS_EQ_STATION(k, n)    IF_X(X_TAG_MINUS_EQ_STATION(k, n))
#define IF_TAG_EQ_STATION(k)             IF_X(X_TAG_EQ_STATION(k))

/* 태그 동작 이름(AGV_STOP, AGV_TURN, SEQ_START, ACS_SEND …)은 agv/tag_dsl.h. 태그 표 파일은 그 헤더를 포함한다 */

#endif /* AGV_SEQ_DSL_H */
