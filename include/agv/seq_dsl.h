/*
 * seq_dsl.h - 사이트 표 작성 매크로. 시퀀스·ACT·태그 표를 C 파일에 한 줄 한 스텝으로 쓴다.
 * 설계와 예: docs/03_시퀀스_DSL.md
 *
 * 규칙
 *  - 시간 인자는 전부 1/100초. 300 = 3초.
 *  - ON·OFF·IN·L 같은 짧은 이름을 쓰므로 표 파일은 legacy_compat.h 와 함께 포함하지 않는다.
 *  - 컴파일 옵션 --c99 필요 (가변 인자 매크로).
 *  - 조건 칸(WHEN·WAIT·WAIT_TO·WAIT_STABLE(_TO)·ERROR_HOLD·OUT_FLICKER_UNTIL_IN(_TO)·MAN·TAG_IF·TAG_RANGE_IF)은 IN( … ) 말고
 *    AND( ON·OFF·ANY·상태 낱말 … ) 도 받는다(agv/dsl_and.h). AND 의 낱말은 매크로가 아니라서 사이트가 FORWARD·ID·KEY·PARAM 같은
 *    이름의 매크로를 정의하면 깨진다. 칸에 안 들어가는 조합은 컴파일 오류다.
 */
#ifndef AGV_SEQ_DSL_H
#define AGV_SEQ_DSL_H

#include "agv/seq.h"
#include "agv/manual.h"
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
#include "agv/dsl_and.h"

/* ID 가 0~31 또는 IO_NONE 이 아니면 컴파일 오류 (앰프 신호를 조건에 쓰는 경우 등) */
#define DSL_ID_OK(x)     (0 * sizeof(char[(((x) < 32) || ((x) == IO_NONE)) ? 1 : -1]))
#define DSL_BIT(x)       ((((x) == IO_NONE) ? 0UL : (1UL << ((x) & 31))) + DSL_ID_OK(x))

/* 논리 0 이 활성인 입력(기본 active-low 배선에서 핀 1 = 활성). 같은 핀을 차종마다 반대 극성으로 쓸 때
 * 사이트 io_map.h 가 이름 정의에 표시한다: #define SENH_1 IN_ACTIVE_HIGH( IN_EXT_1 ).
 * ON()·OFF() 만 받고 반대쪽 절반으로 보낸다. ANY_ON·ANY2_ON·출력·LOAD_SENSORS 에 쓰면 컴파일 오류다.
 * C 코드의 io_in( ) 에는 쓰지 않는다(논리 ID 를 쓴다) */
#define DSL_ACTIVE_HIGH  0x100
#define IN_ACTIVE_HIGH(x)  ((x) + DSL_ACTIVE_HIGH + DSL_ID_OK(x))
#define IN_INVERTED(x)     IN_ACTIVE_HIGH(x)    /* 같은 뜻의 이름(v0.58.0) */
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
/* 조건 칸 값은 64비트 식(ON·OFF·IN·NO_IN)이어야 한다. ANY2_ON 은 32비트(uint32_t)로 만들어 조건 칸에 들어오면(IN( ANY2_ON( … ) ) 등)
 * 크기가 달라 컴파일 오류다. 정수 상수 같은 다른 크기의 값이 조건 칸에 들어와도 오류다 */
#define DSL_COND64(c)    (0 * sizeof(char[(sizeof(c) == sizeof(dsl_cond_t)) ? 1 : -1]))
#define DSL_COND_OFF(c)  ((uint32_t)((((dsl_cond_t)(c)) + DSL_COND64(c)) >> 32))
/* 양쪽 절반에 같이 선 비트 = "그중 하나라도 ON". ANY_ON 만 그렇게 쓴다(아래 IN 이 겹침을 막는다) */

#define DSL_STEP(kind, target, a0, a1, to, on, off, out)                                     \
    { (uint16_t)(kind), (uint16_t)(target), (uint16_t)(a0), (uint16_t)(a1), (uint16_t)(to), 0u, \
      (uint32_t)(on), (uint32_t)(off), (uint32_t)(out) },
#define DSL_STEP_C(kind, target, a0, a1, to, c, out)                                         \
    DSL_STEP(kind, target, a0, a1, to, DSL_COND_ON(c), DSL_COND_OFF(c), out)
/* 확장 조건 스텝. x 는 X_* 세 값(코드, a, b)으로 펼쳐져 xk, xa, xb 로 들어온다. a, b 는 arg0, arg1 */
/* 시퀀스에서는 태그 번호가 늘 0 이라 X_TAG_NUM(TAG_IS)은 뜻이 없다: 컴파일 오류 */
#define DSL_NOTAGNUM(xk) (0 * sizeof(char[((((xk) & 0x7F) != XC_TAG_NUM)) ? 1 : -1]))
#define DSL_STEP_X(kind, target, to, xk, xa, xb, c, any2)                                     \
    { (uint16_t)(kind), (uint16_t)(target), (uint16_t)(xa), (uint16_t)(xb), (uint16_t)(to), (uint16_t)((xk) + DSL_NOTAGNUM(xk)), \
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
#define DSL_XK(p, c)     (((dsl_cond_t)(p) & (((dsl_cond_t)(c) >> 32) | ((dsl_cond_t)(c) << 32))) + DSL_COND64(c))   /* 겸해서 c 가 64비트 식인지 본다 */
#define DSL_IN1(a)       ((a) + DSL_COND64(a))
#define DSL_IN2(a, b)    (((a) | (b)) + DSL_COND64(a) + DSL_NOX(DSL_XK(a, b)))
#define DSL_IN3(a, b, c)             (((a) | (b) | (c)) + DSL_COND64(a) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c)))
#define DSL_IN4(a, b, c, d)          (((a) | (b) | (c) | (d)) + DSL_COND64(a) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) | DSL_XK((a) | (b) | (c), d)))
#define DSL_IN5(a, b, c, d, e)       (((a) | (b) | (c) | (d) | (e)) + DSL_COND64(a) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) \
                                      | DSL_XK((a) | (b) | (c), d) | DSL_XK((a) | (b) | (c) | (d), e)))
#define DSL_IN6(a, b, c, d, e, f)    (((a) | (b) | (c) | (d) | (e) | (f)) + DSL_COND64(a) + DSL_NOX(DSL_XK(a, b) | DSL_XK((a) | (b), c) \
                                      | DSL_XK((a) | (b) | (c), d) | DSL_XK((a) | (b) | (c) | (d), e) | DSL_XK((a) | (b) | (c) | (d) | (e), f)))
/* 7·8 개는 AND 가 항목 7 개까지 받으며 안에서 쓴다 */
#define DSL_IN7(a1, a2, a3, a4, a5, a6, a7) (((a1) | (a2) | (a3) | (a4) | (a5) | (a6) | (a7)) + DSL_COND64(a1) + DSL_NOX(DSL_XK(a1, a2) | DSL_XK((a1) | (a2), a3) \
                                      | DSL_XK((a1) | (a2) | (a3), a4) | DSL_XK((a1) | (a2) | (a3) | (a4), a5) \
                                      | DSL_XK((a1) | (a2) | (a3) | (a4) | (a5), a6) | DSL_XK((a1) | (a2) | (a3) | (a4) | (a5) | (a6), a7)))
#define DSL_IN8(a1, a2, a3, a4, a5, a6, a7, a8) (((a1) | (a2) | (a3) | (a4) | (a5) | (a6) | (a7) | (a8)) + DSL_COND64(a1) + DSL_NOX(DSL_XK(a1, a2) | DSL_XK((a1) | (a2), a3) \
                                      | DSL_XK((a1) | (a2) | (a3), a4) | DSL_XK((a1) | (a2) | (a3) | (a4), a5) \
                                      | DSL_XK((a1) | (a2) | (a3) | (a4) | (a5), a6) | DSL_XK((a1) | (a2) | (a3) | (a4) | (a5) | (a6), a7) \
                                      | DSL_XK((a1) | (a2) | (a3) | (a4) | (a5) | (a6) | (a7), a8)))
#define IN(...)          DSL_CAT(DSL_IN, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define NO_IN            ((dsl_cond_t)0)
/* 두 번째 "하나라도 ON" 묶음: (a||b) && (c||d). WHEN_ANY2·WAIT_ANY2 의 둘째 인자. 32비트 값이라 조건 칸에 들어가면 DSL_COND64 가 컴파일 오류로 막는다 */
#define ANY2_ON(...)     ((uint32_t)DSL_MASK(__VA_ARGS__))

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
/* 지금 태그 번호 == 식. 식은 스테이션 큐 식(아래 STATION): STATION(k), STATION(k) ± n, MOD( STATION(k), m ),
 * n + MOD( STATION(k), m ). 태그 표 전용. 예: 레거시 "Station[0] == Tag - 100" 은 X_TAG_NUM( STATION(0) + 100 ) */
#define X_TAG_NUM(e)                 (XC_TAG_NUM | DSL_SLOT(DSL_ST_K(e))), (uint16_t)(DSL_ST_N(e) + DSL_ST_OK(e)), (uint16_t)DSL_ST_M(e)
/* 파라미터 값 == v. 이름은 FND 이름(아래 DSL_PRM_*). 자릿수 파라미터의 한 자리는 이름_자리(TFB_1 전후진 모드, TFB_10 회전 모드) */
#define X_PARAM(name, v)             DSL_XPRM(DSL_PRM_##name, v)
#define DSL_XPRM(...)                DSL_XPRM_(__VA_ARGS__)
#define DSL_XPRM_(p, place, v)       (XC_PARAM_EQ | DSL_SLOT(p)), (v), (place)
#define DSL_PRM_DRS   P_SPEED_HIGH, 0
#define DSL_PRM_LOS   P_SPEED_LOW, 0
#define DSL_PRM_TNS   P_SPEED_TURN, 0
#define DSL_PRM_ATD   P_AUTO_DRIVE_MODE, 0
#define DSL_PRM_ITM   P_INTERFACE_MODE, 0
#define DSL_PRM_SND   P_SOUND, 0
#define DSL_PRM_BPS   P_BUMPER_USE, 0
#define DSL_PRM_OBS   P_OBS_USE, 0
#define DSL_PRM_DSM   P_DISPLAY_MODE, 0
#define DSL_PRM_BO    P_BAT_OFFSET_DV, 0
#define DSL_PRM_JSP   P_SPEED_JOG, 0
#define DSL_PRM_ACC   P_ACC, 0
#define DSL_PRM_DCC   P_DCC, 0
#define DSL_PRM_AS    P_AUTO_START_SEC, 0
#define DSL_PRM_ST    P_STATION_NO, 0
#define DSL_PRM_TFB   P_TURN_FORBACK, 0
#define DSL_PRM_TFB_1  P_TURN_FORBACK, 1
#define DSL_PRM_TFB_10 P_TURN_FORBACK, 10
#define DSL_PRM_SL    P_STOP_LEN_CM, 0
#define DSL_PRM_K_P   P_STEER_KP, 0
#define DSL_PRM_K_I   P_STEER_KI, 0
#define DSL_PRM_WS    P_WHEEL_DIA_MM, 0
#define DSL_PRM_RG    P_GEAR_RATIO, 0
#define DSL_PRM_WW    P_WHEEL_BASE_MM, 0
#define DSL_PRM_ID    P_AGV_ID, 0
#define DSL_PRM_AT    P_AGV_TYPE, 0
#define DSL_PRM_LO    P_LAMP_EXT_USE, 0
#define DSL_PRM_SPN   P_SPEED_MIN, 0
#define DSL_PRM_CML   P_COMM_LINK, 0
#define DSL_PRM_FL    P_STOP_LEN_FWD_CM, 0
#define DSL_PRM_ASS   P_ACS_STANDBY, 0
#define DSL_PRM_BGO   P_BRANCH_OFFSET, 0
#define DSL_PRM_CD    P_CROSS_IN_CM, 0
#define DSL_PRM_WRT   P_ACS_RETRY_LEGACY, 0
#define DSL_PRM_CT    P_ACS_RETRY_SEC, 0
#define DSL_PRM_EST   P_ERR_REPORT_MIN, 0
#define DSL_PRM_RCI   P_CROSS_RETRY_100MS, 0
#define DSL_PRM_SR    P_STEER_RPM_OFFSET, 0
#define DSL_PRM_GTO   P_GUIDE_TIMEOUT, 0
#define DSL_PRM_AV    P_ACS_VERSION, 0
#define DSL_PRM_GD    P_GUIDE_DIST_MM, 0
#define DSL_PRM_ES    P_ENC_STALL_TICKS, 0
#define DSL_PRM_EDS   P_EMPTY_SPEED, 0
#define DSL_PRM_ETN   P_EMPTY_TURN, 0
#define DSL_PRM_EAC   P_EMPTY_ACC, 0
#define DSL_PRM_EDC   P_EMPTY_DCC, 0
#define DSL_PRM_LDS   P_LOAD_DETECT, 0
#define DSL_PRM_AD    P_ACT_DELAY, 0
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
#define PROFILE(profile, type, from, to)  { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to), 0UL, 0 },
/* 적재 센서가 있는 프로파일. mask = LOAD_SENSORS(SEN_1, ...) 하나라도 켜지면 적재 */
#define PROFILE_LOAD(profile, type, from, to, mask)  { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to), (uint32_t)(mask), 0 },
#define LOAD_SENSORS(...)          DSL_MASK(__VA_ARGS__)
/* 적재 조건 하나나 둘, 하나라도 참이면 적재. 표 앞에 이름 있는 상수로 두고 행은 그 이름을 쓴다:
 *   LOAD_IF( 이름, IN( … )[, IN( … )] )              static const uint32_t 이름[4]
 *   PROFILE_LOAD_IF( 프로파일, 타입, 처음, 끝, 이름 )
 * 조건이 비면(NO_IN) 컴파일 오류. 이름이 LOAD_IF 배열이 아니면 컴파일 오류. seq_site_t.load_fn = manual_load 일 때 쓴다(agv/manual.h).
 * v0.53.0 은 조건을 행 안의 compound literal 로 두었다(C28x 에서도 .econst 였다). 놓는 곳을 이름으로 보이게 바꿨다 */
#define DSL_LIF_OK(c)        (0 * sizeof(char[((c) != 0) ? 1 : -1]))
#define DSL_LIF1(c)          DSL_LIF2(c, c)
#define DSL_LIF2(c1, c2)     { DSL_COND_ON(c1) + DSL_LIF_OK(c1), DSL_COND_OFF(c1), \
                               DSL_COND_ON(c2) + DSL_LIF_OK(c2), DSL_COND_OFF(c2) }
#define LOAD_IF(name, ...)   static const uint32_t name[4] = DSL_CAT(DSL_LIF, DSL_NARG(__VA_ARGS__))(__VA_ARGS__);
#define PROFILE_LOAD_IF(profile, type, from, to, name)                               \
    { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to), 0UL, \
      (name) + 0 * sizeof(char[(sizeof(name) == sizeof(uint32_t[4])) ? 1 : -1]) },
#define PROFILE_TABLE_END          };

/* ---- 라벨·분기 ---- */
#define L(label)         DSL_STEP(SK_LABEL, label, 0, 0, 0, 0, 0, 0)
#define GOTO(label)      DSL_STEP(SK_GOTO,  label, 0, 0, 0, 0, 0, 0)

/* SELECT_BEGIN·SELECT_END 는 읽기 위한 표시이며 스텝을 만들지 않는다. 가지 수 제한 없음 */
#define SELECT_BEGIN
/* 조건 칸 c: IN( … ) 꼴 그대로(괄호로 시작) 또는 AND( … ) 기록(DSL_SQ_). AND 의 상태 낱말은 확장 조건 칸(xk, xa, xb)으로 간다 */
#define WHEN(c, label)               DSL_CAT(DSL_WHEN_, DSL_IS_PAREN(c))(c, label)
#define DSL_WHEN_1(c, label)         DSL_STEP_C(SK_WHEN, label, 0, 0, 0, c, 0)
#define DSL_WHEN_0(c, label)         DSL_APPLY(DSL_STEP_X, SK_WHEN, label, 0, DSL_SQ_##c)
#define WHEN_STATE(id, val, label)   DSL_STEP(SK_WHEN_STATE, label, id, val, 0, 0, 0, 0)
#define WHEN_IF(x, label)            DSL_STEP_X(SK_WHEN, label, 0, x, NO_IN, 0)
#define WHEN_IF_IN(x, c, label)      DSL_STEP_X(SK_WHEN, label, 0, x, c, 0)
#define WHEN_ANY2(c, any2, label)    DSL_STEP_C(SK_WHEN, label, 0, 0, 0, c, any2)
/* WHEN_STABLE( 조건, 틱, 라벨 )·WHEN_ACCUM( 조건, 틱, 라벨 )(v0.61.0): 묶음 안 유지 가지. 조건(입력만)이 틱 초과 이어서(누적은 모두 합쳐)
 * 참이면 라벨로. 틱은 WAIT_ACCUM 과 같다(0~65534 또는 PARAM_TIME). 거짓이면 셈 0(누적은 그대로). 앞 가지가 참인 틱에는 셈을 보지 않는다.
 * 묶음당 유지 가지 둘까지(셋째는 실행 때 표 오류 290. 매크로는 못 잡는다). ACT 에서도 된다 */
#define DSL_WNHOLD(c, s, acc, label)        DSL_APPLY(DSL_WNHOLD_, c, acc, label, DSL_HT(s, DSL_HOLD_OK))
#define DSL_WNHOLD_(c, acc, label, a0, u)   DSL_STEP_C(SK_WHEN_STABLE, label, a0, (u) | (acc), 0, DSL_CN(c), 0)
#define WHEN_STABLE(c, s, label)     DSL_WNHOLD(c, s, 0, label)
#define WHEN_ACCUM(c, s, label)      DSL_WNHOLD(c, s, SEQ_HOLD_ACCUM, label)
/* 호기별 분기. 프로파일을 나누지 않고 한 표에서 호기 번호로 가른다 */
#define WHEN_ID(lo, hi, label)       WHEN_IF(X_ID(lo, hi), label)
#define OTHERWISE(label)             DSL_STEP(SK_WHEN, label, 0, 0, 0, 0, 0, 0)
#define SELECT_END
/* 제한 t 는 1~0x7FFF 틱(0 은 한 틱 뒤 바로 라벨로 가 고르기가 아니다. UNMATCHED 는 이 스텝에서 받지 않는다) */
#define DSL_SELTO_OK(t)              (0 * sizeof(char[(((t) >= 1) && ((t) < 0x8000L)) ? 1 : -1]))
#define SELECT_END_TO(t, label)      DSL_STEP(SK_WHEN_TIMEOUT, label, 0, 0, (t) + DSL_SELTO_OK(t), 0, 0, 0)

/* ---- 출력 ---- */
#define OUT_ON(...)      DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, DSL_MASK(__VA_ARGS__), 0, 0)
#define OUT_OFF(...)     DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, 0, DSL_MASK(__VA_ARGS__), 0)
/* 켜는 시간, 끄는 시간, 횟수 */
#define OUT_PULSE(o, on_t, off_t, n) \
    DSL_STEP(SK_OUT_PULSE, SEQ_NO_LABEL, on_t, off_t, n, 0, 0, DSL_BIT(o))
/* 접점 통신: 출력을 period 간격으로 점멸하다가 조건이 참이면 끄고 다음 스텝.
 * 조건 칸은 IN( … ) 또는 AND( … ) 다. AND 에 상태 낱말이 있으면 점멸 스텝의 칸 배치가 달라진다(seq.c flk_cells: 주기·출력이 out 칸
 * 하나로 들어간다). 그래서 AND 의 둘째 ANY 와 둘째 상태 낱말은 컴파일 오류다 */
#define DSL_FLKB_OK(p)       (0 * sizeof(char[(((p) >= 0) && ((p) <= 0xFFFF)) ? 1 : -1]))
#define DSL_FLKB(o, p)       ((((uint32_t)(p) + DSL_FLKB_OK(p)) << 8) | (uint32_t)(((o) == IO_NONE) ? 0 : (((o) & 31) + 1)))
#define DSL_FLK(o, p, t, label, c)    DSL_CAT(DSL_FLK_, DSL_IS_PAREN(c))(o, p, t, label, c)
#define DSL_FLK_1(o, p, t, label, c)  DSL_STEP_C(SK_OUT_FLICKER, label, p, 0, t, c, DSL_BIT(o))
#define DSL_FLK_0(o, p, t, label, c)  DSL_APPLY(DSL_FLKX, o, p, t, label, DSL_FK_##c)
#define DSL_FLKX(o, p, t, label, k1, a1, b1, c, chk) \
    DSL_STEP_X(SK_OUT_FLICKER, label, t, k1, ((k1) ? (a1) : (p)), ((k1) ? (b1) : 0), c, \
               ((k1) ? DSL_FLKB(o, p) : DSL_BIT(o)) + (chk))
#define OUT_FLICKER_UNTIL_IN(o, period, c) \
    DSL_FLK(o, period, 0, SEQ_NO_LABEL, c)
#define OUT_FLICKER_UNTIL_IN_TO(o, period, c, t, label) \
    DSL_FLK(o, period, (t) + DSL_TO_OK(t), label, c)
/* 출력 켬, 응답 켜짐 대기, 응답이 켜지면 바로 다음 스텝(v0.58.0). 출력은 끄지 않는다(OUT_OFF 등으로 표가 끈다).
 * 제한 t 를 넘으면 라벨로 가고 출력은 그대로다. 응답이 이미 켜져 있으면 바로 끝난다 */
#define HANDSHAKE(o, in_ack, t, label) \
    DSL_STEP(SK_HANDSHAKE, label, 0, 0, (t) + DSL_TO_OK(t), DSL_BIT(in_ack), 0, DSL_BIT(o))

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
/* 조건 칸 c 는 WHEN 과 같다: IN( … ) 또는 AND( … ) */
#define WAIT(c)                     DSL_CAT(DSL_WAIT_, DSL_IS_PAREN(c))(c)
#define DSL_WAIT_1(c)               DSL_STEP_C(SK_WAIT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
#define DSL_WAIT_0(c)               DSL_APPLY(DSL_STEP_X, SK_WAIT, SEQ_NO_LABEL, 0, DSL_SQ_##c)
/* WAIT_TO( 조건, 제한, 라벨 ): 제한은 틱 숫자(0 = 제한 없음, 0x8000 이상은 컴파일 오류, UNMATCHED 도 오류) 또는
 * PARAM_TIME( FND 또는 P_…, TICK | DSEC | SEC | MIN )(옛 WAIT_TO_PARAM 과 같은 스텝). 파라미터 제한에는 입력 조건만 쓴다
 * (상태 낱말·둘째 ANY 는 스텝 칸이 겹쳐 컴파일 오류). UNMATCHED 는 WAIT_STABLE_TO 에서만 받는다 */
#define DSL_TO_OK(t)                (0 * sizeof(char[(((t) >= 0) && ((t) < 0x8000L)) ? 1 : -1]))
#define PARAM_TIME(p, unit)         DSL_PTM(DSL_PTSEL(p), DSL_UNIT_##unit)
#define DSL_PTSEL(p)                DSL_APPLY(DSL_PTSEL_X, DSL_NARG(DSL_PRM_##p), p, DSL_PRM_##p)
#define DSL_PTSEL_X(n, p, ...)      DSL_CAT(DSL_PTSEL_, n)(p, __VA_ARGS__)
#define DSL_PTSEL_1(p, x)           (p)
#define DSL_PTSEL_2(p, a, b)        ((a) + (0 * sizeof(char[((b) == 0) ? 1 : -1])))
#define DSL_PTMX_DSL_PTM(p, u)      p, u
#define DSL_WTOW_DSL_PTM(p, u)      ~, DSL_WTOPARAM
#define WAIT_TO(c, t, label)        DSL_CAT(DSL_WTOP_, DSL_IS_PAREN(t))(c, t, label)
#define DSL_WTOP_1(c, t, label)     DSL_WTOC(c, t, label)
#define DSL_WTOP_0(c, t, label)     DSL_ARG2(DSL_WTOW_##t, DSL_WTOC)(c, t, label)
#define DSL_WTOC(c, t, label)       DSL_CAT(DSL_WAITTO_, DSL_IS_PAREN(c))(c, (t) + DSL_TO_OK(t), label)
#define DSL_WAITTO_1(c, t, label)   DSL_STEP_C(SK_WAIT, label, 0, 0, t, c, 0)
#define DSL_WAITTO_0(c, t, label)   DSL_APPLY(DSL_STEP_X, SK_WAIT, label, t, DSL_SQ_##c)
#define DSL_WTOPARAM(c, t, label)   DSL_APPLY(DSL_WTOPARAM_, c, label, DSL_PTMX_##t)
#define DSL_WTOPARAM_(c, label, p, u)  DSL_STEP_C(SK_WAIT_PARAM, label, (p) + DSL_PARAM_OK(p), u, 0, DSL_CN(c), 0)
/* 유지 틱 칸 s(WAIT_STABLE(_TO)·WAIT_ACCUM(_TO)·WHEN_STABLE·WHEN_ACCUM, v0.61.0): 틱 숫자 또는 PARAM_TIME( FND 또는 P_…, TICK | DSEC | SEC ).
 * 두 값 "arg0, arg1 상위 8비트" 로 펼친다. 파라미터 시간은 arg0 = 파라미터 번호, arg1 상위 8비트 = 단위 틱(1·10·100. MIN 은 8비트에
 * 안 들어가 컴파일 오류). 한도 = 값 x 단위를 틱마다 다시 읽는다(음수 0, 65534 로 묶음, 값 0 = 첫 참 틱에 통과, seq.c hold_limit).
 * chk 는 숫자 틱의 검사 매크로: WAIT_STABLE(_TO) 는 예전처럼 검사 없음, 새 매크로는 0~65534 */
#define DSL_HOLD_OK(t)              (0 * sizeof(char[(((t) >= 0) && ((t) <= 0xFFFEL)) ? 1 : -1]))
#define DSL_HOLD_NOCHK(t)           0
#define DSL_HUNIT_OK(u)             (0 * sizeof(char[((u) <= 0xFF) ? 1 : -1]))
#define DSL_HT(s, chk)              DSL_CAT(DSL_HTP_, DSL_IS_PAREN(s))(s, chk)
#define DSL_HTP_1(s, chk)           DSL_HTN(s, chk)
#define DSL_HTP_0(s, chk)           DSL_ARG2(DSL_HTW_##s, DSL_HTN)(s, chk)
#define DSL_HTW_DSL_PTM(p, u)       ~, DSL_HTPRM
#define DSL_HTN(s, chk)             ((s) + chk(s)), 0
#define DSL_HTPRM(s, chk)           DSL_APPLY(DSL_HTPRM_, DSL_PTMX_##s)
#define DSL_HTPRM_(p, u)            ((p) + DSL_PARAM_OK(p)), ((uint16_t)((u) + DSL_HUNIT_OK(u)) << 8)
#define DSL_WHOLD(c, s, acc, t, label, chk)   DSL_APPLY(DSL_WHOLD_, c, acc, t, label, DSL_HT(s, chk))
#define DSL_WHOLD_(c, acc, t, label, a0, u)   DSL_STEP_C(SK_WAIT_STABLE, label, a0, (u) | (acc), t, DSL_CN(c), 0)
#define WAIT_STABLE(c, s)           DSL_WHOLD(c, s, 0, 0, SEQ_NO_LABEL, DSL_HOLD_NOCHK)
#define WAIT_STABLE_TO(c, s, t, label)  DSL_WHOLD(c, s, 0, t, label, DSL_HOLD_NOCHK)
/* WAIT_ACCUM( 조건, 틱 )·WAIT_ACCUM_TO( 조건, 틱, 제한, 라벨 )(v0.61.0): WAIT_STABLE(_TO) 와 같은 스텝이고 arg1 = SEQ_HOLD_ACCUM.
 * 조건이 참인 틱만 세고 거짓인 틱에는 셈을 멈춘다(지우지 않는다). 참인 틱이 모두 합쳐 틱을 넘는 틱에 통과.
 * 틱 0~65534 또는 PARAM_TIME(밖은 컴파일 오류). 조건은 입력만(상태 낱말·둘째 ANY 는 오류). 제한은 WAIT_STABLE_TO 와 같다(UNMATCHED 도 받는다) */
#define WAIT_ACCUM(c, s)            DSL_WHOLD(c, s, SEQ_HOLD_ACCUM, 0, SEQ_NO_LABEL, DSL_HOLD_OK)
#define WAIT_ACCUM_TO(c, s, t, label)   DSL_WHOLD(c, s, SEQ_HOLD_ACCUM, t, label, DSL_HOLD_OK)
#define WAIT_OUT(c)                 DSL_STEP_C(SK_WAIT_OUT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
/* WHEN_OUT( 출력 조건, 라벨 )(v0.61.0): WHEN 묶음의 가지. 출력 래치가 조건(WAIT_OUT 과 같다: IN( ON( 출력… ), OFF( … ) ))이면 라벨로.
 * AND 는 입력 항목만(상태 낱말·둘째 ANY 는 오류) */
#define WHEN_OUT(c, label)          DSL_STEP_C(SK_WHEN_OUT, label, 0, 0, 0, DSL_CN(c), 0)
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
#define WAIT_PARAM_TO(c, p, unit, label)  WAIT_TO_PARAM(c, p, unit, label)   /* WAIT_TO( c, PARAM_TIME( p, unit ), label ) 의 옛 이름 쪽 별칭 */

/* 켜기·끄기를 한 스텝에 (BELT_ON 과 같은 SK_OUT). 사이트가 같은 토큰열로 정의해 쓰던 것을 코어로 옮겼다(v0.58.0: 재정의 경고 없음) */
#define OUT_SET( on, off )  DSL_STEP( SK_OUT, SEQ_NO_LABEL, 0, 0, 0, (on), (off), 0 )

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
#define DSL_INFO_DISPLAY    'D'   /* 표시 번호(v0.61.0). 번호 0 이하는 보내지 않는다(acs.c) */
#define DSL_INFO_IDLE       'X'   /* 오류 5초 뒤 자동 보고(acs.c)와 같은 Info·같은 슬롯. 같은 슬롯이라 둘이 겹치면 나중 것으로 바뀌고 ACK 하나로 풀린다 */
/* 번호 자리의 스테이션 큐 번호: ACS_SEND( NEXT, STATION(1) ) = 레거시 DA_ACS_Next( ACS_Station_Call_Num[1] ). k 0~23.
 * 상수식이라 정적 표에 쓴다(ACS( 종류, STATION(k) ) 의 출처 자리에서는 이름으로 붙는다).
 * 스테이션 큐 식: STATION(k) ± n, MOD( STATION(k), m ), n + MOD( STATION(k), m ) 를 C 연산 그대로 쓴다(IF_TAG_NUM, 태그 ACS_SEND).
 * 값 = 표식 2^29 + k << 16 + m << 21 + n. n 은 -2048~2047(넘으면 비트 12~15 가 서서 컴파일 오류), m 1~255.
 * 시퀀스 ACS·ACS_SEND 는 STATION(k) 와 STATION(0) + 1 만 받는다(다른 ± n, MOD 는 컴파일 오류) */
#define DSL_STATION_MARK       0x20000000L
/* STATION( k ) = 큐 값(위). STATION( k, lo, hi ) = AND 안의 상태 항목(스테이션[k] 가 lo 이상 hi 이하, 예전 X_STATION) */
#define STATION(...)           DSL_CAT(DSL_STATION_, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define DSL_STATION_1(k)       (DSL_STATION_MARK + ((long)(k) << 16) + (0 * sizeof(char[(((k) >= 0) && ((k) < ACS_STATION_MAX)) ? 1 : -1])))
#define DSL_STATION_3(k, lo, hi)  DSL_STI(k, lo, hi)
#define DSL_ST_T(v)            ((long)(v) - DSL_STATION_MARK + 0x800L)
#define DSL_IS_STATION(v)      ((DSL_ST_T(v) >= 0L) && (DSL_ST_T(v) < (1L << 29)) && ((DSL_ST_T(v) & 0xF000L) == 0L))
#define DSL_ST_K(v)            ((int)((DSL_ST_T(v) >> 16) & 0x1FL))
#define DSL_ST_N(v)            ((long)(DSL_ST_T(v) & 0xFFFL) - 0x800L)
#define DSL_ST_M(v)            ((int)((DSL_ST_T(v) >> 21) & 0xFFL))
#define DSL_ST_OK(v)           (0 * sizeof(char[DSL_IS_STATION(v) ? 1 : -1]))
#define DSL_ST_PLAIN(v)        (DSL_IS_STATION(v) && (DSL_ST_N(v) == 0L) && (DSL_ST_M(v) == 0))
#define MOD(x, m)              ((x) + ((long)(m) << 21) + (0 * sizeof(char[(DSL_ST_PLAIN(x) && ((m) >= 1) && ((m) <= 255)) ? 1 : -1])))
#define DSL_SEND_OK(n)         (0 * sizeof(char[((((n) >= 0) && ((n) < 0x10000L)) || DSL_ST_PLAIN(n)) ? 1 : -1]))
#define DSL_SEND_SRC(n)        (DSL_IS_STATION(n) ? ((DSL_ST_K(n) == 0) ? SEQ_NUM_STATION0 : SEQ_NUM_STATION(DSL_ST_K(n))) : SEQ_NUM_CONST)
/* ACS( 종류, 번호 ): 번호 칸은 숫자 0~65535, STATION(k), STATION(0) + 1, AGV_ID. 펌웨어에 없는 조합(STATION(k) ± n 의 k 가 0 이 아닌 것,
 * STATION(0) ± 1 말고 다른 n, MOD, 70000 같은 범위 밖 숫자)은 컴파일 오류다.
 * 옛 출처 낱말 STATION0·STATION0_PLUS1·AGV_ID 는 같은 스텝의 별칭이다. CONST 는 늘 0 을 보고하는 낱말이라 컴파일 오류다(숫자 0 을 쓴다).
 * 낱말은 x 를 펼치기 전에 붙여서 고른다(DSL_SACSW_##x). 그래서 사이트가 STATION0·AGV_ID 같은 이름의 매크로(#define AGV_ID 25)를 정의해도
 * 낱말이 숫자로 바뀌지 않는다(예전 SEQ_NUM_##src 와 같다). 대신 번호 칸 첫 글자가 "(" 인 식(ACS( 종류, (70 + 7) ))은 붙일 수 없어 못 받는다:
 * 괄호 없이 70 + 7, 또는 ACS_SEND( 종류, (70 + 7) ) 로 쓴다 */
#define ACS(info, x)           DSL_ARG2(DSL_SACSW_##x, DSL_ACSN)(info, x)
#define DSL_SACSW_CONST          ~, DSL_ACSV_CONST
#define DSL_SACSW_STATION0       ~, DSL_ACSV_STATION0
#define DSL_SACSW_STATION0_PLUS1 ~, DSL_ACSV_STATION0_PLUS1
#define DSL_SACSW_AGV_ID         ~, DSL_ACSV_AGV_ID
#define DSL_ACSV_CONST(info, x)          DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, ACS_CONST_ALWAYS_REPORTS_ZERO_USE_NUMBER_0, 0, 0, 0, 0)
#define DSL_ACSV_STATION0(info, x)       DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_STATION0, 0, 0, 0, 0)
#define DSL_ACSV_STATION0_PLUS1(info, x) DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_STATION0_PLUS1, 0, 0, 0, 0)
#define DSL_ACSV_AGV_ID(info, x)         DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_AGV_ID, 0, 0, 0, 0)
/* STATION(0) + 1 만 STATION0_PLUS1, STATION(k) 는 큐[k]. k 가 0 이 아니면 n 은 0 이어야 한다 */
#define DSL_NSRC(n)    (DSL_IS_STATION(n) ? ((DSL_ST_K(n) == 0) ? ((DSL_ST_N(n) == 0L) ? SEQ_NUM_STATION0 : SEQ_NUM_STATION0_PLUS1) \
                                                                  : SEQ_NUM_STATION(DSL_ST_K(n))) : SEQ_NUM_CONST)
#define DSL_NOK(n)     (0 * sizeof(char[((((n) >= 0) && ((n) < 0x10000L)) \
                           || (DSL_IS_STATION(n) && (DSL_ST_M(n) == 0) && ((DSL_ST_N(n) == 0L) || ((DSL_ST_K(n) == 0) && (DSL_ST_N(n) == 1L))))) ? 1 : -1]))
#define DSL_ACSN(info, x)      DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, DSL_NSRC(x) + DSL_NOK(x), DSL_IS_STATION(x) ? 0 : (x), 0, 0, 0)
/* 옛 이름: ACS( 종류, 번호 ) 의 별칭(숫자·STATION(k) 만 쓰던 꼴이 그대로 된다) */
#define ACS_SEND(info, num)    DSL_ACSN(info, num)
#define ACS_WAIT_ACK(t, label) DSL_STEP(SK_ACS_WAIT_ACK, label, 0, 0, t, 0, 0, 0)
#define ACS_STATION(idx, val, cnt)  DSL_STEP(SK_ACS_STATION, SEQ_NO_LABEL, idx, val, cnt, 0, 0, 0)
#define STATION_RESET          DSL_STEP(SK_STATION_RESET, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)

/* ---- 주행 관련 (ACT 에서 금지, seq_bind 가 부팅 때 거부) ----
 * 방식 옵션은 인자 맨 뒤에 적는다. 안 적으면 일반 동작이다. 정해 두지 않은 옵션·조합은 컴파일 오류다.
 * 태그 표(tag_dsl.h)와 뜻이 같은 동작은 같은 이름이다(AGV_SPEED·AGV_SPEED_MMIN·AGV_FORBACK·AGV_TURN) */
/* AGV_SPEED( HIGH | LOW | LLOW | MIN | m/min ): 단계 낱말 또는 m/min 4~255(숫자·식). 0~3 은 단계 번호와 겹쳐 받지 않는다.
 * 낱말은 붙여서 고르므로 사이트가 HIGH 같은 이름의 매크로를 정의하면 깨진다. AGV_SPEED_MMIN 은 숫자만 받는 옛 이름(별칭) */
#define DSL_MMIN_OK(v)         (0 * sizeof(char[(((v) >= 4) && ((v) <= 255)) ? 1 : -1]))
#define DSL_SPW_HIGH           ~, SEQ_SPD_HIGH
#define DSL_SPW_LOW            ~, SEQ_SPD_LOW
#define DSL_SPW_LLOW           ~, SEQ_SPD_LLOW
#define DSL_SPW_MIN            ~, SEQ_SPD_MIN
/* 낱말은 x 를 펼치기 전에 붙여서 고른다(사이트의 #define HIGH 1 이 낱말을 덮지 않는다). 첫 글자가 "(" 인 식 AGV_SPEED( (20 + 10) ) 은 붙일 수 없어
 * 못 받는다(gcc 붙이기 오류): AGV_SPEED( 20 + 10 ) 이나 AGV_SPEED_MMIN( (20 + 10) ) 으로 쓴다 */
#define AGV_SPEED(x)           DSL_STEP(SK_SPEED, SEQ_NO_LABEL, DSL_ARG2(DSL_SPW_##x, ((x) + DSL_MMIN_OK(x))), 0, 0, 0, 0, 0)
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
#define ERROR_HOLD(code, c, label)  DSL_STEP_C(SK_ERROR_HOLD, label, code, 0, 0, DSL_CN(c), 0)   /* 조건은 입력만(상태·둘째 ANY 는 오류) */
#define END_RESTART            DSL_STEP(SK_END_RESTART, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)

/* ---- 공통 ---- */
#define OBS_ON                 DSL_STEP(SK_OBS, SEQ_NO_LABEL, 1, 0, 0, 0, 0, 0)
#define OBS_OFF                DSL_STEP(SK_OBS, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)
#define OBS_OFF_AUTO           DSL_STEP(SK_OBS, SEQ_NO_LABEL, 2, 0, 0, 0, 0, 0)   /* 끔. 시퀀스가 통째로 지워지면 다시 켠다(D.obs_use) */
#define SET_STATE(id, val)     DSL_STEP(SK_SET_STATE, SEQ_NO_LABEL, id, val, 0, 0, 0, 0)
#define WAIT_STATE(id, val)    DSL_STEP(SK_WAIT_STATE, SEQ_NO_LABEL, id, val, 0, 0, 0, 0)
#define ACT_START(id)          DSL_STEP(SK_ACT_START, SEQ_NO_LABEL, id, 0, 0, 0, 0, 0)
#define WAIT_ACT(id)           DSL_STEP(SK_WAIT_ACT, SEQ_NO_LABEL, id, 0, 0, 0, 0, 0)
#define CALL(idx)              DSL_STEP(SK_CALL, SEQ_NO_LABEL, idx, 0, 0, 0, 0, 0)
#define END_STOP               DSL_STEP(SK_END_STOP, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)
#define END_ERROR(code)        DSL_STEP(SK_END_ERROR, SEQ_NO_LABEL, code, 0, 0, 0, 0, 0)

/* ---- 수동 조작 표 (agv/manual.h, docs/03 5.4절) ----
 * MAN( 프로파일, 모드, 조건, 출력… ) 사슬의 가지, MAN_ELSE( 프로파일, 모드, 출력… ) 사슬 끝. 모든 사슬은 MAN_ELSE 로 닫는다
 * (할 일이 없으면 NO_OUT). 한 프로파일에 사슬이 여럿이어도 된다.
 * 프로파일: 사이트 프로파일 상수 그대로(1~63, 0·범위 밖은 컴파일 오류). 지금 프로파일(seq_profile())과 같은 줄만 맞는다(v0.54.0,
 * 예전 ID( 처음, 끝 ) 호기 범위). 여러 프로파일이 같은 동작이면 묶음을 쓴다(v0.63.0): MAN_PROFILES( PROFILE_SET( k ), 프로파일… )
 * 묶음 줄(k 0~15, 프로파일 1~63 을 16 개까지, 같은 k 로 줄을 더 쓰면 OR) 뒤에 MAN·MAN_ELSE·MAN_FB_SW·MAN_OBS_B·MAN_START_SW 의
 * 프로파일 칸에 PROFILE_SET( k ) 를 쓰면 지금 프로파일이 그 묶음에 들 때 맞는다. 묶음 줄은 표를 훑으며 읽으므로 쓰는 줄보다 앞에 둔다.
 * 모드: IDLE 자동 화면에서 대기(run_is_idle(), 준비 아님, STOP 아님), JOG 조그 화면이고 STOP 아님,
 *   ANY 모드 무관(manual_tick 이 도는 모든 틱, 자동 주행·시퀀스 중·STOP 도. 같은 틱의 시퀀스·ACT 출력을 덮는다, v0.63.0).
 * 조건: IN( ON(…), OFF(…) ), KEY( UP )(지금 눌린 키, 소비하지 않음), NO_IN(늘 참), AND( ON·OFF·ANY… [, KEY( k )] )(키는 하나까지,
 *   다른 상태 낱말·둘째 ANY 는 컴파일 오류. 줄은 키와 입력 조건이 둘 다 맞아야 한다).
 * 출력(여럿이면 모두): ON( 출력… ) 켬, OFF( 출력… ) 끔, BELT( n, CW | CCW | OFF ). 한 출력을 켜면서 끄면 컴파일 오류.
 *   BELT 는 사이트 BELTn_CW·BELTn_CCW 마스크로 그 방향 마스크를 켜고 그 벨트의 나머지 출력을 끈다(BELT_ON 스텝과 같은 값).
 * MAN_FB_SW( 프로파일, 입력 ): 수동 방향 전환 스위치 입력(맞는 줄이 없으면 IN_IF_5). MAN_OBS_B( 프로파일, 입력 ): 매 틱
 * agv_drive.obs_b_ext = 그 입력. 둘 다 모드와 상관없이 매 틱.
 * MAN_START_SW( 프로파일, 입력 ): 그 입력이 켜져 있으면 START 스위치가 눌린 것과 똑같이 본다(본체 START 와 OR, 줄이 여럿이면 모두 OR, manual_tick 이 넘겨 한 틱 늦다) */
#define MANUAL_BEGIN(name)  const man_row_t name[] = {
#define MANUAL_END          };
/* 프로파일 1~63(압축 태그 표의 프로파일 칸 6비트와 같은 범위) 또는 묶음 PROFILE_SET( 0~15 ) */
#define DSL_PSET(s)         ((((s) & ~15) == MAN_PSET) ? 1 : 0)
#define DSL_MPF(p)          ((uint16_t)(p) + (0 * sizeof(char[((((p) >= 1) && ((p) <= 63)) || DSL_PSET(p)) ? 1 : -1])))
#define PROFILE_SET(k)      (MAN_PSET + (k) + 0 * sizeof(char[(((k) >= 0) && ((k) <= 15)) ? 1 : -1]))
#define DSL_PLO(p)          ((((p) < 32) ? (1UL << ((p) & 31)) : 0UL) + 0 * sizeof(char[(((p) >= 1) && ((p) <= 63)) ? 1 : -1]))
#define DSL_PHI(p)          (((p) >= 32) ? (1UL << ((p) & 31)) : 0UL)
#define MAN_PROFILES(s, ...) { (uint16_t)((s) + 0 * sizeof(char[DSL_PSET(s) ? 1 : -1])), MAN_K_PROFILES, \
                               DSL_FMASK(DSL_PLO, __VA_ARGS__), DSL_FMASK(DSL_PHI, __VA_ARGS__), 0UL, 0UL },
#define DSL_MM_IDLE         MAN_K_IDLE
#define DSL_MM_JOG          MAN_K_JOG
#define DSL_MM_ANY          MAN_K_ANY
/* 조건 칸은 첫 낱말을 붙여 "키, 입력 조건" 둘로 펼친다 */
#define DSL_MC_IN(...)      0, IN(__VA_ARGS__)
#define DSL_MC_NO_IN        0, NO_IN
#define DSL_MC_KEY(k)       KEY_##k, NO_IN
/* 출력 값은 조건과 같은 64비트: 하위 32비트 켬, 상위 32비트 끔 */
#define DSL_MWSET(set, clr) ((dsl_cond_t)(uint32_t)(set) | ((dsl_cond_t)(uint32_t)(clr) << 32))
#define DSL_MW1(x)          ((dsl_cond_t)(x))
#define DSL_MW(...)         DSL_FMASK(DSL_MW1, __VA_ARGS__)
#define BELT(n, d)              DSL_MBELT_##d(BELT##n##_CW, BELT##n##_CCW)
#define DSL_MBELT_CW(cw, ccw)   DSL_MWSET((cw) + DSL_BELT_OK(cw), (ccw) & ~(uint32_t)(cw))
#define DSL_MBELT_CCW(cw, ccw)  DSL_MWSET((ccw) + DSL_BELT_OK(ccw), (cw) & ~(uint32_t)(ccw))
#define DSL_MBELT_OFF(cw, ccw)  DSL_MWSET(0UL, (cw) | (ccw))
#define DSL_MROW_(...)      DSL_MROW(__VA_ARGS__)
#define DSL_MROW(pf, kind, key, c, w)                                                                 \
    { DSL_MPF(pf), (uint16_t)((uint16_t)(kind) | ((uint16_t)(key) << 8)) + DSL_NOX(DSL_COND_ON(w) & DSL_COND_OFF(w)), \
      DSL_COND_ON(c), DSL_COND_OFF(c), DSL_COND_ON(w), DSL_COND_OFF(w) },
#define MAN(pf, mode, cond, ...)    DSL_MROW_(pf, DSL_MM_##mode, DSL_MC_##cond, DSL_MW(__VA_ARGS__))
#define MAN_ELSE(pf, mode, ...)     DSL_MROW_(pf, DSL_MM_##mode | MAN_F_ELSE, 0, NO_IN, DSL_MW(__VA_ARGS__))
#define MAN_FB_SW(pf, pin)   { DSL_MPF(pf), MAN_K_FB_SW, 0UL, 0UL, (uint32_t)(pin) + DSL_ID_OK(pin), 0UL },
#define MAN_OBS_B(pf, pin)   { DSL_MPF(pf), MAN_K_OBS_B, 0UL, 0UL, (uint32_t)(pin) + DSL_ID_OK(pin), 0UL },
#define MAN_START_SW(pf, pin) { DSL_MPF(pf), MAN_K_START_SW, 0UL, 0UL, (uint32_t)(pin) + DSL_ID_OK(pin), 0UL },

/* ---- 태그 표 ---- */
#define TAG_TABLE_BEGIN(name)  const tag_rule_t name[] = {
#define TAG_TABLE_END          };
#define ANY_PROFILE            0
/* 위에서 아래로 첫 번째로 맞는 규칙 하나만 실행. 액션은 최대 4칸 */
/* 동작 칸은 값이 4 개까지다(AGV_TURN·ACS_SEND 는 2 칸, 값 개수를 센다). 넘으면 컴파일 오류(검사값은 태그 번호 칸에 더한다) */
#define DSL_ACTS_OK(acts)      (0 * sizeof(char[(DSL_NARG(DSL_UNPAREN acts) <= 4) ? 1 : -1]))
#define TAG(tag, profile, ...)  { (uint16_t)((tag) + DSL_ACTS_OK((__VA_ARGS__))), (uint16_t)(profile), TAG_COND_NONE, 0, 0, { __VA_ARGS__ } },
/* cond: AND( … ) 하나(기록 1값), 또는 IF_* 하나(확장 조건 둘과 입력 조건 하나를 AND 한다, 8값). 값 개수로 가린다.
 * 맨 X_…(3값)는 받지 않는다: 둘째 X_ 가 동작 칸으로 들어가 실행되는 조용한 오류가 된다(상태 조건 하나는 AND( BACKWARD ) 나 IF_X( X_… )).
 * 다른 개수는 DSL_TAGC? 가 없어 컴파일 오류 */
#define TAG_IF(tag, profile, cond, ...)  DSL_TAGIF_(tag, profile, (__VA_ARGS__), cond)
#define DSL_TAGIF_(t, p, acts, ...)      DSL_BPASTE(DSL_TAGC, DSL_NARG(__VA_ARGS__))(t, p, acts, __VA_ARGS__)
#define DSL_TAGC1(t, p, acts, rec)       DSL_APPLY(DSL_TAG_, t, p, DSL_TG_##rec, acts)
#define DSL_TAGC8(t, p, acts, ...)       DSL_TAG_(t, p, __VA_ARGS__, acts)
#define DSL_TAG_(t, p, k1, a1, b1, k2, a2, b2, on, off, acts)                                         \
    { (uint16_t)((t) + DSL_ACTS_OK(acts)), (uint16_t)(p), (uint16_t)(k1), (uint16_t)(a1), (uint16_t)(b1), { DSL_UNPAREN acts }, \
      (uint16_t)(k2), (uint16_t)(a2), (uint16_t)(b2), (uint32_t)(on), (uint32_t)(off) },
/* 범위 규칙: 한 규칙이 태그 lo~hi 를 받는다(lo <= hi <= 1023). 고르는 순서는 다른 규칙과 같다(표에 적힌 순서, 먼저 맞는 하나) */
#define DSL_TRANGE_OK(lo, hi)  (0 * sizeof(char[(((lo) >= 0) && ((lo) <= (hi)) && ((hi) <= 1023)) ? 1 : -1]))
#define TAG_RANGE(lo, hi, profile, ...)  \
    { (uint16_t)((lo) + DSL_ACTS_OK((__VA_ARGS__))), (uint16_t)(profile), TAG_COND_NONE, 0, 0, { __VA_ARGS__ }, 0, 0, 0, 0UL, 0UL, (uint16_t)((hi) + DSL_TRANGE_OK(lo, hi)) },
#define TAG_RANGE_IF(lo, hi, profile, cond, ...)  DSL_TAGRIF_(lo, hi, profile, (__VA_ARGS__), cond)
#define DSL_TAGRIF_(lo, hi, p, acts, ...)    DSL_BPASTE(DSL_TAGRC, DSL_NARG(__VA_ARGS__))(lo, hi, p, acts, __VA_ARGS__)
#define DSL_TAGRC1(lo, hi, p, acts, rec)     DSL_APPLY(DSL_TAGR_, lo, hi, p, DSL_TG_##rec, acts)
#define DSL_TAGRC8(lo, hi, p, acts, ...)     DSL_TAGR_(lo, hi, p, __VA_ARGS__, acts)
#define DSL_TAGR_(lo, hi, p, k1, a1, b1, k2, a2, b2, on, off, acts)                                    \
    { (uint16_t)((lo) + DSL_ACTS_OK(acts)), (uint16_t)(p), (uint16_t)(k1), (uint16_t)(a1), (uint16_t)(b1), { DSL_UNPAREN acts }, \
      (uint16_t)(k2), (uint16_t)(a2), (uint16_t)(b2), (uint32_t)(on), (uint32_t)(off), (uint16_t)((hi) + DSL_TRANGE_OK(lo, hi)) },

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
/* 범위 규칙이 있는 압축 표: TAG_RANGE_P( 규칙 번호, hi ) 를 규칙 번호 순서로. 그 규칙의 TAGP 태그가 lo 다 */
#define TAG_RANGES_BEGIN(name) static const tag_range_t name[] = {
#define TAG_RANGE_P(i, hi)     { (uint16_t)(i), (uint16_t)(hi) },
#define TAG_PACKED_R(name, rules, x, in, act, rng)  \
    const tag_packed_t name = { rules, x, in, act, (uint16_t)(sizeof(rules) / sizeof(rules[0])), rng, (uint16_t)(sizeof(rng) / sizeof(rng[0])) };

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
#define IF_TAG_NUM(e)                    IF_X(X_TAG_NUM(e))
/* 이름은 붙여서 넘긴다(사이트 매크로가 펼쳐지지 않게) */
#define IF_PARAM(name, v)                IF_X(DSL_XPRM(DSL_PRM_##name, v))

/* 태그 동작 이름(AGV_STOP, AGV_TURN, SEQ_START, ACS_SEND …)은 agv/tag_dsl.h. 태그 표 파일은 그 헤더를 포함한다 */

#endif /* AGV_SEQ_DSL_H */
