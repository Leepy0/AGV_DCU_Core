/*
 * dsl_and.h - AND( … ) 조건 틀. seq_dsl.h 가 포함한다(사이트 파일은 직접 포함하지 않는다).
 * 사용법과 낱말 대응표: docs/03_시퀀스_DSL.md 2.5절
 *
 * AND( 항목, … ) 한 덩이가 입력 조건과 상태 조건을 아무 순서로 받는다. 항목은 다음 셋이다.
 *   입력  ON( … ) OFF( … ) ANY( … )           ANY 는 최대 2개: 첫째는 입력 조건 안(ANY_ON 과 같다), 둘째는 따로 둔다
 *   상태  FORWARD BACKWARD FORBACK_MODE LOW_BATTERY ID( lo, hi ) STATION( k, lo, hi ) STATION_MOD( k, n, m ) STATION_DIV( k, n, m )
 *         TAG_IS( 식 ) PARAM( FND, v ) PARAM_MIN( FND, v ) PARAM_MAX( FND, v ) CROSS_IN CROSS_OUT CROSS_IN_WAIT CROSS_OUT_WAIT
 *         ERR_IS( n ) KEY( k ) STATE( 칸, 값 ) STATE( 칸, 처음, 끝 ) ALWAYS NOT( 상태 )
 *   입력 묶음 IN( … ) 도 항목으로 받는다
 * 낱말은 매크로가 아니라 불활성 머리다(붙이기로 고른다). 사이트가 같은 이름의 매크로(#define FORWARD 1 등)를 정의하면 깨진다.
 *
 * 구현: 항목 하나 -> 기록 (c, m, p) 셋.
 *   c 입력 조건(64비트, 하위 켬·상위 끔)  m 이 항목의 ANY 마스크(32비트, 없으면 0)  p 상태 하나(코드 | a << 16 | b << 32, 없으면 0)
 * AND 는 기록들을 접어 DSL_ANDREC( 상태1 셋, 상태2 셋, 입력 조건, 둘째 ANY 마스크 ) 하나로 만든다(8값). 받는 쪽(소비자)이
 * DSL_SQ_(시퀀스), DSL_TG_(태그), DSL_CN_(입력만), DSL_MN_(수동), DSL_FK_(점멸) 로 자기 칸에 맞게 풀고, 칸에 안 들어가는 조합은
 * 컴파일 오류(음수 크기 배열)로 막는다.
 */
#ifndef AGV_DSL_AND_H
#define AGV_DSL_AND_H

/* ---- 괄호 탐침: x 가 "(" 로 시작하면 1, 아니면 0 (x 는 먼저 펼쳐진다) ---- */
#define DSL_CK_(a, b, ...) b
#define DSL_CK(...)        DSL_CK_(__VA_ARGS__, 0, ~)
#define DSL_PP_PROBE(...)  ~, 1,
#define DSL_IS_PAREN(x)    DSL_CK(DSL_PP_PROBE x)
#define DSL_APPLY(m, ...)  m(__VA_ARGS__)
/* 낱말 가리기: x 가 낱말이면 DSL_xxx_낱말 이 "~, 값" 으로 펼쳐져 둘째 값이 뽑히고, 아니면 dflt 가 뽑힌다 */
#define DSL_ARG2_(a, b, ...)     b
#define DSL_ARG2(...)            DSL_ARG2_(__VA_ARGS__, ~)

/* ---- ANY: 불활성 머리 + 마스크. 숫자로 바뀌는 ON·OFF 와 달리 낱말로 남는다 ---- */
#define ANY(...)          DSL_ANYTOK(DSL_MASK(__VA_ARGS__))

/* ---- 항목 한 개 -> 기록 (c, m, p) ---- */
#define DSL_PKV(k, a, b)  (((unsigned long long)(uint16_t)(k)) | (((unsigned long long)(uint16_t)(a)) << 16) | (((unsigned long long)(uint16_t)(b)) << 32))
#define DSL_RS(...)       DSL_RS_(__VA_ARGS__)
#define DSL_RS_(k, a, b)  (0ULL, 0UL, DSL_PKV(k, a, b))
#define DSL_AI(i)         DSL_CAT(DSL_AI_, DSL_IS_PAREN(i))(i)
/* 괄호로 시작하는 항목은 입력 조건(ON·OFF·IN)이어야 한다: 64비트 식이 아니면(사이트가 FORWARD 를 (1) 로 정의한 경우 등) 오류 */
#define DSL_AI_1(i)       (((i) + DSL_COND64(i)), 0UL, 0ULL)
#define DSL_AI_0(i)       DSL_AIW(i)
#define DSL_AIW(i)        DSL_AX_##i

/* 상태 낱말. 오른쪽은 예전 X_ 이름 */
#define DSL_AX_DSL_ANYTOK(m)  (0ULL, (m), 0ULL)
#define DSL_AX_ALWAYS         DSL_RS(X_ALWAYS)
#define DSL_AX_FORWARD        DSL_RS(X_FORWARD)
#define DSL_AX_BACKWARD       DSL_RS(X_BACKWARD)
#define DSL_AX_FORBACK_MODE   DSL_RS(X_FORBACK_MODE)
#define DSL_AX_LOW_BATTERY    DSL_RS(X_LOW_BATTERY)
#define DSL_AX_CROSS_IN       DSL_RS(X_CROSS_IN)
#define DSL_AX_CROSS_OUT      DSL_RS(X_CROSS_OUT)
#define DSL_AX_CROSS_IN_WAIT  DSL_RS(X_CROSS_IN_ON)
#define DSL_AX_CROSS_OUT_WAIT DSL_RS(X_CROSS_OUT_ON)
#define DSL_AX_ID(lo, hi)           DSL_RS(X_ID(lo, hi))
#define DSL_AX_DSL_STI(k, lo, hi)   DSL_RS(X_STATION(DSL_STK(k), lo, hi))
#define DSL_AX_STATION_MOD(k, n, m) DSL_RS(X_STATION_MOD(DSL_STK(k), n, m))
#define DSL_AX_STATION_DIV(k, n, m) DSL_RS(X_STATION_DIV(DSL_STK(k), n, m))
/* 스테이션 큐 칸 k 는 0~ACS_STATION_MAX-1(밖이면 acs_station_get 이 늘 0 을 돌려 조건이 조용히 거짓이 된다) */
#define DSL_STK(k)                  ((k) + (0 * sizeof(char[(((k) >= 0) && ((k) < ACS_STATION_MAX)) ? 1 : -1])))
#define DSL_AX_TAG_IS(e)            DSL_RS(X_TAG_NUM(e))
#define DSL_AX_ERR_IS(n)            DSL_RS(X_ERR_EQ(n))
#define DSL_AX_KEY(k)               DSL_RS(X_KEY(k))
#define DSL_AX_PARAM(name, v)       DSL_RS(DSL_XPRM(DSL_PRM_##name, v))
#define DSL_AX_PARAM_MIN(name, v)   DSL_RS(DSL_XPRMC(DSL_PRM_##name, v, XC_PARAM_GE))
#define DSL_AX_PARAM_MAX(name, v)   DSL_RS(DSL_XPRMC(DSL_PRM_##name, v, XC_PARAM_LE))
/* 사이트 상태 칸(SET_STATE 와 같은 칸, v0.61.0): STATE( 칸, 값 ) 같음, STATE( 칸, 처음, 끝 ) 범위(부호 없는 16비트). 칸 0~15,
 * 값 0~65535, 처음 <= 끝(밖은 컴파일 오류). "> 0" 은 STATE( 칸, 1, 65535 ) */
#define DSL_AX_STATE(...)           DSL_BPASTE(DSL_AXST_, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define DSL_AXST_2(k, v)            DSL_AXST_3(k, v, v)
#define DSL_AXST_3(k, lo, hi)       DSL_RS((XC_STATE | DSL_SLOT(k)), (lo) + DSL_STATE_OK(k, lo, hi), hi)
#define DSL_STATE_OK(k, lo, hi)     (0 * sizeof(char[(((k) >= 0) && ((k) < SEQ_STATE_COUNT) && ((lo) >= 0) && ((lo) <= (hi)) && ((hi) <= 0xFFFFL)) ? 1 : -1]))
/* PARAM_MIN·MAX 는 자리 없는 파라미터만 받는다(자릿수 파라미터의 한 자리 TFB_1 등은 컴파일 오류) */
#define DSL_XPRMC(...)            DSL_XPRMC_(__VA_ARGS__)
#define DSL_XPRMC_(p, place, v, kind)  ((kind) | DSL_SLOT(p)), (v), (0 * sizeof(char[((place) == 0) ? 1 : -1]))
/* NOT 은 상태 낱말에만(입력 항목에 쓰면 붙이기 오류). NOT( NOT( … ) ) 는 부정이 겹쳐 같은 뜻이 안 되니 오류 */
#define DSL_AX_NOT(x)        DSL_RNOT(DSL_AX_##x)
#define DSL_RNOT(r)          DSL_RNOT_ r
#define DSL_RNOT_(c, m, p)   (c, m, (((p) | (unsigned long long)XC_NOT) + (0 * sizeof(char[(((p) & (unsigned long long)XC_NOT) == 0) ? 1 : -1]))))
#define DSL_RC_(c, m, p)  c
#define DSL_RM_(c, m, p)  m
#define DSL_RP_(c, m, p)  p
#define DSL_RC(r)         DSL_RC_ r
#define DSL_RM(r)         DSL_RM_ r
#define DSL_RP(r)         DSL_RP_ r

/* ---- 접기 도구: 첫째·둘째로 0 이 아닌 값, 0 이 아닌 값의 개수 ---- */
#define DSL_F1(a1) (a1)
#define DSL_S1(a1) 0ULL
#define DSL_C1(a1) (!!(a1))
#define DSL_F2(a1, a2) ((a1) ? (a1) : DSL_F1(a2))
#define DSL_S2(a1, a2) ((a1) ? DSL_F1(a2) : DSL_S1(a2))
#define DSL_C2(a1, a2) (!!(a1) + !!(a2))
#define DSL_F3(a1, a2, a3) ((a1) ? (a1) : DSL_F2(a2, a3))
#define DSL_S3(a1, a2, a3) ((a1) ? DSL_F2(a2, a3) : DSL_S2(a2, a3))
#define DSL_C3(a1, a2, a3) (!!(a1) + !!(a2) + !!(a3))
#define DSL_F4(a1, a2, a3, a4) ((a1) ? (a1) : DSL_F3(a2, a3, a4))
#define DSL_S4(a1, a2, a3, a4) ((a1) ? DSL_F3(a2, a3, a4) : DSL_S3(a2, a3, a4))
#define DSL_C4(a1, a2, a3, a4) (!!(a1) + !!(a2) + !!(a3) + !!(a4))
#define DSL_F5(a1, a2, a3, a4, a5) ((a1) ? (a1) : DSL_F4(a2, a3, a4, a5))
#define DSL_S5(a1, a2, a3, a4, a5) ((a1) ? DSL_F4(a2, a3, a4, a5) : DSL_S4(a2, a3, a4, a5))
#define DSL_C5(a1, a2, a3, a4, a5) (!!(a1) + !!(a2) + !!(a3) + !!(a4) + !!(a5))
#define DSL_F6(a1, a2, a3, a4, a5, a6) ((a1) ? (a1) : DSL_F5(a2, a3, a4, a5, a6))
#define DSL_S6(a1, a2, a3, a4, a5, a6) ((a1) ? DSL_F5(a2, a3, a4, a5, a6) : DSL_S5(a2, a3, a4, a5, a6))
#define DSL_C6(a1, a2, a3, a4, a5, a6) (!!(a1) + !!(a2) + !!(a3) + !!(a4) + !!(a5) + !!(a6))
#define DSL_F7(a1, a2, a3, a4, a5, a6, a7) ((a1) ? (a1) : DSL_F6(a2, a3, a4, a5, a6, a7))
#define DSL_S7(a1, a2, a3, a4, a5, a6, a7) ((a1) ? DSL_F6(a2, a3, a4, a5, a6, a7) : DSL_S6(a2, a3, a4, a5, a6, a7))
#define DSL_C7(a1, a2, a3, a4, a5, a6, a7) (!!(a1) + !!(a2) + !!(a3) + !!(a4) + !!(a5) + !!(a6) + !!(a7))

/* ---- AND: 항목 1~7 개 ---- */
#define DSL_ANYB(m)       ((((dsl_cond_t)(m))) | (((dsl_cond_t)(m)) << 32))
#define DSL_CHK_LE(c, n)  (0 * sizeof(char[((c) <= (n)) ? 1 : -1]))
/* 상태 항목 최대 2개(p1·p2, cp 개수), 입력 ANY 최대 2개(첫째는 cond 안, 둘째 m2, cm 개수) */
/* 입력 항목(ON·OFF·IN)을 합친 plain 의 양쪽 절반에 같이 선 비트 = 그 안의 ANY_ON. ANY( … ) 항목이 따로 있으면 둘이 한 묶음으로 합쳐져
 * 조건이 조용히 약해지므로 오류(둘째 묶음이 필요하면 ANY 둘로 쓴다) */
#define DSL_ANYMERGE(plain, cm) (0 * sizeof(char[(((cm) == 0) || ((((plain) >> 32) & (plain) & 0xFFFFFFFFULL) == 0)) ? 1 : -1]))
#define DSL_ANDOUT(plain, cond, m2, cm, p1, p2, cp) \
    DSL_ANDREC( (uint16_t)((p1) & 0xFFFF), (uint16_t)(((p1) >> 16) & 0xFFFF), (uint16_t)(((p1) >> 32) & 0xFFFF), \
                (uint16_t)((p2) & 0xFFFF), (uint16_t)(((p2) >> 16) & 0xFFFF), (uint16_t)(((p2) >> 32) & 0xFFFF), \
                (cond), (uint32_t)(m2) + DSL_CHK_LE(cm, 2) + DSL_CHK_LE(cp, 2) + DSL_ANYMERGE(plain, cm) )
#define DSL_AND1(i1) DSL_ANDM1(DSL_AI(i1))
#define DSL_ANDM1(r1) DSL_ANDOUT( (DSL_RC(r1)), DSL_IN2(DSL_RC(r1), DSL_ANYB(DSL_F1(DSL_RM(r1)))), DSL_S1(DSL_RM(r1)), DSL_C1(DSL_RM(r1)), \
                                  DSL_F1(DSL_RP(r1)), DSL_S1(DSL_RP(r1)), DSL_C1(DSL_RP(r1)) )
#define DSL_AND2(i1, i2) DSL_ANDM2(DSL_AI(i1), DSL_AI(i2))
#define DSL_ANDM2(r1, r2) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2)), DSL_IN3(DSL_RC(r1), DSL_RC(r2), DSL_ANYB(DSL_F2(DSL_RM(r1), DSL_RM(r2)))), DSL_S2(DSL_RM(r1), DSL_RM(r2)), DSL_C2(DSL_RM(r1), DSL_RM(r2)), \
                                  DSL_F2(DSL_RP(r1), DSL_RP(r2)), DSL_S2(DSL_RP(r1), DSL_RP(r2)), DSL_C2(DSL_RP(r1), DSL_RP(r2)) )
#define DSL_AND3(i1, i2, i3) DSL_ANDM3(DSL_AI(i1), DSL_AI(i2), DSL_AI(i3))
#define DSL_ANDM3(r1, r2, r3) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2) | DSL_RC(r3)), DSL_IN4(DSL_RC(r1), DSL_RC(r2), DSL_RC(r3), DSL_ANYB(DSL_F3(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3)))), DSL_S3(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3)), DSL_C3(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3)), \
                                  DSL_F3(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3)), DSL_S3(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3)), DSL_C3(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3)) )
#define DSL_AND4(i1, i2, i3, i4) DSL_ANDM4(DSL_AI(i1), DSL_AI(i2), DSL_AI(i3), DSL_AI(i4))
#define DSL_ANDM4(r1, r2, r3, r4) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2) | DSL_RC(r3) | DSL_RC(r4)), DSL_IN5(DSL_RC(r1), DSL_RC(r2), DSL_RC(r3), DSL_RC(r4), DSL_ANYB(DSL_F4(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4)))), DSL_S4(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4)), DSL_C4(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4)), \
                                  DSL_F4(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4)), DSL_S4(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4)), DSL_C4(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4)) )
#define DSL_AND5(i1, i2, i3, i4, i5) DSL_ANDM5(DSL_AI(i1), DSL_AI(i2), DSL_AI(i3), DSL_AI(i4), DSL_AI(i5))
#define DSL_ANDM5(r1, r2, r3, r4, r5) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2) | DSL_RC(r3) | DSL_RC(r4) | DSL_RC(r5)), DSL_IN6(DSL_RC(r1), DSL_RC(r2), DSL_RC(r3), DSL_RC(r4), DSL_RC(r5), DSL_ANYB(DSL_F5(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5)))), DSL_S5(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5)), DSL_C5(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5)), \
                                  DSL_F5(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5)), DSL_S5(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5)), DSL_C5(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5)) )
#define DSL_AND6(i1, i2, i3, i4, i5, i6) DSL_ANDM6(DSL_AI(i1), DSL_AI(i2), DSL_AI(i3), DSL_AI(i4), DSL_AI(i5), DSL_AI(i6))
#define DSL_ANDM6(r1, r2, r3, r4, r5, r6) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2) | DSL_RC(r3) | DSL_RC(r4) | DSL_RC(r5) | DSL_RC(r6)), DSL_IN7(DSL_RC(r1), DSL_RC(r2), DSL_RC(r3), DSL_RC(r4), DSL_RC(r5), DSL_RC(r6), DSL_ANYB(DSL_F6(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6)))), DSL_S6(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6)), DSL_C6(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6)), \
                                  DSL_F6(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6)), DSL_S6(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6)), DSL_C6(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6)) )
#define DSL_AND7(i1, i2, i3, i4, i5, i6, i7) DSL_ANDM7(DSL_AI(i1), DSL_AI(i2), DSL_AI(i3), DSL_AI(i4), DSL_AI(i5), DSL_AI(i6), DSL_AI(i7))
#define DSL_ANDM7(r1, r2, r3, r4, r5, r6, r7) DSL_ANDOUT( (DSL_RC(r1) | DSL_RC(r2) | DSL_RC(r3) | DSL_RC(r4) | DSL_RC(r5) | DSL_RC(r6) | DSL_RC(r7)), DSL_IN8(DSL_RC(r1), DSL_RC(r2), DSL_RC(r3), DSL_RC(r4), DSL_RC(r5), DSL_RC(r6), DSL_RC(r7), DSL_ANYB(DSL_F7(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6), DSL_RM(r7)))), DSL_S7(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6), DSL_RM(r7)), DSL_C7(DSL_RM(r1), DSL_RM(r2), DSL_RM(r3), DSL_RM(r4), DSL_RM(r5), DSL_RM(r6), DSL_RM(r7)), \
                                  DSL_F7(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6), DSL_RP(r7)), DSL_S7(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6), DSL_RP(r7)), DSL_C7(DSL_RP(r1), DSL_RP(r2), DSL_RP(r3), DSL_RP(r4), DSL_RP(r5), DSL_RP(r6), DSL_RP(r7)) )
#define AND(...)          DSL_CAT(DSL_AND, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)

/* ---- 소비자: 기록 DSL_ANDREC( k1, a1, b1, k2, a2, b2, c, m2 ) -> 칸 ---- */
/* 시퀀스 WHEN·WAIT·WAIT_TO: xk, xa, xb, c, any2. 상태는 하나(둘째가 있으면 오류). 상태가 있으면 둘째 ANY 는 못 쓴다
 * (해석기는 둘 다 받지만 표 편집기 사양과 같게 막는다: 상태 낱말과 둘째 ANY 는 한 조건에 함께 쓰지 않는다) */
#define DSL_SQ_DSL_ANDREC(k1, a1, b1, k2, a2, b2, c, m2) k1, a1, b1, c, ((m2) + (0 * sizeof(char[((k2) == 0) ? 1 : -1])) \
                                                          + (0 * sizeof(char[(((k1) == 0) || ((m2) == 0)) ? 1 : -1])))
/* 태그: k1 a1 b1 k2 a2 b2 on off. 상태는 둘까지, 둘째 ANY 는 오류(태그 규칙에 둘 칸이 없다) */
#define DSL_TG_DSL_ANDREC(k1, a1, b1, k2, a2, b2, c, m2) k1, a1, b1, k2, a2, b2, (DSL_COND_ON(c) + (0 * sizeof(char[((m2) == 0) ? 1 : -1]))), DSL_COND_OFF(c)
/* 입력 조건만 받는 칸(WAIT_STABLE·ERROR_HOLD·파라미터 시간 대기): 상태·둘째 ANY 가 있으면 오류. IN( … ) 꼴은 그대로 */
#define DSL_CN_DSL_ANDREC(k1, a1, b1, k2, a2, b2, c, m2) \
    ((c) + (0 * sizeof(char[((k1) == 0) && ((m2) == 0) ? 1 : -1])))
#define DSL_CN(c)         DSL_CAT(DSL_CNP_, DSL_IS_PAREN(c))(c)
#define DSL_CNP_1(c)      c
#define DSL_CNP_0(c)      DSL_CN_##c
/* 수동 표 조건 칸: 키(KEY 하나) + 입력 조건. 키 말고 다른 상태·둘째 상태(KEY 둘 포함)·둘째 ANY 는 오류. 칸 둘 "키, 입력 조건" */
#define DSL_MN_DSL_ANDREC(k1, a1, b1, k2, a2, b2, c, m2) \
    ((k1) ? (a1) : 0) + (0 * sizeof(char[((((k1) == 0) || ((k1) == XC_KEY)) && ((k2) == 0)) ? 1 : -1])), \
    ((c) + (0 * sizeof(char[((m2) == 0) ? 1 : -1])))
#define DSL_MC_AND(...)   DSL_CAT(DSL_MN_, AND(__VA_ARGS__))
/* 점멸(seq_dsl.h DSL_FLK_0): k1 이 0 이면 예전 칸 배치, 아니면 상태 인자가 arg0·arg1 을 쓰므로 주기·출력을 out 에 넣는다(seq.c flk_cells).
 * 둘째 상태·둘째 ANY 는 오류. 값: xk, xa, xb, 입력 조건, 검사값(0) */
#define DSL_FK_DSL_ANDREC(k1, a1, b1, k2, a2, b2, c, m2) \
    k1, a1, b1, c, ((m2) + (0 * sizeof(char[((k2) == 0) ? 1 : -1])) + (0 * sizeof(char[((m2) == 0) ? 1 : -1])))

#endif /* AGV_DSL_AND_H */
