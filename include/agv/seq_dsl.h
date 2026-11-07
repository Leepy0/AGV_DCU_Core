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
#include "agv/tagact.h"
#include "agv/io.h"

/* ---- 내부 도구 ---- */
#define DSL_CAT_(a, b)   a##b
#define DSL_CAT(a, b)    DSL_CAT_(a, b)
#define DSL_NARG_(a, b, c, d, e, f, g, h, n, ...) n
#define DSL_NARG(...)    DSL_NARG_(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0)

/* ID 가 0~31 또는 IO_NONE 이 아니면 컴파일 오류 (앰프 신호를 조건에 쓰는 경우 등) */
#define DSL_ID_OK(x)     (0 * sizeof(char[(((x) < 32) || ((x) == IO_NONE)) ? 1 : -1]))
#define DSL_BIT(x)       ((((x) == IO_NONE) ? 0UL : (1UL << ((x) & 31))) + DSL_ID_OK(x))

#define DSL_MASK1(a)                 DSL_BIT(a)
#define DSL_MASK2(a, b)              (DSL_BIT(a) | DSL_BIT(b))
#define DSL_MASK3(a, b, c)           (DSL_MASK2(a, b) | DSL_BIT(c))
#define DSL_MASK4(a, b, c, d)        (DSL_MASK3(a, b, c) | DSL_BIT(d))
#define DSL_MASK5(a, b, c, d, e)     (DSL_MASK4(a, b, c, d) | DSL_BIT(e))
#define DSL_MASK6(a, b, c, d, e, f)  (DSL_MASK5(a, b, c, d, e) | DSL_BIT(f))
#define DSL_MASK7(a, b, c, d, e, f, g)     (DSL_MASK6(a, b, c, d, e, f) | DSL_BIT(g))
#define DSL_MASK8(a, b, c, d, e, f, g, h)  (DSL_MASK7(a, b, c, d, e, f, g) | DSL_BIT(h))
#define DSL_MASK(...)    DSL_CAT(DSL_MASK, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)

/* 조건은 64비트 상수 하나: 하위 32비트 must_on, 상위 32비트 must_off */
typedef unsigned long long dsl_cond_t;
#define DSL_COND_ON(c)   ((uint32_t)(c))
#define DSL_COND_OFF(c)  ((uint32_t)((c) >> 32))
/* 같은 신호를 ON 과 OFF 양쪽에 쓰면 컴파일 오류 */
#define DSL_COND_OK(c)   (0 * sizeof(char[(DSL_COND_ON(c) & DSL_COND_OFF(c)) ? -1 : 1]))

#define DSL_STEP(kind, target, a0, a1, to, on, off, out)                                     \
    { (uint16_t)(kind), (uint16_t)(target), (uint16_t)(a0), (uint16_t)(a1), (uint16_t)(to),  \
      (uint32_t)(on), (uint32_t)(off), (uint32_t)(out) },
#define DSL_STEP_C(kind, target, a0, a1, to, c, out)                                         \
    DSL_STEP(kind, target, a0, a1, to, DSL_COND_ON(c) + DSL_COND_OK(c), DSL_COND_OFF(c), out)

/* ---- 조건 ---- */
#define ON(...)          ((dsl_cond_t)DSL_MASK(__VA_ARGS__))
#define OFF(...)         (((dsl_cond_t)DSL_MASK(__VA_ARGS__)) << 32)
#define DSL_IN1(a)       (a)
#define DSL_IN2(a, b)    ((a) | (b))
#define IN(...)          DSL_CAT(DSL_IN, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)

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
#define PROFILE(profile, type, from, to)  { (uint16_t)(profile), (uint16_t)(type), (uint16_t)(from), (uint16_t)(to) },
#define PROFILE_TABLE_END          };

/* ---- 라벨·분기 ---- */
#define L(label)         DSL_STEP(SK_LABEL, label, 0, 0, 0, 0, 0, 0)
#define GOTO(label)      DSL_STEP(SK_GOTO,  label, 0, 0, 0, 0, 0, 0)

/* SELECT_BEGIN·SELECT_END 는 읽기 위한 표시이며 스텝을 만들지 않는다. 가지 수 제한 없음 */
#define SELECT_BEGIN
#define WHEN(c, label)               DSL_STEP_C(SK_WHEN, label, 0, 0, 0, c, 0)
#define WHEN_STATE(id, val, label)   DSL_STEP(SK_WHEN_STATE, label, id, val, 0, 0, 0, 0)
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
#define BELT_ON(n, dir)                                                             \
    DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0,                                         \
             BELT##n##_##dir + DSL_BELT_OK(BELT##n##_##dir),                        \
             (BELT##n##_CW | BELT##n##_CCW) & ~(uint32_t)(BELT##n##_##dir), 0)
#define BELT_OFF(n)      DSL_STEP(SK_OUT, SEQ_NO_LABEL, 0, 0, 0, 0, BELT##n##_CW | BELT##n##_CCW, 0)

/* ---- 대기 ---- */
#define WAIT(c)                     DSL_STEP_C(SK_WAIT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
#define WAIT_TO(c, t, label)        DSL_STEP_C(SK_WAIT, label, 0, 0, t, c, 0)
#define WAIT_STABLE(c, s)           DSL_STEP_C(SK_WAIT_STABLE, SEQ_NO_LABEL, s, 0, 0, c, 0)
#define WAIT_STABLE_TO(c, s, t, label)  DSL_STEP_C(SK_WAIT_STABLE, label, s, 0, t, c, 0)
#define WAIT_OUT(c)                 DSL_STEP_C(SK_WAIT_OUT, SEQ_NO_LABEL, 0, 0, 0, c, 0)
#define DELAY(t)                    DSL_STEP(SK_DELAY, SEQ_NO_LABEL, t, 0, 0, 0, 0, 0)

/* ---- ACS ---- */
#define DSL_INFO_L 'L'
#define DSL_INFO_U 'U'
#define DSL_INFO_N 'N'
#define DSL_INFO_A 'A'
#define DSL_INFO_F 'F'
#define DSL_INFO_P 'P'
#define DSL_INFO_B 'B'
#define DSL_INFO_C 'C'
#define ACS(info, src)         DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_##src, 0, 0, 0, 0)
#define ACS_N(info, num)       DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_CONST, num, 0, 0, 0)
#define ACS_WAIT_ACK(t, label) DSL_STEP(SK_ACS_WAIT_ACK, label, 0, 0, t, 0, 0, 0)
#define ACS_STATION(idx, val, cnt)  DSL_STEP(SK_ACS_STATION, SEQ_NO_LABEL, idx, val, cnt, 0, 0, 0)
#define STATION_RESET          DSL_STEP(SK_STATION_RESET, SEQ_NO_LABEL, 0, 0, 0, 0, 0, 0)

/* ---- 주행 관련 (ACT 에서 금지, seq_bind 가 부팅 때 거부) ---- */
#define SPEED(level)           DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_##level, 0, 0, 0, 0, 0)
#define SPEED_MMIN(v)          DSL_STEP(SK_SPEED, SEQ_NO_LABEL, v, 0, 0, 0, 0, 0)
#define FORBACK(dir)           DSL_STEP(SK_FORBACK, SEQ_NO_LABEL, DIR_##dir, 0, 0, 0, 0, 0)
#define TURN(dir, deg, fb, autostart, obs) \
    DSL_STEP(SK_TURN, SEQ_NO_LABEL, TURN_##dir, deg, (fb) | ((autostart) << 4) | ((obs) << 8), 0, 0, 0)
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
#define TAG_IF(tag, profile, cond, ...)  { (uint16_t)(tag), (uint16_t)(profile), cond, { __VA_ARGS__ } },

#define IF_FORBACK_MODE        TAG_COND_FORBACK_MODE, 0, 0
#define IF_LOW_BATTERY         TAG_COND_LOW_BATTERY, 0, 0
#define IF_FORWARD             TAG_COND_FORWARD, 0, 0
#define IF_BACKWARD            TAG_COND_BACKWARD, 0, 0
#define IF_STATION(idx, lo, hi)  (uint16_t)(TAG_COND_STATION_RANGE | ((idx) << 8)), (uint16_t)(lo), (uint16_t)(hi)

#define A_STOP                 TA_STOP
#define A_AUTOSTART(sec)       (TA_STOP_AUTOSTART | (sec))
#define A_AUTOSTART_PARAM      TA_STOP_AUTOSTART
#define A_SPEED_HIGH           (TA_SPEED | 0)
#define A_SPEED_LOW            (TA_SPEED | 1)
#define A_SPEED_LLOW           (TA_SPEED | 2)
#define A_SPEED_MIN            (TA_SPEED | 3)
#define A_FORBACK(dir)         (TA_FORBACK | DIR_##dir)
#define A_TURN(dir, deg, fb, autostart, obs) \
    (TA_TURN | TURN_##dir | (((deg) / 90) << 2)), ((fb) | ((autostart) << 4) | ((obs) << 8))
#define A_BRANCH(dir)          (TA_BRANCH | BRANCH_##dir)
#define A_SEQ(id)              (TA_SEQ | (id))
#define A_TAPE_STOP(id)        (TA_TAPE_STOP | (id))
#define A_ACT(id)              (TA_ACT | (id))
#define A_OBS_ON               (TA_OBS | 1)
#define A_OBS_OFF              (TA_OBS | 0)
#define A_STATION_RESET        TA_STATION_RESET
#define A_CALL(idx)            (TA_CALL | (idx))

#endif /* AGV_SEQ_DSL_H */
