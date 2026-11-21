/* test_seq.c - 시퀀스·ACT 인터프리터 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/seq_dsl.h"
#include "agv/acs.h"
#include "agv/run.h"
#include "fakes.h"

#define D agv_drive

#define SEN_A   IN_EXT_1
#define SEN_B   IN_EXT_2
#define SEN_C   IN_EXT_3
#define SEN_D   IN_EXT_4
#define SEN_E   IN_EXT_5
#define O_A     OUT_EXT_1
#define O_B     OUT_EXT_2
#define O_C     OUT_EXT_3
#define BIT(x)  (1UL << (x))

#define BELT1_CW   OUTS(O_A)
#define BELT1_CCW  OUTS(O_A, O_B)
#define BELT2_CW   OUTS(O_C)
#define BELT2_CCW  OUTS(O_C, OUT_EXT_4)
#define BELT3_CW   OUTS(OUT_EXT_5)
#define BELT3_CCW  OUTS(OUT_EXT_5, OUT_EXT_6)
#define BELT4_CW   OUTS(OUT_EXT_9)
#define BELT4_CCW  NO_OUT

enum { PF_ONE = 1, PF_TWO = 2 };
enum { ACT_PULSE = 1, ACT_HOLD, ACT_FAIL, ACT_BAD_FN, ACT_SPARE1, ACT_SPARE2, ACT_SPARE3 };
enum { ST_0 = 0, ST_1 };

SEQ_LABELS( L_RETRY, L_OK, L_ERR, L_A, L_B, L_C, L_LOOP, L_MISSING )

/* 분기, 안정 대기, 오류 유지, 재시도 */
SEQ_BEGIN( s_branch )
    SELECT_BEGIN
        WHEN( IN( ON(SEN_A), OFF(SEN_B) ), L_A )
        WHEN( IN( ON(SEN_B) ),             L_B )
    SELECT_END_TO( 100, L_C )
    L( L_A )
    OUT_ON( O_A )
    WAIT_STABLE( IN( ON(SEN_C) ), 3 )
    OUT_OFF( O_A )
    END_RESTART
    L( L_B )
    OUT_ON( O_B )
    END_STOP
    L( L_C )
    END_ERROR( 123 )
SEQ_END

/* ANY_ON: 4개 중 하나라도 켜지고 SEN_E 꺼짐 */
SEQ_BEGIN( s_any )
    SELECT_BEGIN
        WHEN( IN( ANY_ON(SEN_A, SEN_B, SEN_C, SEN_D), OFF(SEN_E) ), L_A )
    SELECT_END_TO( 100, L_C )
    L( L_A )
    OUT_ON( O_A )
    WAIT( ANY_ON(SEN_B, SEN_C) )
    OUT_ON( O_B )
    END_STOP
    L( L_C )
    END_ERROR( 123 )
SEQ_END

/* 안정 + 불일치 누적 타임아웃, 재진입 때 타이머 0 */
SEQ_BEGIN( s_unmatched )
    L( L_RETRY )
    WAIT_STABLE_TO( IN( OFF(SEN_A) ), 5, UNMATCHED(10), L_ERR )
    END_STOP
    L( L_ERR )
    ERROR_HOLD( 290, IN( ON(IN_START_SW) ), L_RETRY )
SEQ_END

SEQ_BEGIN( s_timing )
    DELAY( 2 )
    OUT_ON( O_A )
    WAIT_TO( IN( ON(SEN_A) ), 3, L_ERR )
    GOTO( L_OK )
    L( L_ERR )
    OUT_ON( O_C )
    L( L_OK )
    END_STOP
SEQ_END

SEQ_BEGIN( s_acs )
    ACS( LOADING, STATION0 )
    ACS( UNLOADING, STATION0_PLUS1 )
    ACS( ARRIVE, AGV_ID )
    ACS_SEND( FINISH, 87 )
    ACS( IDLE, 55 )
    ACS_WAIT_ACK( 5, L_ERR )
    OUT_ON( O_A )
    STATION_RESET
    ACS_STATION( 1, 42, 2 )
    END_STOP
    L( L_ERR )
    OUT_ON( O_B )
    END_STOP
SEQ_END

SEQ_BEGIN( s_drive )
    AGV_SPEED( LOW )
    AGV_SPEED_MMIN( 20 )
    AGV_FORBACK( BACK )
    OBS_OFF
    AGV_TURN( CW, 90, 1, 1, 0 )
    OBS_ON
    SET_STATE( ST_0, 7 )
    WAIT_STATE( ST_1, 3 )
    WHEN_STATE( ST_0, 9, L_ERR )
    WHEN_STATE( ST_0, 7, L_OK )
    L( L_ERR )
    OUT_ON( O_C )
    L( L_OK )
    END_RESTART
SEQ_END

/* 한 스캔 8스텝 상한 */
SEQ_BEGIN( s_budget )
    OUT_ON( OUT_EXT_1 ) OUT_ON( OUT_EXT_2 ) OUT_ON( OUT_EXT_3 ) OUT_ON( OUT_EXT_4 )
    L( L_LOOP )
    OUT_ON( OUT_EXT_5 ) OUT_ON( OUT_EXT_6 ) OUT_ON( OUT_EXT_7 ) OUT_ON( OUT_EXT_8 )
    OUT_ON( OUT_EXT_9 )
    END_STOP
SEQ_END

SEQ_BEGIN( s_outs )
    OUT_PULSE( O_A, 2, 1, 2 )
    OUT_ON( O_C )
SEQ_END

SEQ_BEGIN( s_flicker )
    OUT_FLICKER_UNTIL_IN_TO( O_A, 2, IN( ON(SEN_A) ), 20, L_ERR )
    OUT_ON( O_B )
    END_STOP
    L( L_ERR )
    OUT_ON( O_C )
SEQ_END

SEQ_BEGIN( s_handshake )
    HANDSHAKE( O_A, SEN_A, 5, L_ERR )
    OUT_ON( O_B )
    END_STOP
    L( L_ERR )
    OUT_ON( O_C )
SEQ_END

/* 점멸 끝 조건에 상태 낱말(AND): 칸 배치가 달라진다(주기·출력이 out 한 칸) */
SEQ_BEGIN( s_flicker_st )
    OUT_FLICKER_UNTIL_IN_TO( O_A, 2, AND( ON(SEN_A), FORWARD ), 20, L_ERR )
    OUT_ON( O_B )
    END_STOP
    L( L_ERR )
    OUT_ON( O_C )
SEQ_END

/* AND: 입력(켬·끔·하나라도)과 상태(호기)를 한 조건으로, 파라미터 제한 대기, 대기 보고 */
SEQ_BEGIN( s_and )
    WAIT( AND( ON(SEN_A), ANY(SEN_B, SEN_C), ID(5, 9) ) )
    OUT_ON( O_A )
    WAIT_STABLE( AND( ON(SEN_D), ANY(SEN_B, SEN_C) ), 3 )
    OUT_ON( O_B )
    WAIT_TO( AND( ON(SEN_E), BACKWARD ), 5, L_ERR )
    OUT_ON( O_C )
    END_STOP
    L( L_ERR )
    OUT_ON( OUT_EXT_5 )
    END_STOP
SEQ_END

/* 누적 대기(v0.61.0): 참인 틱만 세고 거짓인 틱에는 멈춘다(지우지 않는다). 제한은 진입부터·거짓 틱만, 조건이 먼저 */
SEQ_BEGIN( s_accum )
    WAIT_ACCUM( IN( ON(SEN_A) ), 5 )
    OUT_ON( O_A )
    WAIT_ACCUM_TO( IN( ON(SEN_B) ), 3, 10, L_ERR )
    OUT_ON( O_B )
    WAIT_ACCUM_TO( IN( ON(SEN_C) ), 2, UNMATCHED(4), L_ERR )
    OUT_ON( O_C )
    END_STOP
    L( L_ERR )
    OUT_ON( OUT_EXT_5 )
    END_STOP
SEQ_END

/* 파라미터 시간 유지 대기(v0.61.0): 한도 = 값 x 단위, 틱마다 다시 읽는다. 음수 0, 65534 로 묶음 */
SEQ_BEGIN( s_hold_param )
    WAIT_STABLE( IN( ON(SEN_A) ), PARAM_TIME( ST, TICK ) )
    OUT_ON( O_A )
    WAIT_STABLE( IN( ON(SEN_C) ), PARAM_TIME( BO, TICK ) )
    OUT_ON( O_C )
    WAIT_ACCUM( IN( ON(SEN_B) ), PARAM_TIME( P_AUTO_START_SEC, SEC ) )
    OUT_ON( O_B )
    END_STOP
SEQ_END

/* 묶음 안 유지 셈(v0.61.0): 앞 가지가 이기고, 유지 가지 둘은 따로 센다(a·b), 제한은 timer */
SEQ_BEGIN( s_when_hold )
    SELECT_BEGIN
        WHEN( IN( ON(SEN_A) ), L_A )
        WHEN_STABLE( IN( ON(SEN_B) ), 3, L_B )
        WHEN_ACCUM( IN( ON(SEN_C) ), PARAM_TIME( ST, TICK ), L_C )
    SELECT_END_TO( 50, L_ERR )
    L( L_A )
    OUT_ON( O_A )
    END_STOP
    L( L_B )
    OUT_ON( O_B )
    END_STOP
    L( L_C )
    OUT_ON( O_C )
    END_STOP
    L( L_ERR )
    OUT_ON( OUT_EXT_5 )
    END_STOP
SEQ_END

/* 셋째 유지 가지는 닿으면 표 오류 */
SEQ_BEGIN( s_when_hold3 )
    SELECT_BEGIN
        WHEN( IN( ON(SEN_D) ), L_A )
        WHEN_STABLE( IN( ON(SEN_A) ), 1, L_A )
        WHEN_STABLE( IN( ON(SEN_B) ), 1, L_A )
        WHEN_ACCUM( IN( ON(SEN_C) ), 1, L_A )
    SELECT_END
    L( L_A )
    OUT_ON( O_A )
    END_STOP
SEQ_END

SEQ_BEGIN( s_standby )
    ACS( STANDBY, 7 )
    ACS( CALL, 9 )
    ACS( STANDBY, STATION(0) + 1 )
    END_STOP
SEQ_END

SEQ_BEGIN( s_wait_out )
    WAIT_OUT( IN( ON(O_A), OFF(O_B) ) )
    END_STOP
SEQ_END

/* 벨트 정지 조건: 셋이 각자 끝 센서에서 서고 모두 서면(출력이 꺼지면) 다음 */
SEQ_BEGIN( s_belts )
    BELT_ON( 1, CW, UNTIL( ON(SEN_A) ) )
    BELT_ON( 2, CCW, UNTIL( ON(SEN_B) ) )
    BELT_ON( 3, CW, UNTIL( IN( ON(SEN_C), OFF(SEN_D, IN_START_SW) ) ) )
    WAIT_BELTS( 1, 2, 3 )
    OUT_ON( OUT_EXT_7 )
    END_STOP
SEQ_END

/* 조건을 건 채 BELT_OFF, 같은 벨트를 다시 BELT_ON(조건 지움), 조건을 건 채 대기·중단 */
SEQ_BEGIN( s_belt_off )
    BELT_ON( 1, CW, UNTIL( ON(SEN_A) ) )
    BELT_ON( 2, CW, UNTIL( ON(SEN_B) ) )
    BELT_OFF( 1 )
    BELT_ON( 2, CCW )
    WAIT( IN( ON(SEN_E) ) )
    BELT_ON( 3, CW, UNTIL( ON(SEN_C) ) )
    WAIT( IN( ON(SEN_D) ) )
    END_STOP
SEQ_END

/* 조건을 건 채 끝난다 */
SEQ_BEGIN( s_belt_end )
    BELT_ON( 1, CW, UNTIL( ON(SEN_A) ) )
    END_STOP
SEQ_END

/* 칸(SEQ_BELTS 3)이 모자라면 표 오류 */
SEQ_BEGIN( s_belt_many )
    BELT_ON( 1, CW, UNTIL( ON(SEN_A) ) )
    BELT_ON( 2, CW, UNTIL( ON(SEN_A) ) )
    BELT_ON( 3, CW, UNTIL( ON(SEN_A) ) )
    BELT_ON( 4, CW, UNTIL( ON(SEN_A) ) )
    END_STOP
SEQ_END
ACT_BEGIN( a_bad_belt )
    BELT_ON( 1, CW, UNTIL( ON(SEN_A) ) )
ACT_END

/* 확장 조건: 방향, 호기 + 입력 */
SEQ_BEGIN( s_xwhen )
    SELECT_BEGIN
        WHEN_IF( X_BACKWARD, L_A )
        WHEN_IF_IN( X_ID(5, 9), IN( ON(SEN_A) ), L_B )
    SELECT_END
    L( L_A )
    OUT_ON( O_A )
    END_STOP
    L( L_B )
    OUT_ON( O_B )
    END_STOP
SEQ_END

/* 교차로·부정·오류·키·파라미터, 제한 시간 */
SEQ_BEGIN( s_xwait )
    WAIT_IF( X_CROSS_IN )
    OUT_ON( O_A )
    WAIT_IF( X_NOT(X_CROSS_IN_ON) )
    OUT_ON( O_B )
    WAIT_IF_IN( X_ERR_EQ(281), IN( OFF(SEN_A) ) )
    WAIT_IF( X_KEY(UP) )
    WAIT_IF_TO( X_PARAM_EQ(P_STATION_NO, 5), 3, L_ERR )
    OUT_ON( O_C )
    END_STOP
    L( L_ERR )
    OUT_ON( OUT_EXT_5 )
    END_STOP
SEQ_END

/* (A||B) && (C||D) */
SEQ_BEGIN( s_any2 )
    WAIT_ANY2( IN( ANY_ON(SEN_A, SEN_B) ), ANY2_ON(SEN_C, SEN_D) )
    OUT_ON( O_A )
    SELECT_BEGIN
        WHEN_ANY2( IN( ANY_ON(SEN_A, SEN_B), OFF(SEN_E) ), ANY2_ON(SEN_C, SEN_D), L_A )
    SELECT_END
    L( L_A )
    OUT_ON( O_B )
    END_STOP
SEQ_END

/* 파라미터 분 단위: 16비트 틱을 넘는다 */
SEQ_BEGIN( s_dparam )
    DELAY_PARAM( P_AUTO_START_SEC, MIN )
    OUT_ON( O_A )
    END_STOP
SEQ_END

SEQ_BEGIN( s_tparam )
    DELAY_PARAM( P_STATION_NO, TICK )
    OUT_ON( O_A )
    WAIT_TO_PARAM( IN( ON(SEN_A) ), P_AUTO_START_SEC, SEC, L_ERR )
    OUT_ON( O_B )
    END_STOP
    L( L_ERR )
    OUT_ON( O_C )
    END_STOP
SEQ_END

SEQ_BEGIN( s_nowait )
    AGV_TURN( CCW, 90, 0, 1, 0, NOWAIT )
    OUT_ON( O_A )
    ACS( LOADING, STATION(2) )
    END_STOP
SEQ_END

static int call_count, call_ret;
static int custom_fn(seq_ctx_t *c)
{
    call_count++;
    c->scratch[0]++;
    if (c->sub == 0) { c->sub = 1; return 0; }
    return call_ret;
}
static const seq_call_fn s_calls[] = { custom_fn };

SEQ_BEGIN( s_call )
    CALL( 0 )
    OUT_ON( O_A )
    END_STOP
SEQ_END

SEQ_BEGIN( s_bad_label )
    GOTO( L_MISSING )
SEQ_END

SEQ_BEGIN( s_act_start )
    ACT_START( ACT_PULSE )
    WAIT_ACT( ACT_PULSE )
    OUT_ON( O_C )
    END_STOP
SEQ_END

SEQ_BEGIN( s_prof1 )
    OUT_ON( O_A )
SEQ_END
SEQ_BEGIN( s_prof_any )
    OUT_ON( O_B )
SEQ_END

ACT_BEGIN( a_pulse )
    OUT_PULSE( O_A, 2, 2, 2 )
ACT_END
ACT_BEGIN( a_hold )
    OUT_ON( O_B )
    WAIT( IN( ON(SEN_A) ) )
    OUT_OFF( O_B )
ACT_END
ACT_BEGIN( a_fail )
    END_ERROR( 290 )
ACT_END
ACT_BEGIN( a_bad_turn )
    AGV_TURN( CW, 90, 0, 0 )
ACT_END
ACT_BEGIN( a_bad_restart )
    END_RESTART
ACT_END
ACT_BEGIN( a_bad_speed )
    AGV_SPEED( LOW )
ACT_END
ACT_BEGIN( a_bad_forback )
    AGV_FORBACK( FWD )
ACT_END
ACT_BEGIN( a_bad_hold )
    ERROR_HOLD( 290, IN( ON(SEN_A) ), L_OK )
ACT_END
ACT_BEGIN( a_bad_nowait )
    AGV_TURN( CW, 90, 0, 0, 0, NOWAIT )
ACT_END
ACT_BEGIN( a_bad_move )
    AGV_MOVE( 10 )
ACT_END

/* 거리 이동: 출력 A 켬 → AGV_MOVE(10) → 출력 B 켬 */
SEQ_BEGIN( s_move )
    OUT_ON( OUT_EXT_1 )
    AGV_MOVE( 10 )
    OUT_ON( OUT_EXT_2 )
    END_STOP
SEQ_END

/* 정렬 회전: 회전 → 정착·보정이 끝나야 출력 B. 오류로 끝나면 B 를 켜지 않는다 */
SEQ_BEGIN( s_align )
    AGV_TURN( CW, 90, 0, 0, 12, ALIGN, OBS )
    OUT_ON( OUT_EXT_2 )
    END_STOP
SEQ_END
ACT_BEGIN( a_bad_align )
    AGV_TURN( CW, 90, 0, 0, 0, ALIGN )
ACT_END

/* 교차선까지 이동: 서기 전에는 B 를 켜지 않고, 못 찾아 오류(258)로 끝나면 B 를 켜지 않는다 */
SEQ_BEGIN( s_move_tape )
    OUT_ON( OUT_EXT_1 )
    AGV_MOVE( 150, TAPE )
    OUT_ON( OUT_EXT_2 )
    END_STOP
SEQ_END
ACT_BEGIN( a_bad_move_tape )
    AGV_MOVE( 10, TAPE )
ACT_END
SEQ_BEGIN( s_obs_auto )
    OBS_OFF_AUTO
    END_STOP
SEQ_END

/* 32비트 칸의 상위 워드(입력·출력 ID 16 이상, 압축 표의 칸 7·9·11)를 쓰는 스텝 */
SEQ_BEGIN( s_high )
    WAIT( IN( ON(IN_IF_1), OFF(IN_IF_2) ) )
    OUT_ON( OUT_IF_1, OUT_LAMP_G )
    SELECT_BEGIN
        WHEN( IN( ANY_ON(IN_IF_3, IN_IF_4), OFF(IN_IF_5) ), L_A )
        WHEN( IN( ON(SEN_A) ), L_B )
    SELECT_END_TO( 50, L_C )
    L( L_A )
    OUT_OFF( OUT_LAMP_G )
    OUT_ON( OUT_BUZZER )
    END_STOP
    L( L_B )
    OUT_PULSE( OUT_IF_2, 1, 1, 2 )
    END_STOP
    L( L_C )
    END_ERROR( 123 )
SEQ_END

enum {
    S_BRANCH = SEQ_SITE_BASE, S_UNMATCHED, S_TIMING, S_ACS, S_DRIVE, S_BUDGET, S_OUTS, S_FLICKER,
    S_ANY, S_HANDSHAKE, S_WAIT_OUT, S_CALL, S_BAD_LABEL, S_ACT_START, S_PROF, S_NO_TABLE,
    S_XWHEN, S_XWAIT, S_ANY2, S_DPARAM, S_TPARAM, S_NOWAIT, S_MOVE, S_ALIGN, S_MOVE_TAPE,
    S_BELTS, S_BELT_OFF, S_BELT_END, S_BELT_MANY, S_OBS_AUTO, S_HIGH
};
enum { S_FLICKER_ST = SEQ_UNLOAD_PART + 3, S_AND, S_STANDBY, S_ACCUM, S_HOLD_PARAM, S_WHEN_HOLD, S_WHEN_HOLD3 };   /* 사이트 ID 가 SEQ_MAX(64)를 넘지 않게 코어 ID 빈 자리를 쓴다 */

SEQ_MAP_BEGIN( site_seqs )
    SEQ_MAP( 0, S_BRANCH, s_branch )
    SEQ_MAP( 0, S_UNMATCHED, s_unmatched )
    SEQ_MAP( 0, S_TIMING, s_timing )
    SEQ_MAP( 0, S_ACS, s_acs )
    SEQ_MAP( 0, S_DRIVE, s_drive )
    SEQ_MAP( 0, S_BUDGET, s_budget )
    SEQ_MAP( 0, S_OUTS, s_outs )
    SEQ_MAP( 0, S_FLICKER, s_flicker )
    SEQ_MAP( 0, S_ANY, s_any )
    SEQ_MAP( 0, S_HANDSHAKE, s_handshake )
    SEQ_MAP( 0, S_WAIT_OUT, s_wait_out )
    SEQ_MAP( 0, S_CALL, s_call )
    SEQ_MAP( 0, S_BAD_LABEL, s_bad_label )
    SEQ_MAP( 0, S_ACT_START, s_act_start )
    SEQ_MAP( PF_TWO, S_PROF, s_prof_any )
    SEQ_MAP( PF_ONE, S_PROF, s_prof1 )
    SEQ_MAP( 0, S_PROF, s_prof_any )
    SEQ_MAP( 0, S_XWHEN, s_xwhen )
    SEQ_MAP( 0, S_XWAIT, s_xwait )
    SEQ_MAP( 0, S_ANY2, s_any2 )
    SEQ_MAP( 0, S_DPARAM, s_dparam )
    SEQ_MAP( 0, S_TPARAM, s_tparam )
    SEQ_MAP( 0, S_NOWAIT, s_nowait )
    SEQ_MAP( 0, S_MOVE, s_move )
    SEQ_MAP( 0, S_ALIGN, s_align )
    SEQ_MAP( 0, S_MOVE_TAPE, s_move_tape )
    SEQ_MAP( 0, S_BELTS, s_belts )
    SEQ_MAP( 0, S_BELT_OFF, s_belt_off )
    SEQ_MAP( 0, S_BELT_END, s_belt_end )
    SEQ_MAP( 0, S_BELT_MANY, s_belt_many )
    SEQ_MAP( 0, S_OBS_AUTO, s_obs_auto )
    SEQ_MAP( 0, S_FLICKER_ST, s_flicker_st )
    SEQ_MAP( 0, S_AND, s_and )
    SEQ_MAP( 0, S_STANDBY, s_standby )
    SEQ_MAP( 0, S_ACCUM, s_accum )
    SEQ_MAP( 0, S_HOLD_PARAM, s_hold_param )
    SEQ_MAP( 0, S_WHEN_HOLD, s_when_hold )
    SEQ_MAP( 0, S_WHEN_HOLD3, s_when_hold3 )
    SEQ_MAP( 0, S_HIGH, s_high )
SEQ_MAP_END

ACT_MAP_BEGIN( site_acts )
    ACT_MAP( 0, ACT_PULSE, a_pulse )
    ACT_MAP( 0, ACT_HOLD, a_hold )
    ACT_MAP( 0, ACT_FAIL, a_fail )
    ACT_MAP( 0, ACT_SPARE1, a_hold )
    ACT_MAP( 0, ACT_SPARE2, a_hold )
    ACT_MAP( 0, ACT_SPARE3, a_hold )
ACT_MAP_END

PROFILE_TABLE_BEGIN( site_profiles )
    PROFILE( PF_ONE, 100, 1, 5 )
    PROFILE( PF_TWO, 100, 6, 9 )
PROFILE_TABLE_END

#define NSEQS  (sizeof(site_seqs) / sizeof(site_seqs[0]))
#define NACTS  (sizeof(site_acts) / sizeof(site_acts[0]))
#define NPROFS (sizeof(site_profiles) / sizeof(site_profiles[0]))

static const seq_site_t s_site_ok = { site_seqs, NSEQS, site_acts, NACTS, s_calls, 1, site_profiles, NPROFS };

/* 사용자 함수가 같은 ACT 슬롯을 다시 시작하고 "다음"(1)을 돌려준다(74차 검토). 다음 스텝은 새 표의 1 번이다 */
enum { ACT_R1 = 1, ACT_R2 };
static int restart_fn(seq_ctx_t *c)
{
    (void)c;
    act_abort_all();
    (void)act_start(ACT_R2);
    return 1;
}
static const seq_call_fn s_rcalls[] = { restart_fn };
ACT_BEGIN( a_r1 )
    OUT_ON( O_A )
    CALL( 0 )
    OUT_ON( O_B )            /* 옛 표의 2 번: 여기로 가면 안 된다 */
    DELAY( 2 )
ACT_END
ACT_BEGIN( a_r2 )
    OUT_ON( O_C )
    OUT_ON( OUT_EXT_4 )
    DELAY( 3 )
    OUT_OFF( OUT_EXT_4 )
ACT_END
ACT_MAP_BEGIN( r_acts )
    ACT_MAP( 0, ACT_R1, a_r1 )
    ACT_MAP( 0, ACT_R2, a_r2 )
ACT_MAP_END
static const seq_site_t s_site_r = { 0, 0, r_acts, 2, s_rcalls, 1, site_profiles, NPROFS };

/* ---- 압축 표(v0.56.0): 같은 시험을 압축 표로 한 번 더 돈다. 꼴은 agv/seq.h seq_packed_t ---- */
static int          s_packed_mode;
static uint16_t     s_pw[8192], s_pwn;
static seq_pmap_t   s_pseqs[64], s_pacts[16];
static seq_packed_t s_pk;
static seq_site_t   s_bound;

static uint16_t pack_steps(const seq_step_t *st, uint16_t n, uint16_t at)
{
    uint16_t k;
    for (k = 0; k < n; k++) {
        const seq_step_t *s = &st[k];
        const uint16_t v[12] = { s->kind, s->target, s->arg0, s->arg1, s->timeout, s->xc,
                                 (uint16_t)s->on, (uint16_t)(s->on >> 16), (uint16_t)s->off, (uint16_t)(s->off >> 16),
                                 (uint16_t)s->out, (uint16_t)(s->out >> 16) };
        uint16_t head = at++, m = 0, c = 0, b;
        seq_step_t u;
        for (b = 0; b < 12; b++) {
            if (v[b] != 0u) { m |= (uint16_t)(1u << b); s_pw[at++] = v[b]; c++; }
        }
        assert(at < sizeof(s_pw) / sizeof(s_pw[0]));
        s_pw[head] = (uint16_t)(m | (c << 12));
        /* 왕복: 푼 스텝이 펼친 스텝과 칸마다 같다 */
        assert(seq_unpack(&s_pw[head], &u) == SEQ_PK_LEN(s_pw[head]) && SEQ_PK_LEN(s_pw[head]) == (uint16_t)(at - head));
        assert(u.kind == s->kind && u.target == s->target && u.arg0 == s->arg0 && u.arg1 == s->arg1 && u.timeout == s->timeout
               && u.xc == s->xc && u.on == s->on && u.off == s->off && u.out == s->out);
    }
    return at;
}

static uint16_t pack_map(const seq_def_t *d, uint16_t n, seq_pmap_t *pm, uint16_t at)
{
    uint16_t i;
    for (i = 0; i < n; i++) {
        pm[i].pid = (uint16_t)(d[i].profile | (d[i].id << 8));
        pm[i].off = at;
        pm[i].n = d[i].nsteps;
        at = pack_steps(d[i].steps, d[i].nsteps, at);
    }
    return at;
}

/* 펼친 표 모드는 그대로, 압축 표 모드는 압축해서 seqs·acts 없이 묶는다(코어가 펼친 표를 건드리면 죽는다) */
static uint16_t bind(const seq_site_t *src)
{
    uint16_t at;
    if (!s_packed_mode) { return seq_bind(src); }
    assert(src->nseqs <= 64 && src->nacts <= 16);
    at = pack_map(src->seqs, src->nseqs, s_pseqs, 0);
    s_pwn = pack_map(src->acts, src->nacts, s_pacts, at);
    s_pk.w = s_pw;
    s_pk.seqs = s_pseqs;  s_pk.nseqs = src->nseqs;
    s_pk.acts = s_pacts;  s_pk.nacts = src->nacts;
    s_bound = *src;
    s_bound.seqs = 0;  s_bound.nseqs = 0;
    s_bound.acts = 0;  s_bound.nacts = 0;
    s_bound.packed = &s_pk;
    return seq_bind(&s_bound);
}

static void start(uint16_t id)
{
    seq_abort_all();
    act_abort_all();
    fake_in = fake_out = 0;
    fake_restart_count = 0;
    D.action_err = D.stop_err = 0;
    seq_request(id);
}

static void bad_act_rejected(const seq_step_t *steps, uint16_t n)
{
    seq_def_t bad[2];
    seq_site_t site = s_site_ok;
    bad[0] = site_acts[0];
    bad[1].profile = 0;
    bad[1].id = 77;
    bad[1].steps = steps;
    bad[1].nsteps = n;
    site.acts = bad;
    site.nacts = 2;
    assert(bind(&site) == 77);
    assert(act_start(ACT_PULSE) == 2);   /* 거부된 표의 ACT 는 하나도 시작하지 않는다 */
}

#define BAD(t) bad_act_rejected(t##_steps, (uint16_t)(sizeof(t##_steps) / sizeof(seq_step_t)))

static void run_all(void)
{
    uint16_t k;

    for (k = 0; k < SEQ_STATE_COUNT; k++) { seq_state_set(k, 0); }   /* 둘째 판(압축 표)도 같은 처음 상태 */
    fake_reset();
    fake_params();
    memset(&D, 0, sizeof(D));
    D.ui_mode = UI_AUTO;
    (void)param_set(P_AGV_ID, 7);
    (void)param_set(P_ACS_RETRY_LEGACY, 5);
    acs_reset();

    /* ACT 금지 스텝 부팅 검사 */
    BAD(a_bad_turn);
    BAD(a_bad_restart);
    BAD(a_bad_speed);
    BAD(a_bad_forback);
    BAD(a_bad_hold);
    BAD(a_bad_nowait);

    assert(bind(&s_site_ok) == 0);
    assert(seq_profile() == 0);

    /* AGV_MOVE: 앞 출력 → 이동 요청(100 mm) → 도착해 설(MOVE_DONE) 때까지 대기 → 뒤 출력. ACT 에서는 금지 */
    start(S_MOVE);
    fake_move_mm = 0;
    fake_move_ret = 0;                                           /* 회전 중 등: 못 건다. 통과하지 않고 다시 건다 */
    seq_run(); seq_run();
    assert(fake_out == BIT(OUT_EXT_1) && fake_move_mm == 100 && !D.move_on && seq_is_active());
    fake_move_ret = 1;
    seq_run();
    assert(D.move_on);
    seq_run(); seq_run();
    assert(seq_is_active() && fake_out == BIT(OUT_EXT_1));      /* 서기 전에는 B 를 켜지 않는다 */
    D.move_on = MOVE_DONE;                                       /* 도착해 섰다(stop.c finish) */
    seq_run();
    assert(fake_out == (BIT(OUT_EXT_1) | BIT(OUT_EXT_2)) && !seq_is_active() && (D.move_on == 0));
    start(S_MOVE);                                               /* 도착 전에 다른 원인으로 섰다: B 없이 오류 277 */
    seq_run();
    D.move_on = MOVE_FAIL;
    seq_run(); seq_run();
    assert((fake_out & BIT(OUT_EXT_2)) == 0 && (D.stop_err == ERR_STOP_APPROACH) && (D.move_on == 0));
    D.stop_err = 0;
    seq_abort_all();
    fake_out = 0;
    start(S_MOVE);                                               /* 거리 범위 밖: 줄이지 않고 지워지지 않는 오류 290(시퀀스는 run_after_motor 가 끝낸다) */
    fake_move_ret = -1;
    seq_run(); seq_run();
    assert(!D.move_on && ((fake_out & BIT(OUT_EXT_2)) == 0) && (D.stop_err == SEQ_ERR_TABLE));
    D.stop_err = 0;
    seq_abort_all();
    fake_move_ret = 1;
    fake_out = 0;
    BAD(a_bad_move);
    assert(bind(&s_site_ok) == 0);

    /* 정렬 회전(ALIGN): 정렬이 끝나야(tape_align 0, 회전 끝) 다음 스텝. 오류가 서 있으면 다음으로 가지 않는다 */
    start(S_ALIGN);
    fake_turn_count = 0;
    seq_run();
    assert(fake_turn_count == 1 && fake_turn_next == ((1u << 8) | (12u << 11)) && D.tape_align == 1);
    seq_run(); seq_run();
    assert(seq_is_active() && fake_out == 0);                   /* 정렬 중 */
    D.tape_align = 0;
    D.system_err = 258;                                          /* 못 찾고 끝났다: 회전은 끝났어도 B 는 켜지 않는다 */
    seq_run(); seq_run();
    assert(seq_is_active() && fake_out == 0 && seq_active_step() == 0);
    D.system_err = 0;
    D.stop_err = 277;
    seq_run();
    assert(fake_out == 0);
    D.stop_err = 0;
    D.tape_fix_end = 0;                                          /* 가운데에 섰다 */
    seq_run();                                                   /* 오류 없이 끝났다 */
    assert(fake_out == BIT(OUT_EXT_2) && !seq_is_active());
    fake_out = 0;
    /* 오류 없이 실패로 끝남(선을 잡은 뒤 범위 초과 -1, 보정 못 끝냄 1, 선 놓침 3): B 를 켜지 않고 지워지지 않는 258(system_err) */
    {
        static const int16_t fails[] = { -1, 1, 2, 3, 4 };
        for (int k = 0; k < 5; k++) {
            start(S_ALIGN);
            seq_run();
            D.tape_align = 0;
            D.tape_fix_end = fails[k];
            seq_run(); seq_run();
            assert(fake_out == 0 && (D.system_err == ERR_GUIDE_SENSOR) && (seq_active_step() == 0));
            D.system_err = 0;
            seq_abort_all();
        }
    }
    /* 거부(회전 모드 0 등): 다음 스텝으로 가지 않고 지워지지 않는 290(stop_err) */
    fake_align_refuse = 1;
    start(S_ALIGN);
    seq_run(); seq_run();
    assert(fake_out == 0 && (D.stop_err == SEQ_ERR_TURN) && (seq_active_step() == 0));
    fake_align_refuse = 0;
    D.stop_err = 0;
    seq_abort_all();
    D.tape_fix_end = 0;
    fake_turn_count = 0;                                         /* 뒤 시험은 처음부터 센다 */
    BAD(a_bad_align);
    assert(bind(&s_site_ok) == 0);

    /* 교차선까지 이동(AGV_MOVE( 150, TAPE )): 설 때까지 대기, 섰는데 오류가 서 있으면 B 를 켜지 않는다 */
    start(S_MOVE_TAPE);
    fake_move_mm = 0;
    seq_run();
    assert(fake_out == BIT(OUT_EXT_1) && fake_move_mm == -1500 && fake_move_kind == 0 && D.move_on);
    seq_run(); seq_run();
    assert(seq_is_active() && fake_out == BIT(OUT_EXT_1));
    D.system_err = 258;                                          /* 한계 안에 못 찾았다 */
    seq_run();
    assert(seq_is_active() && fake_out == BIT(OUT_EXT_1));
    D.system_err = 0;
    D.move_on = MOVE_DONE;
    seq_run();                                                   /* 찾아 섰다 */
    assert(fake_out == (BIT(OUT_EXT_1) | BIT(OUT_EXT_2)) && !seq_is_active());
    BAD(a_bad_move_tape);
    assert(bind(&s_site_ok) == 0);

    /* WHEN: 대기 → 첫 참의 가지 → 안정 초과 → 재출발 */
    start(S_BRANCH);
    seq_run();
    assert(seq_is_active() && seq_active_id() == S_BRANCH && seq_active_step() == 0);
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == BIT(O_A));
    fake_in |= BIT(SEN_C);
    seq_run(); seq_run(); seq_run();
    assert(seq_is_active());
    fake_in &= ~BIT(SEN_C);        /* 끊기면 처음부터 */
    seq_run();
    fake_in |= BIT(SEN_C);
    seq_run(); seq_run(); seq_run();
    assert(seq_is_active());
    seq_run();                     /* 4 틱째 = 3 초과 */
    assert(!seq_is_active() && fake_out == 0 && fake_restart_count == 1);
    assert(seq_request_flag[S_BRANCH] == 0);

    /* ANY_ON: 0개 켜짐 = 거짓, 1개 = 참, OFF 항목 위반 = 거짓 */
    start(S_ANY);
    seq_run(); seq_run();
    assert(seq_is_active() && seq_active_step() == 0 && fake_out == 0);
    fake_in = BIT(SEN_C) | BIT(SEN_E);          /* ANY 만족, SEN_E 켜져 있어 거짓 */
    seq_run();
    assert(fake_out == 0);
    fake_in = BIT(SEN_D);                       /* 4번째 하나만 */
    seq_run();
    assert(fake_out == BIT(O_A) && seq_is_active());
    seq_run();                                  /* WAIT ANY_ON(B,C): D 만으로는 거짓 */
    assert(fake_out == BIT(O_A));
    fake_in = BIT(SEN_D) | BIT(SEN_B);
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)));

    /* WHEN 묶음 시간 제한 */
    start(S_BRANCH);
    for (int i = 0; i < 100; i++) { seq_run(); assert(seq_is_active()); }
    seq_run();
    assert(!seq_is_active() && D.action_err == 123);

    /* 우선순위: 낮은 ID 먼저 */
    start(S_TIMING);
    seq_request(S_BRANCH);
    seq_run();
    assert(seq_active_id() == S_BRANCH);
    seq_abort_all();
    assert(fake_seq_off_count > 0 && !seq_is_active() && seq_request_flag[S_TIMING] == 0);

    /* 표 없는 요청은 버린다 */
    start(S_NO_TABLE);
    seq_run();
    assert(!seq_is_active() && seq_request_flag[S_NO_TABLE] == 0);

    /* 불일치 누적: 거짓 틱만 세고, 오류에서 복귀하면 다시 0 */
    start(S_UNMATCHED);
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 10; i++) { seq_run(); assert(D.action_err == 0); }
    fake_in = 0;
    seq_run(); seq_run(); seq_run();                     /* 참 틱은 누적에 안 든다 */
    fake_in = BIT(SEN_A);
    seq_run();
    assert(D.action_err == 290);
    D.action_err = 0;
    seq_run();
    assert(D.action_err == 290);                         /* 유지 */
    D.action_err = 0;
    fake_in = BIT(IN_START_SW);                          /* 복귀, SEN_A 꺼짐 = 조건 참 */
    seq_run();
    D.action_err = 0;                                    /* 복귀한 스캔에는 아직 오류가 서 있다 */
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 10; i++) { seq_run(); assert(D.action_err == 0); }  /* 두 번째 시도도 전체 시간 */
    seq_run();
    assert(D.action_err == 290);

    /* DELAY(N) 은 N+1 틱, WAIT_TO 는 N 초과에서 라벨 */
    start(S_TIMING);
    seq_run(); seq_run();
    assert(fake_out == 0);
    seq_run();
    assert(fake_out == BIT(O_A));
    seq_run(); seq_run();
    assert(fake_out == BIT(O_A));
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_C)) && !seq_is_active());

    /* WAIT_TO 조건 통과 */
    start(S_TIMING);
    fake_in = BIT(SEN_A);
    seq_run(); seq_run(); seq_run();
    assert(fake_out == BIT(O_A) && !seq_is_active());

    /* ACS: 번호 출처 4종, ACK 대기, 스테이션 */
    acs_station_reset();
    (void)acs_station_push(10);
    start(S_ACS);
    seq_run();
    assert(acs_pending() == 5);
    fake_tx_count = 0;                                      /* IDLE 보고(v0.59.0): Info 'X', 번호 055, 슬롯 하나 */
    for (int i = 0; i < 5; i++) { acs_schedule(); }
    assert(fake_tx_count == 5 && fake_tx[4][6] == 'X' && fake_tx[4][7] == '0' && fake_tx[4][8] == '5' && fake_tx[4][9] == '5');
    seq_run();
    assert(fake_out == 0);
    acs_on_ack('L', '0'); acs_on_ack('U', '0'); acs_on_ack('A', '0'); acs_on_ack('F', '0'); acs_on_ack('X', '0');
    seq_run();
    assert(fake_out == BIT(O_A) && acs_station_get(1) == 42 && acs_station_count() == 2 && acs_station_get(0) == 0);
    /* ACK 시간 제한 */
    acs_report('L', 1);
    start(S_ACS);
    for (int i = 0; i < 5; i++) { seq_run(); }
    assert(fake_out == 0);
    seq_run();
    assert(fake_out == BIT(O_B));
    acs_reset();

    /* OBS_OFF_AUTO 스텝: 값 2(끔, 자동 복귀) */
    start(S_OBS_AUTO);
    D.obs_use = 1;
    seq_run();
    assert(D.obs_use == 2);
    seq_abort_all();

    /* 주행 스텝 */
    start(S_DRIVE);
    D.turn_pulses = 100;
    D.obs_use = 1;
    seq_run();
    assert(fake_speed_arg == 20 && fake_forback_arg == DIR_BACK && D.obs_use == 0 && fake_turn_count == 1);
    assert(fake_turn_next == (1u | (1u << 4)));                /* 셋째 칸 그대로 tagact_turn_next 로(속도 0 = 자동) */
    seq_run();
    assert(seq_active_step() == 4 && fake_turn_count == 1);      /* 완료 대기, 다시 요청하지 않음 */
    D.turn_pulses = 0;
    D.system_err = 258;                                          /* 오류로 회전이 끝난 틱: 다음 스텝으로 가지 않는다 */
    seq_run();
    assert(seq_active_step() == 4);
    D.system_err = 0;
    seq_run();
    assert(D.obs_use == 1 && seq_state_get(ST_0) == 7);
    seq_run();
    assert(seq_is_active());
    seq_state_set(ST_1, 3);
    seq_run();
    assert(!seq_is_active() && fake_out == 0 && fake_restart_count == 1);

    /* 한 스캔 8스텝, 라벨은 세지 않는다 */
    start(S_BUDGET);
    seq_run();
    assert(fake_out == 0xFFUL && seq_is_active());
    seq_run();
    assert(fake_out == 0x1FFUL && !seq_is_active());

    /* 펄스: 2 켜고 1 끄기 2회. END 없이 끝나면 비활성 */
    start(S_OUTS);
    { static const int want[] = { 1, 1, 0, 1, 1 };
      for (int i = 0; i < 5; i++) { seq_run(); assert(((fake_out & BIT(O_A)) != 0) == want[i]); } }
    seq_run();
    assert(fake_out == BIT(O_C) && !seq_is_active());

    /* 점멸: 조건 참이면 끄고 다음 */
    start(S_FLICKER);
    { int on = 0, off = 0;
      for (int i = 0; i < 8; i++) { seq_run(); if (fake_out & BIT(O_A)) { on++; } else { off++; } }
      assert(on > 0 && off > 0); }
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == BIT(O_B));
    start(S_FLICKER);
    for (int i = 0; i < 20; i++) { seq_run(); }
    assert(fake_out != BIT(O_C));
    seq_run();
    assert((fake_out & BIT(O_A)) == 0 && fake_out == BIT(O_C));

    /* 핸드셰이크(v0.58.0): 출력 켬, 응답 켜짐 대기, 켜지면 바로 다음 스텝. 출력은 끄지 않는다(표가 끈다) */
    start(S_HANDSHAKE);
    seq_run();
    assert(fake_out == BIT(O_A) && seq_is_active());
    seq_run();
    assert(fake_out == BIT(O_A) && seq_is_active());       /* 응답 전: 출력을 켠 채 기다린다 */
    fake_in = BIT(SEN_A);
    seq_run();                                              /* 응답 켜짐: 같은 스캔에 다음 스텝(O_B)까지. O_A 는 그대로 켜져 있다 */
    assert(fake_out == (BIT(O_A) | BIT(O_B)) && !seq_is_active());
    fake_in = 0;
    start(S_HANDSHAKE);                                     /* 제한 5 틱은 응답 켜짐 대기에 적용: 응답이 없으면 라벨로, 출력은 그대로 */
    for (int i = 0; i < 6; i++) { seq_run(); }
    assert(fake_out == (BIT(O_A) | BIT(O_C)));
    start(S_HANDSHAKE);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = BIT(SEN_A);                                   /* 제한 안에 응답 */
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)));
    start(S_HANDSHAKE);                                     /* 응답이 이미 켜져 있으면 켠 틱에 끝난다(따로 막지 않는다) */
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)) && !seq_is_active());
    fake_in = 0;

    /* 점멸 끝 조건에 AND 상태 낱말: 방향이 맞아야 끝난다. 칸 배치(xc != 0)는 주기·출력을 out 한 칸에 넣는다 */
    assert(s_flicker_steps[0].xc == 0 && s_flicker_steps[0].arg0 == 2 && s_flicker_steps[0].out == BIT(O_A));
    assert(s_flicker_st_steps[0].xc != 0 && s_flicker_st_steps[0].arg0 == 0 && s_flicker_st_steps[0].out == ((2uL << 8) | (O_A + 1)));
    D.backward = 1;
    start(S_FLICKER_ST);
    fake_in = BIT(SEN_A);                                   /* 입력은 참, 후진이라 상태가 거짓 */
    { int on = 0, off = 0;
      for (int i = 0; i < 8; i++) { seq_run(); if (fake_out & BIT(O_A)) { on++; } else { off++; } }
      assert(on > 0 && off > 0 && seq_is_active()); }
    D.backward = 0;                                         /* 전진: 끄고 다음 스텝 */
    seq_run();
    assert(fake_out == BIT(O_B) && !seq_is_active());
    fake_in = 0;
    D.backward = 1;
    start(S_FLICKER_ST);
    for (int i = 0; i < 40; i++) { seq_run(); }             /* 제한 20 틱 */
    assert(fake_out == BIT(O_C));
    D.backward = 0;
    fake_in = 0;

    /* AND 조건(입력 + 하나라도 + 호기) */
    (void)param_set(P_AGV_ID, 7);
    start(S_AND);
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == 0);                                  /* SEN_B·SEN_C 둘 다 꺼짐 */
    fake_in = BIT(SEN_A) | BIT(SEN_C);
    seq_run();
    assert(fake_out == BIT(O_A));                           /* 통과. 다음은 유지 대기 */
    fake_in = BIT(SEN_C);
    for (int i = 0; i < 6; i++) { seq_run(); }
    assert(fake_out == BIT(O_A));                           /* SEN_D 없음 */
    fake_in = BIT(SEN_C) | BIT(SEN_D);
    for (int i = 0; i < 3; i++) { seq_run(); }
    assert(fake_out == BIT(O_A));                           /* 유지 3 틱 초과 전 */
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)));
    fake_in = BIT(SEN_E);
    D.backward = 0;                                         /* 상태(후진)가 거짓: 제한 5 틱 */
    for (int i = 0; i < 4; i++) { seq_run(); }
    assert(fake_out == (BIT(O_A) | BIT(O_B)));
    D.backward = 1;
    seq_run();
    assert(fake_out & BIT(O_C));
    D.backward = 0;
    fake_in = 0;
    seq_abort_all();

    /* 누적 대기: 참 3 → 거짓 5(셈 유지) → 참 2 는 아직(5 를 넘지 않음), 참 1 더(6) 에 통과 */
    start(S_ACCUM);
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = 0;
    for (int i = 0; i < 5; i++) { seq_run(); }
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 2; i++) { seq_run(); }
    assert(fake_out == 0);
    seq_run();                                              /* 같은 스캔에 다음 대기 첫 틱(SEN_B 꺼짐) */
    assert(fake_out == BIT(O_A));
    fake_in = BIT(SEN_B);                                   /* 제한 10(진입부터): 참 2, 거짓 7, 11 번째 틱 참은 셈 3(넘지 않음) → 제한 */
    for (int i = 0; i < 2; i++) { seq_run(); }
    fake_in = 0;
    for (int i = 0; i < 7; i++) { seq_run(); }
    assert(fake_out == BIT(O_A));
    fake_in = BIT(SEN_B);
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(OUT_EXT_5)));
    start(S_ACCUM);                                         /* 제한과 같은 틱에 셈이 넘으면 조건이 이긴다 */
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 6; i++) { seq_run(); }
    assert(fake_out == BIT(O_A));                           /* 둘째 대기 1 틱(거짓) */
    fake_in = BIT(SEN_B);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = 0;
    for (int i = 0; i < 6; i++) { seq_run(); }
    assert(fake_out == BIT(O_A));                           /* 틱 10, 셈 3 */
    fake_in = BIT(SEN_B);
    seq_run();                                              /* 틱 11 > 10 이지만 셈 4 > 3 이 먼저. 셋째 대기 첫 틱은 거짓(거짓 1) */
    assert(seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B)));
    fake_in = BIT(SEN_C);                                   /* UNMATCHED(4): 거짓 틱만 센다. 참 1, 거짓 3(합 4, 넘지 않음), 참 2 → 셈 3 > 2 */
    seq_run();
    fake_in = 0;
    for (int i = 0; i < 3; i++) { seq_run(); }
    assert(seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B)));
    fake_in = BIT(SEN_C);
    seq_run();
    assert(seq_is_active());
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B) | BIT(O_C)));
    start(S_ACCUM);                                         /* 거짓 틱 5(> 4) 면 제한 라벨 */
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 6; i++) { seq_run(); }
    fake_in = BIT(SEN_B);
    for (int i = 0; i < 4; i++) { seq_run(); }
    assert(fake_out == (BIT(O_A) | BIT(O_B)));              /* 셋째 대기 거짓 1 */
    fake_in = 0;
    for (int i = 0; i < 3; i++) { seq_run(); }
    assert(seq_is_active());
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B) | BIT(OUT_EXT_5)));
    fake_in = 0;

    /* 파라미터 시간 유지 대기: ST 0 = 첫 참 틱에 통과 */
    (void)param_set(P_STATION_NO, 0);
    (void)param_set(P_BAT_OFFSET_DV, -5);
    (void)param_set(P_AUTO_START_SEC, 700);                 /* 70000 틱 → 65534 로 묶음 */
    start(S_HOLD_PARAM);
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == BIT(O_A));
    start(S_HOLD_PARAM);                                    /* ST 3: 참 3 → 거짓(0) → 참 3, 도는 중 5 로 올리면 셈 5 까지 못 넘고, 2 로 내리면 다음 참 틱에 */
    (void)param_set(P_STATION_NO, 3);
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = 0;
    seq_run();
    fake_in = BIT(SEN_A);
    for (int i = 0; i < 3; i++) { seq_run(); }
    assert(fake_out == 0);
    (void)param_set(P_STATION_NO, 5);
    for (int i = 0; i < 2; i++) { seq_run(); }              /* 셈 5 */
    assert(fake_out == 0);
    (void)param_set(P_STATION_NO, 2);
    seq_run();                                              /* 셈 6 > 2. 다음 대기(BO -5 → 0) 첫 틱은 SEN_C 꺼짐 */
    assert(fake_out == BIT(O_A));
    fake_in = BIT(SEN_C);
    seq_run();                                              /* 음수 = 0: 첫 참 틱. 누적 대기 첫 틱은 SEN_B 꺼짐 */
    assert(fake_out == (BIT(O_A) | BIT(O_C)));
    fake_in = BIT(SEN_B);
    for (long t = 0; t < 65534L; t++) { seq_run(); }
    assert(seq_is_active() && !(fake_out & BIT(O_B)));
    seq_run();
    assert(!seq_is_active() && (fake_out & BIT(O_B)));
    (void)param_set(P_BAT_OFFSET_DV, 0);
    (void)param_set(P_AUTO_START_SEC, 0);
    fake_in = 0;

    /* 묶음 안 유지 셈 */
    (void)param_set(P_STATION_NO, 2);
    start(S_WHEN_HOLD);                                     /* 유지: 참 3 → 거짓(0) → 참 3 은 아직, 4 번째에 */
    fake_in = BIT(SEN_B);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = 0;
    seq_run();
    fake_in = BIT(SEN_B);
    for (int i = 0; i < 3; i++) { seq_run(); }
    assert(seq_is_active() && fake_out == 0);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_B));
    start(S_WHEN_HOLD);                                     /* 앞 가지가 이긴다: 유지가 넘는 틱에 SEN_A 도 켜지면 L_A */
    fake_in = BIT(SEN_B);
    for (int i = 0; i < 3; i++) { seq_run(); }
    fake_in = BIT(SEN_A) | BIT(SEN_B);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_A));
    start(S_WHEN_HOLD);                                     /* 둘이 따로 센다: B·C 2, B 만 1(B 3, C 2 유지), 둘 다 거짓(B 0, C 2), C 1 → C 3 > 2 */
    fake_in = BIT(SEN_B) | BIT(SEN_C);
    for (int i = 0; i < 2; i++) { seq_run(); }
    fake_in = BIT(SEN_B);
    seq_run();
    fake_in = 0;
    seq_run();
    assert(seq_is_active() && fake_out == 0);
    fake_in = BIT(SEN_C);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_C));
    start(S_WHEN_HOLD);                                     /* 제한은 timer: 셈과 따로. 참·거짓이 섞여도 50 틱 넘으면 L_ERR */
    for (int i = 0; i < 50; i++) { fake_in = (i & 1) ? BIT(SEN_B) : 0u; seq_run(); }
    assert(seq_is_active() && fake_out == 0);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(OUT_EXT_5));
    start(S_WHEN_HOLD3);                                    /* 셋째 유지 가지: 앞 가지가 참인 틱에는 오류 없음 */
    fake_in = BIT(SEN_D);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_A) && D.action_err == 0);
    start(S_WHEN_HOLD3);
    seq_run();
    assert(!seq_is_active() && fake_out == 0 && D.action_err == SEQ_ERR_TABLE);
    D.action_err = 0;
    (void)param_set(P_STATION_NO, 0);
    fake_in = 0;

    /* 시퀀스의 대기 보고(STANDBY)는 보고 + 자동 재전송(run_standby)이고 정지는 걸지 않는다. 다른 보고는 슬롯만 */
    fake_standby_count = 0;
    acs_reset();
    acs_station_reset();
    (void)acs_station_push(10);
    start(S_STANDBY);
    seq_run();
    assert(fake_standby_count == 2 && !seq_is_active());
    assert((fake_standby_flags & RUN_STANDBY_AUTO) && (fake_standby_flags & RUN_STANDBY_NOSTOP) && (fake_standby_flags & RUN_STANDBY_SEQ) && !(fake_standby_flags & RUN_STANDBY_RESET));
    assert(fake_standby_id == 11);                          /* 마지막: STATION(0) + 1 */
    assert(acs_pending() == 1);                             /* CALL 만 슬롯에 있다(STANDBY 슬롯은 run_standby 가 채우는데 fake 다) */
    acs_reset();

    /* 출력 대기 */
    start(S_WAIT_OUT);
    fake_out = BIT(O_A) | BIT(O_B);
    seq_run();
    assert(seq_is_active());
    fake_out = BIT(O_A);
    seq_run();
    assert(!seq_is_active());

    /* 벨트 정지 조건(BELT_ON … UNTIL, WAIT_BELTS). 표 칸: 켤·끌 출력은 옵션 없는 BELT_ON 과 같고 조건은 out·arg0·arg1 */
    assert(s_belts_steps[0].kind == SK_BELT_UNTIL && s_belts_steps[0].on == BIT(O_A) && s_belts_steps[0].off == BIT(O_B)
           && s_belts_steps[0].out == BIT(SEN_A) && s_belts_steps[0].arg0 == 0 && s_belts_steps[0].arg1 == 0);
    assert(s_belts_steps[2].out == BIT(SEN_C) && s_belts_steps[2].arg0 == (uint16_t)BIT(SEN_D)
           && s_belts_steps[2].arg1 == (uint16_t)(BIT(IN_START_SW) >> 16));
    assert(s_belts_steps[3].kind == SK_WAIT_OUT && s_belts_steps[3].on == 0
           && s_belts_steps[3].off == (BIT(O_A) | BIT(O_B) | BIT(O_C) | BIT(OUT_EXT_4) | BIT(OUT_EXT_5) | BIT(OUT_EXT_6)));
    assert(s_belt_off_steps[3].kind == SK_OUT && s_belt_off_steps[3].on == (BIT(O_C) | BIT(OUT_EXT_4)) && s_belt_off_steps[3].off == 0);
    BAD(a_bad_belt);                                              /* ACT 금지 */
    assert(bind(&s_site_ok) == 0);
    /* 셋이 다른 틱에 선다. 끝 센서가 켜졌다 꺼져도 그 벨트는 끝났다. 셋째는 OFF 조건까지 맞아야 선다. 마지막이 서는 틱에 통과 */
    start(S_BELTS);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_A) | BIT(O_C) | BIT(OUT_EXT_4) | BIT(OUT_EXT_5)));
    fake_in = BIT(SEN_A);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4) | BIT(OUT_EXT_5)));
    fake_in = 0;
    seq_run();
    fake_in = BIT(SEN_C) | BIT(SEN_D);
    seq_run();
    fake_in = BIT(SEN_C) | BIT(IN_START_SW);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4) | BIT(OUT_EXT_5)));
    fake_in = BIT(SEN_C);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4)));
    fake_in = BIT(SEN_B);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(OUT_EXT_7));
    /* 켤 때 이미 참이면 그 틱에 끄고 같은 틱에 통과 */
    start(S_BELTS);
    fake_in = BIT(SEN_A) | BIT(SEN_B) | BIT(SEN_C);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(OUT_EXT_7));
    /* BELT_OFF 와 같은 벨트의 BELT_ON 이 조건을 지운다 */
    start(S_BELT_OFF);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4)));
    fake_in = BIT(SEN_A) | BIT(SEN_B);
    seq_run();
    assert(fake_out == (BIT(O_C) | BIT(OUT_EXT_4)));               /* 벨트 2 는 조건 없이 돈다 */
    fake_in = BIT(SEN_E);
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4) | BIT(OUT_EXT_5)));
    fake_in = BIT(SEN_C);
    seq_belt_tick();                                              /* seq_run 을 부르지 않는 틱(이동 중 등)에도 본다 */
    assert(seq_is_active() && fake_out == (BIT(O_C) | BIT(OUT_EXT_4)));
    /* 중단·끝에서 조건만 지운다 */
    start(S_BELT_OFF);
    seq_run();
    fake_in = BIT(SEN_E);
    seq_run();                                                    /* 벨트 3 조건을 건 채 */
    seq_abort_all();
    fake_out = BIT(OUT_EXT_5);
    fake_in = BIT(SEN_C);
    seq_belt_tick();
    seq_run();
    assert(fake_out == BIT(OUT_EXT_5));
    start(S_BELT_OFF);
    seq_run();
    fake_in = BIT(SEN_E);
    seq_run();                                                    /* 벨트 3 조건을 건 채 */
    seq_cancel(S_BELT_OFF);
    fake_out = BIT(OUT_EXT_5);
    fake_in = BIT(SEN_C);
    seq_belt_tick();
    assert(fake_out == BIT(OUT_EXT_5));
    start(S_BELT_END);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_A));
    fake_in = BIT(SEN_A);
    seq_belt_tick();
    seq_run();
    assert(fake_out == BIT(O_A));
    /* 칸이 모자라면 표 오류 */
    start(S_BELT_MANY);
    seq_run();
    assert(!seq_is_active() && D.action_err == SEQ_ERR_TABLE);
    D.action_err = 0;

    /* 사용자 함수: 0 대기, 1 다음, 2 오류 */
    call_ret = 0; call_count = 0;
    start(S_CALL);
    seq_run(); seq_run(); seq_run();
    assert(call_count == 3 && seq_is_active() && fake_out == 0);
    call_ret = 1;
    seq_run();
    assert(fake_out == BIT(O_A));
    call_ret = 2;
    start(S_CALL);
    seq_run(); seq_run();
    assert(!seq_is_active() && D.action_err == SEQ_ERR_CALL);

    /* 없는 라벨 */
    start(S_BAD_LABEL);
    seq_run();
    assert(!seq_is_active() && D.action_err == SEQ_ERR_TABLE);

    /* seq_cancel */
    start(S_BRANCH);
    seq_run();
    seq_cancel(S_BRANCH);
    assert(!seq_is_active() && seq_request_flag[S_BRANCH] == 0);

    /* 프로파일: 표 순서대로 첫 일치 */
    (void)param_set(P_AGV_TYPE, 100);
    (void)param_set(P_AGV_ID, 3);
    assert(bind(&s_site_ok) == 0 && seq_profile() == PF_ONE);
    start(S_PROF);
    seq_run();
    assert(fake_out == BIT(O_A));
    (void)param_set(P_AGV_ID, 7);
    assert(bind(&s_site_ok) == 0 && seq_profile() == PF_TWO);
    start(S_PROF);
    seq_run();
    assert(fake_out == BIT(O_B));
    (void)param_set(P_AGV_ID, 50);
    assert(bind(&s_site_ok) == 0 && seq_profile() == 0);
    (void)param_set(P_AGV_ID, 7);
    assert(bind(&s_site_ok) == 0);

    /* ACT: 펄스, 재요청 무시, 슬롯 4개, 미정의 */
    start(0);
    assert(act_start(ACT_PULSE) == 0);
    assert(act_start(ACT_PULSE) == 1);
    assert(act_is_busy(ACT_PULSE));
    act_run();
    assert(fake_out == BIT(O_A));
    act_abort_all();
    assert(fake_out == 0 && !act_is_busy(ACT_PULSE));
    assert(act_start(ACT_HOLD) == 0 && act_start(ACT_SPARE1) == 0 && act_start(ACT_SPARE2) == 0 && act_start(ACT_SPARE3) == 0);
    assert(act_start(ACT_PULSE) == 2);
    assert(act_start(99) == 2);
    act_abort_all();

    /* ACT 는 시퀀스와 동시에 돈다. 끝나면 슬롯이 빈다 */
    start(S_ACT_START);
    seq_run();
    act_run();
    assert(act_is_busy(ACT_PULSE) && seq_is_active());
    for (int i = 0; i < 7; i++) { seq_run(); act_run(); assert((fake_out & BIT(O_C)) == 0); }
    seq_run();
    assert(!act_is_busy(ACT_PULSE) && (fake_out & BIT(O_C)) != 0);

    /* abort: 그 ACT 가 켠 출력만 끈다 */
    start(0);
    fake_out = BIT(O_C);                 /* 다른 쪽이 켠 출력 */
    assert(act_start(ACT_HOLD) == 0);
    act_run();
    assert(fake_out == (BIT(O_B) | BIT(O_C)));
    act_abort_all();
    assert(fake_out == BIT(O_C));

    /* 스스로 끈 출력은 owned 에서 빠진다: abort 해도 안 건드린다 */
    start(0);
    fake_in = BIT(SEN_A);
    assert(act_start(ACT_HOLD) == 0);
    act_run();
    assert(!act_is_busy(ACT_HOLD) && fake_out == 0);

    /* 오류에서 중단 */
    start(0);
    assert(act_start(ACT_HOLD) == 0);
    act_run();
    D.stop_err = 257;
    act_run();
    assert(!act_is_busy(ACT_HOLD) && fake_out == 0);
    D.stop_err = 0;
    assert(act_start(ACT_HOLD) == 0);
    act_run();
    D.system_err = 200;
    act_run();
    assert(!act_is_busy(ACT_HOLD));
    D.system_err = 0;

    /* ACT END_ERROR 는 stop_err, 시퀀스 END_ERROR 는 action_err */
    start(0);
    assert(act_start(ACT_FAIL) == 0);
    act_run();
    assert(D.stop_err == 290 && D.action_err == 0 && !act_is_busy(ACT_FAIL));
    D.stop_err = 0;

    /* 일시정지(action_err)로는 멈추지 않는다 */
    start(0);
    assert(act_start(ACT_PULSE) == 0);
    D.action_err = 281;
    act_run();
    assert(act_is_busy(ACT_PULSE));
    act_abort_all();

    /* seq_abort_all 은 위험 감속 표시도 내린다. LG 1 에 남은 b0 은 무시한다 */
    D.risk_stop = 1;
    seq_abort_all();
    assert(D.risk_stop == 0);
    (void)param_set(P_LEGACY_QUIRKS, 1);
    D.risk_stop = 1;
    seq_abort_all();
    assert(D.risk_stop == 0);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* ---- 확장 조건 ---- */
    D.backward = 0;
    start(S_XWHEN);
    seq_run(); seq_run();
    assert(seq_is_active() && fake_out == 0);
    fake_in = BIT(SEN_A);                        /* 호기 7 + SEN_A */
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_B));
    start(S_XWHEN);
    D.backward = 1;
    seq_run();
    assert(fake_out == BIT(O_A));
    D.backward = 0;
    (void)param_set(P_AGV_ID, 10);               /* 범위 밖 */
    start(S_XWHEN);
    fake_in = BIT(SEN_A);
    seq_run();
    assert(seq_is_active() && fake_out == 0);
    (void)param_set(P_AGV_ID, 7);

    start(S_XWAIT);
    seq_run();
    assert(fake_out == 0);
    D.cross_in_flag = 1;                         /* 대기 */
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)) && seq_active_step() == 4);   /* 유지 아님이라 바로 통과 */
    D.cross_in_flag = 0;
    D.cross_in_on = 1;
    start(S_XWAIT);
    seq_run();
    assert(fake_out == BIT(O_A) && seq_active_step() == 2);   /* 유지 중이면 머문다 */
    D.cross_in_on = 0;
    seq_run();
    assert(fake_out == (BIT(O_A) | BIT(O_B)) && seq_active_step() == 4);
    fake_in = BIT(SEN_A);
    D.action_err = 281;
    seq_run();
    assert(seq_active_step() == 4);              /* SEN_A 켜짐 */
    fake_in = 0;
    seq_run();
    assert(seq_active_step() == 5);
    D.action_err = 0;
    fake_key = KEY_DOWN;
    seq_run();
    assert(seq_active_step() == 5);
    fake_key = KEY_UP;
    (void)param_set(P_STATION_NO, 5);
    seq_run();
    assert(!seq_is_active() && (fake_out & BIT(O_C)));
    fake_key = 0;
    (void)param_set(P_STATION_NO, 4);            /* 3 틱 초과면 타임아웃 */
    start(S_XWAIT);
    D.cross_in_flag = 1;
    seq_run();
    D.cross_in_flag = 0;
    D.action_err = 281;
    fake_key = KEY_UP;
    seq_run();
    assert(seq_active_step() == 6);
    seq_run(); seq_run();
    assert(seq_is_active());
    seq_run();
    assert(!seq_is_active() && (fake_out & BIT(OUT_EXT_5)) && !(fake_out & BIT(O_C)));
    fake_key = 0;
    D.action_err = 0;

    /* 확장 조건 평가 직접 */
    acs_station_set(0, 43, 3);
    acs_station_set(1, 42, 3);
    acs_station_set(2, 145, 3);
    assert(seq_xcond(X_STATION(1, 41, 43), 0) && !seq_xcond(X_STATION(1, 43, 50), 0));
    assert(seq_xcond(X_STATION_MOD(0, 10, 3), 0) && !seq_xcond(X_STATION_MOD(0, 10, 4), 0));
    assert(seq_xcond(X_STATION_DIV(2, 100, 1), 0) && seq_xcond(X_STATION_DIV(2, 10, 14), 0));
    /* 태그 번호 == 큐[k] ± n, 큐[k] % m + n(n 앞뒤 어느 쪽이든) */
    assert(seq_xcond(X_TAG_NUM(STATION(1) + 20), 62) && !seq_xcond(X_TAG_NUM(STATION(1) + 20), 61));
    assert(seq_xcond(X_TAG_NUM(STATION(0)), 43) && !seq_xcond(X_TAG_NUM(STATION(0)), 44));
    assert(seq_xcond(X_TAG_NUM(STATION(2) - 45), 100) && !seq_xcond(X_TAG_NUM(STATION(2) - 45), 190));
    assert(seq_xcond(X_TAG_NUM(MOD(STATION(2), 100) + 7), 52) && seq_xcond(X_TAG_NUM(7 + MOD(STATION(2), 100)), 52));
    assert(seq_xcond(X_TAG_NUM(MOD(STATION(0), 10)), 3) && !seq_xcond(X_TAG_NUM(MOD(STATION(0), 10)), 43));
    assert(seq_xcond(X_TAG_NUM(STATION(23) - 2048), (uint16_t)-2048) && seq_xcond(X_TAG_NUM(STATION(0) + 2047), 2090));
    /* 파라미터 값 == v, 자릿수 파라미터의 한 자리(TFB 십의 자리 회전 모드, 일의 자리 전후진 모드) */
    assert(param_set(P_TURN_FORBACK, 12) == 0);
    assert(seq_xcond(X_PARAM(TFB, 12), 0) && seq_xcond(X_PARAM(TFB_1, 2), 0) && seq_xcond(X_PARAM(TFB_10, 1), 0));
    assert(!seq_xcond(X_PARAM(TFB_1, 1), 0) && !seq_xcond(X_PARAM(TFB, 2), 0) && seq_xcond(X_NOT(X_PARAM(TFB_1, 1)), 0));
    assert(param_set(P_TURN_FORBACK, 11) == 0);
    assert(!seq_xcond(X_PARAM(TFB_1, 2), 0) && seq_xcond(X_PARAM(TFB_1, 1), 0));
    assert(seq_xcond(X_PARAM_GE(P_STATION_NO, 4), 0) && !seq_xcond(X_PARAM_GE(P_STATION_NO, 5), 0));
    assert(seq_xcond(X_PARAM_LE(P_STATION_NO, 4), 0) && !seq_xcond(X_PARAM_LE(P_STATION_NO, 3), 0));
    assert(seq_xcond(X_ID(7, 7), 0) && !seq_xcond(X_NOT(X_ID(7, 7)), 0));
    assert(seq_xcond(X_FORWARD, 0) && !seq_xcond(X_BACKWARD, 0));
    D.flag_stopping = 1;
    assert(seq_xcond(X_STATION(1, 42, 42), 0) && !seq_xcond(X_STATION_RUNNING(1, 42, 42), 0));
    D.flag_stopping = 0;
    assert(seq_xcond(X_STATION_RUNNING(1, 42, 42), 0));
    D.cross_out_flag = 1;
    assert(seq_xcond(X_CROSS_OUT, 0) && !seq_xcond(X_CROSS_OUT_ON, 0) && !seq_xcond(X_CROSS_IN, 0));
    D.cross_out_flag = 0;
    D.cross_out_on = 1;
    assert(seq_xcond(X_CROSS_OUT, 0) && seq_xcond(X_CROSS_OUT_ON, 0));
    D.cross_out_on = 0;
    assert(seq_xcond(X_ALWAYS, 0) && !seq_xcond(0x7F, 0, 0, 0));
    acs_station_reset();

    /* 두 번째 묶음 */
    start(S_ANY2);
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == 0);
    fake_in = BIT(SEN_D);
    seq_run();
    assert(fake_out == 0);
    fake_in = BIT(SEN_B) | BIT(SEN_D) | BIT(SEN_E);   /* WAIT 통과, WHEN 은 SEN_E 로 거짓 */
    seq_run();
    assert(fake_out == BIT(O_A) && seq_is_active());
    fake_in = BIT(SEN_A) | BIT(SEN_C);
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B)));

    /* 파라미터 11분 = 66000 틱 (16비트 넘음) */
    (void)param_set(P_AUTO_START_SEC, 11);
    start(S_DPARAM);
    for (long t = 0; t < 66000L; t++) { seq_run(); }
    assert(seq_is_active() && fake_out == 0);
    seq_run();
    assert(!seq_is_active() && fake_out == BIT(O_A));

    /* 틱 단위는 DELAY 와 같다(값 초과), 초 단위 제한 시간, 값 0 = 제한 없음 */
    (void)param_set(P_STATION_NO, 2);
    (void)param_set(P_AUTO_START_SEC, 1);
    start(S_TPARAM);
    seq_run(); seq_run();
    assert(fake_out == 0);
    seq_run();
    assert(fake_out == BIT(O_A));
    for (int t = 0; t < 99; t++) { seq_run(); }
    assert(seq_is_active() && fake_out == BIT(O_A));
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(O_C)));
    (void)param_set(P_AUTO_START_SEC, 0);
    start(S_TPARAM);
    for (int t = 0; t < 500; t++) { seq_run(); }
    assert(seq_is_active() && fake_out == BIT(O_A));
    fake_in = BIT(SEN_A);
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(O_A) | BIT(O_B)));
    (void)param_set(P_AUTO_START_SEC, 0);

    /* 회전 비대기: 회전 중에도 다음 스캔에 다음 스텝. 번호 큐[2] */
    acs_reset();
    acs_station_set(2, 123, 3);
    fake_tx_count = 0;
    start(S_NOWAIT);
    fake_turn_count = 0;
    D.turn_pulses = 100;
    seq_run();
    assert(fake_turn_count == 1 && fake_out == 0);
    seq_run();
    assert(fake_turn_count == 1 && fake_out == BIT(O_A) && !seq_is_active() && acs_pending() == 1);
    acs_schedule();
    assert(fake_tx_count == 1 && fake_tx[0][6] == 'L' && fake_tx[0][7] == '1' && fake_tx[0][8] == '2' && fake_tx[0][9] == '3');
    D.turn_pulses = 0;
    acs_reset();

    /* 32비트 칸의 상위 워드: 입력 IF_1~5(ID 27~31), 출력 IF_1·램프·부저(ID 16 이상) */
    start(S_HIGH);
    seq_run();
    assert(seq_is_active() && fake_out == 0);
    fake_in = BIT(IN_IF_1) | BIT(IN_IF_2);               /* 끔 조건이 상위 워드 */
    seq_run();
    assert(fake_out == 0);
    fake_in = BIT(IN_IF_1);
    seq_run();
    assert(fake_out == (BIT(OUT_IF_1) | BIT(OUT_LAMP_G)));
    fake_in = BIT(IN_IF_4) | BIT(IN_IF_5);               /* 하나라도 켬(IF_3·4) + 끔(IF_5): 아직 거짓 */
    seq_run();
    assert(seq_is_active() && fake_out == (BIT(OUT_IF_1) | BIT(OUT_LAMP_G)));
    fake_in = BIT(IN_IF_4);
    seq_run();
    assert(!seq_is_active() && fake_out == (BIT(OUT_IF_1) | BIT(OUT_BUZZER)));
    if (s_packed_mode) {                                 /* 압축 표에 칸 7·9·11(상위 워드)이 실제로 있다 */
        uint16_t k, at = s_pseqs[NSEQS - 1u].off, m = 0;
        for (k = 0; k < s_pseqs[NSEQS - 1u].n; k++) { m |= s_pw[at]; at = (uint16_t)(at + SEQ_PK_LEN(s_pw[at])); }
        assert((m & (1u << 7)) && (m & (1u << 9)) && (m & (1u << 11)));
    }
}

/* 펼친 표와 압축 표를 같은 난수 입력으로 돌려 틱마다 견준다(점프·WHEN 묶음·타임아웃·CALL·ACT·벨트 포함) */
#define LS_TICKS 40000u
static uint32_t s_ls[LS_TICKS][5];

static long lockstep(int packed)
{
    uint32_t rng = 0x2545F491u, t;
    long busy = 0;

    s_packed_mode = packed;
    for (t = 0; t < SEQ_STATE_COUNT; t++) { seq_state_set((uint16_t)t, 0); }
    fake_reset();
    memset(&D, 0, sizeof(D));
    D.ui_mode = UI_AUTO;
    acs_reset();
    call_count = 0;
    assert(bind(&s_site_ok) == 0);
    seq_abort_all();
    act_abort_all();
    for (t = 0; t < LS_TICKS; t++) {
        uint32_t rec[5];
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        if ((rng & 7u) == 0u) {
            fake_in = ((rng >> 3) & 0x3FFu) | (((rng >> 20) & 1u) ? BIT(IN_START_SW) : 0u) | (((rng >> 21) & 31u) << IN_IF_1);
        }
        if (!seq_is_active() && (((rng >> 8) & 15u) == 0u)) { seq_request((uint16_t)(S_BRANCH + ((rng >> 12) % (S_HIGH - S_BRANCH + 1)))); }
        if (((rng >> 16) & 63u) == 0u) { D.move_on = MOVE_DONE; }
        if (((rng >> 22) & 255u) == 0u) { D.action_err = 0; D.stop_err = 0; D.system_err = 0; }
        if (((rng >> 24) & 31u) == 0u) { (void)act_start((uint16_t)(ACT_PULSE + ((rng >> 26) % 3u))); }
        if (((rng >> 4) & 1023u) == 0u) { seq_abort_all(); act_abort_all(); }
        call_ret = (int)((rng >> 28) % 3u);
        D.turn_pulses = 0; D.turn_motion = 0; D.flag_turn = 0; D.tape_align = 0; D.tape_fix_end = 0;
        seq_run();
        act_run();
        busy += seq_is_active();
        rec[0] = fake_out;
        rec[1] = ((uint32_t)seq_active_id() << 16) | (seq_is_active() ? seq_active_step() : 0u);   /* 쉬면 스텝 번호는 앞 판 값이 남는다 */
        rec[2] = ((uint32_t)(uint16_t)D.action_err << 16) | (uint16_t)D.stop_err;
        rec[3] = (uint32_t)(act_is_busy(ACT_PULSE) | (act_is_busy(ACT_HOLD) << 1) | (act_is_busy(ACT_FAIL) << 2))
               | ((uint32_t)fake_restart_count << 8) | ((uint32_t)call_count << 16);
        rec[4] = ((uint32_t)fake_tx_count << 16) | (uint16_t)D.obs_use;
        if (!packed) { memcpy(s_ls[t], rec, sizeof(rec)); }
        else if (memcmp(s_ls[t], rec, sizeof(rec)) != 0) {
            printf("  압축 표 틱 %lu 다름: out %08lx/%08lx seq %08lx/%08lx err %08lx/%08lx\n", (unsigned long)t,
                   (unsigned long)s_ls[t][0], (unsigned long)rec[0], (unsigned long)s_ls[t][1], (unsigned long)rec[1],
                   (unsigned long)s_ls[t][2], (unsigned long)rec[2]);
            assert(0);
        }
    }
    seq_abort_all();
    act_abort_all();
    return busy;
}

/* 사용자 함수 안에서 같은 ACT 슬롯 다시 시작: 두 표로 돌려 틱마다 견준다 */
static void lockstep_restart(int packed, uint32_t rec[200][2])
{
    uint16_t t;
    s_packed_mode = packed;
    fake_reset();
    memset(&D, 0, sizeof(D));
    assert(bind(&s_site_r) == 0);
    act_abort_all();
    for (t = 0; t < 200u; t++) {
        if (!act_is_busy(ACT_R1) && !act_is_busy(ACT_R2) && ((t % 7u) == 0u)) {
            fake_out = 0;
            (void)act_start(ACT_R1);
        }
        act_run();
        rec[t][0] = fake_out;
        rec[t][1] = (uint32_t)(act_is_busy(ACT_R1) | (act_is_busy(ACT_R2) << 1));
    }
    act_abort_all();
}

int main(void)
{
    static uint32_t rr[2][200][2];
    long busy;
    uint16_t t;

    lockstep_restart(0, rr[0]);
    lockstep_restart(1, rr[1]);
    assert((rr[0][0][0] == BIT(OUT_EXT_4)) && (rr[0][0][1] == 2u));   /* "다음" = 새 표의 1 번(EXT_4). 옛 표 2 번(O_B) 아님 */
    for (t = 0; t < 200u; t++) {
        if ((rr[0][t][0] != rr[1][t][0]) || (rr[0][t][1] != rr[1][t][1])) {
            printf("  ACT 다시 시작: 틱 %u 다름 out %08lx/%08lx busy %lu/%lu\n", (unsigned)t, (unsigned long)rr[0][t][0],
                   (unsigned long)rr[1][t][0], (unsigned long)rr[0][t][1], (unsigned long)rr[1][t][1]);
            assert(0);
        }
    }
    puts("  사용자 함수 안 ACT 다시 시작: 펼친 표·압축 표 같음");
    busy = lockstep(0);
    (void)lockstep(1);
    printf("  펼친 표·압축 표 틱마다 같음(%u 틱, 시퀀스 도는 틱 %ld)\n", LS_TICKS, busy);
    s_packed_mode = 0;
    run_all();
    puts("  펼친 표 ok");
    s_packed_mode = 1;
    run_all();
    printf("  압축 표 ok(스텝 %u 워드, 대응표 %u+%u 줄)\n", (unsigned)s_pwn, (unsigned)NSEQS, (unsigned)NACTS);
    puts("test_seq ok");
    return 0;
}
