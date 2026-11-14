/* test_seq.c - 시퀀스·ACT 인터프리터 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/seq_dsl.h"
#include "agv/acs.h"
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

enum {
    S_BRANCH = SEQ_SITE_BASE, S_UNMATCHED, S_TIMING, S_ACS, S_DRIVE, S_BUDGET, S_OUTS, S_FLICKER,
    S_ANY, S_HANDSHAKE, S_WAIT_OUT, S_CALL, S_BAD_LABEL, S_ACT_START, S_PROF, S_NO_TABLE,
    S_XWHEN, S_XWAIT, S_ANY2, S_DPARAM, S_TPARAM, S_NOWAIT, S_MOVE, S_ALIGN, S_MOVE_TAPE,
    S_BELTS, S_BELT_OFF, S_BELT_END, S_BELT_MANY, S_OBS_AUTO
};

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
    assert(seq_bind(&site) == 77);
    assert(act_start(ACT_PULSE) == 2);   /* 거부된 표의 ACT 는 하나도 시작하지 않는다 */
}

#define BAD(t) bad_act_rejected(t##_steps, (uint16_t)(sizeof(t##_steps) / sizeof(seq_step_t)))

int main(void)
{
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

    assert(seq_bind(&s_site_ok) == 0);
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
    assert(seq_bind(&s_site_ok) == 0);

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
    assert(seq_bind(&s_site_ok) == 0);

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
    assert(seq_bind(&s_site_ok) == 0);

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
    assert(acs_pending() == 4);
    seq_run();
    assert(fake_out == 0);
    acs_on_ack('L', '0'); acs_on_ack('U', '0'); acs_on_ack('A', '0'); acs_on_ack('F', '0');
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

    /* 핸드셰이크 */
    start(S_HANDSHAKE);
    seq_run();
    assert(fake_out == BIT(O_A));
    fake_in = BIT(SEN_A);
    seq_run();
    assert(fake_out == 0);
    seq_run();
    assert(seq_is_active());
    fake_in = 0;
    seq_run();
    assert(fake_out == BIT(O_B));
    start(S_HANDSHAKE);
    for (int i = 0; i < 6; i++) { seq_run(); }
    assert(fake_out == BIT(O_C));

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
    assert(seq_bind(&s_site_ok) == 0);
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
    assert(seq_bind(&s_site_ok) == 0 && seq_profile() == PF_ONE);
    start(S_PROF);
    seq_run();
    assert(fake_out == BIT(O_A));
    (void)param_set(P_AGV_ID, 7);
    assert(seq_bind(&s_site_ok) == 0 && seq_profile() == PF_TWO);
    start(S_PROF);
    seq_run();
    assert(fake_out == BIT(O_B));
    (void)param_set(P_AGV_ID, 50);
    assert(seq_bind(&s_site_ok) == 0 && seq_profile() == 0);
    (void)param_set(P_AGV_ID, 7);
    assert(seq_bind(&s_site_ok) == 0);

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

    puts("test_seq ok");
    return 0;
}
