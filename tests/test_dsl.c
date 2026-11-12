/*
 * test_dsl.c - seq_dsl.h 매크로가 만드는 표의 값을 검사한다.
 *
 * 호스트: gcc -std=c99 -DAGV_HOST_BUILD -Iinclude tests/test_dsl.c -o build/test_dsl && build/test_dsl
 * 타깃:   cl2000 -v28 -ml -mt --c99 -Iinclude --compile_only tests/test_dsl.c
 * 컴파일 오류 시험: -DNEG_AMP_INPUT, -DNEG_NO_DIRECTION, -DNEG_PARAM_ZERO, -DNEG_MOD_ZERO, -DNEG_HIGH_ANY, -DNEG_TURN_SPEED,
 * -DNEG_ACS1_RESET, -DNEG_PIN_TWICE, -DNEG_PIN_ANY, -DNEG_PIN_ONE 각각 컴파일이 실패해야 한다(tests/run_host.ps1 이 하나씩 본다).
 */
#include "agv/seq_dsl.h"

#define SEN_A     IN_EXT_1
#define SEN_B     IN_EXT_2
#define SEN_C     IN_EXT_3
#define SEN_H     IN_ACTIVE_HIGH(IN_EXT_4)
#define CV1_RUN   OUT_EXT_1
#define CV1_DIR   OUT_EXT_2
#define CV2_CW    OUT_EXT_3
#define CV2_CCW   OUT_EXT_4
#define NOT_WIRED IO_NONE

#define BELT1_CW   OUTS(CV1_RUN)
#define BELT1_CCW  OUTS(CV1_RUN, CV1_DIR)
#define BELT2_CW   OUTS(CV2_CW)
#define BELT2_CCW  OUTS(CV2_CCW)
#define BELT3_CW   OUTS(OUT_EXT_5)
#define BELT3_CCW  NO_OUT

enum { ACT_DEMO = 1 };

SEQ_LABELS( FIRST, SECOND, ERR )

SEQ_BEGIN( demo )
    L( FIRST )
    SELECT_BEGIN
        WHEN( IN( ON(SEN_A), OFF(SEN_B, SEN_C) ),  SECOND )
        WHEN( IN( ON(SEN_B) ),                     ERR )
        WHEN( IN( OFF(SEN_A, SEN_B, SEN_C) ),      ERR )
        WHEN( IN( ON(SEN_A, SEN_B, SEN_C) ),       ERR )
        OTHERWISE( ERR )
    SELECT_END
    L( SECOND )
    BELT_ON( 1, CCW )
    BELT_ON( 2, CW )
    BELT_OFF( 1 )
    OUT_ON( NOT_WIRED )
    WAIT_STABLE_TO( IN( OFF(SEN_A) ), 300, UNMATCHED(2000), ERR )
    OUT_PULSE( OUT_EXT_7, 50, 50, 3 )
    OUT_FLICKER_UNTIL_IN( OUT_EXT_8, 50, IN( ON(SEN_C) ) )
    L( ERR )
    ERROR_HOLD( 290, IN( ON(IN_START_SW) ), SECOND )
    FORBACK( BACK )
    TURN( CW, 90, 1, 1, 0 )
    END_RESTART
    WAIT( ANY_ON(SEN_A, SEN_B) )
    WAIT( IN( ANY_ON(SEN_A, SEN_B), OFF(IN_EXT_4) ) )
#ifdef NEG_AMP_INPUT
    WAIT( IN( ON(IN_AMP_L_OVERLOAD) ) )
#endif
#ifdef NEG_NO_DIRECTION
    BELT_ON( 3, CCW )
#endif
#ifdef NEG_PARAM_ZERO
    DELAY_PARAM( 0, SEC )
#endif
SEQ_END

/* 확장 조건, 두 번째 묶음, 파라미터 시간, 회전 비대기, 큐[k] 번호 */
SEQ_BEGIN( ext )
    SELECT_BEGIN
        WHEN_IF( X_BACKWARD, SECOND )                                        /* 0 */
        WHEN_IF_IN( X_ID(21, 23), IN( ON(SEN_A) ), SECOND )                  /* 1 */
        WHEN_ID( 30, 30, ERR )                                               /* 2 */
        WHEN_ANY2( IN( ANY_ON(SEN_A, SEN_B) ), ANY2_ON(SEN_C, IN_EXT_4), ERR )   /* 3 */
    SELECT_END
    WAIT_IF( X_NOT(X_CROSS_IN_ON) )                                          /* 4 */
    WAIT_IF_TO( X_PARAM_GE(P_STATION_NO, 5), 300, ERR )                      /* 5 */
    WAIT_IF_IN_TO( X_KEY(UP), IN( OFF(SEN_A) ), 100, ERR )                   /* 6 */
    WAIT_ANY2_TO( IN( ANY_ON(SEN_A, SEN_B), OFF(SEN_C) ), ANY2_ON(IN_EXT_5, IN_EXT_6), 50, ERR )  /* 7 */
    DELAY_PARAM( P_AUTO_START_SEC, MIN )                                     /* 8 */
    WAIT_TO_PARAM( IN( ON(SEN_B) ), P_STATION_NO, DSEC, ERR )                /* 9 */
    TURN_NOWAIT( CCW, 90, 0, 1, 0 )                                          /* 10 */
    ACS( LOADING, STATION(3) )                                               /* 11 */
    ACS( NEXT, STATION0 )                                                    /* 12 */
    WAIT( IN( ON(IN_EXT_1), ON(IN_EXT_2), ON(IN_EXT_3), OFF(IN_EXT_4), OFF(IN_EXT_5), ANY_ON(IN_EXT_6, IN_EXT_7) ) )  /* 13 */
    OUT_ON( OUT_EXT_1, OUT_EXT_2, OUT_EXT_3, OUT_EXT_4, OUT_EXT_5, OUT_EXT_6, OUT_EXT_7, OUT_EXT_8,
            OUT_EXT_9, OUT_EXT_10, OUT_EXT_11, OUT_EXT_12, OUT_EXT_13, OUT_EXT_14, OUT_EXT_15, OUT_EXT_16 )  /* 14 */
    WAIT_IF( X_ERR_EQ(281) )                                                 /* 15 */
    WAIT( IN( ON(SEN_H, SEN_A), OFF(IN_ACTIVE_HIGH(IN_EXT_3), SEN_B) ) )     /* 16 논리 0 활성 */
    ACS_SEND( NEXT, STATION(0) )                                             /* 17 = ACS( NEXT, STATION0 ) */
    ACS_SEND( NEXT, STATION(2) )                                             /* 18 = ACS( NEXT, STATION(2) ) */
    ACS_SEND( LOADING, 599 )                                                 /* 19 */
    WAIT( IN( OFF(SEN_H), ON(IN_EXT_4), ON(SEN_A, SEN_A) ) )                 /* 20 같은 핀 같은 뜻 겹침은 된다 */
#ifdef NEG_HIGH_ANY
    WAIT( ANY_ON(SEN_H) )
#endif
#ifdef NEG_PIN_TWICE
    WAIT( IN( ON(SEN_H), ON(IN_EXT_4) ) )                                    /* 같은 핀 반대 극성: 핀 0 이면서 1 */
#endif
#ifdef NEG_PIN_ANY
    WAIT( IN( ANY_ON(IN_EXT_4, SEN_A), ON(SEN_H) ) )                         /* ON(SEN_H) 가 ANY_ON 에 묻힌다 */
#endif
#ifdef NEG_PIN_ONE
    WAIT( IN( ON(SEN_H, IN_EXT_4) ) )
#endif
SEQ_END

ACT_BEGIN( signal_3x )
    OUT_PULSE( OUT_EXT_7, 50, 50, 3 )
ACT_END

SEQ_MAP_BEGIN( site_seqs )
    SEQ_MAP( 1, SEQ_LOAD, demo )
    SEQ_MAP( 1, SEQ_UNLOAD, ext )
SEQ_MAP_END

ACT_MAP_BEGIN( site_acts )
    ACT_MAP( 1, ACT_DEMO, signal_3x )
ACT_MAP_END

#include "agv/tag_dsl.h"   /* 아래부터 ACS·ACS_SEND·STATION_RESET 은 태그 동작 */

TAG_TABLE_BEGIN( site_tags )
    TAG( 4, 1, AGV_STOP, SEQ_START( LOAD ) )
    TAG( 21, ANY_PROFILE, AGV_TURN(CW, 90, 0, 1, 0) )
    TAG_IF( 36, 1, IF_STATION(1, 101, 116), ACT_START(ACT_DEMO) )
    TAG_IF( 6,  1, IF_IN( ON(SEN_C) ), SEQ_START( UNLOAD ), AGV_FORBACK(TOGGLE) )                     /* 3 */
    TAG_IF( 10, 1, IF_IN( ANY_ON(SEN_A, SEN_B) ), SEQ_START( LOAD ) )                               /* 4 */
    TAG_IF( 61, 1, IF_X2( X_ID(5, 5), X_STATION(1, 42, 42) ), AGV_TURN(CCW, 90, 0, 1, 0) )        /* 5 */
    TAG_IF( 62, 1, IF_X_IN( X_ID(1, 9), IN( OFF(SEN_A) ) ), AGV_STOP )                              /* 6 */
    TAG_IF( 30, 1, IF_TAG_MINUS_EQ_STATION(1, 20), AGV_STOP )                                       /* 7 */
    TAG_IF( 31, 1, IF_STATION_MOD(0, 10, 3), AGV_STOP )                                             /* 8 */
    TAG_IF( 32, 1, IF_STATION_DIV(2, 100, 1), AGV_STOP )                                            /* 9 */
    TAG_IF( 33, 1, IF_TAG_EQ_STATION(0), AGV_STOP )                                                 /* 10 */
    TAG_IF( 34, 1, IF_X2_IN( X_FORWARD, X_NOT(X_CROSS_OUT), IN( ON(SEN_A) ) ), AGV_STOP )           /* 11 */
    TAG_IF( 35, 1, IF_BACKWARD, AGV_STOP )                                                          /* 12 */
    TAG( 43, 1, ACS_SEND( ARRIVE, 115 ), ACS( ARRIVE, STATION0 ), STATION_RESET )                 /* 13 */
    TAG( 40, 1, AGV_STOP, ACS_SEND( STANDBY, AGV_ID_MINUS(10) ), STATION_RESET )                  /* 14 */
    TAG( 41, 1, ACS_SEND( STANDBY_RESET, 14 ), ACS( ARRIVE, STATION0 ), AGV_STOP )                /* 15 */
    TAG( 42, 1, ACS_SEND( ARRIVE, 115 ), ACS_SEND( ARRIVE, STATION(1) ) )                         /* 16 */
    TAG( 38, 1, AGV_TURN_ALIGN( CCW, 180, DIR_TOGGLE, 1, 1, 15 ) )                                /* 17 회전 속도 */
#ifdef NEG_MOD_ZERO
    TAG_IF( 37, 1, IF_STATION_MOD(0, 0, 3), AGV_STOP )
#endif
#ifdef NEG_TURN_SPEED
    TAG( 39, 1, AGV_TURN( CW, 90, 0, 1, 0, 32 ) )
#endif
#ifdef NEG_ACS1_RESET
    TAG( 44, 1, ACS( STANDBY_RESET, STATION0 ) )                   /* 1칸에 대기 리셋 비트가 들어갈 자리가 없다 */
#endif
TAG_TABLE_END

#ifdef __TMS320C28XX__
/* 타깃 크기 확인: 스텝 12워드, 태그 규칙 16워드 */
typedef char step_size_is_12[(sizeof(seq_step_t) == 12) ? 1 : -1];
typedef char rule_size_is_16[(sizeof(tag_rule_t) == 16) ? 1 : -1];
#endif

#ifdef AGV_HOST_BUILD
#include <assert.h>
#include <stdio.h>
int main(void)
{
    const seq_step_t *s = demo_steps;

    assert(s[0].kind == SK_LABEL && s[0].target == FIRST && FIRST == 1);
    assert(s[1].kind == SK_WHEN && s[1].on == 0x1UL && s[1].off == 0x6UL && s[1].target == SECOND);
    assert(s[3].on == 0 && s[3].off == 0x7UL);
    assert(s[5].kind == SK_WHEN && s[5].on == 0 && s[5].off == 0);      /* OTHERWISE: 항상 참 */

    /* BELT1 CCW: RUN·DIR 를 켠다. 켜는 쪽과 겹치는 출력은 끄지 않는다 */
    assert(s[7].on == 0x3UL && s[7].off == 0);
    /* BELT2 CW: CCW 출력을 끄고 CW 출력을 켠다 */
    assert(s[8].on == 0x4UL && s[8].off == 0x8UL);
    assert(s[9].on == 0 && s[9].off == 0x3UL);                           /* BELT_OFF */
    assert(s[10].on == 0 && s[10].off == 0);                             /* 미연결 출력 */
    assert(s[11].arg0 == 300 && s[11].timeout == (2000 | SEQ_TO_UNMATCHED) && s[11].target == ERR);
    assert(s[12].kind == SK_OUT_PULSE && s[12].out == 0x40UL && s[12].arg0 == 50 && s[12].arg1 == 50 && s[12].timeout == 3);
    assert(s[15].kind == SK_ERROR_HOLD && s[15].arg0 == 290 && s[15].on == (1UL << IN_START_SW) && s[15].target == SECOND);
    assert(s[16].arg0 == DIR_BACK);
    assert(s[17].arg0 == TURN_CW && s[17].arg1 == 90 && s[17].timeout == (1 | (1 << 4)));

    /* ANY_ON: 같은 마스크가 양쪽 절반에 */
    assert(s[19].on == 0x3UL && s[19].off == 0x3UL);
    assert(s[20].on == 0x3UL && s[20].off == 0xBUL);                     /* any=A·B + 전부 OFF=EXT_4 */

    assert(site_seqs[0].nsteps == sizeof(demo_steps) / sizeof(seq_step_t));
    assert(site_acts[0].id == ACT_DEMO && site_acts[0].nsteps == 1);
    assert(site_tags[1].act[0] == (TA_TURN | TURN_CW | (1 << 2)) && site_tags[1].act[1] == (1 << 4));
    assert((site_tags[2].cond & 0xFF) == TAG_COND_STATION_RANGE && (site_tags[2].cond >> 8) == 1);
    assert(site_tags[2].cond_a == 101 && site_tags[2].cond_b == 116);
    assert(site_tags[2].act[0] == (TA_ACT | ACT_DEMO));

    /* 확장 조건: xc = 코드, arg0·arg1 = 인자, out = 두 번째 묶음 */
    {
        const seq_step_t *e = ext_steps;
        assert(e[0].kind == SK_WHEN && e[0].xc == XC_BACKWARD && e[0].on == 0 && e[0].off == 0 && e[0].out == 0 && e[0].target == SECOND);
        assert(e[1].xc == XC_ID && e[1].arg0 == 21 && e[1].arg1 == 23 && e[1].on == 0x1UL && e[1].off == 0);
        assert(e[2].xc == XC_ID && e[2].arg0 == 30 && e[2].arg1 == 30 && e[2].target == ERR);
        assert(e[3].xc == 0 && e[3].on == 0x3UL && e[3].off == 0x3UL && e[3].out == 0xCUL);
        assert(e[4].kind == SK_WAIT && e[4].xc == (XC_CROSS | (2 << 8) | XC_NOT) && e[4].timeout == 0);
        assert(e[5].xc == (XC_PARAM_GE | (P_STATION_NO << 8)) && e[5].arg0 == 5 && e[5].timeout == 300 && e[5].target == ERR);
        assert(e[6].xc == XC_KEY && e[6].arg0 == KEY_UP && e[6].on == 0 && e[6].off == 0x1UL && e[6].timeout == 100);
        assert(e[7].xc == 0 && e[7].on == 0x3UL && e[7].off == 0x7UL && e[7].out == 0x30UL && e[7].timeout == 50);
        assert(e[8].kind == SK_DELAY_PARAM && e[8].arg0 == P_AUTO_START_SEC && e[8].arg1 == 6000);
        assert(e[9].kind == SK_WAIT_PARAM && e[9].arg0 == P_STATION_NO && e[9].arg1 == 10 && e[9].on == 0x2UL && e[9].xc == 0);
        assert(e[10].kind == SK_TURN_NOWAIT && e[10].arg0 == TURN_CCW && e[10].arg1 == 90 && e[10].timeout == (1 << 4));
        assert(e[11].kind == SK_ACS && e[11].arg0 == 'L' && e[11].arg1 == SEQ_NUM_STATION(3));
        assert(e[12].arg1 == SEQ_NUM_STATION0);
        assert(e[13].on == 0x67UL && e[13].off == 0x78UL);                  /* IN 6개 */
        assert(e[14].kind == SK_OUT && e[14].on == 0xFFFFUL);               /* 신호 16개 */
        assert(e[15].xc == XC_ERR_EQ && e[15].arg0 == 281);
        assert(e[17].arg0 == 'N' && e[17].arg1 == SEQ_NUM_STATION0 && e[17].timeout == 0);
        assert(e[18].arg1 == SEQ_NUM_STATION(2) && e[18].timeout == 0);
        assert(e[19].arg0 == 'L' && e[19].arg1 == SEQ_NUM_CONST && e[19].timeout == 599);
        assert(e[16].on == 0x5UL && e[16].off == 0xAUL);                    /* ON(H) 는 off 쪽, OFF(H) 는 on 쪽 */
        assert(e[20].on == 0x9UL && e[20].off == 0);                        /* OFF(H) = 핀 활성 = ON(IN_EXT_4) */
        assert(demo_steps[1].xc == 0);                                       /* 예전 스텝은 0 */
    }
    /* 태그: 확장 조건 둘 + 입력 조건. 새 칸은 act 뒤 */
    assert(site_tags[0].cond == 0 && site_tags[0].cond2 == 0 && site_tags[0].in_on == 0 && site_tags[0].in_off == 0);
    assert(site_tags[2].cond2 == 0 && site_tags[2].in_on == 0);
    assert(site_tags[3].cond == 0 && site_tags[3].in_on == 0x4UL && site_tags[3].in_off == 0);
    assert(site_tags[3].act[0] == (TA_SEQ | SEQ_UNLOAD) && site_tags[3].act[1] == (TA_FORBACK | DIR_TOGGLE));
    assert(site_tags[4].in_on == 0x3UL && site_tags[4].in_off == 0x3UL);
    assert(site_tags[5].cond == XC_ID && site_tags[5].cond_a == 5 && site_tags[5].cond_b == 5);
    assert(site_tags[5].cond2 == (XC_STATION | (1 << 8)) && site_tags[5].cond2_a == 42 && site_tags[5].cond2_b == 42);
    assert(site_tags[5].act[0] == (TA_TURN | TURN_CCW | (1 << 2)) && site_tags[5].act[1] == (1 << 4) && site_tags[5].in_on == 0);
    assert(site_tags[6].cond == XC_ID && site_tags[6].cond2 == 0 && site_tags[6].in_off == 0x1UL);
    assert(site_tags[7].cond == (XC_TAG_MINUS_EQ_ST | (1 << 8)) && site_tags[7].cond_a == 20);
    assert(site_tags[8].cond == XC_STATION_MOD && site_tags[8].cond_a == 10 && site_tags[8].cond_b == 3);
    assert(site_tags[9].cond == (XC_STATION_DIV | (2 << 8)) && site_tags[9].cond_a == 100 && site_tags[9].cond_b == 1);
    assert(site_tags[10].cond == XC_TAG_EQ_ST);
    assert(site_tags[11].cond == XC_FORWARD && site_tags[11].cond2 == (XC_CROSS | (1 << 8) | XC_NOT) && site_tags[11].in_on == 0x1UL);
    assert(site_tags[12].cond == TAG_COND_BACKWARD && site_tags[12].act[0] == TA_STOP);
    /* 태그 보고·스테이션 (tag_dsl.h) */
    assert(site_tags[13].act[0] == (TA_ACS_NUM | 115) && site_tags[13].act[1] == 'A'
           && site_tags[13].act[2] == (TA_ACS | 'A') && site_tags[13].act[3] == TA_STATION_RESET);
    assert(site_tags[14].act[0] == TA_STOP && site_tags[14].act[1] == (TA_ACS_NUM | 10)
           && site_tags[14].act[2] == ('B' | TA_ACS_AGV_MINUS) && site_tags[14].act[3] == TA_STATION_RESET);
    assert(site_tags[15].act[0] == (TA_ACS_NUM | 14) && site_tags[15].act[1] == ('B' | TA_ACS_STANDBY_RESET)
           && site_tags[15].act[2] == (TA_ACS | 'A') && site_tags[15].act[3] == TA_STOP);
    assert(site_tags[16].act[0] == (TA_ACS_NUM | 115) && site_tags[16].act[1] == 'A'
           && site_tags[16].act[2] == (TA_ACS_NUM | 1) && site_tags[16].act[3] == ('A' | TA_ACS_STATION));
    assert(site_tags[17].act[0] == (TA_TURN_ALIGN | TURN_CCW | (2 << 2))
           && site_tags[17].act[1] == (DIR_TOGGLE | (1 << 4) | (1 << 8) | (15 << TA_TURN_SPEED_SHIFT)));

    puts("test_dsl ok");
    return 0;
}
#endif
