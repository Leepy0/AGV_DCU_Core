/*
 * test_dsl.c - seq_dsl.h 매크로가 만드는 표의 값을 검사한다.
 *
 * 호스트: gcc -std=c99 -DAGV_HOST_BUILD -Iinclude tests/test_dsl.c -o build/test_dsl && build/test_dsl
 * 타깃:   cl2000 -v28 -ml -mt --c99 -Iinclude --compile_only tests/test_dsl.c
 * 컴파일 오류 시험: -DNEG_SAME_SIGNAL, -DNEG_AMP_INPUT, -DNEG_NO_DIRECTION 각각 컴파일이 실패해야 한다.
 */
#include "agv/seq_dsl.h"

#define SEN_A     IN_EXT_1
#define SEN_B     IN_EXT_2
#define SEN_C     IN_EXT_3
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
#ifdef NEG_SAME_SIGNAL
    WAIT( IN( ON(SEN_A), OFF(SEN_A) ) )
#endif
#ifdef NEG_AMP_INPUT
    WAIT( IN( ON(IN_AMP_L_OVERLOAD) ) )
#endif
#ifdef NEG_NO_DIRECTION
    BELT_ON( 3, CCW )
#endif
SEQ_END

ACT_BEGIN( signal_3x )
    OUT_PULSE( OUT_EXT_7, 50, 50, 3 )
ACT_END

SEQ_MAP_BEGIN( site_seqs )
    SEQ_MAP( 1, SEQ_LOAD, demo )
SEQ_MAP_END

ACT_MAP_BEGIN( site_acts )
    ACT_MAP( 1, ACT_DEMO, signal_3x )
ACT_MAP_END

TAG_TABLE_BEGIN( site_tags )
    TAG( 4, 1, A_STOP, A_SEQ(SEQ_LOAD) )
    TAG( 21, ANY_PROFILE, A_TURN(CW, 90, 0, 1, 0) )
    TAG_IF( 36, 1, IF_STATION(1, 101, 116), A_ACT(ACT_DEMO) )
TAG_TABLE_END

#ifdef __TMS320C28XX__
/* 타깃 크기 확인: 스텝 12워드, 태그 규칙 9워드 */
typedef char step_size_is_12[(sizeof(seq_step_t) == 12) ? 1 : -1];
typedef char rule_size_is_9[(sizeof(tag_rule_t) == 9) ? 1 : -1];
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

    assert(site_seqs[0].nsteps == sizeof(demo_steps) / sizeof(seq_step_t));
    assert(site_acts[0].id == ACT_DEMO && site_acts[0].nsteps == 1);
    assert(site_tags[1].act[0] == (TA_TURN | TURN_CW | (1 << 2)) && site_tags[1].act[1] == (1 << 4));
    assert((site_tags[2].cond & 0xFF) == TAG_COND_STATION_RANGE && (site_tags[2].cond >> 8) == 1);
    assert(site_tags[2].cond_a == 101 && site_tags[2].cond_b == 116);
    assert(site_tags[2].act[0] == (TA_ACT | ACT_DEMO));

    puts("test_dsl ok");
    return 0;
}
#endif
