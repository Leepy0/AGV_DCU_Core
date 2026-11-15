#include "agv/seq_dsl.h"
#include "same.h"
#define OLD_WAIT_TO(c, t, label)  DSL_STEP_C(SK_WAIT, label, 0, 0, t, c, 0)   /* 옛 정의 사본(비교용) */
#define MY_TO 250
#ifndef NEG
SEQ_LABELS( A, B )
SEQ_BEGIN( s_new )
    WAIT_TO( IN( ON(SEN_1) ), 500, A )
    WAIT_TO( IN( ON(SEN_1) ), MY_TO, A )
    WAIT_TO( IN( ON(SEN_1) ), (5 * 100), A )
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( AS, SEC ), A )
    WAIT_TO( IN( OFF(SEN_2) ), PARAM_TIME( P_AUTO_START_SEC, TICK ), B )
    WAIT_TO( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), PARAM_TIME( ST, DSEC ), B )
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( DCC, MIN ), B )
    WAIT_TO( AND( ON(SEN_1), ANY(SEN_2, SEN_3) ), PARAM_TIME( ST, DSEC ), B )
    WAIT_TO( AND( ON(SEN_1), FORWARD ), 300, A )
    WAIT_TO( AND( ON(SEN_1), ANY(SEN_2, SEN_3), ANY(SEN_4, SEN_5) ), 300, A )
    WAIT_TO( AND( OFF(SEN_2) ), PARAM_TIME( P_ACC, SEC ), A )
    WAIT_TO( NO_IN, 32767, A )
SEQ_END
SEQ_BEGIN( s_old )
    OLD_WAIT_TO( IN( ON(SEN_1) ), 500, A )
    OLD_WAIT_TO( IN( ON(SEN_1) ), MY_TO, A )
    OLD_WAIT_TO( IN( ON(SEN_1) ), (5 * 100), A )
    WAIT_TO_PARAM( IN( ON(SEN_1) ), P_AUTO_START_SEC, SEC, A )
    WAIT_TO_PARAM( IN( OFF(SEN_2) ), P_AUTO_START_SEC, TICK, B )
    WAIT_TO_PARAM( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), P_STATION_NO, DSEC, B )
    WAIT_TO_PARAM( IN( ON(SEN_1) ), P_DCC, MIN, B )
    WAIT_TO_PARAM( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), P_STATION_NO, DSEC, B )
    WAIT_IF_IN_TO( X_FORWARD, IN( ON(SEN_1) ), 300, A )
    WAIT_ANY2_TO( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), ANY2_ON(SEN_4, SEN_5), 300, A )
    WAIT_TO_PARAM( IN( OFF(SEN_2) ), P_ACC, SEC, A )
    OLD_WAIT_TO( NO_IN, 32767, A )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_old_steps);
MAIN_END
#else
SEQ_LABELS( A, B )
SEQ_BEGIN( n )
#if defined(NEG_UNMATCHED)
    WAIT_TO( IN( ON(SEN_1) ), UNMATCHED(2000), A )
#elif defined(NEG_BIG)
    WAIT_TO( IN( ON(SEN_1) ), 40000, A )
#elif defined(NEG_NEG)
    WAIT_TO( IN( ON(SEN_1) ), (0 - 5), A )
#elif defined(NEG_STATE_PARAM)
    WAIT_TO( AND( ON(SEN_1), FORWARD ), PARAM_TIME( AS, SEC ), A )
#elif defined(NEG_ANY2_PARAM)
    WAIT_TO( AND( ANY(SEN_1, SEN_2), ANY(SEN_3, SEN_4) ), PARAM_TIME( AS, SEC ), A )
#elif defined(NEG_UNIT)
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( AS, HOUR ), A )
#elif defined(NEG_PARAM0)
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( P_NONE_0, SEC ), A )
#elif defined(NEG_FND)
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( ZZZ, SEC ), A )
#elif defined(NEG_PLACE)
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( TFB_1, SEC ), A )
#elif defined(NEG_PTM_ALONE)
    WAIT( IN( ON(SEN_1) ) ) DELAY( PARAM_TIME( AS, SEC ) )
#elif defined(NEG_UNMATCHED_AND)
    WAIT_TO( AND( ON(SEN_1), FORWARD ), UNMATCHED(2000), A )
#elif defined(NEG_PCOUNT)
    WAIT_TO( IN( ON(SEN_1) ), PARAM_TIME( P_COUNT, SEC ), A )
#endif
SEQ_END
#endif
