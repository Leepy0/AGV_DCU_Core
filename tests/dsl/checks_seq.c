#include "agv/seq_dsl.h"
#include "same.h"
/* ANY2_ON 표식(64비트 맨 윗비트)이 WHEN_ANY2·WAIT_ANY2 의 둘째 인자에서는 잘려 바이트가 그대로이고, 조건 칸에 들어가면 오류다.
 * SELECT_END_TO·시퀀스 WHEN_IF 의 X_TAG_NUM 검사도 여기서 본다 */
SEQ_LABELS( A, B )
SEQ_BEGIN( s_any2 )
    WHEN_ANY2( IN( ANY_ON(SEN_1, SEN_2), OFF(SEN_3) ), ANY2_ON(SEN_4, SEN_5), A )
    WHEN_ANY2( NO_IN, ANY2_ON(SEN_1), B )
    WAIT_ANY2( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), ANY2_ON(SEN_4, SEN_5, SEN_1) )
    WAIT_ANY2_TO( IN( ANY_ON(SEN_2, SEN_3) ), ANY2_ON(SEN_4, SEN_5), 100, B )
    WAIT_TO( IN( ON(SEN_1), OFF(SEN_2) ), 100, A )
    WAIT_STABLE_TO( IN( OFF(SEN_1) ), 300, UNMATCHED(2000), B )
    SELECT_BEGIN
    WHEN( IN( ON(SEN_1) ), A )
    SELECT_END_TO( 500, B )
SEQ_END
/* 예전 정의를 그대로 풀어 쓴 표 */
SEQ_BEGIN( s_ref )
    DSL_STEP_C( SK_WHEN, A, 0, 0, 0, IN( ANY_ON(SEN_1, SEN_2), OFF(SEN_3) ), DSL_MASK(SEN_4, SEN_5) )
    DSL_STEP_C( SK_WHEN, B, 0, 0, 0, NO_IN, DSL_MASK(SEN_1) )
    DSL_STEP_C( SK_WAIT, SEQ_NO_LABEL, 0, 0, 0, IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), DSL_MASK(SEN_4, SEN_5, SEN_1) )
    DSL_STEP_C( SK_WAIT, B, 0, 0, 100, IN( ANY_ON(SEN_2, SEN_3) ), DSL_MASK(SEN_4, SEN_5) )
    DSL_STEP_C( SK_WAIT, A, 0, 0, 100, IN( ON(SEN_1), OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, B, 300, 0, (2000 | SEQ_TO_UNMATCHED), IN( OFF(SEN_1) ), 0 )
    DSL_STEP_C( SK_WHEN, A, 0, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP( SK_WHEN_TIMEOUT, B, 0, 0, 500, 0, 0, 0 )
SEQ_END
#ifndef NEG
MAIN_BEGIN
    SAME(s_any2_steps, s_ref_steps);
MAIN_END
#else
SEQ_BEGIN( n )
#if defined(NEG_WHEN_ANY2)
    WHEN( ANY2_ON( SEN_1, SEN_2 ), A )
#elif defined(NEG_IN_ANY2)
    WHEN( IN( ANY2_ON( SEN_1, SEN_2 ) ), A )
#elif defined(NEG_IN_ANY2B)
    WAIT( IN( ON( SEN_3 ), ANY2_ON( SEN_1, SEN_2 ) ) )
#elif defined(NEG_WAITTO_ANY2)
    WAIT_TO( IN( ANY2_ON( SEN_1 ) ), 100, A )
#elif defined(NEG_SELTO0)
    SELECT_END_TO( 0, A )
#elif defined(NEG_SELTO_BIG)
    SELECT_END_TO( 40000, A )
#elif defined(NEG_TAGNUM_WHEN_IF)
    WHEN_IF( X_TAG_NUM( STATION(0) + 100 ), A )
#elif defined(NEG_TAGNUM_WAIT_IF)
    WAIT_IF( X_TAG_NUM( STATION(0) + 100 ) )
#elif defined(NEG_TAGNUM_WAIT_IF_IN_TO)
    WAIT_IF_IN_TO( X_TAG_NUM( STATION(0) + 100 ), IN( ON(SEN_1) ), 100, A )
#elif defined(NEG_TAGNUM_AND)
    WHEN( AND( TAG_IS( STATION(0) + 100 ) ), A )
#elif defined(NEG_XST24)
    WHEN_IF( X_STATION( 24, 1, 2 ), A )
#elif defined(NEG_XSTRUN24)
    WHEN_IF( X_STATION_RUNNING( 24, 1, 2 ), A )
#elif defined(NEG_XSTMOD24)
    WHEN_IF( X_STATION_MOD( 24, 2, 0 ), A )
#elif defined(NEG_XSTDIV24)
    WHEN_IF( X_STATION_DIV( 24, 2, 0 ), A )
#elif defined(NEG_IFST24)
    IF_STATION( 24, 1, 2 )  GOTO( A )
#elif defined(NEG_HANDSHAKE_UNMATCHED)
    HANDSHAKE( OUT_EXT_1, IN_EXT_1, UNMATCHED(100), A )
#elif defined(NEG_FLICKER_UNMATCHED)
    OUT_FLICKER_UNTIL_IN_TO( OUT_EXT_1, 10, IN( ON(SEN_1) ), UNMATCHED(100), A )
#endif
SEQ_END
MAIN_BEGIN
MAIN_END
#endif
