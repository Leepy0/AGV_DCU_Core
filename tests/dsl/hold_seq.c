/* 시퀀스 대기 계열(v0.61.0): 새 꼴이 정한 칸에 들어가는가(손으로 쓴 스텝과 바이트 비교), 기존 꼴 바이트 불변, -DNEG_* 컴파일 오류 */
#include "agv/seq_dsl.h"
#include "same.h"
#define MY_T 250
#ifndef NEG
SEQ_LABELS( A, B )
SEQ_BEGIN( s_new )
    WAIT_ACCUM( IN( ON(SEN_1) ), 200 )
    WAIT_ACCUM( AND( ON(SEN_1), OFF(SEN_2) ), 0 )
    WAIT_ACCUM_TO( IN( ON(SEN_1) ), 65534, 500, A )
    WAIT_ACCUM_TO( IN( OFF(SEN_2) ), 30, UNMATCHED(100), B )
    WAIT_STABLE( IN( ON(SEN_1) ), 300 )
    WAIT_STABLE_TO( IN( ON(SEN_1) ), 300, UNMATCHED(2000), A )
    WAIT_STABLE( IN( ON(SEN_1) ), MY_T )
    WAIT_STABLE( IN( ON(SEN_1) ), (2 * 50) )
    WAIT_STABLE( IN( ON(SEN_1) ), PARAM_TIME( ST, TICK ) )
    WAIT_STABLE_TO( AND( OFF(SEN_2) ), PARAM_TIME( P_AUTO_START_SEC, SEC ), UNMATCHED(100), A )
    WAIT_ACCUM( IN( ON(SEN_1) ), PARAM_TIME( AS, DSEC ) )
    WAIT_ACCUM_TO( IN( ON(SEN_1) ), PARAM_TIME( ACC, SEC ), 300, B )
    WHEN_STABLE( IN( ON(SEN_1) ), 300, A )
    WHEN_STABLE( IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), 0, B )
    WHEN_ACCUM( AND( ON(SEN_1), OFF(SEN_2) ), PARAM_TIME( ST, SEC ), B )
    WHEN_ACCUM( IN( OFF(SEN_H) ), 65534, A )
SEQ_END
SEQ_BEGIN( s_ref )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 200, 1, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 0, 1, 0, IN( ON(SEN_1), OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, A, 65534, 1, 500, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, B, 30, 1, 0x8000 | 100, IN( OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 300, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, A, 300, 0, 0x8000 | 2000, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 250, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 100, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, P_STATION_NO, 1u << 8, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, A, P_AUTO_START_SEC, 100u << 8, 0x8000 | 100, IN( OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, P_AUTO_START_SEC, (10u << 8) | 1u, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, B, P_ACC, (100u << 8) | 1u, 300, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WHEN_STABLE, A, 300, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WHEN_STABLE, B, 0, 0, 0, IN( ON(SEN_1), ANY_ON(SEN_2, SEN_3) ), 0 )
    DSL_STEP_C( SK_WHEN_STABLE, B, P_STATION_NO, (100u << 8) | 1u, 0, IN( ON(SEN_1), OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WHEN_STABLE, A, 65534, 1, 0, IN( OFF(SEN_H) ), 0 )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_ref_steps);
MAIN_END
#elif defined(NEG_TAG_ACCUM) || defined(NEG_TAG_ACCUM_TO) || defined(NEG_TAG_WHEN_STABLE) || defined(NEG_TAG_WHEN_ACCUM)
#include "agv/tag_dsl.h"
TAG_TABLE_BEGIN( n )
#if defined(NEG_TAG_ACCUM)
    TAG( 1, 0, WAIT_ACCUM( IN( ON(SEN_1) ), 10 ) )
#elif defined(NEG_TAG_ACCUM_TO)
    TAG( 1, 0, WAIT_ACCUM_TO( IN( ON(SEN_1) ), 10, 100, 1 ) )
#elif defined(NEG_TAG_WHEN_STABLE)
    TAG( 1, 0, WHEN_STABLE( IN( ON(SEN_1) ), 10, 1 ) )
#else
    TAG( 1, 0, WHEN_ACCUM( IN( ON(SEN_1) ), 10, 1 ) )
#endif
TAG_TABLE_END
#else
SEQ_LABELS( A, B )
SEQ_BEGIN( n )
#if defined(NEG_ACCUM_STATE)
    WAIT_ACCUM( AND( ON(SEN_1), FORWARD ), 10 )
#elif defined(NEG_ACCUM_ANY2)
    WAIT_ACCUM_TO( AND( ANY(SEN_1, SEN_2), ANY(SEN_3, SEN_4) ), 10, 100, A )
#elif defined(NEG_ACCUM_BIG)
    WAIT_ACCUM( IN( ON(SEN_1) ), 65535 )
#elif defined(NEG_ACCUM_NEG)
    WAIT_ACCUM( IN( ON(SEN_1) ), (0 - 1) )
#elif defined(NEG_HOLD_MIN)
    WAIT_STABLE( IN( ON(SEN_1) ), PARAM_TIME( AS, MIN ) )
#elif defined(NEG_HOLD_PLACE)
    WAIT_STABLE( IN( ON(SEN_1) ), PARAM_TIME( TFB_1, SEC ) )
#elif defined(NEG_HOLD_PCOUNT)
    WAIT_ACCUM( IN( ON(SEN_1) ), PARAM_TIME( P_COUNT, SEC ) )
#elif defined(NEG_HOLD_FND)
    WAIT_STABLE_TO( IN( ON(SEN_1) ), PARAM_TIME( ZZZ, SEC ), 100, A )
#elif defined(NEG_HOLD_STATE_PARAM)
    WAIT_STABLE( AND( ON(SEN_1), FORWARD ), PARAM_TIME( AS, SEC ) )
#elif defined(NEG_WHENHOLD_STATE)
    WHEN_STABLE( AND( ON(SEN_1), BACKWARD ), 10, A )
#elif defined(NEG_WHENHOLD_ANY2)
    WHEN_ACCUM( AND( ANY(SEN_1, SEN_2), ANY(SEN_3, SEN_4) ), 10, A )
#elif defined(NEG_WHENHOLD_BIG)
    WHEN_STABLE( IN( ON(SEN_1) ), 65535, A )
#elif defined(NEG_WHENHOLD_MIN)
    WHEN_ACCUM( IN( ON(SEN_1) ), PARAM_TIME( AS, MIN ), A )
#endif
SEQ_END
#endif
