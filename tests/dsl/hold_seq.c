/* 시퀀스 대기 계열(v0.61.0): 새 꼴이 정한 칸에 들어가는가(손으로 쓴 스텝과 바이트 비교), 기존 꼴 바이트 불변, -DNEG_* 컴파일 오류 */
#include "agv/seq_dsl.h"
#include "same.h"
#ifndef NEG
SEQ_LABELS( A, B )
SEQ_BEGIN( s_new )
    WAIT_ACCUM( IN( ON(SEN_1) ), 200 )
    WAIT_ACCUM( AND( ON(SEN_1), OFF(SEN_2) ), 0 )
    WAIT_ACCUM_TO( IN( ON(SEN_1) ), 65534, 500, A )
    WAIT_ACCUM_TO( IN( OFF(SEN_2) ), 30, UNMATCHED(100), B )
    WAIT_STABLE( IN( ON(SEN_1) ), 300 )
    WAIT_STABLE_TO( IN( ON(SEN_1) ), 300, UNMATCHED(2000), A )
SEQ_END
SEQ_BEGIN( s_ref )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 200, 1, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 0, 1, 0, IN( ON(SEN_1), OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, A, 65534, 1, 500, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, B, 30, 1, 0x8000 | 100, IN( OFF(SEN_2) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, SEQ_NO_LABEL, 300, 0, 0, IN( ON(SEN_1) ), 0 )
    DSL_STEP_C( SK_WAIT_STABLE, A, 300, 0, 0x8000 | 2000, IN( ON(SEN_1) ), 0 )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_ref_steps);
MAIN_END
#elif defined(NEG_TAG_ACCUM) || defined(NEG_TAG_ACCUM_TO)
#include "agv/tag_dsl.h"
TAG_TABLE_BEGIN( n )
#if defined(NEG_TAG_ACCUM)
    TAG( 1, 0, WAIT_ACCUM( IN( ON(SEN_1) ), 10 ) )
#else
    TAG( 1, 0, WAIT_ACCUM_TO( IN( ON(SEN_1) ), 10, 100, 1 ) )
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
#endif
SEQ_END
#endif
