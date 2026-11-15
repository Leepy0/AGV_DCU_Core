#include "agv/seq_dsl.h"
#include "same.h"
/* 사이트가 낱말과 같은 이름의 숫자 매크로를 정의해도(legacy_const.h 의 #define AGV_ID 25 등) 시퀀스의 ACS 번호 출처·AGV_SPEED 낱말이 그대로다 */
#define AGV_ID     25
#define STATION0   5
#define HIGH       1
#define LOW        2
SEQ_BEGIN( s_new )
    ACS( FINISH, AGV_ID )
    ACS( LOADING, STATION0 )
    ACS( NEXT, STATION(0) + 1 )
    ACS( CALL, 70 + 7 )
    AGV_SPEED( HIGH )
    AGV_SPEED( LOW )
    AGV_SPEED( 4 )
SEQ_END
#undef AGV_ID
#undef STATION0
#undef HIGH
#undef LOW
SEQ_BEGIN( s_ref )
    DSL_STEP(SK_ACS, SEQ_NO_LABEL, 'F', SEQ_NUM_AGV_ID, 0, 0, 0, 0)
    DSL_STEP(SK_ACS, SEQ_NO_LABEL, 'L', SEQ_NUM_STATION0, 0, 0, 0, 0)
    DSL_STEP(SK_ACS, SEQ_NO_LABEL, 'N', SEQ_NUM_STATION0_PLUS1, 0, 0, 0, 0)
    DSL_STEP(SK_ACS, SEQ_NO_LABEL, 'C', SEQ_NUM_CONST, 77, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_HIGH, 0, 0, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_LOW, 0, 0, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, 4, 0, 0, 0, 0, 0)
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_ref_steps);
MAIN_END
