#include "agv/seq_dsl.h"
#include "same.h"
#define OLD_ACS(info, src)       DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_##src, 0, 0, 0, 0)
SEQ_LABELS( A, B, C )
SEQ_BEGIN( s_new )
    AGV_SPEED( HIGH )
    WHEN( AND( ANY(SEN_1, SEN_2), OFF(SEN_3), ANY(SEN_4, SEN_5) ), B )
    ACS( NEXT, STATION(0) + 1 )
    AGV_SPEED( 25 )
    WAIT_TO( AND( ANY(SEN_1, SEN_2), ID(1, 9) ), 200, C )
    ACS( CALL, 12 )
SEQ_END
SEQ_BEGIN( s_old )
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_HIGH, 0, 0, 0, 0, 0)
    WHEN_ANY2( IN( ANY_ON(SEN_1, SEN_2), OFF(SEN_3) ), ANY2_ON(SEN_4, SEN_5), B )
    OLD_ACS( NEXT, STATION0_PLUS1 )
    AGV_SPEED_MMIN( 25 )
    WAIT_IF_IN_TO( X_ID(1, 9), IN( ANY_ON(SEN_1, SEN_2) ), 200, C )
    ACS_SEND( CALL, 12 )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_old_steps);
MAIN_END
