#include "agv/seq_dsl.h"
#include "same.h"
#define SPEED_FAST 30
#ifndef NEG
SEQ_BEGIN( s_new )
    AGV_SPEED( HIGH )
    AGV_SPEED( LOW )
    AGV_SPEED( LLOW )
    AGV_SPEED( MIN )
    AGV_SPEED( 4 )
    AGV_SPEED( 30 )
    AGV_SPEED( 255 )
    AGV_SPEED( SPEED_FAST )
    AGV_SPEED( 20 + 10 )
SEQ_END
SEQ_BEGIN( s_old )
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_HIGH, 0, 0, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_LOW, 0, 0, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_LLOW, 0, 0, 0, 0, 0)
    DSL_STEP(SK_SPEED, SEQ_NO_LABEL, SEQ_SPD_MIN, 0, 0, 0, 0, 0)
    AGV_SPEED_MMIN( 4 )
    AGV_SPEED_MMIN( 30 )
    AGV_SPEED_MMIN( 255 )
    AGV_SPEED_MMIN( SPEED_FAST )
    AGV_SPEED_MMIN( 30 )
SEQ_END
SEQ_BEGIN( s_mmin )
    AGV_SPEED( 4 )
    AGV_SPEED( 30 )
    AGV_SPEED( 255 )
    AGV_SPEED( SPEED_FAST )
SEQ_END
SEQ_BEGIN( s_mmin_old )
    AGV_SPEED_MMIN( 4 )
    AGV_SPEED_MMIN( 30 )
    AGV_SPEED_MMIN( 255 )
    AGV_SPEED_MMIN( SPEED_FAST )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_old_steps);
    SAME(s_mmin_steps, s_mmin_old_steps);
MAIN_END
#else
SEQ_BEGIN( n )
#if defined(NEG_0)
    AGV_SPEED( 0 )
#elif defined(NEG_3)
    AGV_SPEED( 3 )
#elif defined(NEG_256)
    AGV_SPEED( 256 )
#elif defined(NEG_WORD)
    AGV_SPEED( FAST )
#elif defined(NEG_NEG)
    AGV_SPEED( -5 )
#elif defined(NEG_EXPR)
    AGV_SPEED( (20 + 10) )
#elif defined(NEG_LOWER)
    AGV_SPEED( high )
#elif defined(NEG_EMPTY)
    AGV_SPEED( )
#endif
SEQ_END
#endif
