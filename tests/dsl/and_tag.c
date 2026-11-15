#include "agv/tag_dsl.h"
#include "same.h"

TAG_TABLE_BEGIN(t_new)
    TAG_IF( 1, ANY_PROFILE, AND( BACKWARD ),                                   AGV_STOP )
    TAG_IF( 2, ANY_PROFILE, AND( ON(SEN_1), BACKWARD ),                        AGV_STOP )
    TAG_IF( 3, ANY_PROFILE, AND( ANY(SEN_1, SEN_2), FORWARD, ID(1, 9) ),       AGV_TURN( CCW, 90, DIR_TOGGLE, 1 ) )
    TAG_IF( 4, ANY_PROFILE, AND( OFF(SEN_1, SEN_2) ),                          AGV_STOP )
    TAG_IF( 5, ANY_PROFILE, AND( NOT(ID(83, 83)), STATION(0, 70, 71), ON(SEN_3) ), AGV_STOP, OBS_ON )
    TAG_IF( 6, ANY_PROFILE, AND( PARAM(TFB_1, 2), CROSS_OUT ),                 AGV_STOP )
    TAG_IF( 7, ANY_PROFILE, AND( ID(1, 9), FORWARD ),                          AGV_STOP )
    TAG_IF( 8, ANY_PROFILE, AND( OFF(SEN_1), ANY(SEN_3, SEN_4), ON(SEN_2), LOW_BATTERY, NOT(CROSS_IN) ), SEQ_START( TAPE, LOAD ) )
    TAG_IF( 9, ANY_PROFILE, X_BACKWARD,                                        AGV_STOP )
    TAG_IF( 10, ANY_PROFILE, IF_X2( X_FORWARD, X_ID(1, 9) ),                   AGV_STOP )
TAG_TABLE_END

TAG_TABLE_BEGIN(t_old)
    TAG_IF( 1, ANY_PROFILE, IF_BACKWARD,                                       AGV_STOP )
    TAG_IF( 2, ANY_PROFILE, IF_X_IN( X_BACKWARD, IN( ON(SEN_1) ) ),            AGV_STOP )
    TAG_IF( 3, ANY_PROFILE, IF_X2_IN( X_FORWARD, X_ID(1, 9), IN( ANY_ON(SEN_1, SEN_2) ) ), AGV_TURN( CCW, 90, DIR_TOGGLE, 1 ) )
    TAG_IF( 4, ANY_PROFILE, IF_IN( IN( OFF(SEN_1, SEN_2) ) ),                  AGV_STOP )
    TAG_IF( 5, ANY_PROFILE, IF_X2_IN( X_NOT(X_ID(83, 83)), X_STATION(0, 70, 71), IN( ON(SEN_3) ) ), AGV_STOP, OBS_ON )
    TAG_IF( 6, ANY_PROFILE, IF_X2( X_PARAM(TFB_1, 2), X_CROSS_OUT ),           AGV_STOP )
    TAG_IF( 7, ANY_PROFILE, IF_X2( X_ID(1, 9), X_FORWARD ),                    AGV_STOP )
    TAG_IF( 8, ANY_PROFILE, IF_X2_IN( X_LOW_BATTERY, X_NOT(X_CROSS_IN), IN( ANY_ON(SEN_3, SEN_4), OFF(SEN_1), ON(SEN_2) ) ), SEQ_START( TAPE, LOAD ) )
    TAG_IF( 9, ANY_PROFILE, IF_BACKWARD,                                       AGV_STOP )
    TAG_IF( 10, ANY_PROFILE, IF_X2( X_FORWARD, X_ID(1, 9) ),                   AGV_STOP )
TAG_TABLE_END

MAIN_BEGIN
    SAME(t_new, t_old);
MAIN_END
