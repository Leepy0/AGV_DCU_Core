/* 태그 표에서 상태 칸 조건 낱말 STATE(v0.61.0, 시퀀스와 같은 확장 조건)와 ACS( DISPLAY, n ): 손으로 쓴 규칙과 바이트 비교 */
#include "agv/tag_dsl.h"
#include "same.h"
#define XS(k, lo, hi) (XC_STATE | ((k) << 8)), lo, hi   /* 확장 조건 세 값(비교용) */
#ifndef NEG
TAG_TABLE_BEGIN( t_new )
    TAG_IF( 10, 0, AND( STATE( 3, 1 ), ID( 46, 50 ) ), AGV_STOP )
    TAG_IF( 11, 0, AND( ON(SEN_1), NOT( STATE( 2, 1, 65535 ) ) ), OBS_OFF )
    TAG_RANGE_IF( 20, 30, 0, AND( STATE( 15, 4 ), STATE( 14, 0, 3 ) ), ACS( DISPLAY, 13 ) )
    TAG( 12, 0, SET_STATE( 3, 7 ) )
    TAG( 13, 0, AGV_STOP, SET_STATE( 15, 65535 ) )
    TAG( 14, 0, SET_STATE( 0, 0 ) )
TAG_TABLE_END
TAG_TABLE_BEGIN( t_ref )
    TAG_IF( 10, 0, IF_X2( XS( 3, 1, 1 ), X_ID( 46, 50 ) ), AGV_STOP )
    TAG_IF( 11, 0, IF_X_IN( X_NOT( XS( 2, 1, 65535u ) ), IN( ON(SEN_1) ) ), OBS_OFF )
    TAG_RANGE_IF( 20, 30, 0, IF_X2( XS( 15, 4, 4 ), XS( 14, 0, 3 ) ), ACS_SEND( DISPLAY, 13 ) )
    TAG( 12, 0, TA_STATE | 3, 7 )
    TAG( 13, 0, AGV_STOP, TA_STATE | 15, 65535u )
    TAG( 14, 0, TA_STATE | 0, 0 )
TAG_TABLE_END
MAIN_BEGIN
    SAME(t_new, t_ref);
MAIN_END
#else
TAG_TABLE_BEGIN( n )
#if defined(NEG_TAG_STATE3)
    TAG_IF( 10, 0, AND( STATE( 3, 1 ), STATE( 4, 1 ), FORWARD ), AGV_STOP )
#elif defined(NEG_TAG_STATE_SLOT)
    TAG_IF( 10, 0, AND( STATE( 16, 1 ) ), AGV_STOP )
#elif defined(NEG_TAG_DISPLAY_BIG)
    TAG( 10, 0, ACS( DISPLAY, 256 ) )
#elif defined(NEG_TAG_SETSTATE_SLOT)
    TAG( 10, 0, SET_STATE( 16, 1 ) )
#elif defined(NEG_TAG_SETSTATE_SLOTNEG)
    TAG( 10, 0, SET_STATE( -1, 1 ) )
#elif defined(NEG_TAG_SETSTATE_BIG)
    TAG( 10, 0, SET_STATE( 3, 65536 ) )
#elif defined(NEG_TAG_SETSTATE_NEG)
    TAG( 10, 0, SET_STATE( 3, -1 ) )
#endif
TAG_TABLE_END
#endif
