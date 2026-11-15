#include "agv/tag_dsl.h"
#include "same.h"
/* 사이트의 숫자 매크로(#define AGV_ID 25 등)가 태그의 ACS 번호 출처·AGV_SPEED 낱말을 덮지 않는다 */
#define AGV_ID     25
#define STATION0   5
#define HIGH       1
#define MIN        9
TAG_TABLE_BEGIN(t_new)
    TAG( 1, ANY_PROFILE, ACS( FINISH, AGV_ID ) )
    TAG( 2, ANY_PROFILE, ACS( NEXT, STATION0 ) )
    TAG( 3, ANY_PROFILE, ACS( NEXT, TAG_NUM + 5 ) )
    TAG( 4, ANY_PROFILE, AGV_SPEED( HIGH ), AGV_SPEED( MIN ), AGV_SPEED( 20 ) )
TAG_TABLE_END
#undef AGV_ID
#undef STATION0
#undef HIGH
#undef MIN
TAG_TABLE_BEGIN(t_ref)
    TAG( 1, ANY_PROFILE, ACS_SEND( FINISH, AGV_ID_MINUS(0) ) )
    TAG( 2, ANY_PROFILE, DSL_TACS_STATION0( NEXT ) )
    TAG( 3, ANY_PROFILE, ACS_SEND( NEXT, TAG_NUM + 5 ) )
    TAG( 4, ANY_PROFILE, TA_SPEED | DSL_TSPD_HIGH, TA_SPEED | DSL_TSPD_MIN, TA_SPEED | 20 )
TAG_TABLE_END
MAIN_BEGIN
    SAME(t_new, t_ref);
MAIN_END
