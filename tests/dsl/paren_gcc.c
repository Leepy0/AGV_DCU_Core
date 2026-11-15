#include "agv/tag_dsl.h"
#include "same.h"
/* 낱말을 펼치기 전에 붙이는 ACS·AGV_SPEED 는 번호·속도 칸이 "(" 로 시작하면 못 받는다(gcc 붙이기 오류. cl2000 은 경고만 내고 지나가서
 * 이 시험은 gcc 만 돈다). 괄호 없이 쓰거나 ACS_SEND·AGV_SPEED_MMIN 을 쓴다 */
TAG_TABLE_BEGIN(n)
#if defined(NEG_ACS_PAREN)
    TAG( 1, ANY_PROFILE, ACS( NEXT, (70 + 7) ) )
#elif defined(NEG_SPEED_PAREN)
    TAG( 1, ANY_PROFILE, AGV_SPEED( (20 + 10) ) )
#else
    TAG( 1, ANY_PROFILE, ACS( NEXT, 70 + 7 ), AGV_SPEED( 20 + 10 ) )
    TAG( 2, ANY_PROFILE, ACS_SEND( NEXT, (70 + 7) ), AGV_SPEED_MMIN( (20 + 10) ) )
#endif
TAG_TABLE_END
MAIN_BEGIN
MAIN_END
