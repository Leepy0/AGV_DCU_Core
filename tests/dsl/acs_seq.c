#include "agv/seq_dsl.h"
#include "same.h"
/* 옛 정의 사본(비교용) */
#define OLD_ACS(info, src)       DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, SEQ_NUM_##src, 0, 0, 0, 0)
#define OLD_ACS_SEND(info, num)  DSL_STEP(SK_ACS, SEQ_NO_LABEL, DSL_INFO_##info, DSL_SEND_SRC(num) + DSL_SEND_OK(num), \
                                          DSL_IS_STATION(num) ? 0 : (num), 0, 0, 0)
#define MY_NO 77
#define DSL_INFO_RAWX 'X'   /* IDLE(v0.59.0)를 글자 'X' 로 직접 적은 옛 꼴 */
#ifndef NEG
SEQ_BEGIN( s_new )
    ACS( LOADING, STATION(0) )
    ACS( NEXT, STATION(0) + 1 )
    ACS( ARRIVE, STATION(5) )
    ACS( ARRIVE, STATION(23) )
    ACS( FINISH, AGV_ID )
    ACS( CALL, 77 )
    ACS( CALL, 0 )
    ACS( CALL, 65535 )
    ACS( CALL, MY_NO )
    ACS( CALL, 70 + 7 )
    ACS( LOADING, STATION0 )
    ACS( NEXT, STATION0_PLUS1 )
    ACS( STANDBY, AGV_ID )
    ACS_SEND( STANDBY, STATION(3) )
    ACS_SEND( NEXT, 12 )
    ACS( IDLE, STATION0 )
    ACS( IDLE, 77 )
    ACS( IDLE, STATION(0) + 1 )
    ACS( IDLE, AGV_ID )
    ACS_SEND( IDLE, STATION(2) )
SEQ_END
SEQ_BEGIN( s_old )
    OLD_ACS( LOADING, STATION0 )
    OLD_ACS( NEXT, STATION0_PLUS1 )
    OLD_ACS_SEND( ARRIVE, STATION(5) )
    OLD_ACS_SEND( ARRIVE, STATION(23) )
    OLD_ACS( FINISH, AGV_ID )
    OLD_ACS_SEND( CALL, 77 )
    OLD_ACS_SEND( CALL, 0 )
    OLD_ACS_SEND( CALL, 65535 )
    OLD_ACS_SEND( CALL, MY_NO )
    OLD_ACS_SEND( CALL, (70 + 7) )
    OLD_ACS( LOADING, STATION0 )
    OLD_ACS( NEXT, STATION0_PLUS1 )
    OLD_ACS( STANDBY, AGV_ID )
    OLD_ACS( STANDBY, STATION(3) )
    OLD_ACS_SEND( NEXT, 12 )
    OLD_ACS( RAWX, STATION0 )
    OLD_ACS_SEND( RAWX, 77 )
    OLD_ACS( RAWX, STATION0_PLUS1 )
    OLD_ACS( RAWX, AGV_ID )
    OLD_ACS_SEND( RAWX, STATION(2) )
SEQ_END
/* 칸 23(끝)과 255(큐 길이만)는 받는다 */
SEQ_BEGIN( s_stok )
    ACS_STATION( 23, 1, 2 )
    ACS_STATION( 255, 0, 2 )
SEQ_END
MAIN_BEGIN
    SAME(s_new_steps, s_old_steps);
MAIN_END
#else
SEQ_BEGIN( n )
#if defined(NEG_ST1_PLUS1)
    ACS( NEXT, STATION(1) + 1 )
#elif defined(NEG_ST0_PLUS2)
    ACS( NEXT, STATION(0) + 2 )
#elif defined(NEG_ST0_MINUS1)
    ACS( NEXT, STATION(0) - 1 )
#elif defined(NEG_MOD)
    ACS( NEXT, MOD( STATION(0), 5 ) )
#elif defined(NEG_BIG)
    ACS( NEXT, 70000 )
#elif defined(NEG_NEG)
    ACS( NEXT, -1 )
#elif defined(NEG_STK)
    ACS( NEXT, STATION(24) )
#elif defined(NEG_ACSST24)
    ACS_STATION( 24, 1, 2 )
#elif defined(NEG_ACSSTNEG)
    ACS_STATION( -1, 1, 2 )
#elif defined(NEG_BADWORD)
    ACS( NEXT, STATION1 )
#elif defined(NEG_AGVMINUS)
    ACS( NEXT, AGV_ID_MINUS(3) )
#elif defined(NEG_TAGNUM)
    ACS( NEXT, TAG_NUM + 1 )
#elif defined(NEG_CONST)
    ACS( UNLOADING, CONST )
#endif
SEQ_END
#endif
