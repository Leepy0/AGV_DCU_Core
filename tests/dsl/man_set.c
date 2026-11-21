#include "agv/seq_dsl.h"
#include "same.h"
/* 수동 표 묶음(v0.63.0): 묶음 줄 MAN_PROFILES 와 열쇠 PROFILE_SET( k ) 가 손으로 쓴 줄과 같은 바이트.
 * 묶음 줄 = 열쇠 MAN_PSET + k, 종류 MAN_K_PROFILES, on = 프로파일 1~31 비트, off = 32~63 비트 */
MANUAL_BEGIN( m_new )
    MAN_PROFILES( PROFILE_SET( 2 ), 1, 5, 31, 32, 63, 5 )
    MAN( PROFILE_SET( 2 ), JOG, IN( ON(SEN_1) ),  ON( OUT_EXT_3 ) )
    MAN( 7, IDLE, KEY( UP ),                      ON( OUT_EXT_4 ) )
    MAN_ELSE( PROFILE_SET( 2 ), JOG,              OFF( OUT_EXT_3 ) )
    MAN_FB_SW( PROFILE_SET( 15 ), SEN_2 )
    MAN_OBS_B( PROFILE_SET( 0 ), SEN_3 )
    MAN_START_SW( PROFILE_SET( 2 ), SEN_4 )
    MAN( 9, ANY, IN( OFF(SEN_5) ),                ON( OUT_EXT_5 ) )   /* 모드 무관(v0.63.0) */
    MAN_ELSE( PROFILE_SET( 2 ), ANY,              NO_OUT )
MANUAL_END
MANUAL_BEGIN( m_old )
    { MAN_PSET + 2u, MAN_K_PROFILES, (1UL << 1) | (1UL << 5) | (1UL << 31), (1UL << 0) | (1UL << 31), 0UL, 0UL },
    { MAN_PSET + 2u, MAN_K_JOG, DSL_COND_ON(IN( ON(SEN_1) )), DSL_COND_OFF(IN( ON(SEN_1) )), OUTS(OUT_EXT_3), 0UL },
    { 7, (uint16_t)(MAN_K_IDLE | (KEY_UP << 8)), 0UL, 0UL, OUTS(OUT_EXT_4), 0UL },
    { MAN_PSET + 2u, MAN_K_JOG | MAN_F_ELSE, 0UL, 0UL, 0UL, OUTS(OUT_EXT_3) },
    { MAN_PSET + 15u, MAN_K_FB_SW, 0UL, 0UL, SEN_2, 0UL },
    { MAN_PSET + 0u, MAN_K_OBS_B, 0UL, 0UL, SEN_3, 0UL },
    { MAN_PSET + 2u, MAN_K_START_SW, 0UL, 0UL, SEN_4, 0UL },
    { 9, MAN_K_ANY, DSL_COND_ON(IN( OFF(SEN_5) )), DSL_COND_OFF(IN( OFF(SEN_5) )), OUTS(OUT_EXT_5), 0UL },
    { MAN_PSET + 2u, MAN_K_ANY | MAN_F_ELSE, 0UL, 0UL, 0UL, 0UL },
MANUAL_END
/* 후진 정지 FL(v0.63.0): STOP_BACK_FL( p ) = 프로파일 칸 비트 8(PF_F_STOP_BACK_FL) */
LOAD_IF( p_lif, IN( ON(SEN_1) ) )
PROFILE_TABLE_BEGIN( p_new )
    PROFILE( STOP_BACK_FL( 3 ), 0, 1, 2 )
    PROFILE_LOAD( STOP_BACK_FL( 63 ), 100, 5, 9, LOAD_SENSORS( SEN_1 ) )
    PROFILE_LOAD_IF( STOP_BACK_FL( 5 ), 0, 10, 10, p_lif )
    PROFILE( 3, 0, 3, 7 )
PROFILE_TABLE_END
PROFILE_TABLE_BEGIN( p_old )
    { 0x103u, 0, 1, 2, 0UL, 0 },
    { 0x13Fu, 100, 5, 9, LOAD_SENSORS( SEN_1 ), 0 },
    { 0x105u, 0, 10, 10, 0UL, p_lif },
    { 3, 0, 3, 7, 0UL, 0 },
PROFILE_TABLE_END
#if defined(NEG_FL_SEQMAP) || defined(NEG_MAP_64)
SEQ_BEGIN( s_x )  END_STOP  SEQ_END
#endif
#ifdef NEG_FL_0
PROFILE_TABLE_BEGIN( p_neg )  PROFILE( STOP_BACK_FL( 0 ), 0, 1, 1 )  PROFILE_TABLE_END          /* 인자 1~63 밖 */
#endif
#ifdef NEG_FL_64
PROFILE_TABLE_BEGIN( p_neg )  PROFILE( STOP_BACK_FL( 64 ), 0, 1, 1 )  PROFILE_TABLE_END
#endif
#ifdef NEG_FL_TWICE
PROFILE_TABLE_BEGIN( p_neg )  PROFILE( STOP_BACK_FL( STOP_BACK_FL( 1 ) ), 0, 1, 1 )  PROFILE_TABLE_END
#endif
#ifdef NEG_FL_SET
PROFILE_TABLE_BEGIN( p_neg )  PROFILE( STOP_BACK_FL( PROFILE_SET( 0 ) ), 0, 1, 1 )  PROFILE_TABLE_END
#endif
#ifdef NEG_FL_MAN
MANUAL_BEGIN( m_neg )  MAN_ELSE( STOP_BACK_FL( 1 ), JOG, NO_OUT )  MANUAL_END                 /* 프로파일 표가 아닌 곳 */
#endif
#ifdef NEG_FL_TAG
static const tag_rule_t t_neg[] = { TAG( 1, STOP_BACK_FL( 1 ), TA_STOP ) };
#endif
#ifdef NEG_FL_TAGIF
static const tag_rule_t t_neg[] = { TAG_IF( 1, STOP_BACK_FL( 1 ), IF_ID( 1, 2 ), TA_STOP ) };
#endif
#ifdef NEG_FL_TAGRANGE
static const tag_rule_t t_neg[] = { TAG_RANGE( 1, 2, STOP_BACK_FL( 1 ), TA_STOP ) };
#endif
#ifdef NEG_FL_TAGRANGEIF
static const tag_rule_t t_neg[] = { TAG_RANGE_IF( 1, 2, STOP_BACK_FL( 1 ), IF_ID( 1, 2 ), TA_STOP ) };
#endif
#ifdef NEG_FL_TAGP
TAGP_BEGIN( t_neg )  TAGP( 1, STOP_BACK_FL( 1 ), 0, 0, 0, 1 )  TAG_POOL_END
#endif
#ifdef NEG_FL_SEQMAP
SEQ_MAP_BEGIN( d_neg )  SEQ_MAP( STOP_BACK_FL( 1 ), 1, s_x )  SEQ_MAP_END
#endif
#ifdef NEG_TAG_64
static const tag_rule_t t_neg[] = { TAG( 1, 64, TA_STOP ) };                                   /* 태그 표 프로파일 0~63 밖 */
#endif
#ifdef NEG_MAP_64
SEQ_MAP_BEGIN( d_neg )  SEQ_MAP( 64, 1, s_x )  SEQ_MAP_END                                    /* 대응표 프로파일 0~63 밖 */
#endif
#ifdef NEG_SET_16
MANUAL_BEGIN( m_neg )  MAN_PROFILES( PROFILE_SET( 16 ), 1 )  MANUAL_END                    /* 묶음 번호 0~15 밖 */
#endif
#ifdef NEG_SET_NEG
MANUAL_BEGIN( m_neg )  MAN_ELSE( PROFILE_SET( -1 ), JOG, NO_OUT )  MANUAL_END
#endif
#ifdef NEG_SET_PLAIN
MANUAL_BEGIN( m_neg )  MAN_PROFILES( 1, 2 )  MANUAL_END                                    /* 묶음 줄의 열쇠가 묶음이 아님 */
#endif
#ifdef NEG_SET_PF0
MANUAL_BEGIN( m_neg )  MAN_PROFILES( PROFILE_SET( 1 ), 1, 0 )  MANUAL_END                  /* 묶음에 넣는 프로파일 1~63 밖 */
#endif
#ifdef NEG_SET_PF64
MANUAL_BEGIN( m_neg )  MAN_PROFILES( PROFILE_SET( 1 ), 64 )  MANUAL_END
#endif
#ifdef NEG_SET_EMPTY
MANUAL_BEGIN( m_neg )  MAN_PROFILES( PROFILE_SET( 1 ) )  MANUAL_END                         /* 프로파일 없음 */
#endif
#ifdef NEG_SET_17
MANUAL_BEGIN( m_neg )  MAN_PROFILES( PROFILE_SET( 1 ), 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 )  MANUAL_END   /* 한 줄 16 개까지 */
#endif
#ifdef NEG_SET_KEY
MANUAL_BEGIN( m_neg )  MAN_START_SW( MAN_PSET + 16, SEN_1 )  MANUAL_END                    /* 묶음도 프로파일도 아님 */
#endif
#ifdef NEG_ANY_LOWER
MANUAL_BEGIN( m_neg )  MAN( 1, any, NO_IN, NO_OUT )  MANUAL_END                           /* 정하지 않은 모드 낱말 */
#endif
#ifdef NEG_SET_64
MANUAL_BEGIN( m_neg )  MAN( 64, JOG, NO_IN, NO_OUT )  MANUAL_END                           /* 맨 숫자 64 는 묶음이 아님 */
#endif
MAIN_BEGIN
    SAME(m_new, m_old);
    SAME(p_new, p_old);
MAIN_END
