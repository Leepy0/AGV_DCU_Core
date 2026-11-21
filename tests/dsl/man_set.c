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
MANUAL_END
MANUAL_BEGIN( m_old )
    { MAN_PSET + 2u, MAN_K_PROFILES, (1UL << 1) | (1UL << 5) | (1UL << 31), (1UL << 0) | (1UL << 31), 0UL, 0UL },
    { MAN_PSET + 2u, MAN_K_JOG, DSL_COND_ON(IN( ON(SEN_1) )), DSL_COND_OFF(IN( ON(SEN_1) )), OUTS(OUT_EXT_3), 0UL },
    { 7, (uint16_t)(MAN_K_IDLE | (KEY_UP << 8)), 0UL, 0UL, OUTS(OUT_EXT_4), 0UL },
    { MAN_PSET + 2u, MAN_K_JOG | MAN_F_ELSE, 0UL, 0UL, 0UL, OUTS(OUT_EXT_3) },
    { MAN_PSET + 15u, MAN_K_FB_SW, 0UL, 0UL, SEN_2, 0UL },
    { MAN_PSET + 0u, MAN_K_OBS_B, 0UL, 0UL, SEN_3, 0UL },
    { MAN_PSET + 2u, MAN_K_START_SW, 0UL, 0UL, SEN_4, 0UL },
MANUAL_END
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
#ifdef NEG_SET_64
MANUAL_BEGIN( m_neg )  MAN( 64, JOG, NO_IN, NO_OUT )  MANUAL_END                           /* 맨 숫자 64 는 묶음이 아님 */
#endif
MAIN_BEGIN
    SAME(m_new, m_old);
MAIN_END
