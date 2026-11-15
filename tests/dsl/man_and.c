#include "agv/seq_dsl.h"
#include "same.h"
/* 수동 표 조건 칸에 AND( 입력…, KEY( k ) ): 키 하나 + 입력 조건. 같은 뜻을 예전 꼴로 적은 줄과 같은 바이트 */
#define BELT1_CW   OUTS(OUT_EXT_1)
#define BELT1_CCW  OUTS(OUT_EXT_1, OUT_EXT_2)
MANUAL_BEGIN( m_new )
    MAN( 5, JOG,  AND( ON(SEN_1), KEY( UP ) ),  BELT( 1, CW ) )
    MAN( 5, JOG,  AND( KEY( UP ) ),             BELT( 1, CW ) )
    MAN( 5, IDLE, AND( ON(SEN_1), OFF(SEN_2), ANY(SEN_3, SEN_4) ), ON( OUT_EXT_3 ) )
    MAN( 5, JOG,  AND( ALWAYS ),                OFF( OUT_EXT_3 ) )
MANUAL_END
MANUAL_BEGIN( m_old )
    { 5, (uint16_t)(MAN_K_JOG | (KEY_UP << 8)), DSL_COND_ON(IN( ON(SEN_1) )), DSL_COND_OFF(IN( ON(SEN_1) )), OUTS(OUT_EXT_1), OUTS(OUT_EXT_2) },
    MAN( 5, JOG,  KEY( UP ),                    BELT( 1, CW ) )
    MAN( 5, IDLE, IN( ON(SEN_1), OFF(SEN_2), ANY_ON(SEN_3, SEN_4) ), ON( OUT_EXT_3 ) )
    MAN( 5, JOG,  NO_IN,                        OFF( OUT_EXT_3 ) )
MANUAL_END
#ifdef NEG_MAN_STATE
MANUAL_BEGIN( m_neg )
    MAN( 5, JOG,  AND( FORWARD, KEY( UP ) ),    BELT( 1, CW ) )
MANUAL_END
#endif
#ifdef NEG_MAN_ANY2
MANUAL_BEGIN( m_neg )
    MAN( 5, JOG,  AND( ANY(SEN_1, SEN_2), ANY(SEN_3, SEN_4) ), BELT( 1, CW ) )
MANUAL_END
#endif
MAIN_BEGIN
    SAME(m_new, m_old);
MAIN_END
