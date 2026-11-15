/* 사이트가 낱말과 같은 이름의 매크로를 정의하면 깨져야 한다(조용히 틀리면 안 된다). -DNEG_SITE_x 는 컴파일 오류여야 한다 */
#include "agv/seq_dsl.h"
#include "same.h"
SEQ_LABELS( A, B )
#if defined(NEG_SITE_NUM)
#define FORWARD 1
#elif defined(NEG_SITE_PAREN)
#define FORWARD (1)
#elif defined(NEG_SITE_IDENT)
#define FORWARD DIR_FWD
#elif defined(NEG_SITE_ID)
#define ID 5
#endif
SEQ_BEGIN( n )
#if defined(NEG_SITE_ID)
    WHEN( AND( ID(1, 9), ON(SEN_1) ), A )
#else
    WHEN( AND( FORWARD, ON(SEN_1) ), A )
#endif
SEQ_END

MAIN_BEGIN
MAIN_END
