/* test_tagpk.c - 압축 태그 표에서 사용자 함수가 tagact_run 을 다시 불러도
 * 바깥 규칙의 남은 동작이 한 번씩, 바깥 규칙의 것으로 실행되는지 본다 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/seq_dsl.h"
#include "agv/site.h"
#include "agv/sys.h"
#include "agv/tagact.h"
#include "hal_host.h"

static uint16_t s_log[32];
static uint16_t s_nlog;

static int rec(uint16_t id) { if (s_nlog < 32u) { s_log[s_nlog++] = id; } return 1; }
static int fn1(seq_ctx_t *c) { (void)c; return rec(1); }
static int fn2(seq_ctx_t *c) { (void)c; return rec(2); }
static int fn3(seq_ctx_t *c) { (void)c; return rec(3); }
static int fn0(seq_ctx_t *c)
{
    (void)c;
    rec(0);
    (void)tagact_run(51);      /* 안쪽: 맞는 규칙 있음 */
    (void)tagact_run(53);      /* 안쪽: 후보는 있으나 조건이 거짓 */
    (void)tagact_run(54);      /* 안쪽: 후보 없음 */
    return 1;
}
static const seq_call_fn s_calls[] = { fn0, fn1, fn2, fn3 };
static const seq_site_t s_site = { 0, 0, 0, 0, s_calls, 4, 0, 0, 0 };

TAG_X_BEGIN( s_x )
    TAG_X( X_BACKWARD )                      /* 1: 전진 중이라 거짓 */
TAG_POOL_END
TAG_IN_BEGIN( s_in )
TAG_POOL_END
TAG_ACT_BEGIN( s_act )
    TAG_ACTS( A_CALL(0), A_CALL(1), A_CALL(1), A_CALL(1) )   /* 1 */
    TAG_ACTS( A_CALL(2) )                                    /* 2 */
    TAG_ACTS( A_CALL(3), A_CALL(3), A_CALL(3), A_CALL(3) )   /* 3 */
TAG_POOL_END
TAGP_BEGIN( s_prules )
    TAGP( 50, 0, 0, 0, 0, 1 )
    TAGP( 51, 0, 0, 0, 0, 2 )
    TAGP( 53, 0, 1, 0, 0, 3 )
TAG_POOL_END
TAG_PACKED( s_pk, s_prules, s_x, s_in, s_act )

void site_register(void) {}
void site_manual(void) {}

int main(void)
{
    static const uint16_t want[] = { 0, 2, 1, 1, 1 };
    uint16_t i;

    host_reset();
    memset(&agv_drive, 0, sizeof(agv_drive));
    (void)seq_bind(&s_site);
    tagact_bind_packed(&s_pk, 0);
    (void)tagact_run(50);
    for (i = 0; i < s_nlog; i++) { printf("%u ", (unsigned)s_log[i]); }
    printf("\n");
    assert(s_nlog == 5u);
    for (i = 0; i < 5u; i++) { assert(s_log[i] == want[i]); }
    printf("test_tagpk ok\n");
    return 0;
}
