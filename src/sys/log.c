/* log.c - 링 버퍼 256워드. 쓰기는 메인 문맥에서만 한다 */
#include "agv/log.h"

#define LOG_RECS 64u

static log_rec_t s_rec[LOG_RECS];
static uint16_t  s_seq;

void log_event(uint16_t code, int16_t a, int16_t b)
{
    log_rec_t *r = &s_rec[s_seq & (LOG_RECS - 1u)];
    r->code = code;
    r->a = a;
    r->b = b;
    r->seq = s_seq;
    s_seq++;
}

uint16_t log_count(void) { return s_seq; }

int log_read(uint16_t back, log_rec_t *out)
{
    if ((back >= s_seq) || (back >= LOG_RECS)) { return 0; }
    *out = s_rec[(uint16_t)(s_seq - 1u - back) & (LOG_RECS - 1u)];
    return 1;
}
