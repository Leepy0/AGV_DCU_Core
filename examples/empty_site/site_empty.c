/* site_empty.c - 표가 없는 빈 사이트. 코어 단독 링크 검증용이고 차를 움직이는 표는 없다 */
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/seq.h"
#include "agv/site.h"
#include "agv/tagact.h"

static const seq_site_t s_site = { 0, 0, 0, 0, 0, 0, 0, 0 };

void site_register(void)
{
    param_default_types(0, 0, 0);
    (void)seq_bind(&s_site);
    tagact_bind(0, 0, 0);
}

void site_manual(void) { }
