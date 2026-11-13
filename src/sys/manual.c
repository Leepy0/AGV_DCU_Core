/* manual.c - 수동 조작 표와 프로파일 적재 조건 (agv/manual.h) */
#include "agv/manual.h"
#include "agv/drive.h"
#include "agv/io.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/ui.h"

static const man_row_t     *s_rows;
static const profile_row_t *s_prof;
static uint16_t             s_nrows, s_nprof;

void manual_bind(const man_row_t *rows, uint16_t n, const profile_row_t *prof, uint16_t nprof)
{
    s_rows = rows;
    s_nrows = n;
    s_prof = prof;
    s_nprof = nprof;
}

void manual_tick(void)
{
    uint16_t id = (uint16_t)param_get(P_AGV_ID);
    uint32_t in = io_in_mask();
    uint16_t mode_ok[2];
    uint16_t fb = IN_IF_5;           /* run.c 의 기본 입력 */
    uint16_t done = 0, i;

    mode_ok[MAN_K_IDLE] = (uint16_t)(run_is_idle() && !agv_drive.flag_ready && !agv_drive.stop_sw);
    mode_ok[MAN_K_JOG] = (uint16_t)((agv_drive.ui_mode == UI_JOG) && !agv_drive.stop_sw);

    for (i = 0; i < s_nrows; i++) {
        const man_row_t *r = &s_rows[i];
        uint16_t k = r->kind & 0x0Fu;
        uint16_t key = r->kind >> 8;
        uint16_t hit = (uint16_t)((id >= (r->id & 0xFFu)) && (id <= (r->id >> 8)));

        if (k == MAN_K_FB_SW) {
            if (hit) { fb = (uint16_t)r->set; }
        }
        else if (k == MAN_K_OBS_B) {
            if (hit) { agv_drive.obs_b_ext = (int16_t)io_in((uint16_t)r->set); }
        }
        else {
            if (hit && !done && mode_ok[k] && ((key == 0u) || (ui_key() == key)) && seq_mask_true(in, r->on, r->off)) {
                io_out_masks(r->set, r->clr);
                done = 1;
            }
            if (r->kind & MAN_F_ELSE) { done = 0; }     /* 사슬 끝 */
        }
    }
    run_forback_input(fb);
}

/* seq_pick_profile 과 같은 행을 고른다 */
int16_t manual_load(void)
{
    int16_t type = param_get(P_AGV_TYPE);
    uint16_t id = (uint16_t)param_get(P_AGV_ID);
    uint32_t in = io_in_mask();
    uint16_t i;

    for (i = 0; i < s_nprof; i++) {
        const profile_row_t *r = &s_prof[i];
        if (((r->agv_type == 0u) || (r->agv_type == (uint16_t)type)) && (id >= r->id_from) && (id <= r->id_to)) {
            const uint32_t *c = r->load_if;
            if (c == 0) { return -1; }
            return (int16_t)(seq_mask_true(in, c[0], c[1]) || seq_mask_true(in, c[2], c[3]));
        }
    }
    return -1;
}
