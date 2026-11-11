/* guide.c - 가이드센서 응답 해석, LED 덩어리 검출, 위치 추적 */
#include "agv/guide.h"
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/tagact.h"

#define H_PLACEHOLDER      10      /* 구동작의 'h' 자리표 */
#define POS_OFF_NARROW     17      /* 'a' 전부 꺼짐 표시 */
#define POS_OFF_WIDE       33
#define POS_ON_NARROW      16      /* 'a' 전부 켜짐 표시 */
#define POS_ON_WIDE        32
#define LEN_A_NARROW       7       /* STX a 부호 십 일 BCC ETX */
#define LEN_A_WIDE         8       /* STX a 부호 십 일 V BCC ETX. V = 세로 센서 '1'/'0' */
#define LEN_H_NARROW       8       /* STX h 16진×4 BCC ETX */
#define LEN_H_WIDE         12      /* STX h 16진×8 BCC ETX. 세로 LED 는 'h' 에 없다 */
#define LEN_S              4       /* 'S' 의 최소 길이: STX S BCC ETX */

#define RX_BUF_SIZE        16      /* 가장 긴 응답 12 바이트 */
#define ASCII_STX          0x02
#define ASCII_ETX          0x03
#define ASCII_ACK          0x06

static guide_cfg_t s_cfg;
static struct {
    uint16_t buf[RX_BUF_SIZE];
    uint16_t cnt;
    uint16_t in_packet;
} s_rx;
static struct {
    uint16_t armed;       /* 첫 유효 응답을 받았다 */
    uint16_t waiting;     /* 보낸 요청의 응답이 아직 없다 */
    uint16_t miss;
    uint16_t bad;
    int16_t  error;
} s_comm;
static int16_t     s_prev;        /* 마지막 위치. invert 전 값 */
static int16_t     s_prev_valid;  /* LOST 뒤에는 변화량 제한을 걸지 않는다. 걸면 다시 못 잡는다 */
/* 테이프 폭 학습과 가장자리 고정. 비스듬한 교차선이 주행선과 붙으면 덩어리 가운데가 교차선 쪽으로 끌린다 */
static uint16_t    s_tape_q4;     /* 배운 테이프 폭 × 16. 0 = 모름. 센서를 바꿔도 유지한다(같은 테이프) */
static int16_t     s_edge_s, s_edge_e;   /* 정상 추적 때의 가장자리 칸. 붙어 있는 동안은 고치지 않는다.
                                            매 스캔 고치면 지나가는 교차선의 가장자리를 따라 끌려간다 */
static uint16_t    s_release;
/* 다시 잡을 때 지속 확인(주행 위치 경로만). 잡았다가 잃은 뒤 처음 보이는 덩어리는 후보로만 두고
 * GUIDE_REACQ_N 응답을 이어 같은 자리에 있어야 받는다. 끊긴 자리의 바닥 반사 한두 칸을 주행선으로 받지 않는다 */
static int16_t     s_drive;       /* 이번 호출은 주행 위치 경로다: 가장자리 고정·폭 학습·지속 확인을 한다 */
static int16_t     s_straight_n;  /* 직진 조건이 이어진 틱 수(포화) */
static int16_t     s_straight;    /* 직진 중이다. 폭 학습은 이때만 한다 */
static int16_t     s_relearn;     /* 가장자리 고정·유지 상한을 넘겨 폭을 다시 배우는 중 */
static uint16_t    s_merge_age;   /* 필터 전 덩어리가 둘 이상이던 마지막 스캔 뒤 지난 스캔 수(포화). 붙어 있는 동안은 멈춘다 */
static uint16_t    s_wide_n;      /* 붙음이 아닌데 배운 폭 + 1 보다 넓은 응답이 이어진 수(포화 = 올려 배우는 중) */
static int16_t     s_trk_last;    /* 지난 주행 위치 스캔의 guide_track_t */
static int16_t     s_ever;        /* 시작·센서 전환 뒤 선을 잡은 적이 있다. 첫 획득은 바로 받는다 */
static int16_t     s_cand_pos, s_cand_n;

void guide_cfg_default(guide_cfg_t *cfg)
{
    cfg->invert_fwd = 0;
    cfg->h_scale = 1.0f;
    cfg->invert_back = 0;
    cfg->min_width = 1;
    cfg->max_width = GUIDE_LED_WIDE;
    cfg->max_step = 0;
    cfg->curve_span = GUIDE_CURVE_SPAN_DEFAULT;
}

void guide_configure(const guide_cfg_t *cfg) { s_cfg = *cfg; }
void guide_cfg_get(guide_cfg_t *cfg)         { *cfg = s_cfg; }
float guide_h_scale(void)                     { return s_cfg.h_scale; }

static void comm_clear(void)
{
    s_comm.armed = 0;
    s_comm.waiting = 0;
    s_comm.miss = 0;
    s_comm.bad = 0;
    s_comm.error = 0;
}

void guide_on_run_start(void)
{
    comm_clear();
    s_ever = 0;
    s_cand_n = 0;
}

void guide_on_sensor_switch(void)
{
    s_rx.cnt = 0;
    s_rx.in_packet = 0;
    s_prev = 0;           /* 앞 센서의 위치로 뒤 센서 덩어리를 고르면 안 된다 */
    s_prev_valid = 0;
    s_release = 0;
    s_ever = 0;
    s_cand_n = 0;
    s_relearn = 0;
    s_wide_n = 0;
    s_merge_age = GUIDE_MERGE_SCANS;
    s_trk_last = GUIDE_TRK_NORMAL;
    comm_clear();
}

void guide_reset(void)
{
    guide_cfg_default(&s_cfg);
    s_tape_q4 = 0;
    guide_on_sensor_switch();
}

static int16_t hex_value(uint16_t c)
{
    if ((c >= '0') && (c <= '9')) { return (int16_t)(c - '0'); }
    if ((c >= 'a') && (c <= 'f')) { return (int16_t)(c - 'a' + 10); }
    if ((c >= 'A') && (c <= 'F')) { return (int16_t)(c - 'A' + 10); }
    return -1;
}

static uint16_t bcc_ok(const uint16_t *buf, uint16_t len)
{
    uint16_t i, sum = 0, last = (uint16_t)(len - 2);
    for (i = 1; i < last; i++) { sum = (uint16_t)(sum + buf[i]); }
    sum &= 0x0F;
    sum = (uint16_t)((sum < 10) ? ('0' + sum) : ('a' + sum - 10));   /* 센서는 소문자로 보낸다 */
    return sum == buf[last];
}

static void parse_position(const uint16_t *buf, uint16_t len, guide_rx_t *rx)
{
    int16_t off, on, low;

    rx->kind = GUIDE_RX_POSITION;
    rx->wide = (len != LEN_A_NARROW);
    if (((len != LEN_A_NARROW) && (len != LEN_A_WIDE)) || ((buf[2] != '-') && (buf[2] != '+'))
        || (buf[3] < '0') || (buf[3] > '9') || (buf[4] < '0') || (buf[4] > '9')
        || (rx->wide && (buf[5] != '0') && (buf[5] != '1'))) {
        rx->kind = GUIDE_RX_BAD_FORMAT;
        return;
    }
    if (rx->wide) { rx->vert = (int16_t)(buf[5] - '0'); }
    rx->pos = (int16_t)((buf[3] - '0') * 10 + (buf[4] - '0'));
    if (buf[2] == '-') { rx->pos = (int16_t)(-rx->pos); }
    off = rx->wide ? POS_OFF_WIDE : POS_OFF_NARROW;
    on  = rx->wide ? POS_ON_WIDE : POS_ON_NARROW;
    low = rx->wide ? -31 : -15;
    if (rx->pos == off) {
        rx->status = GUIDE_LOST;
    }
    else if ((rx->pos >= on) || (rx->pos < low)) {
        rx->pos = 0;
        rx->status = GUIDE_ALL_ON;
    }
}

/* 16진 글자 순서가 비트 순서와 반대다: [2] 가 비트 0~3 이다 */
static void parse_led(const uint16_t *buf, uint16_t len, guide_rx_t *rx)
{
    uint16_t i, digits;

    rx->kind = GUIDE_RX_LED;
    rx->wide = (len != LEN_H_NARROW);
    if ((len != LEN_H_NARROW) && (len != LEN_H_WIDE)) {
        rx->kind = GUIDE_RX_BAD_FORMAT;
        return;
    }
    digits = (uint16_t)(len - 4);     /* STX h ... BCC ETX */
    for (i = 0; i < digits; i++) {
        int16_t v = hex_value(buf[2 + i]);
        if (v < 0) {
            rx->kind = GUIDE_RX_BAD_FORMAT;
            return;
        }
        rx->led_lo |= (uint32_t)v << (i * 4);
    }
}

void guide_parse(const uint16_t *buf, uint16_t len, guide_rx_t *rx)
{
    rx->kind = GUIDE_RX_NONE;
    rx->wide = 0;
    rx->pos = 0;
    rx->status = GUIDE_NORMAL;
    rx->vert = 0;
    rx->led_lo = 0;

    if (len < LEN_S) {
        rx->kind = GUIDE_RX_BAD_FORMAT;
        return;
    }
    if (!bcc_ok(buf, len)) {
        rx->kind = GUIDE_RX_BAD_BCC;
        return;
    }
    switch (buf[1]) {
    case 'a': parse_position(buf, len, rx); break;
    case 'h': parse_led(buf, len, rx); break;
    case 'S': rx->kind = GUIDE_RX_SENSOR_ERR; break;
    default:  break;
    }
}

/* 잘린 패킷 뒤의 STX 는 새 패킷으로 본다. 전환 직후 잡음에서 한 프레임 안에 복구한다 */
int16_t guide_feed(uint16_t byte, guide_rx_t *rx)
{
    byte &= 0xFF;
    if ((byte == ASCII_ACK) && (s_rx.cnt == 0)) {
        s_rx.in_packet = 0;
        return 0;
    }
    if (byte == ASCII_STX) {
        s_rx.in_packet = 1;
        s_rx.cnt = 0;
        s_rx.buf[s_rx.cnt++] = byte;
        return 0;
    }
    if (!s_rx.in_packet) { return 0; }
    if (s_rx.cnt >= RX_BUF_SIZE) {
        s_rx.in_packet = 0;
        s_rx.cnt = 0;
        return 0;
    }
    s_rx.buf[s_rx.cnt++] = byte;
    if (byte != ASCII_ETX) { return 0; }

    guide_parse(s_rx.buf, s_rx.cnt, rx);
    s_rx.in_packet = 0;
    s_rx.cnt = 0;

    s_comm.waiting = 0;   /* 깨졌어도 응답은 왔다. 두절이 아니라 체크섬 실패로 센다 */
    s_comm.miss = 0;
    if ((rx->kind == GUIDE_RX_BAD_BCC) || (rx->kind == GUIDE_RX_BAD_FORMAT)) {
        if (s_comm.armed) {
            s_comm.bad++;
            if (s_comm.bad >= GUIDE_BAD_LIMIT) { s_comm.error = ERR_GUIDE_COMM; }
        }
    }
    else {
        s_comm.armed = 1;
        s_comm.bad = 0;
    }
    return 1;
}

int16_t guide_on_request(void)
{
    if (s_comm.armed && s_comm.waiting) {
        uint16_t limit = (uint16_t)param_get(P_GUIDE_TIMEOUT);
        if (limit == 0) { limit = GUIDE_MISS_LIMIT; }
        s_comm.miss++;
        if (s_comm.miss >= limit) { s_comm.error = ERR_GUIDE_COMM; }
    }
    s_comm.waiting = 1;
    return s_comm.error;
}

int16_t guide_comm_error(void) { return s_comm.error; }

/* 오류 해제. 감시는 유지한다. 주행 중 리셋 뒤에도 무응답이면 다시 잡는다 */
void guide_comm_clear(void)
{
    s_comm.waiting = 0;
    s_comm.miss = 0;
    s_comm.bad = 0;
    s_comm.error = 0;
}

uint32_t guide_lateral(const guide_rx_t *rx)
{
    return rx->wide ? rx->led_lo : (rx->led_lo & 0xFFFFUL);
}

uint16_t guide_blobs(uint32_t lateral, uint16_t leds, guide_blob_t *out)
{
    uint16_t i, count = 0, start = 0;

    for (i = 1; i <= leds; i++) {
        uint16_t on = (uint16_t)((lateral >> (i - 1)) & 1UL);
        if (on && (start == 0)) { start = i; }
        if ((start != 0) && (!on || (i == leds))) {
            out[count].start = start;
            out[count].end = on ? i : (uint16_t)(i - 1);
            count++;
            start = 0;
        }
    }
    return count;
}

static int16_t blob_pos(const guide_blob_t *b, uint16_t leds)
{
    return (int16_t)(b->start + b->end - (leds + 1));
}

static uint16_t width_ok(const guide_blob_t *b)
{
    uint16_t w = (uint16_t)(b->end - b->start + 1);
    return (w >= s_cfg.min_width) && (w <= s_cfg.max_width);
}

static uint16_t step_ok(int16_t pos)
{
    int16_t d = (int16_t)(pos - s_prev);
    if ((s_cfg.max_step == 0) || !s_prev_valid) { return 1; }
    if (d < 0) { d = (int16_t)(-d); }
    return (uint16_t)d <= s_cfg.max_step;
}

/* 반환: 고른 덩어리 번호, 없으면 -1. 거리가 같으면 칸 번호가 작은 쪽이다 */
static int16_t pick(const guide_blob_t *b, uint16_t n, uint16_t leds, int16_t branch, int16_t backward, int16_t invert)
{
    uint16_t i;
    int16_t best = -1;
    int16_t best_d = 0;

    if (branch != 0) {
        /* 왼쪽 = 음의 위치 쪽. 분기는 일부러 건너뛰는 동작이라 변화량 제한을 걸지 않는다 */
        uint16_t high = (uint16_t)((branch == BRANCH_RIGHT) ^ (backward != 0) ^ (invert != 0));
        for (i = 0; i < n; i++) {
            if (width_ok(&b[i])) {
                best = (int16_t)i;
                if (!high) { break; }
            }
        }
        return best;
    }
    for (i = 0; i < n; i++) {
        int16_t pos = blob_pos(&b[i], leds);
        int16_t d = (int16_t)(pos - s_prev);
        if (d < 0) { d = (int16_t)(-d); }
        if (width_ok(&b[i]) && step_ok(pos) && ((best < 0) || (d < best_d))) {
            best = (int16_t)i;
            best_d = d;
        }
    }
    return best;
}

/* 10 mm 테이프는 1칸·2칸을 오간다. 평균 1.5 를 2 로 올리고, 가끔 한 칸 넓어지는 것은 무시한다.
 * 하한 2칸: 한 칸만 켜지는 구간이 길어도 1 로 굳지 않는다. 0 = 아직 모름 */
uint16_t guide_tape_width(void)
{
    uint16_t w = (uint16_t)((s_tape_q4 + 11u) >> 4);
    return (s_tape_q4 == 0) ? 0u : ((w < 2u) ? 2u : w);
}
void     guide_track_release(void) { s_release = 1; }
int16_t  guide_reacq_pending(void) { return (int16_t)(s_cand_n > 0); }

/* 후진도 크기로 본다. 차 ≤ 평균 × PCT / 100  ⇔  차 × 200 ≤ 합 × PCT */
int16_t guide_straight_tick(int16_t rpm_l, int16_t rpm_r, int16_t min_rpm)
{
    int32_t l = (rpm_l < 0) ? -(int32_t)rpm_l : rpm_l;
    int32_t r = (rpm_r < 0) ? -(int32_t)rpm_r : rpm_r;
    int32_t d = (l > r) ? (l - r) : (r - l);

    if (((l + r) >= 2L * min_rpm) && ((d * 200L) <= ((l + r) * GUIDE_STRAIGHT_PCT))) {
        if (s_straight_n < GUIDE_STRAIGHT_TICKS) { s_straight_n++; }
    }
    else { s_straight_n = 0; }
    s_straight = (int16_t)(s_straight_n >= GUIDE_STRAIGHT_TICKS);
    return s_straight;
}

/* sure: 넓어진 테이프로 확인됐다. 느린 평균의 나머지(16 미만)도 한 응답에 1 씩 올려 끝까지 배운다 */
static void learn_width(uint16_t w, int16_t sure)
{
    int16_t d = (int16_t)((int16_t)(w << 4) - (int16_t)s_tape_q4);
    if (s_tape_q4 == 0) { s_tape_q4 = (uint16_t)(w << 4); }
    else if (sure && (d > 0) && (d < 16)) { s_tape_q4++; }
    else { s_tape_q4 = (uint16_t)((int16_t)s_tape_q4 + d / ((d < 0) ? 4 : 16)); }   /* 좁아지는 쪽은 빨리 따라간다 */
}

/* 고른 덩어리의 가장자리를 테이프 폭에 맞춰 고친다. 반환 guide_track_t */
static int16_t edge_fix(int16_t *s, int16_t *e, uint16_t leds, uint16_t blobs, int16_t tracking)
{
    int16_t ref = (int16_t)guide_tape_width();
    int16_t w = (int16_t)(*e - *s + 1);
    int16_t ds = (int16_t)(*s - s_edge_s), de = (int16_t)(*e - s_edge_e);

    if (ds < 0) { ds = (int16_t)(-ds); }
    if (de < 0) { de = (int16_t)(-de); }
    /* 상한을 넘겼다: 폭을 지우지 않고 다시 배운다. 넓은 덩어리를 가운데로 받으며 느린 평균으로 올린다 */
    if (s_release) { s_release = 0; s_relearn = 1; }
    if (!s_straight) { s_wide_n = 0; }                /* 곡선의 넓어짐은 세지 않는다 */
    /* 세는 중에는 배운 폭 + 1 안으로 들어오면, 올려 배우는 중에는 배운 폭에 닿으면 끝낸다 */
    if (w <= ref + ((s_wide_n >= GUIDE_WIDE_LEARN_SCANS) ? 0 : 1)) { s_wide_n = 0; }
    if (ref == 0) {                                  /* 처음: 보이는 대로 받고 배운다 */
    }
    else if (w < ref) {                              /* 센서 끝에 반만 걸렸으면 안쪽 가장자리가 기준이다. 배우지 않는다 */
        if (*s == 1)                  { *s = (int16_t)(*e - ref + 1); return GUIDE_TRK_NORMAL; }
        if (*e == (int16_t)leds)      { *e = (int16_t)(*s + ref - 1); return GUIDE_TRK_NORMAL; }
    }
    else if (s_relearn) {
        if (w <= ref + 1) { s_relearn = 0; }
    }
    else if (!tracking) {                            /* 다시 잡는 중: 견줄 가장자리가 없다 */
    }
    else if ((*e < s_edge_s - 1) || (*s > s_edge_e + 1)) {
        return GUIDE_TRK_NORMAL;                     /* 떨어진 다른 덩어리로 옮겼다. 붙은 것이 아니다 */
    }
    else if (w > ref + 1) {
        /* 교차선·합류선은 붙기 전에 떨어진 덩어리로 먼저 보인다. 그런 적이 없이 넓어지면 커브다: 가운데, 폭 그대로 */
        if ((s_merge_age >= GUIDE_MERGE_SCANS) || ((ds == 1) && (de == 1))) {   /* 양쪽으로 한 칸씩: 커브 */
            /* 커브의 폭 증가는 잠깐이다. GUIDE_WIDE_LEARN_SCANS 이어지면 넓어진 테이프다: 아래에서 올려 배운다 */
            if (s_wide_n < GUIDE_WIDE_LEARN_SCANS) { if (s_straight) { s_wide_n++; } return GUIDE_TRK_NORMAL; }
        }
        else {
            s_wide_n = 0;
            if ((ds == 0) || ((ds == 1) && (de > 1))) { *e = (int16_t)(*s + ref - 1); return GUIDE_TRK_EDGE; }
            if ((de == 0) || ((de == 1) && (ds > 1))) { *s = (int16_t)(*e - ref + 1); return GUIDE_TRK_EDGE; }
            return GUIDE_TRK_HOLD;
        }
    }
    /* 직진 중에만 배운다. 처음(ref 0)은 서 있을 때·출발 직선이 보통이고 좁아지는 학습이 곧 맞춘다 */
    if ((blobs == 1) && (*s > 1) && (*e < (int16_t)leds) && (s_straight || (ref == 0))) { learn_width((uint16_t)w, s_wide_n >= GUIDE_WIDE_LEARN_SCANS); }
    return GUIDE_TRK_NORMAL;
}

/* 잃은 뒤 다시 잡는 중인가. 1 = 아직 후보다(LOST 로 낸다). 후보 자리에 덩어리가 있으면 그것을, 없으면 고른 것을 새 후보로 */
static int16_t reacq_wait(const guide_blob_t *b, uint16_t n, uint16_t leds, int16_t *chosen)
{
    uint16_t i;
    int16_t pos;

    if (s_cand_n > 0) {
        for (i = 0; i < n; i++) {
            int16_t d = (int16_t)(blob_pos(&b[i], leds) - s_cand_pos);
            if (width_ok(&b[i]) && (d <= GUIDE_REACQ_TOL) && (d >= -GUIDE_REACQ_TOL)) { break; }
        }
        if (i < n) { *chosen = (int16_t)i; s_cand_n++; }
        else       { s_cand_n = 1; }
    }
    else {
        s_cand_n = 1;
    }
    pos = blob_pos(&b[*chosen], leds);
    s_cand_pos = pos;
    return s_cand_n < GUIDE_REACQ_N;
}

void guide_track_drive(const guide_rx_t *rx, int16_t branch, int16_t backward, guide_out_t *out)
{
    s_drive = 1;
    guide_track(rx, branch, backward, out);
    s_drive = 0;
}

void guide_track(const guide_rx_t *rx, int16_t branch, int16_t backward, guide_out_t *out)
{
    guide_blob_t blob[GUIDE_BLOB_MAX];
    uint16_t leds = rx->wide ? GUIDE_LED_WIDE : GUIDE_LED_NARROW;
    uint32_t lat = guide_lateral(rx);
    uint32_t all = rx->wide ? 0xFFFFFFFFUL : 0xFFFFUL;
    int16_t  invert = backward ? s_cfg.invert_back : s_cfg.invert_fwd;
    int16_t  chosen;

    out->blobs = guide_blobs(lat, leds, blob);
    out->track = GUIDE_TRK_NORMAL;
    if (s_drive) {
        if (out->blobs >= 2u) { s_merge_age = 0; }
        else if ((s_trk_last == GUIDE_TRK_NORMAL) && (s_merge_age < GUIDE_MERGE_SCANS)) { s_merge_age++; }
    }
    if (lat == all) {
        out->status = GUIDE_ALL_ON;
        out->width = leds;
    }
    else {
        chosen = (out->blobs == 0) ? -1 : pick(blob, out->blobs, leds, branch, backward, invert);
        if (chosen < 0) { s_cand_n = 0; }
        if ((chosen < 0)
            || (s_drive && (branch == 0) && s_ever && !s_prev_valid && reacq_wait(blob, out->blobs, leds, &chosen))) {
            out->status = GUIDE_LOST;
            out->width = 0;
            s_prev_valid = 0;
        }
        else {
            int16_t s = (int16_t)blob[chosen].start, e = (int16_t)blob[chosen].end;
            out->status = GUIDE_NORMAL;
            out->width = (uint16_t)(e - s + 1);
            /* 분기는 일부러 선을 갈아타는 동작이라 가장자리를 고치지 않는다. 정렬 회전·튜닝 직접 호출도 고치지 않는다 */
            if ((branch == 0) && s_drive) { out->track = edge_fix(&s, &e, leds, out->blobs, s_prev_valid); }
            if (out->track != GUIDE_TRK_NORMAL) { out->width = guide_tape_width(); }   /* 붙은 폭을 커브로 보지 않는다 */
            if (out->track != GUIDE_TRK_HOLD) {
                s_prev = (int16_t)(s + e - (int16_t)(leds + 1));
                s_prev_valid = 1;
                s_ever = 1;
                s_cand_n = 0;
            }
            if (out->track == GUIDE_TRK_NORMAL) {
                s_edge_s = s;
                s_edge_e = e;
            }
        }
    }
    if (s_drive) { s_trk_last = out->track; }
    out->pos = invert ? (int16_t)(-s_prev) : s_prev;
}

int16_t guide_h_position(const guide_rx_t *rx, int16_t prev)
{
    uint32_t lat = guide_lateral(rx);
    if (lat == 0) { return rx->wide ? POS_OFF_WIDE : POS_OFF_NARROW; }
    if (lat == (rx->wide ? 0xFFFFFFFFUL : 0xFFFFUL)) { return rx->wide ? POS_ON_WIDE : POS_ON_NARROW; }
    return param_quirk(QUIRK_H_POSITION) ? H_PLACEHOLDER : prev;
}

int16_t guide_source_led(void)
{
    return !param_quirk(QUIRK_SRC_A);
}

uint16_t guide_curve_span(void)
{
    uint16_t w = (uint16_t)(guide_tape_width() + 2u);   /* 25 mm 테이프는 직선에서도 4칸이 켜진다 */
    return (w > s_cfg.curve_span) ? w : s_cfg.curve_span;
}

int16_t guide_want_h(int16_t branch_start, int16_t branch_step)
{
    return guide_source_led() || ((branch_start != 0) && (branch_step != 3));
}
