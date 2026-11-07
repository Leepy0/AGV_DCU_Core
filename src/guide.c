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
#define LEN_H_NARROW       8       /* STX h 16진×4 BCC ETX */
#define LEN_H_WIDE         13      /* STX h 16진×9 BCC ETX */
#define LEN_S              4       /* 'S' 의 최소 길이: STX S BCC ETX */

#define RX_BUF_SIZE        16      /* 가장 긴 응답 13 바이트 */
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

void guide_cfg_default(guide_cfg_t *cfg)
{
    uint16_t i;
    cfg->invert_fwd = 0;
    cfg->invert_back = 0;
    for (i = 0; i < GUIDE_VERT_COUNT; i++) { cfg->vert_bit[i] = (uint16_t)(GUIDE_LED_WIDE + i); }
    cfg->min_width = 1;
    cfg->max_width = GUIDE_LED_WIDE;
    cfg->max_step = 0;
    cfg->curve_span = GUIDE_CURVE_SPAN_DEFAULT;
}

void guide_configure(const guide_cfg_t *cfg) { s_cfg = *cfg; }

static void comm_clear(void)
{
    s_comm.armed = 0;
    s_comm.waiting = 0;
    s_comm.miss = 0;
    s_comm.bad = 0;
    s_comm.error = 0;
}

void guide_on_run_start(void) { comm_clear(); }

void guide_on_sensor_switch(void)
{
    s_rx.cnt = 0;
    s_rx.in_packet = 0;
    s_prev = 0;           /* 앞 센서의 위치로 뒤 센서 덩어리를 고르면 안 된다 */
    s_prev_valid = 0;
    comm_clear();
}

void guide_reset(void)
{
    guide_cfg_default(&s_cfg);
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
    if ((len < LEN_A_NARROW) || ((buf[2] != '-') && (buf[2] != '+'))
        || (buf[3] < '0') || (buf[3] > '9') || (buf[4] < '0') || (buf[4] > '9')) {
        rx->kind = GUIDE_RX_BAD_FORMAT;
        return;
    }
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
    rx->wide = (len == LEN_H_WIDE);
    if ((len != LEN_H_NARROW) && (len != LEN_H_WIDE)) {
        rx->kind = GUIDE_RX_BAD_FORMAT;
        return;
    }
    digits = rx->wide ? 9 : 4;
    for (i = 0; i < digits; i++) {
        int16_t v = hex_value(buf[2 + i]);
        if (v < 0) {
            rx->kind = GUIDE_RX_BAD_FORMAT;
            return;
        }
        if (i < 8) { rx->led_lo |= (uint32_t)v << (i * 4); }
        else       { rx->led_hi = (uint16_t)v; }
    }
}

void guide_parse(const uint16_t *buf, uint16_t len, guide_rx_t *rx)
{
    rx->kind = GUIDE_RX_NONE;
    rx->wide = 0;
    rx->pos = 0;
    rx->status = GUIDE_NORMAL;
    rx->led_lo = 0;
    rx->led_hi = 0;

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

static uint16_t raw_bit(const guide_rx_t *rx, uint16_t n)
{
    if (n < 32) { return (uint16_t)((rx->led_lo >> n) & 1UL); }
    return (uint16_t)((rx->led_hi >> (n - 32)) & 1u);
}

static uint16_t is_vert_bit(uint16_t n)
{
    uint16_t i;
    for (i = 0; i < GUIDE_VERT_COUNT; i++) {
        if (s_cfg.vert_bit[i] == n) { return 1; }
    }
    return 0;
}

uint32_t guide_lateral(const guide_rx_t *rx)
{
    uint32_t lat = 0;
    uint16_t n, led = 0;

    if (!rx->wide) { return rx->led_lo & 0xFFFFUL; }
    for (n = 0; n < GUIDE_BITS_WIDE; n++) {
        if (is_vert_bit(n)) { continue; }
        if (raw_bit(rx, n)) { lat |= 1UL << led; }
        led++;
    }
    return lat;
}

uint16_t guide_vertical(const guide_rx_t *rx)
{
    uint16_t i, v = 0;
    if (!rx->wide) { return 0; }
    for (i = 0; i < GUIDE_VERT_COUNT; i++) {
        if (raw_bit(rx, s_cfg.vert_bit[i])) { v |= (uint16_t)(1u << i); }
    }
    return v;
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

void guide_track(const guide_rx_t *rx, int16_t branch, int16_t backward, guide_out_t *out)
{
    guide_blob_t blob[GUIDE_BLOB_MAX];
    uint16_t leds = rx->wide ? GUIDE_LED_WIDE : GUIDE_LED_NARROW;
    uint32_t lat = guide_lateral(rx);
    uint32_t all = rx->wide ? 0xFFFFFFFFUL : 0xFFFFUL;
    int16_t  invert = backward ? s_cfg.invert_back : s_cfg.invert_fwd;
    int16_t  chosen;

    out->vert = guide_vertical(rx);
    out->blobs = guide_blobs(lat, leds, blob);
    if (lat == all) {
        out->status = GUIDE_ALL_ON;
        out->width = leds;
    }
    else {
        chosen = (out->blobs == 0) ? -1 : pick(blob, out->blobs, leds, branch, backward, invert);
        if (chosen < 0) {
            out->status = GUIDE_LOST;
            out->width = 0;
            s_prev_valid = 0;
        }
        else {
            out->status = GUIDE_NORMAL;
            out->width = (uint16_t)(blob[chosen].end - blob[chosen].start + 1);
            s_prev = blob_pos(&blob[chosen], leds);
            s_prev_valid = 1;
        }
    }
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
    return param_get(P_GUIDE_CURVE_DETECT) == GUIDE_SRC_LED_ONLY;
}

uint16_t guide_curve_span(void) { return s_cfg.curve_span; }

int16_t guide_want_h(int16_t branch_start, int16_t branch_step)
{
    return guide_source_led() || ((branch_start != 0) && (branch_step != 3));
}
