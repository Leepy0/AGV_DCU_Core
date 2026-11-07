/* frame.c - STX/ETX 코덱, 체크섬, 20·12바이트 파서, 자리별 검사, AGV ID 필터 */
#include "agv/frame.h"
#include "agv/param.h"

/* ETX 를 종료로 인정하는 최소 cnt. 그 이하의 0x03 은 데이터다 (원본 동작) */
#define FRAME_ETX_MIN_CNT 5

static frame_t      s_rx;
static uint16_t     s_cnt;
static frame_stat_t s_stat;
static uint16_t     s_msgid = '1';

static int16_t is_digit(uint16_t c)  { return (c >= '0') && (c <= '9'); }
static int16_t is_hex_upper(uint16_t c) { return is_digit(c) || ((c >= 'A') && (c <= 'F')); }
static int16_t digits(const uint16_t *b, uint16_t from, uint16_t to)
{
    uint16_t i;
    for (i = from; i <= to; i++) {
        if (!is_digit(b[i])) { return 0; }
    }
    return 1;
}

void frame_reset(void)
{
    uint16_t i;
    uint16_t *p = (uint16_t *)&s_stat;
    s_cnt = 0;
    s_msgid = '1';
    for (i = 0; i < sizeof(s_stat) / sizeof(uint16_t); i++) { p[i] = 0; }
}

const frame_t *frame_rx(void)            { return &s_rx; }
const frame_stat_t *frame_stat(void)     { return &s_stat; }

uint16_t frame_checksum(const uint16_t *p, uint16_t n)
{
    uint16_t i, sum = 0;
    for (i = 0; i < n; i++) { sum = (uint16_t)(sum + p[i]); }
    sum &= 0x0F;
    return (uint16_t)((sum < 10) ? ('0' + sum) : ('A' + sum - 10));
}

uint16_t frame_msgid_next(void)
{
    uint16_t id = s_msgid;
    if (s_msgid == '9')      { s_msgid = 'A'; }
    else if (s_msgid == 'F') { s_msgid = '1'; }
    else                     { s_msgid++; }
    return id;
}

/* 프레임의 AGV ID 자리가 내 ID 인가 */
static int16_t agv_id_matches(const uint16_t *b)
{
    return is_digit(b[4]) && is_digit(b[5])
        && (((b[4] - '0') * 10 + (b[5] - '0')) == param_get(P_AGV_ID));
}

/* 체크섬 불량이어도 재시도 보상 후보로 쓸 수 있는 'T' 프레임 형태 */
static int16_t is_tag_shape(const uint16_t *b)
{
    return (s_cnt == FRAME_LEN_LONG - 1) && (b[1] == 'E') && (b[6] == 'T') && (b[10] == '0')
        && agv_id_matches(b);
}

/* 종류별 숫자 자리. 체크섬이 4비트뿐이라 내용까지 본다 */
static int16_t fields_ok(const uint16_t *b)
{
    switch (b[1]) {
    case 'E':
        if (b[10] != '0') { return 0; }
        /* 'T' 번호는 [2][8][9] 이고 [7] 은 번호 자리가 아니다 */
        if (b[6] == 'T') { return is_digit(b[2]) && digits(b, 8, 9); }
        return digits(b, 7, 9);
    case 'W':
        if (b[6] == 'C') { return is_digit(b[9]) && digits(b, 10, 12); }
        return 1;
    case 'A':
        return (b[7] >= 'A') && (b[7] <= 'Z');
    case 'N':
        return digits(b, 6, 14);
    case 'C':
        if (b[6] == 'C') { return is_digit(b[9]); }
        if (b[6] == 'S') { return digits(b, 7, 9); }
        return 0;
    default:     /* 'D' 는 상위에서 무시한다 */
        return 1;
    }
}

static frame_ev_t judge(void)
{
    const uint16_t *b = s_rx.b;
    uint16_t type = b[1];

    if ((s_cnt != FRAME_LEN_LONG - 1) && (s_cnt != FRAME_LEN_SHORT - 1)) {
        s_stat.len_err++;
        return FRAME_EV_NONE;
    }
    s_rx.len = (uint16_t)(s_cnt + 1);
    if (frame_checksum(&b[1], (uint16_t)(s_cnt - 2)) != b[s_cnt - 1]) {
        s_stat.bcc_err++;
        return is_tag_shape(b) ? FRAME_EV_RFID_BAD : FRAME_EV_NONE;
    }
    if ((s_cnt == FRAME_LEN_LONG - 1)
            ? ((type != 'E') && (type != 'W') && (type != 'A') && (type != 'N') && (type != 'D'))
            : (type != 'C')) {
        s_stat.type_err++;
        return FRAME_EV_NONE;
    }
    /* [2] 는 PAN 이지만 'T' 는 번호의 백의 자리, 'S' 는 부명령이라 '0' 도 정상이다 */
    if (!is_hex_upper(b[2])) { s_stat.pan_err++;   return FRAME_EV_NONE; }
    /* 구 ACS 호환: 수신 '0' 허용. 리더 보드는 'E' 의 MsgID 자리에 'Z' 를 보낸다(실차 확인) */
    if ((type != 'E') && !is_hex_upper(b[3])) { s_stat.msgid_err++; return FRAME_EV_NONE; }
    if (!agv_id_matches(b))  { s_stat.id_mismatch++; return FRAME_EV_NONE; }
    if (!fields_ok(b))       { s_stat.field_err++; return FRAME_EV_NONE; }
    s_stat.ok++;
    return FRAME_EV_OK;
}

frame_ev_t frame_feed(uint16_t byte)
{
    frame_ev_t ev;

    byte &= 0xFF;
    if (byte == FRAME_STX) {
        if (s_cnt > 0) { s_stat.aborted++; }
        s_cnt = 0;
        s_rx.b[s_cnt++] = byte;
        return FRAME_EV_NONE;
    }
    if ((byte == FRAME_ETX) && (s_cnt > FRAME_ETX_MIN_CNT)) {
        s_stat.closed++;
        ev = judge();
        s_cnt = 0;
        return ev;
    }
    if (s_cnt == 0) { return FRAME_EV_NONE; }     /* STX 전 잡음 */
    if (s_cnt >= FRAME_LEN_LONG) {
        s_stat.len_err++;
        s_cnt = 0;
        return FRAME_EV_NONE;
    }
    s_rx.b[s_cnt++] = byte;
    return FRAME_EV_NONE;
}

static void put_id_and_close(uint16_t *buf)
{
    int16_t id = param_get(P_AGV_ID);
    buf[0] = FRAME_STX;
    buf[4] = (uint16_t)('0' + param_digit(id, 10));
    buf[5] = (uint16_t)('0' + param_digit(id, 1));
    buf[18] = frame_checksum(&buf[1], 17);
    buf[19] = FRAME_ETX;
}

void frame_build(uint16_t *buf, uint16_t type, uint16_t pan, uint16_t msgid,
                 uint16_t info, int16_t num, int16_t bat)
{
    uint16_t i;
    buf[1] = type;
    buf[2] = pan;
    buf[3] = msgid;
    buf[6] = info;
    buf[7]  = (uint16_t)('0' + param_digit(num, 100));
    buf[8]  = (uint16_t)('0' + param_digit(num, 10));
    buf[9]  = (uint16_t)('0' + param_digit(num, 1));
    buf[10] = (uint16_t)('0' + param_digit(bat, 100));
    buf[11] = (uint16_t)('0' + param_digit(bat, 10));
    buf[12] = (uint16_t)('0' + param_digit(bat, 1));
    for (i = 13; i <= 17; i++) { buf[i] = '0'; }
    /* put_id_and_close 가 ID 를 먼저 넣고 BCC 를 계산한다 */
    put_id_and_close(buf);
}

void frame_build_ack(uint16_t *buf, uint16_t pan, uint16_t msgid, uint16_t c6, uint16_t c7, uint16_t c9)
{
    uint16_t i;
    buf[1] = 'A';
    buf[2] = pan;
    buf[3] = msgid;
    buf[6] = c6;
    buf[7] = c7;
    buf[8] = 'A';
    buf[9] = c9;
    for (i = 10; i <= 17; i++) { buf[i] = '0'; }
    put_id_and_close(buf);
}
