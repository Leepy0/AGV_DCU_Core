/* test_frame.c - 코덱·파서 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/frame.h"
#include "fakes.h"

/* 본문 17자 또는 9자에 STX·BCC·ETX 를 붙인다 */
static int make(uint16_t *out, const char *body, int bcc_delta)
{
    int n = (int)strlen(body);
    out[0] = FRAME_STX;
    for (int i = 0; i < n; i++) { out[i + 1] = (uint16_t)body[i]; }
    out[n + 1] = (uint16_t)(frame_checksum(&out[1], (uint16_t)n) + bcc_delta);
    out[n + 2] = FRAME_ETX;
    return n + 3;
}

static frame_ev_t feed(const uint16_t *b, int n)
{
    frame_ev_t ev = FRAME_EV_NONE;
    for (int i = 0; i < n; i++) { ev = frame_feed(b[i]); }
    return ev;
}

static frame_ev_t send(const char *body, int bcc_delta)
{
    uint16_t f[40];
    int n = make(f, body, bcc_delta);
    return feed(f, n);
}

#define T_OK "E3107T01200000000"     /* [10]='0' 인 태그 프레임 */

int main(void)
{
    uint16_t t[20];
    frame_stat_t before;
    static const uint16_t v1[] = { 'H' };
    static const uint16_t v2[] = { '1','2','3','4','5','6','7','8' };
    static const uint16_t v3[] = { 'E','3','1','0','7','I','0','1','2','2','4','5',' ' };

    fake_reset();
    fake_params();
    assert(param_set(P_AGV_ID, 7) == 0);
    frame_reset();

    /* 프로토콜 레퍼런스 벡터 */
    assert(frame_checksum(v1, 1) == '8');
    assert(frame_checksum(v2, 8) == '4');
    assert(frame_checksum(v3, 13) == '7');

    /* 정상 20·12 바이트 */
    assert(strlen(T_OK) == 17);
    assert(send(T_OK, 0) == FRAME_EV_OK);
    assert(frame_rx()->len == 20 && frame_rx()->b[6] == 'T');
    assert(send("C3107C001", 0) == FRAME_EV_OK);
    assert(frame_rx()->len == 12);
    assert(send("C3107S123", 0) == FRAME_EV_OK);
    assert(send("W3107C00000000000", 0) == FRAME_EV_OK);
    assert(send("A3107EI0000000000", 0) == FRAME_EV_OK);
    assert(send("N31071231240000000", 0) == FRAME_EV_NONE);      /* 18 자는 길이 오류 */
    assert(send("N3107123124000000", 0) == FRAME_EV_OK);
    assert(send("D3107XXXXXXXXXXXX", 0) == FRAME_EV_OK);

    /* 수신 MsgID '0' 허용, 'A'~'F' 허용 */
    assert(send("E3007T01200000000", 0) == FRAME_EV_OK);
    assert(send("E3F07T01200000000", 0) == FRAME_EV_OK);
    /* [2] 는 'T' 번호의 백의 자리라 '0' 도 정상이다 */
    assert(send("E0107T01200000000", 0) == FRAME_EV_OK);
    /* 리더 보드는 'E' 의 MsgID 로 'Z' 를 보낸다. 다른 종류는 그대로 검사한다 */
    assert(send("E0Z07T00400000000", 0) == FRAME_EV_OK);
    assert(send("W3Z07C00000000000", 0) == FRAME_EV_NONE);
    /* 'T' 는 [7] 이 번호 자리가 아니다 */
    assert(send("E3107TX1200000000", 0) == FRAME_EV_OK);

    /* 길이 오류 */
    before = *frame_stat();
    assert(send("E3107T0120000000", 0) == FRAME_EV_NONE);
    assert(frame_stat()->len_err == before.len_err + 1);

    /* STX 중간 삽입: 진행 중 프레임을 버리고 다시 시작 */
    { uint16_t f[40];
      int n = make(f, T_OK, 0);
      before = *frame_stat();
      (void)feed(f, 8);
      assert(feed(f, n) == FRAME_EV_OK);
      assert(frame_stat()->aborted == before.aborted + 1); }

    /* ETX 조기: cnt <= 5 면 데이터로 들어가 길이가 어긋난다 */
    { uint16_t f[40], h[41];
      int n = make(f, T_OK, 0);
      memcpy(h, f, 3 * sizeof(uint16_t));
      h[3] = FRAME_ETX;
      memcpy(&h[4], &f[3], (size_t)(n - 3) * sizeof(uint16_t));
      assert(feed(h, n + 1) == FRAME_EV_NONE); }
    /* cnt 6 이상의 ETX 는 종료다 */
    (void)frame_feed(FRAME_STX);
    for (int i = 0; i < 5; i++) { (void)frame_feed('1'); }
    before = *frame_stat();
    assert(frame_feed(FRAME_ETX) == FRAME_EV_NONE);
    assert(frame_stat()->len_err == before.len_err + 1);

    /* STX 전 잡음 */
    assert(frame_feed('X') == FRAME_EV_NONE);
    assert(frame_feed(FRAME_ETX) == FRAME_EV_NONE);

    /* BCC 오류: 'T' 형태면 재시도 후보 이벤트, 나머지는 폐기 */
    before = *frame_stat();
    assert(send(T_OK, 1) == FRAME_EV_RFID_BAD);
    assert(frame_rx()->b[6] == 'T');
    assert(send("E3107I01200000000", 1) == FRAME_EV_NONE);
    assert(send("E3107T01210000000", 1) == FRAME_EV_NONE);     /* [10] != '0' */
    assert(frame_stat()->bcc_err == before.bcc_err + 3);

    /* AGV ID 불일치. BCC 불량 T 도 내 ID 가 아니면 후보가 아니다 */
    before = *frame_stat();
    assert(send("E3108T01200000000", 0) == FRAME_EV_NONE);
    assert(send("C3108C001", 0) == FRAME_EV_NONE);
    assert(frame_stat()->id_mismatch == before.id_mismatch + 2);
    assert(send("E3108T01200000000", 1) == FRAME_EV_NONE);
    assert(send("E31A7T01200000000", 0) == FRAME_EV_NONE);     /* ID 자리에 문자 */

    /* 종류 */
    before = *frame_stat();
    assert(send("Z3107T01200000000", 0) == FRAME_EV_NONE);
    assert(send("E3107C001", 0) == FRAME_EV_NONE);             /* 12 바이트는 'C' 만 */
    assert(send("C3107C00100000000", 0) == FRAME_EV_NONE);     /* 'C' 는 20 바이트가 아니다 */
    assert(frame_stat()->type_err == before.type_err + 3);

    /* PAN·MsgID 범위 */
    before = *frame_stat();
    assert(send("EG107T01200000000", 0) == FRAME_EV_NONE);
    assert(send("W3G07C00000000000", 0) == FRAME_EV_NONE);
    assert(frame_stat()->pan_err == before.pan_err + 1);
    assert(frame_stat()->msgid_err == before.msgid_err + 1);

    /* 자리별 */
    before = *frame_stat();
    assert(send("E3107I01X00000000", 0) == FRAME_EV_NONE);     /* 번호 자리에 문자 */
    assert(send("E3107I01210000000", 0) == FRAME_EV_NONE);     /* [10] != '0' */
    assert(send("EA107T01200000000", 0) == FRAME_EV_NONE);     /* 'T' 의 [2] 는 숫자 */
    assert(send("N31071A3124000000", 0) == FRAME_EV_NONE);
    assert(send("W3107C00X00000000", 0) == FRAME_EV_NONE);
    assert(send("A3107Ea0000000000", 0) == FRAME_EV_NONE);
    assert(send("C3107CX0X", 0) == FRAME_EV_NONE);
    assert(send("C3107S1X3", 0) == FRAME_EV_NONE);
    assert(send("C3107Q123", 0) == FRAME_EV_NONE);
    assert(frame_stat()->field_err == before.field_err + 9);

    /* 넘침 */
    before = *frame_stat();
    (void)frame_feed(FRAME_STX);
    for (int i = 0; i < 21; i++) { assert(frame_feed('1') == FRAME_EV_NONE); }
    assert(frame_stat()->len_err == before.len_err + 1);

    /* MsgID 송신 순환: '1'~'F', '0' 없음 */
    frame_reset();
    { static const char want[] = "123456789ABCDEF1";
      for (int i = 0; want[i]; i++) { assert(frame_msgid_next() == (uint16_t)want[i]); } }

    /* 빌더 */
    frame_build(t, 'E', '3', '1', 'L', 12, 245);
    assert(t[0] == FRAME_STX && t[1] == 'E' && t[2] == '3' && t[3] == '1');
    assert(t[4] == '0' && t[5] == '7' && t[6] == 'L');
    assert(t[7] == '0' && t[8] == '1' && t[9] == '2' && t[10] == '2' && t[11] == '4' && t[12] == '5');
    assert(t[13] == '0' && t[17] == '0' && t[19] == FRAME_ETX);
    assert(t[18] == frame_checksum(&t[1], 17));
    frame_build_ack(t, '3', '5', 'E', 'T', 'W');
    assert(t[1] == 'A' && t[3] == '5' && t[6] == 'E' && t[7] == 'T' && t[8] == 'A' && t[9] == 'W' && t[10] == '0');
    assert(t[18] == frame_checksum(&t[1], 17));
    /* 빌더가 만든 ACK 는 파서를 통과한다 ([7] 이 대문자) */
    frame_build_ack(t, '3', '5', 'E', 'T', 'W');
    assert(feed(t, 20) == FRAME_EV_OK);

    puts("test_frame ok");
    return 0;
}
