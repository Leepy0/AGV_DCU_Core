/*
 * guide.h - 가이드센서 응답 해석과 LED 패턴 위치 추적. HAL 없이 값만 받는다.
 *
 * 일반 센서: 좌우 16 LED, 'a' 응답 7바이트(STX a 부호 십 일 CS ETX), 'h' 응답 16비트(16진 4자리).
 * wide 센서: 좌우 32 LED + 세로 LED (T자 배치). 'a' 응답 8바이트(STX a 부호 십 일 V CS ETX),
 * 'h' 응답 32비트(16진 8자리, 12바이트)뿐이다.
 *
 * 제조사 사양 확인
 *  - 세로 센서는 wide 'a' 의 V('1' 켜짐, '0' 꺼짐)로 읽는다. 'h' 에는 세로가 없다.
 *  - 'h' LED 패턴 위치는 'a' 와 부호·단위가 같다. 단 'a' 는 가운데 세 칸 점등 상태를 0 으로 보낸다.
 *
 * 필터는 기본값이 전부 꺼짐이다. 실측 없이 켜면 정상 테이프를 버려 정지할 수 있다.
 * 분기 동작은 여기 없다. 이 모듈은 위치와 상태 계산까지만 한다.
 *
 * 전진용·후진용 센서는 같은 SCI-B 를 쓰고 AGV_BACK 릴레이로 전원을 바꾼다. 동시에 읽을 수 없다.
 * 전환하면 guide_on_sensor_switch 를 불러 앞 센서의 상태를 전부 버린다.
 */
#ifndef AGV_GUIDE_H
#define AGV_GUIDE_H

#include "agv/types.h"

#define GUIDE_LED_NARROW   16
#define GUIDE_LED_WIDE     32
#define GUIDE_BLOB_MAX     16      /* 32칸에 번갈아 켜진 경우 */
#define GUIDE_CROSS_LED_MIN 10     /* 이만큼 켜지면 교차선 */

#define GUIDE_CURVE_SPAN_DEFAULT 4     /* 레거시 판정 (점등 4칸 이상) */
#define GUIDE_MISS_LIMIT    3          /* 무응답 연속 3회 = 30 ms. P_GUIDE_TIMEOUT 이 0 일 때 */
#define GUIDE_BAD_LIMIT     10         /* 체크섬 실패 연속 10회 */

/* P_GUIDE_CURVE_DETECT 값. 빈 파라미터 번호가 없어 이 번호를 확장해 소스 선택을 겸한다 */
enum { GUIDE_SRC_POSITION = 0, GUIDE_SRC_MIXED = 1, GUIDE_SRC_LED_ONLY = 2 };

typedef enum { GUIDE_NORMAL = 0, GUIDE_ALL_ON, GUIDE_LOST } guide_status_t;

/* 'h' 추적이 위치를 얻은 방법. 조향이 적분과 칼만 갱신을 여기에 맞춘다 */
typedef enum {
    GUIDE_TRK_NORMAL = 0,  /* 덩어리 가운데 */
    GUIDE_TRK_EDGE,        /* 다른 선과 붙었다. 안 움직인 가장자리에서 테이프 폭으로 잡은 위치 */
    GUIDE_TRK_HOLD         /* 두 가장자리가 다 움직였다. 이전 위치를 유지한다 */
} guide_track_t;
#define GUIDE_HOLD_LIMIT_MM  100   /* 유지 주행 상한. 30도 교차·25 mm 테이프에서 덮이는 구간이 약 43 mm */
#define GUIDE_EDGE_LIMIT_MM  300   /* 가장자리 고정 상한. 넘으면 테이프 폭이 바뀐 것으로 보고 다시 배운다 */

typedef enum {
    GUIDE_RX_NONE = 0,     /* 처리할 내용 없음 (모르는 종류) */
    GUIDE_RX_POSITION,     /* 'a' */
    GUIDE_RX_LED,          /* 'h' */
    GUIDE_RX_SENSOR_ERR,   /* 'S' */
    GUIDE_RX_BAD_BCC,
    GUIDE_RX_BAD_FORMAT
} guide_rx_kind_t;

typedef struct {
    int16_t  kind;         /* guide_rx_kind_t */
    int16_t  wide;
    int16_t  pos;          /* 'a' 위치. 전부 켜짐이면 0 */
    int16_t  status;       /* 'a' 상태. 'h' 상태는 guide_track 이 낸다 */
    int16_t  vert;         /* wide 'a' 의 세로 센서 V. 1 켜짐. 일반·'h' 는 0 */
    uint32_t led_lo;       /* 'h' 비트 0~31 */
} guide_rx_t;

typedef struct {
    int16_t  invert_fwd, invert_back;    /* 1 = 위치 부호 반전. 센서별로 둔다. 분기 끝 해석도 함께 뒤집힌다 */
    uint16_t min_width;                  /* 이보다 좁은 덩어리를 버린다. 기본 1 */
    uint16_t max_width;                  /* 이보다 넓은 덩어리를 버린다. 기본 전체 */
    uint16_t max_step;                   /* 추적 중 스캔당 최대 위치 변화. 0 = 무제한 */
    float    h_scale;                    /* 'h' 위치를 'a' 단위로 바꾸는 배율. 튜닝 모드가 잰다. 기본 1 */
    uint16_t curve_span;                 /* 'h' 소스의 커브 판정: 덩어리 폭이 이 칸 수 이상이면 커브.
                                            테이프 폭 10~25 mm 가 약 1~4칸이라 현장별로 맞춘다. 기본 4 */
} guide_cfg_t;

typedef struct { uint16_t start, end; } guide_blob_t;   /* 칸 번호는 1부터 */

typedef struct {
    int16_t  pos;          /* 위치. ALL_ON·LOST 이면 마지막 위치를 유지한다 */
    int16_t  status;       /* guide_status_t */
    uint16_t width;        /* 선택한 덩어리 폭. 조향의 커브 판정 span 입력으로 쓸 수 있다 */
    uint16_t blobs;        /* 필터 전 덩어리 수 */
    int16_t  track;        /* guide_track_t */
} guide_out_t;

void     guide_reset(void);
void     guide_cfg_default(guide_cfg_t *cfg);
void     guide_configure(const guide_cfg_t *cfg);
void     guide_cfg_get(guide_cfg_t *cfg);
float    guide_h_scale(void);

/* 전진·후진 센서 전환 뒤 호출. 수신 파서, 이전 위치, 덩어리 상태, 오류 카운터를 지운다 */
void     guide_on_sensor_switch(void);
/* 주행 시작 때 호출. 첫 유효 응답까지 두절 판정을 쉰다. 그동안은 준비 시간 초과(257)가 맡는다 */
void     guide_on_run_start(void);

/* 응답 해석. buf 는 STX~ETX, len 은 그 바이트 수 */
void     guide_parse(const uint16_t *buf, uint16_t len, guide_rx_t *rx);
/* 수신 바이트 하나. 프레임이 끝나면 1 을 돌려주고 rx 를 채운다 */
int16_t  guide_feed(uint16_t byte, guide_rx_t *rx);
/* 요청을 보낸 스캔에만 부른다. 무응답 판정은 여기서만 한다. 반환 0 또는 ERR_GUIDE_COMM */
int16_t  guide_on_request(void);
int16_t  guide_comm_error(void);          /* 0 또는 ERR_GUIDE_COMM. 전환·시작·STOP 리셋 때까지 유지 */
void     guide_comm_clear(void);          /* STOP 리셋에서 호출 */

/* 좌우 비트만 칸 순서로 모은다 */
uint32_t guide_lateral(const guide_rx_t *rx);
uint16_t guide_blobs(uint32_t lateral, uint16_t leds, guide_blob_t *out);

/* branch: 0 없음, BRANCH_LEFT, BRANCH_RIGHT. 진행 방향 기준이고 backward 로 센서 좌우로 바꾼다 */
void     guide_track(const guide_rx_t *rx, int16_t branch, int16_t backward, guide_out_t *out);

/* 'a' 소스에서 'h' 응답이 왔을 때의 위치값. 전부 꺼짐·켜짐 표시만 넣고 그 밖에는 이전 값을 둔다 */
int16_t  guide_h_position(const guide_rx_t *rx, int16_t prev);

int16_t  guide_source_led(void);                                  /* 위치를 'h' 로 얻는가 */
uint16_t guide_curve_span(void);                                  /* 설정값과 (배운 테이프 폭 + 2) 중 큰 쪽 */
uint16_t guide_tape_width(void);                                  /* 배운 테이프 폭(칸). 0 = 아직 모름 */
/* 가장자리 고정·유지가 상한 거리를 넘었을 때 부른다. 다음 응답은 덩어리 가운데를 받고 폭을 다시 배운다 */
void     guide_track_release(void);
int16_t  guide_want_h(int16_t branch_start, int16_t branch_step); /* 다음 요청이 'H' 인가 */

/* SCI-B 입출력과 오류 판정. 10 ms (src/sys/guide_poll.c) */
void     guide_poll(void);
void     guide_poll_reset(void);
void     guide_poll_start(void);      /* 주행 시작 */

#endif /* AGV_GUIDE_H */
