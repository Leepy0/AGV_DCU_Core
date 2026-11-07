/*
 * guide.h - 가이드센서 응답 해석과 LED 패턴 위치 추적. HAL 없이 값만 받는다.
 *
 * 일반 센서: 좌우 16 LED, 'h' 응답 16비트.
 * wide 센서: 좌우 32 LED + 세로 4 LED (T자 배치), 'h' 응답 36비트.
 * 세로 LED 는 좌우 위치 계산에 절대 넣지 않는다.
 *
 * 실측 필요
 *  - wide 의 세로 LED 비트 위치. 기본값 32~35 는 추정이고 vert_bit 로 바꿀 수 있다.
 *  - LED 패턴 위치가 'a' 위치와 같은 값인지, 부호가 같은지. 다르면 invert 로 맞춘다.
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
#define GUIDE_BITS_WIDE    36
#define GUIDE_VERT_COUNT   4
#define GUIDE_BLOB_MAX     16      /* 32칸에 번갈아 켜진 경우 */

#define GUIDE_CURVE_SPAN_DEFAULT 4     /* 레거시 판정 (점등 4칸 이상) */
#define GUIDE_MISS_LIMIT    3          /* 무응답 연속 3회 = 30 ms. P_GUIDE_TIMEOUT 이 0 일 때 */
#define GUIDE_BAD_LIMIT     10         /* 체크섬 실패 연속 10회 */

/* P_GUIDE_CURVE_DETECT 값. 빈 파라미터 번호가 없어 이 번호를 확장해 소스 선택을 겸한다 */
enum { GUIDE_SRC_POSITION = 0, GUIDE_SRC_MIXED = 1, GUIDE_SRC_LED_ONLY = 2 };

typedef enum { GUIDE_NORMAL = 0, GUIDE_ALL_ON, GUIDE_LOST } guide_status_t;

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
    uint32_t led_lo;       /* 'h' 비트 0~31 */
    uint16_t led_hi;       /* 'h' 비트 32~35 */
} guide_rx_t;

typedef struct {
    int16_t  invert_fwd, invert_back;    /* 1 = 위치 부호 반전. 센서별로 둔다. 분기 끝 해석도 함께 뒤집힌다 */
    uint16_t vert_bit[GUIDE_VERT_COUNT]; /* wide 세로 LED 의 응답 비트 번호. 나머지 32비트가 좌우다 */
    uint16_t min_width;                  /* 이보다 좁은 덩어리를 버린다. 기본 1 */
    uint16_t max_width;                  /* 이보다 넓은 덩어리를 버린다. 기본 전체 */
    uint16_t max_step;                   /* 추적 중 스캔당 최대 위치 변화. 0 = 무제한 */
    uint16_t curve_span;                 /* 'h' 소스의 커브 판정: 덩어리 폭이 이 칸 수 이상이면 커브.
                                            테이프 폭 10~25 mm 가 약 1~4칸이라 현장별로 맞춘다. 기본 4 */
} guide_cfg_t;

typedef struct { uint16_t start, end; } guide_blob_t;   /* 칸 번호는 1부터 */

typedef struct {
    int16_t  pos;          /* 위치. ALL_ON·LOST 이면 마지막 위치를 유지한다 */
    int16_t  status;       /* guide_status_t */
    uint16_t width;        /* 선택한 덩어리 폭. 조향의 커브 판정 span 입력으로 쓸 수 있다 */
    uint16_t blobs;        /* 필터 전 덩어리 수 */
    uint16_t vert;         /* 세로 4비트 원시값 */
} guide_out_t;

void     guide_reset(void);
void     guide_cfg_default(guide_cfg_t *cfg);
void     guide_configure(const guide_cfg_t *cfg);

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
int16_t  guide_comm_error(void);          /* 0 또는 ERR_GUIDE_COMM. 전환·시작 때까지 유지 */

/* 좌우 비트만 칸 순서로 모은다 */
uint32_t guide_lateral(const guide_rx_t *rx);
uint16_t guide_vertical(const guide_rx_t *rx);
uint16_t guide_blobs(uint32_t lateral, uint16_t leds, guide_blob_t *out);

/* branch: 0 없음, BRANCH_LEFT, BRANCH_RIGHT. 진행 방향 기준이고 backward 로 센서 좌우로 바꾼다 */
void     guide_track(const guide_rx_t *rx, int16_t branch, int16_t backward, guide_out_t *out);

/* 'a' 소스에서 'h' 응답이 왔을 때의 위치값. 전부 꺼짐·켜짐 표시만 넣고 그 밖에는 이전 값을 둔다 */
int16_t  guide_h_position(const guide_rx_t *rx, int16_t prev);

int16_t  guide_source_led(void);                                  /* 위치를 'h' 로 얻는가 */
uint16_t guide_curve_span(void);
int16_t  guide_want_h(int16_t branch_start, int16_t branch_step); /* 다음 요청이 'H' 인가 */

#endif /* AGV_GUIDE_H */
