/* gresp.h - 가이드 센서 응답 시험(튜닝 t3.7 GR). 개발 시험이라 AGV_DEV_TESTS 0 이미지에는 없다.
 * 정지 상태에서 SCI-B 로 'H' 요청을 보내고 응답 시간과 안정 주기를 잰다. 주행 없음.
 *  1 응답 시간: 'H' 송신 시작 → 응답 마지막 바이트 수신(SCI-B 수신 ISR 이 찍은 hal_cycles). 10 ms 간격 GR_N 회의 정상 응답.
 *  2 안정 주기: 요청 간격을 10·8·6·5·4·3·2 ms, 응답 끝나면 바로(0) 로 줄여 가며 GR_N 회 연속 오류 0 인 가장 짧은 간격.
 *    오류가 난 간격은 한 번 더 하고(잡음), 또 나면 거기서 멈춘다. 바로 재요청 단계의 안정 주기는 잰 평균 요청 주기다.
 *  무응답·라인 없음(첫 간격 정상 응답이 모두 LED 0)은 오류가 아니라 없음(state 3, 값 TUNE_NO_DATA).
 *  잡음 오류(체크섬·길이·형식·무응답·SCI 수신 오류)는 err 에 세고 시험은 잇는다.
 * 도는 동안 guide_poll 은 SCI-B 를 건드리지 않는다. 끝나면 링을 비우고 평상시 조회(10 ms, guide_poll)로 돌아간다 */
#ifndef AGV_GRESP_H
#define AGV_GRESP_H
#include "agv/tune.h"
#include "agv/guide.h"

#define GR_N 50              /* 간격마다 요청 수 */

/* 결과(agv_all.gr). 시간은 0.1 ms 단위. 화면 이름 GRL·GRS·GRH 와 같다 */
typedef struct {
    int16_t state;           /* 0 안 함, 1 진행, 2 끝, 3 없음(무응답·라인 없음) */
    int16_t grl;             /* GRL 응답 시간 최소. TUNE_NO_DATA = 없음 */
    int16_t grs;             /* GRS 안정 주기. TUNE_NO_DATA = 없음(10 ms 에서도 오류) */
    int16_t grh;             /* GRH 응답 시간 최대. TUNE_NO_DATA = 없음 */
    int16_t n;               /* GRL·GRH 표본 수(10 ms 간격의 정상 응답) */
    int16_t err;             /* 오류 수(전 간격 합) */
    int16_t step;            /* 마지막으로 한 간격 단계 1~8 */
} gresp_result_t;

/* ---- 스캔 주기 시험(튜닝 t3.8 GS) ----
 * 센서가 LED 를 몇 ms 마다 새로 읽는지(스캔 주기) 잰다. 서서 라인 위에서 START. 시작 때 라인이 안 보이면 바로 없음.
 * 제자리 360도 CW 정렬 회전(tagact_turn_align_stopped, TNS·ACC·DCC, 끝은 정렬 회전 끝 로직으로 라인 위)을 하는 동안
 * 응답이 끝나면 바로 다음 'H' 를 보낸다(무응답 GS_WAIT_MS). guide_poll 은 10 ms 마다 마지막 응답 하나로 회전을 제어한다.
 * 정상 'h' 의 LED 비트가 직전 정상 응답과 다르면 그 응답을 부른 요청의 송신 시각(hal_cycles)을 t 에 넣는다(GS_BUF 개까지).
 * 회전이 끝나면 메인 루프에서 조금씩 GSP 를 셈한다: Δ ≤ GS_PAIR_MS 인 모든 변화 쌍의 Δ 로, 후보 P(2.00~20.00 ms, 0.02 ms 간격)마다
 * (Δ mod P) 위상의 원형 집중도 R(64칸 Q15 cos·sin)을 구한다. 회전 기하(테이프 가장자리가 칸을 지나는 주기 11~20 ms)도
 * 봉우리를 만들어 R 최대만으로는 틀린다(모의). 그래서 스캔이면 나올 기대 R 과 견줘 통과한 후보 가운데 R 최대(gs_candidate_end).
 * 결과와 시각 버퍼는 보존 영역 밖(cmd 의 gscan_ram, RAMH0 곡선 기록기 뒤)이다. 부팅 때 지운다 */
#define GS_BUF        256
#define GS_PAIR_MS    100
#define GS_WAIT_MS    20
typedef struct {
    int16_t  state;          /* 0 안 함, 1 진행, 2 끝, 3 없음(시작 때 라인 없음·무응답·회전 못 함) */
    int16_t  gsn;            /* FND GN: 변화 횟수(버퍼를 넘어도 센다) */
    int16_t  gsl;            /* GSL 연속한 두 변화 간격 최소, 0.1 ms. TUNE_NO_DATA = 없음 */
    int16_t  gsp;            /* GSP 추정 스캔 주기, 0.1 ms. TUNE_NO_DATA = 못 찾음(약 3 ms 이하이거나 요청마다 새로 읽음) */
    int16_t  p100;           /* 고른 P, 0.01 ms. 0 = 없음 */
    int16_t  r1000;          /* 그 P 의 R × 1000 */
    int16_t  pmax100;        /* 참고: R 이 가장 큰 후보 P(0.01 ms)와 그 R × 1000 */
    int16_t  rmax1000;
    uint16_t pairs;          /* 쓴 쌍 수 */
    uint16_t req, err;       /* 회전 중 요청 수, 오류 수(무응답·체크섬·길이·형식·수신 오류) */
    uint16_t nt;             /* t 에 든 시각 수 */
    uint16_t phase;          /* 1 라인 확인, 2 회전, 3 계산 */
    uint32_t span;           /* 회전 중 첫 요청 → 마지막 요청 hal_cycles. 평균 요청 주기 = span / (req - 1) */
    uint32_t t[GS_BUF];      /* 변화를 부른 요청의 송신 시각 hal_cycles(100 MHz) */
} gscan_result_t;

#if AGV_DEV_TESTS
extern gscan_result_t agv_gs;

void gresp_start(void);
void gresp_reset(void);      /* 부팅 때 GS 결과를 지운다(비보존 영역이라 codestart 가 안 지운다) */
int  gresp_busy(void);       /* 1 = 시험이 SCI-B 를 쓴다. guide_poll 이 쉰다 */
void gresp_poll(void);       /* 메인 루프(sys_scan)가 틱과 상관없이 부른다 */
void gscan_start(void);
int  gscan_run(void);        /* tune_run(10 ms). 1 = 아직 진행 */
int  gscan_fast(void);       /* 1 = 회전 중. guide_poll 은 요청을 보내지 않고 gscan_take 의 응답만 쓴다 */
int  gscan_take(guide_rx_t *rx);
/* GSP 계산(호스트 시험도 같은 함수를 쓴다): t[0..nt-1] 로 시작, gscan_calc_step 이 0 을 낼 때까지 */
void gscan_calc_begin(void);
int  gscan_calc_step(uint16_t budget);
#else
#define gresp_busy() 0
#define gscan_fast() 0
#define gscan_take(rx) ((void)(rx), 0)
#endif

#endif /* AGV_GRESP_H */
