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

#if AGV_DEV_TESTS
void gresp_start(void);
int  gresp_busy(void);       /* 1 = 시험 중. guide_poll 이 SCI-B 를 쉰다 */
void gresp_poll(void);       /* 메인 루프(sys_scan)가 틱과 상관없이 부른다 */
#else
#define gresp_busy() 0
#endif

#endif /* AGV_GRESP_H */
