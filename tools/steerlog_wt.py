#!/usr/bin/env python3
"""조향 기록기 덤프에서 t3.9 바퀴 지연 시험 구간을 찾아 바퀴 축 응답과 차체·센서 응답을 맞춘다.

사용: python tools/steerlog_wt.py 덤프.dat [--ww 600] [--gd 250] [--rpm-per-mpm 94.5] [--unit-mm 3.445] [--csv 출력.csv]
      python tools/steerlog_wt.py --selftest
덤프 읽기는 steerlog_decode.py 를 불러 쓴다(같은 폴더, 또는 환경변수 STEERLOG_DECODE_DIR, 또는 이웃 저장소 tools).
시험 구간 표식: 시험 중 steer.c 가 D.p_pi_out 을 ±WT_A_RPM(200) 으로 덮어써(steer.c steer_test_rpm) 기록 p 가 +A·−A·+A 로 남는다.
"""
import argparse
import csv
import math
import os
import random
import sys
import tempfile


def load_decoder():
    here = os.path.dirname(os.path.abspath(__file__))
    dirs = [here, os.environ.get('STEERLOG_DECODE_DIR', ''), os.path.join(here, '..', '..', 'AGV_DCU_App', 'tools')]
    for d in dirs:
        if d and os.path.isfile(os.path.join(d, 'steerlog_decode.py')):
            sys.path.insert(0, d)
            import steerlog_decode
            return steerlog_decode
    raise SystemExit('steerlog_decode.py 를 못 찾았다. STEERLOG_DECODE_DIR 에 그 폴더를 준다')


A_MIN, A_MAX = 150, 250          # 계단 크기 인정 범위(rpm)
SEG = (0, 30, 90, 120)           # 계단 시작 틱, 끝(wtest.c s_seg)
PRE = 5                          # 펌웨어 y0 평균 앞 표본(WT_PRE)
NOISE_TICKS = 50                 # 잡음 구간 0.5 s
FIT_BACK = 30                    # 마지막 계단 끝 뒤 0.3 s
BODY_BACK = 50                   # 0.5 s
FIT_FRONT = 10                   # 첫 계단 앞 0.1 s
L_GRID = range(0, 61, 2)
TAU_GRID = range(0, 401, 2)

# 열 번호(steerlog_decode.CSV_SMP)
C_POS, C_P, C_V, C_RL, C_RR, C_WL, C_WR = 1, 8, 11, 12, 13, 14, 15


def find_steps(p):
    """p 가 +A 30틱, −A 60틱, +A 30틱(각 ±2) 이어지는 첫 자리의 시작 인덱스. 없으면 ValueError."""
    runs = []                    # [부호, 시작, 길이]
    for k, v in enumerate(p):
        s = 1 if A_MIN <= v <= A_MAX else (-1 if -A_MAX <= v <= -A_MIN else 0)
        if runs and runs[-1][0] == s:
            runs[-1][2] += 1
        else:
            runs.append([s, k, 1])
    want = ((1, 30), (-1, 60), (1, 30))
    for i in range(len(runs) - 2):
        r = runs[i:i + 3]
        if all(r[j][0] == want[j][0] and abs(r[j][2] - want[j][1]) <= 2 for j in range(3)) \
                and r[1][1] == r[0][1] + r[0][2] and r[2][1] == r[1][1] + r[1][2]:
            return r[0][1]
    near = [(r[0], r[2]) for r in runs if r[0] and r[2] >= 5]
    raise ValueError('계단(+A 30, -A 60, +A 30틱)을 못 찾았다. 긴 +/-A 구간: %s' % (near[:8] or '없음'))


def mean(x):
    return sum(x) / len(x)


def std(x):
    m = mean(x)
    return math.sqrt(mean([(v - m) ** 2 for v in x]))


def plant(u, tau, L_list):
    """u(틱별 지령)를 1 ms 로 적분한 1차 지연 응답. 순수 지연 L ms 는 출력 이동. L 마다 10 ms 표본 목록을 돌려준다."""
    n = len(u)
    a = 1.0 if tau == 0 else 1.0 - math.exp(-1.0 / tau)
    y = [float(u[0])] * (10 * n + 1)
    c = y[0]
    for m in range(10 * n):
        c += a * (u[m // 10] - c)
        y[m + 1] = c
    return [[y[max(10 * k - L, 0)] for k in range(n)] for L in L_list]


def fit_axis(w, rpm, ks):
    """한 바퀴 w_cmd·rpm(틱별)에서 τ·L 격자 최소제곱. 반환: τ, L, 잔차 RMS, 맞춤 파형(틱별), 잡음 통계."""
    ka, kb = ks - NOISE_TICKS, ks + SEG[3] + FIT_BACK
    u = w[ka:kb + 1]
    pre = rpm[ka:ks]
    bias = mean([r - x for r, x in zip(pre, w[ka:ks])])      # 정속 때 지령 대비 평균 오차는 모형에 상수로 더한다
    i0 = ks - FIT_FRONT - ka
    yy = rpm[ks - FIT_FRONT:kb + 1]
    best = None
    Ls = list(L_GRID)
    for tau in TAU_GRID:
        for L, ym in zip(Ls, plant(u, tau, Ls)):
            e = sum((a - b - bias) ** 2 for a, b in zip(yy, ym[i0:]))
            if best is None or e < best[0]:
                best = (e, tau, L, ym)
    e, tau, L, ym = best
    noise = dict(sd=std(pre), span=max(pre) - min(pre), bias=bias)
    return tau, L, math.sqrt(e / len(yy)), [v + bias for v in ym], noise


def fw_calc(w, rpm, ks):
    """wtest_calc 와 같은 계산: 계단마다 t10·t63 을 '처음 넘는 때'로 τ·L. 반환: (τ 평균, L 평균, 쓴 계단 수), 넘침 %."""
    y = lambda j: rpm[ks + j]
    taus, dls, over = [], [], []
    for s in range(3):
        y0 = mean(rpm[ks - PRE:ks + 1]) if s == 0 else y(SEG[s])
        c = w[ks + SEG[s]]
        span = c - y0
        sg = 1.0 if span > 0 else -1.0
        n = SEG[s + 1] - SEG[s]
        over.append(max(0.0, max(sg * (y(SEG[s] + j) - c) for j in range(1, n + 1))) / abs(span) * 100 if span else 0.0)
        if abs(span) < 100:     # WT_A_RPM(200) 의 절반
            continue
        prev, g10, t10 = 0.0, False, 0.0
        for j in range(1, n + 1):
            r = (y(SEG[s] + j) - y0) / span
            if not g10 and r >= 0.1:
                t10 = ((j - 1) + (0.1 - prev) / (r - prev)) * 10.0
                g10 = True
            if g10 and r >= 0.632:
                t63 = ((j - 1) + (0.632 - prev) / (r - prev)) * 10.0
                tau = (t63 - t10) / 0.89464
                taus.append(tau)
                dls.append(t10 - 0.10536 * tau)
                break
            prev = r
    if not taus:
        return None, over
    return (mean(taus), mean(dls), len(taus)), over


def body_pred(rl, rr, ks, n, ww, gd, rpm_per_mpm):
    """바퀴 rpm 으로 적분한 센서 옆 위치(mm). 첫 계단 시작(틱 ks)에서 방향·옆 위치 0, 틱 ks 부터 n 개."""
    cv = 1000.0 / 60.0 / rpm_per_mpm         # rpm → mm/s
    psi = y = 0.0
    out = []
    for k in range(ks, ks + n):
        vl, vr = rl[k] * cv, rr[k] * cv
        out.append(y)
        wz = (vl - vr) / ww
        y += 0.01 * ((vl + vr) / 2 * math.sin(psi) + gd * wz)
        psi += 0.01 * wz
    return out


def fit_body(pos_mm, pred, ks):
    """실측 ≈ k × 예측(t − d) + c, d 0~400 ms(10 ms). 반환: 표 [(d, k, c, RMS)], 최소 항목."""
    n = len(pred)
    real = pos_mm[ks:ks + n]
    rows = []
    for ds in range(41):
        x = [pred[i - ds] if i >= ds else 0.0 for i in range(n)]
        mx, my = mean(x), mean(real)
        sxx = sum((a - mx) ** 2 for a in x)
        k = sum((a - mx) * (b - my) for a, b in zip(x, real)) / sxx if sxx > 0 else 0.0
        c = my - k * mx
        rows.append((ds * 10, k, c, math.sqrt(mean([(b - k * a - c) ** 2 for a, b in zip(x, real)]))))
    return rows, min(rows, key=lambda r: r[3])


def analyze(rows, ww=600.0, gd=250.0, rpm_per_mpm=94.5, unit_mm=3.445):
    p = [r[C_P] for r in rows]
    ks = find_steps(p)
    kend = ks + SEG[3] + BODY_BACK
    if ks < NOISE_TICKS or kend >= len(rows):
        raise ValueError('표본이 모자란다: 계단 앞 %d틱·뒤 %d틱이 필요(앞 %d, 뒤 %d)' % (
            NOISE_TICKS, SEG[3] + BODY_BACK, ks, len(rows) - ks - 1))
    w = [[r[C_WL] for r in rows], [r[C_WR] for r in rows]]
    rpm = [[r[C_RL] for r in rows], [r[C_RR] for r in rows]]
    res = dict(ks=ks, wheel=[])
    for i in range(2):
        tau, L, rms, fit, nz = fit_axis(w[i], rpm[i], ks)
        fw, over = fw_calc(w[i], rpm[i], ks)
        res['wheel'].append(dict(tau=tau, L=L, rms=rms, ratio=rms / nz['sd'] if nz['sd'] > 0 else float('inf'),
                                 noise=nz, fw=fw, over=over, fit=fit, ka=ks - NOISE_TICKS))
    pred = body_pred(rpm[0], rpm[1], ks, kend - ks + 1, ww, gd, rpm_per_mpm)
    pos_mm = [r[C_POS] * unit_mm for r in rows]
    table, best = fit_body(pos_mm, pred, ks)
    near = [r[0] for r in table if r[3] <= best[3] * 1.1]
    res['body'] = dict(d=best[0], dmin=min(near), dmax=max(near), k=best[1], rms=best[3], rms0=table[0][3], pred=pred)
    res['pos_mm'] = pos_mm
    res['rows'] = rows
    return res


def report(res):
    lines = []
    for name, wh in zip(('왼쪽', '오른쪽'), res['wheel']):
        nz = wh['noise']
        fw = ('τ %.1f ms, 지연 %.1f ms (%d계단)' % wh['fw']) if wh['fw'] else '값 없음'
        lines.append('%s 바퀴: 잡음 σ %.1f rpm, 폭 %d rpm, 평균오차 %+.1f rpm | 맞춤 τ %d ms, L %d ms, 잔차 %.1f rpm (잡음의 %.2f배)'
                     ' | 펌웨어식 %s | 넘침 %s %%' % (name, nz['sd'], nz['span'], nz['bias'], wh['tau'], wh['L'], wh['rms'],
                                                   wh['ratio'], fw, ' '.join('%.0f' % o for o in wh['over'])))
    b = res['body']
    lines.append('차체: 지연 d %d ms (범위 %d~%d ms), k %.2f, 잔차 %.2f mm (d 0 일 때 %.2f mm)' % (
        b['d'], b['dmin'], b['dmax'], b['k'], b['rms'], b['rms0']))
    return lines


def write_csv(path, res):
    ks, rows = res['ks'], res['rows']
    ka, kb = ks - NOISE_TICKS, ks + SEG[3] + BODY_BACK
    with open(path, 'w', encoding='utf-8', newline='') as f:
        cw = csv.writer(f)
        cw.writerow(['t_ms', 'p', 'w_cmd_l', 'w_cmd_r', 'rpm_l', 'rpm_r', 'fit_l', 'fit_r', 'guide_pos', 'pred_mm'])
        for k in range(ka, kb + 1):
            r = rows[k]
            fit = [('%.1f' % wh['fit'][k - ka]) if k - ka < len(wh['fit']) else '' for wh in res['wheel']]
            pr = '%.2f' % (res['body']['pred'][k - ks] if k >= ks else 0.0)
            cw.writerow([r[0], r[C_P], r[C_WL], r[C_WR], r[C_RL], r[C_RR], fit[0], fit[1], r[C_POS], pr])


# ---- 자체 시험 ----

def synth(sd, tau, L, d_body, seed, ww=600.0, gd=250.0, rpm_per_mpm=94.5, unit_mm=3.445, v0=2362, n=400, k0=100):
    """합성 표본(steerlog 표본 형식 dict). 바퀴는 1차 지연+순수 지연, 센서 위치는 차체 지연 d_body 뒤 1단위 양자화."""
    rnd = random.Random(seed)
    p = [0] * n
    for k in range(n):
        j = k - k0
        p[k] = 200 if 0 <= j < 30 or 90 <= j < 120 else (-200 if 30 <= j < 90 else 0)
    w = [(v0 + p[k], v0 - p[k]) for k in range(n)]
    a = 1.0 - math.exp(-1.0 / tau) if tau > 0 else 1.0
    cv = 1000.0 / 60.0 / rpm_per_mpm
    y = [[float(v0), float(v0)]]
    for m in range(10 * n):
        src = w[max(m - L, 0) // 10]
        y.append([y[-1][i] + a * (src[i] - y[-1][i]) for i in range(2)])
    psi = ypos = 0.0
    lat = []
    for m in range(10 * n + 1):
        lat.append(ypos)
        if m >= 10 * k0:
            vl, vr = y[m][0] * cv, y[m][1] * cv
            wz = (vl - vr) / ww
            ypos += 0.001 * ((vl + vr) / 2 * math.sin(psi) + gd * wz)
            psi += 0.001 * wz
    smp = []
    for k in range(n):
        pos = lat[max(10 * k - d_body, 0)] / unit_mm
        smp.append(dict(pos=int(round(pos)), step=-1, trk=0, on=0, off=0, curve=0, risk=0, p=p[k], i=0, okp=0, v=v0,
                        rl=int(round(y[10 * k][0] + rnd.gauss(0, sd))), rr=int(round(y[10 * k][1] + rnd.gauss(0, sd)))))
    return smp


def selftest():
    dec = load_decoder()
    cfg = dict(runs=1, straight_len=0, auto_drive_mode=0, auto_slow_mode=0, tag_pulse2=0, tag_pulse10=0, los=25, drs=31,
               acc=8, dcc=40, obs_use=0, speed_high=45, tst=0)
    cases = ((40, 60, 20, 100, 1, True), (0, 60, 20, 100, 2, False), (40, 10, 8, 100, 3, False))
    for sd, tau, L, db, seed, via_file in cases:
        smp = synth(sd, tau, L, db, seed)
        words = dec.encode(cfg, smp, [], len(smp), len(smp))
        if via_file:
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, 'a.dat')
                dec.write_dat(path, words)
                rows = dec.decode(dec.read_dat(path))[1]
        else:
            rows = dec.decode(words)[1]
        res = analyze(rows)
        print('합성 잡음 %d τ %d L %d 차체 %d ms (시작 틱 %d)' % (sd, tau, L, db, res['ks']))
        for line in report(res):
            print('  ' + line)
        assert res['ks'] == 100
        for wh in res['wheel']:
            assert abs(wh['tau'] - tau) <= 10 and abs(wh['L'] - L) <= 6, (wh['tau'], wh['L'])
        assert abs(res['body']['d'] - db) <= 30, res['body']['d']
        with tempfile.TemporaryDirectory() as d:
            write_csv(os.path.join(d, 'o.csv'), res)
    try:                                         # 계단 없는 자료는 까닭을 말하고 실패해야 한다
        find_steps([0] * 100)
        raise AssertionError('계단 없음을 못 잡았다')
    except ValueError:
        pass
    print('selftest ok')


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('dat', nargs='?')
    a.add_argument('--ww', type=float, default=600.0, help='바퀴 간격 mm')
    a.add_argument('--gd', type=float, default=250.0, help='구동축에서 센서까지 mm')
    a.add_argument('--rpm-per-mpm', type=float, default=94.5, help='모터축 rpm 당 m/min')
    a.add_argument('--unit-mm', type=float, default=3.445, help='guide_pos 1단위 mm')
    a.add_argument('--csv', default=None)
    a.add_argument('--selftest', action='store_true')
    g = a.parse_args()
    if g.selftest:
        selftest()
        return 0
    if not g.dat:
        a.error('덤프 파일이 없다')
    dec = load_decoder()
    try:
        res = analyze(dec.decode(dec.read_dat(g.dat))[1], g.ww, g.gd, g.rpm_per_mpm, g.unit_mm)
    except ValueError as e:
        print('실패: %s' % e)
        return 1
    print('\n'.join(report(res)))
    if g.csv:
        write_csv(g.csv, res)
    return 0


if __name__ == '__main__':
    sys.exit(main())
