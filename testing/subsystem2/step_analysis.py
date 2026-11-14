#!/usr/bin/env python3
"""Speed-step analysis for the PID tuning report (NFR5).

Input: one or more speed-log CSVs from the dashboard ("Download CSV" after
a bench speed step; APP_BENCH firmware).  Columns:
    t_ms, sp_l, v_l, duty_l, sp_r, v_r, duty_r     (mm/s, mm/s, per mille)

For each file and wheel it prints:
    target      final set-point (mm/s)
    ramp_end    time the set-point first reaches the target (ms); the
                firmware ramps set-points at MOTION_ACCEL_MM_S2 (800 mm/s^2)
    rise        10 % -> 90 % of the target (ms)
    overshoot   (peak - target) / target (%)
    settle      time after ramp_end until the speed stays within +-10 %
                of the target for the rest of the log (ms); NFR5: <= 500
    ss_err      mean (speed - target) / target over the last 500 ms (%);
                NFR5: within +-10 %
    duty_max    largest |duty| (per mille; 1000 = saturated)

With matplotlib installed it also writes <file>.png (speed and set-point
against time, both wheels).

    python3 step_analysis.py speed_log_*.csv
"""
import csv
import sys

BAND = 0.10         # NFR5: within +-10 % of the set-point
TAIL_MS = 500       # steady state: the last 500 ms of the log


def load(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: int(v) for k, v in r.items()})
            except (TypeError, ValueError):
                continue                # a sample lost in transit
    return rows


def wheel(rows, side):
    t = [r['t_ms'] for r in rows]
    sp = [r['sp_' + side] for r in rows]
    v = [r['v_' + side] for r in rows]
    duty = [r['duty_' + side] for r in rows]
    target = sp[-1]
    if target == 0:
        return None
    sign = 1 if target > 0 else -1
    tgt = abs(target)
    v = [sign * x for x in v]
    ramp_end = next((t[i] for i in range(len(sp)) if abs(sp[i]) >= tgt), t[-1])
    t10 = next((t[i] for i in range(len(v)) if v[i] >= 0.1 * tgt), None)
    t90 = next((t[i] for i in range(len(v)) if v[i] >= 0.9 * tgt), None)
    peak = max(v)
    last_out = None
    for i in range(len(v)):
        if t[i] >= ramp_end and abs(v[i] - tgt) > BAND * tgt:
            last_out = t[i]
    settle = 0 if last_out is None else (last_out + (t[1] - t[0]) - ramp_end)
    tail = [v[i] for i in range(len(v)) if t[i] >= t[-1] - TAIL_MS]
    ss = (sum(tail) / len(tail) - tgt) / tgt * 100.0 if tail else float('nan')
    return {'target': target, 'ramp_end': ramp_end,
            'rise': (t90 - t10) if (t10 is not None and t90 is not None) else None,
            'overshoot': (peak - tgt) / tgt * 100.0, 'settle': settle,
            'settled': last_out is None or last_out < t[-1] - TAIL_MS,
            'ss_err': ss, 'duty_max': max(abs(x) for x in duty)}


def fmt(x, unit=''):
    if x is None:
        return '—'
    return (f'{x:.1f}' if isinstance(x, float) else str(x)) + unit


def plot(path, rows):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    t = [r['t_ms'] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(t, [r['sp_l'] for r in rows], 'k--', lw=1, label='set-point')
    ax.plot(t, [r['v_l'] for r in rows], lw=1.4, label='left')
    ax.plot(t, [r['v_r'] for r in rows], lw=1.4, label='right')
    ax.set_xlabel('time from the start of the move (ms)')
    ax.set_ylabel('wheel speed (mm/s)')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out = path.rsplit('.', 1)[0] + '.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def main(paths):
    print('| file | wheel | target mm/s | ramp end ms | rise ms | overshoot % | settle ms (<= 500) | steady error % (+-10) | max duty |')
    print('|---|---|---|---|---|---|---|---|---|')
    for p in paths:
        rows = load(p)
        if len(rows) < 20:
            print(f'| {p} | — | too few samples ({len(rows)}) | | | | | | |')
            continue
        for side, name in (('l', 'left'), ('r', 'right')):
            m = wheel(rows, side)
            if m is None:
                continue
            settle = fmt(m['settle']) + ('' if m['settled'] else ' (not settled)')
            print(f"| {p} | {name} | {m['target']} | {m['ramp_end']} | {fmt(m['rise'])} | "
                  f"{fmt(m['overshoot'])} | {settle} | {fmt(m['ss_err'])} | {m['duty_max']} |")
        png = plot(p, rows)
        if png:
            print(f'<!-- plot: {png} -->')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
