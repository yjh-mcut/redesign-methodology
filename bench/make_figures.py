"""Regenerate the three bench figures in the paper from the logs in data/.

    python make_figures.py [--out DIR]

Produces fig_maxspeed.png (Figure 9), fig_static.png (Figure 10) and
fig_window.png (Figure 8).  Nothing is hard-coded: the effective stall torque
is fitted here, from the same four load points the paper reports.
"""
import argparse
import csv
import glob
import math
import os
import statistics as st

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Drive train.  N is the external spur pair fitted at the deployed hip roll.
WM = 5.9987          # no-load speed of the drive train [rad/s], measured
N = 55 / 24
NA, ETA = 2, 0.95    # actuators per joint, assumed mesh efficiency
QWIN = 8.0           # evaluation window, |q| <= QWIN [deg]
DECOUPLED = WM / N   # the no-load bound, which carries no load term

# One representative run per load.  The empty-lever run doubles as the
# post-programme health check, which is why it is called HEALTH.
MAXSPEED_RUNS = [
    (1.1454, 'empty lever', 'data/exp2b_HEALTH-1_20260927_151621.csv', '#2e86c1'),
    (11.7275, 'SF 1.01', 'data/exp2b_C-A-1_20260921_203519.csv', '#27ae60'),
    (17.5904, 'SF 1.52', 'data/exp2b_C-B-1_20260923_184203.csv', '#e67e22'),
    (23.3934, 'SF 2.02', 'data/exp2b_C-C-1_20260925_160329.csv', '#c0392b'),
]

STATIC_SETS = [
    ('SF 1.01  (11.73 N$\\cdot$m)', 'data/exp1_A-A-*.csv', '#2e86c1'),
    ('SF 1.52  (17.59 N$\\cdot$m)', 'data/exp1_A-B-*.csv', '#e67e22'),
    ('SF 2.02  (23.39 N$\\cdot$m)', 'data/exp1_A-C-[234]_*.csv', '#c0392b'),
]


def read(path):
    with open(path, encoding='utf-8-sig') as fh:
        return list(csv.DictReader(fh))


def column(rows, key):
    """Numeric column, skipping the blank cells that abort rows leave behind."""
    out = []
    for row in rows:
        value = row.get(key, '')
        if value not in ('', None):
            try:
                out.append(float(value))
            except ValueError:
                pass
    return out


def slope(ts, qs):
    n = len(ts)
    mt, mq = sum(ts) / n, sum(qs) / n
    num = sum((t - mt) * (q - mq) for t, q in zip(ts, qs))
    den = sum((t - mt) ** 2 for t in ts)
    return num / den


def window_speeds(path, qwin=QWIN):
    """Joint speed per repetition, from the encoder slope inside the window."""
    by_rep = {}
    for row in read(path):
        try:
            rep = int(row['rep'])
            by_rep.setdefault(rep, []).append((float(row['t_s']), float(row['q_deg'])))
        except (ValueError, TypeError, KeyError):
            continue
    speeds = []
    for rep in sorted(by_rep):
        inside = [p for p in by_rep[rep] if abs(p[1]) <= qwin]
        if len(inside) < 15:
            continue
        speeds.append(abs(math.radians(slope([p[0] for p in inside],
                                             [p[1] for p in inside]))))
    return speeds


def coupled(tau, tau_s):
    return WM * (N - tau / (NA * 1.0 * ETA * tau_s)) / N ** 2


def fit_stall_torque(loads, measured):
    """One free parameter over all four loads.  Coarse grid is plenty here."""
    grid = np.arange(8.0, 20.0, 0.0005)
    err = [sum((coupled(t, ts) - w) ** 2 for t, w in zip(loads, measured)) for ts in grid]
    return float(grid[int(np.argmin(err))])


def figure_maxspeed(out):
    points = [(tau, window_speeds(path), label) for tau, label, path, _ in MAXSPEED_RUNS]
    loads = np.array([p[0] for p in points])
    measured = np.array([st.mean(p[1]) for p in points])
    spread = np.array([st.pstdev(p[1]) for p in points])
    tau_s = fit_stall_torque(loads, measured)

    print(f'fitted tau_s = {tau_s:.3f} N.m per actuator '
          f'(group output n_a*beta*eta*tau_s = {NA * ETA * tau_s:.2f} N.m)')
    for tau, w, sd in zip(loads, measured, spread):
        print(f'  {tau:8.4f} N.m   measured {w:.4f} +-{sd:.4f}   '
              f'coupled {coupled(tau, tau_s):.4f} ({100 * (w / coupled(tau, tau_s) - 1):+.1f}%)   '
              f'below the no-load bound by {100 * (1 - w / DECOUPLED):.1f}%')

    fig, ax = plt.subplots(figsize=(7.0, 4.3))
    sweep = np.linspace(0, 26, 400)
    ax.axhline(DECOUPLED, color='#c0392b', ls='--', lw=1.6,
               label=r'no-load bound $\omega_m/N$ = %.2f rad/s (upper bound, load-independent)'
                     % DECOUPLED)
    ax.plot(sweep, [coupled(t, tau_s) for t in sweep], color='#1f4e79', lw=1.8,
            label=r'loaded characteristic, fitted $\tau_s$ = %.2f N$\cdot$m per actuator' % tau_s)
    ax.fill_between(sweep, [coupled(t, tau_s) for t in sweep], DECOUPLED,
                    color='#d6e4f0', alpha=.55, zorder=0,
                    label='gap between the bound and the attainable rate')
    ax.errorbar(loads, measured, yerr=np.maximum(spread, 1e-3) * 3, fmt='o', ms=7,
                color='k', capsize=4, lw=1.2, zorder=5,
                label=r'measured (mean of 3, bars $\pm3\sigma$)')
    for tau, w, (_, _, label) in zip(loads, measured, points):
        ax.annotate(label, (tau, w), textcoords='offset points', xytext=(6, -14), fontsize=8.5)
    for tau, w in zip(loads[1:], measured[1:]):
        ax.annotate('', xy=(tau, w), xytext=(tau, DECOUPLED),
                    arrowprops=dict(arrowstyle='<->', color='#7f8c8d', lw=.9))
        ax.annotate(f'{100 * (1 - w / DECOUPLED):.0f}% below', (tau, (w + DECOUPLED) / 2),
                    fontsize=8, color='#7f8c8d', ha='right', xytext=(-4, 0),
                    textcoords='offset points')
    ax.set_xlabel(r'joint load $\tau$  (N$\cdot$m)')
    ax.set_ylabel('maximum joint speed  (rad/s)')
    ax.set_xlim(0, 26)
    ax.set_ylim(0, 3.0)
    ax.grid(alpha=.25)
    ax.legend(fontsize=8, loc='lower left', framealpha=.95)
    fig.tight_layout()
    fig.savefig(f'{out}/fig_maxspeed.png', dpi=300)
    plt.close(fig)


def figure_static(out):
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.4, 3.8))
    for label, pattern, colour in STATIC_SETS:
        runs = [f for f in sorted(glob.glob(pattern)) if len(read(f)) > 1000]
        for k, path in enumerate(runs):
            rows = read(path)
            t = column(rows, 't_s')
            err = [abs(v) for v in column(rows, 'q_err_deg')]
            hot = [max(a, b) for a, b in zip(column(rows, 'temp1_C'), column(rows, 'temp2_C'))]
            n = min(len(t), len(err))
            left.plot(t[:n], err[:n], color=colour, lw=1.1, alpha=.85,
                      label=label if k == 0 else None)
            n = min(len(t), len(hot))
            right.plot(t[:n], hot[:n], color=colour, lw=1.1, alpha=.85)

    left.axhline(1.0, color='k', ls=':', lw=1.3)
    left.annotate('prespecified tolerance  1.0$^\\circ$', (330, 1.03), fontsize=8)
    left.annotate('quantised at the encoder:\n1 tick = 0.038$^\\circ$ at the joint',
                  (250, 0.76), fontsize=7.5, color='#c0392b')
    left.set_xlabel('time (s)')
    left.set_ylabel('joint angle error  (deg)')
    left.set_xlim(0, 620)
    left.set_ylim(0, 1.32)
    left.grid(alpha=.25)
    left.legend(fontsize=8, loc='lower right', framealpha=.95)
    left.set_title('(a) droop under sustained hold', fontsize=9.5)

    right.axhline(70, color='k', ls=':', lw=1.3)
    right.annotate('abort criterion  70 $^\\circ$C', (8, 71), fontsize=8)
    right.set_xlabel('time (s)')
    right.set_ylabel('hotter actuator, reported temperature ($^\\circ$C)')
    right.set_xlim(0, 620)
    right.set_ylim(20, 78)
    right.grid(alpha=.25)
    right.set_title('(b) thermal state; SF 2.02 terminates before 600 s', fontsize=9.5)

    fig.tight_layout()
    fig.savefig(f'{out}/fig_static.png', dpi=300)
    plt.close(fig)


def figure_window(out):
    """Shows that the evaluated segment sits on the plateau of each traverse."""
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))
    for load, label, path, colour in MAXSPEED_RUNS:
        first = []
        for row in read(path):
            try:
                if int(row['rep']) != 1:
                    continue
                first.append((float(row['t_s']), float(row['q_deg'])))
            except (ValueError, TypeError, KeyError):
                continue
        if not first:
            continue
        t = np.array([p[0] for p in first])
        q = np.array([p[1] for p in first])
        inside = np.abs(q) <= QWIN
        # Align the four loads on the centre of their own windows.
        centre = t[inside].mean() if inside.any() else 0.0
        ms = (t - centre) * 1000
        left.plot(ms, q, color=colour, lw=1.0, alpha=.45)
        left.plot(np.where(inside, ms, np.nan), np.where(inside, q, np.nan),
                  color=colour, lw=2.4, label=f'{label}  ({load:.2f} N$\\cdot$m)')
        speed = np.convolve(np.gradient(np.radians(q), t), np.ones(9) / 9, mode='same')
        right.plot(ms, np.abs(speed), color=colour, lw=1.0, alpha=.45)
        right.plot(np.where(inside, ms, np.nan), np.where(inside, np.abs(speed), np.nan),
                   color=colour, lw=2.4)

    for ax in (left, right):
        ax.set_xlim(-320, 320)
        ax.grid(alpha=.25)
        ax.set_xlabel('time relative to window centre (ms)')
    left.axhline(QWIN, color='k', ls=':', lw=1)
    left.axhline(-QWIN, color='k', ls=':', lw=1)
    left.annotate('$|q|\\leq 8^{\\circ}$ evaluation window', (-300, 10.5), fontsize=8)
    left.set_ylabel('joint angle $q$ (deg)')
    left.set_ylim(-26, 26)
    left.legend(fontsize=7.6, loc='lower right', framealpha=.95)
    left.set_title('(a) trajectory; bold segment is $|q|\\leq 8^{\\circ}$', fontsize=9.5)
    right.set_ylabel('joint speed $|\\dot q|$ (rad/s)')
    right.set_ylim(0, 3.0)
    right.set_title('(b) speed; bold segment is the evaluated window', fontsize=9.5)

    fig.tight_layout()
    fig.savefig(f'{out}/fig_window.png', dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='outputs', help='directory for the PNGs')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    figure_maxspeed(args.out)
    figure_static(args.out)
    figure_window(args.out)
    print(f'\nwrote fig_maxspeed.png, fig_static.png and fig_window.png to {args.out}/')


if __name__ == '__main__':
    main()
