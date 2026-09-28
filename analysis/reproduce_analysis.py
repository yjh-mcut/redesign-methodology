#!/usr/bin/env python3
"""Recompute every analytical quantity in

    "Humanoid Robot Redesign with Existing Actuators: Torque-Speed Constraints,
     Design Margins, and a Bench Test of the Screens"

and regenerate its two analysis figures.  Nothing here is measured; this is the
screening arithmetic, and the point of publishing it is that any assumption in
the PARAMETERS block can be changed and its consequence seen at once.

    python reproduce_analysis.py [--out DIR]

The bench measurements are separate: see ../bench/make_figures.py.
"""
import argparse
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.stats import chi2, binom

# =============================================================================
# PARAMETERS
#
# Each generation is evaluated at its own measured mass and centre-of-mass
# height, taken at the league's robot inspection.  Using one mass for all three
# hides the fact that the platform gained 1.1 kg while it was being improved.
# =============================================================================
g = 9.81

GENERATIONS = [            # label, year, mass [kg], CoM height [m], stance [m]
    ('Generation 1', 2017, 12.1, 0.610, 0.260),
    ('Generation 2', 2018, 12.8, 0.620, 0.179),
    ('Generation 3', 2019, 13.2, 0.625, 0.179),
]

Ts = 0.40                  # single-support period [s], from the gait controller

tau_stall = 10.0           # stall torque per actuator at 14.8 V, manufacturer
tau_usable = 5.4           # maximum characterised in the performance graph at
                           # 12 V, used unscaled at 14.8 V
omega_m = 5.76             # no-load speed at 14.8 V, manufacturer
eta = 0.95                 # assumed spur-mesh efficiency
beta = 1.0                 # assumed load sharing between the paired actuators
kappa_lambda = 1.0         # exact at the reference posture; see Section 3.1
SF = 2.0                   # capacity-ratio target

N_ROLL = 55 / 24

# Measured on the bench, for the comparison in Section 7.4 only.  These do not
# replace the figures above, which are what the screening analysis used.
OMEGA_M_BENCH = 5.9987     # estimated, bidirectional sweeps, drive train
TAU_S_BENCH = 11.169       # fitted to four load points, absorbs beta and eta

AXES = [                   # name, teeth in, teeth out, actuators, requirement
    ('Hip / ankle roll', 24, 55, 2, 'Quasi-static lateral moment'),
    ('Hip / ankle pitch', 24, 40, 2, 'Swing and kicking rate'),
    ('Hip yaw', 24, 36, 1, 'Turning rate'),
]

# Reconstructed from the competition record; see analysis/failure_log_TEMPLATE.csv
FAILURES = [               # label, year, matches, main-tournament matches, failures
    ('Generation 1', 2017, 5, 5, 4),
    ('Generation 2', 2018, 9, 4, 3),
    ('Generation 3', 2019, 12, 6, 2),
]


# =============================================================================
# Models
# =============================================================================
def tau_ref(mass, W):
    """Reference hip-roll moment, Equation (2)."""
    return 0.5 * kappa_lambda * mass * g * W


def tau_out(n_a, N, mesh=eta, tau_u=tau_usable):
    """Available output through an external pair, Equation (6)."""
    return n_a * beta * N * mesh * tau_u


def omega_out(N, w_m=omega_m):
    """No-load output speed, Equation (6).  Attained only at zero torque."""
    return w_m / N


def frontal(mass, z0, W):
    """Frontal-plane quantities from the linear inverted pendulum, Equation (5).

    Returns the reference moment, the lateral sway amplitude, the CoM velocity
    at support exchange, the kinetic-energy index, and the minimum lateral
    distance between the CoM and the stance foot.
    """
    w = math.sqrt(g / z0)
    half = W / 2.0
    v0 = half * w * math.tanh(0.5 * w * Ts)
    E = 0.5 * mass * v0 ** 2
    # The CoM starts a step at the midline moving toward the stance foot and is
    # pushed back out; it turns around where its velocity reaches zero.
    d_min = math.sqrt(max(half ** 2 - (v0 / w) ** 2, 0.0))
    return tau_ref(mass, W), half - d_min, v0, E, d_min


def coupled_speed(tau, N, w_m=omega_m, tau_s=tau_stall):
    """Loaded characteristic through the gear pair."""
    return w_m * (N - tau / (2 * beta * eta * tau_s)) / N ** 2


def admissible_ratios(tau_demand, w_d, w_m=omega_m, tau_s=tau_stall):
    """Interval of N the combined condition allows, or None if empty."""
    A = tau_demand / (2 * beta * eta * tau_s)
    disc = w_m ** 2 - 4 * w_d * w_m * A
    if disc < 0:
        return None
    root = math.sqrt(disc)
    return (w_m - root) / (2 * w_d), (w_m + root) / (2 * w_d)


def poisson_ci(count, exposure, conf=0.95):
    """Exact (Garwood) interval on an incidence rate."""
    a = (1 - conf) / 2
    lo = chi2.ppf(a, 2 * count) / 2 / exposure if count else 0.0
    hi = chi2.ppf(1 - a, 2 * count + 2) / 2 / exposure
    return lo, hi


def rate_ratio_intervals(c1, e1, c2, e2, conf=0.95):
    """Log-Wald and exact conditional-binomial intervals on a rate ratio.

    Two methods because neither is the same as the per-generation intervals,
    and with counts this small the choice is not cosmetic.
    """
    rr = (c1 / e1) / (c2 / e2)
    z = 1.959963985
    se = math.sqrt(1 / c1 + 1 / c2)
    wald = (rr * math.exp(-z * se), rr * math.exp(z * se))
    n = c1 + c2
    lo_p, hi_p = binom_ci(c1, n, conf)
    ratio = e2 / e1
    exact = (lo_p / (1 - lo_p) * ratio if lo_p < 1 else float('inf'),
             hi_p / (1 - hi_p) * ratio if hi_p < 1 else float('inf'))
    return rr, wald, exact


def binom_ci(k, n, conf=0.95):
    """Clopper-Pearson interval on a binomial proportion."""
    a = (1 - conf) / 2
    from scipy.stats import beta as beta_dist
    lo = beta_dist.ppf(a, k, n - k + 1) if k else 0.0
    hi = beta_dist.ppf(1 - a, k + 1, n - k) if k < n else 1.0
    return lo, hi


# =============================================================================
# Tables
# =============================================================================
def table_capacity():
    print('\n=== Hip-roll capacity ratio ===')
    print(f'  {"configuration":<40}{"W [cm]":>8}{"tau_ref":>10}{"tau_out":>10}{"ratio":>8}')
    geared = tau_out(2, N_ROLL)
    plain = tau_out(2, 1.0, mesh=1.0)
    rows = [
        ('Gen. 1: as deployed (12.1 kg)', 0.260, 12.1, plain),
        ('Gearing only, at 12.1 kg (assumed)', 0.260, 12.1, geared),
        ('Gearing only, at 12.8 kg (assumed)', 0.260, 12.8, geared),
        ('Stance narrowing only, at 12.1 kg (assumed)', 0.179, 12.1, plain),
        ('Gen. 2: as deployed (12.8 kg)', 0.179, 12.8, geared),
        ('Gen. 3: as deployed (13.2 kg)', 0.179, 13.2, geared),
    ]
    for label, W, mass, avail in rows:
        need = tau_ref(mass, W)
        print(f'  {label:<40}{W*100:8.1f}{need:10.2f}{avail:10.2f}{avail/need:8.2f}')

    ratio = N_ROLL * eta * (0.260 / 0.179) * (12.1 / 13.2)
    print(f'\n  C_new / C_old = N eta (W_old/W_new)(m_old/m_new) = {ratio:.2f} at eta = {eta}')
    for e in (0.80, 0.95, 1.00):
        print(f'      at eta = {e:.2f}: {N_ROLL * e * (0.260/0.179) * (12.1/13.2):.2f}')
    print(f'  the mass gained costs {100*(1 - 12.1/13.2):.1f}% of what geometry and '
          f'gearing would otherwise have given')

    # Widest stance an ungeared joint sustains, Equation (13)
    for target, label in ((1.0, 'unity'), (SF, f'the target of {SF:.0f}')):
        W_max = 2 * tau_out(2, 1.0, mesh=1.0) / (target * kappa_lambda * 12.1 * g)
        print(f'  widest ungeared stance at {label}: {W_max*100:.1f} cm')


def table_allocation():
    print('\n=== Per-axis allocation ===')
    print(f'  {"axis":<20}{"gear":>8}{"N":>7}{"n_a":>5}{"tau_out":>10}{"w_nl":>8}  requirement')
    for name, zin, zout, n_a, need in AXES:
        N = zout / zin
        print(f'  {name:<20}{f"{zin}:{zout}":>8}{N:7.2f}{n_a:5d}'
              f'{tau_out(n_a, N):10.1f}{omega_out(N):8.2f}  {need}')
    uniform = tau_out(2, 36 / 24) / tau_ref(13.2, 0.179)
    print(f'\n  24:55 at the pitch axes would drop their no-load speed to '
          f'{omega_out(55/24):.2f} rad/s')
    print(f'  24:36 at the roll axes would give a capacity ratio of {uniform:.2f}')


def table_frontal():
    print('\n=== Frontal-plane measures, each generation at its own parameters ===')
    first = frontal(*[(m, z, W) for _, _, m, z, W in GENERATIONS][0])
    third = frontal(*[(m, z, W) for _, _, m, z, W in GENERATIONS][2])
    names = ['Reference moment tau_ref [N.m]', 'Lateral CoM sway amplitude [cm]',
             'CoM velocity at support exchange [m/s]', 'Kinetic-energy index E [J]',
             'Minimum CoM-foot lateral margin [cm]']
    scale = [1, 100, 1, 1, 100]
    for name, a, b, s in zip(names, first, third, scale):
        print(f'  {name:<42}{a*s:9.2f}{b*s:9.2f}{100*(b/a - 1):9.1f}%')
    print('  E is a state quantity of the reduced-order model, not an energy per step')


def table_sensitivity():
    print('\n=== Sensitivity, third generation ===')
    C = tau_out(2, N_ROLL) / tau_ref(13.2, 0.179)
    print(f'  nominal capacity ratio {C:.4f}, which clears the target by '
          f'{100*(C/SF - 1):.1f}%')
    print(f'  eta        >= {eta * SF / C:.4f}')
    print(f'  tau_u      >= {tau_usable * SF / C:.2f} N.m')
    print(f'  beta       >= {SF / C:.4f}')
    print(f'  kappa*lam  <= {C / SF:.4f}')
    print(f'  W          <= {0.179 * C / SF * 100:.2f} cm')
    print('\n  these are one constraint written five ways: C is proportional to')
    print('  beta*eta*tau_u and inversely to kappa*lambda*W')
    print(f'\n  {"assumption":<44}{"tau_u":>7}{"eta":>7}{"beta":>7}{"k*lam":>7}{"C":>7}')
    cases = [
        ('Nominal (reference posture)', 5.4, 0.95, 1.00, 1.00),
        ('Reduced mesh efficiency', 5.4, 0.90, 1.00, 1.00),
        ('Pessimistic mesh efficiency', 5.4, 0.80, 1.00, 1.00),
        ('5% shortfall in combined actuator output', 5.4, 0.95, 0.95, 1.00),
        ('kappa*lambda 5% above the reference posture', 5.4, 0.95, 1.00, 1.05),
        ('kappa*lambda 10% above the reference posture', 5.4, 0.95, 1.00, 1.10),
        ('Reduced output and efficiency', 4.5, 0.90, 1.00, 1.00),
        ('All four adverse', 3.0, 0.80, 0.95, 1.10),
        ('Conditional: beta from the observed current split', 5.4, 0.95, 0.73, 1.00),
    ]
    for label, tu, e, b, kl in cases:
        c = (2 * b * N_ROLL * e * tu) / (0.5 * kl * 13.2 * g * 0.179)
        print(f'  {label:<44}{tu:7.1f}{e:7.2f}{b:7.2f}{kl:7.2f}{c:7.2f}')


def table_binding():
    print('\n=== Which condition binds, third generation ===')
    demand = SF * tau_ref(13.2, 0.179)
    power = 0.25 * 2 * beta * eta * tau_stall * omega_m
    print(f'  demand {demand:.3f} N.m,  available group power {power:.2f} W')
    print(f'  Equation (9) gives w_d <= {power/demand:.2f} rad/s')
    print(f'  no-load bound w_m/N = {omega_out(N_ROLL):.2f} rad/s')
    w_d = 0.374   # stance-sway estimate, Equation (11)
    print(f'  at the stance-sway estimate {w_d:.3f} rad/s: headroom {omega_out(N_ROLL)/w_d:.1f}x '
          f'against the no-load bound, {power/demand/w_d:.1f}x against the loaded one')
    lo = demand / tau_out(2, 1.0, mesh=eta)
    iv = admissible_ratios(demand, w_d)
    print(f'  torque condition needs N >= {lo:.2f}; the quadratic allows '
          f'{iv[0]:.2f} <= N <= {iv[1]:.2f}')
    for label, mass in (('Generation 3', 13.2), ('Generation 2', 12.8)):
        print(f'  minimum ratio meeting the target, {label}: '
              f'{SF * tau_ref(mass, 0.179) / tau_out(2, 1.0, mesh=eta):.3f}')


def table_divergence():
    print('\n=== Separate screens against the combined condition ===')
    print('  (at the bench-estimated w_m and the fitted tau_s)')
    demand = SF * tau_ref(13.2, 0.179)
    lo = demand / tau_out(2, 1.0, mesh=eta)
    print(f'  {"rate":>8}{"separate":>22}{"combined":>22}  outcome')
    for w_d in (0.374, 1.00, 1.30, 1.40):
        hi_sep = OMEGA_M_BENCH / w_d
        iv = admissible_ratios(demand, w_d, OMEGA_M_BENCH, TAU_S_BENCH)
        sep = f'{lo:.2f} <= N <= {hi_sep:.2f}'
        if iv is None or max(lo, iv[0]) > iv[1]:
            comb, note = 'empty', 'disagree'
        else:
            comb = f'{max(lo, iv[0]):.2f} <= N <= {iv[1]:.2f}'
            shrink = 1 - iv[1] / hi_sep
            note = ('agree' if shrink < 0.10 else
                    'agree, combined narrower' if shrink < 0.30 else 'diverge')
        print(f'  {w_d:8.3f}{sep:>22}{comb:>22}  {note}')
    N = 4.0
    print(f'\n  at N = {N}, which both separate screens pass, the calibrated')
    print(f'  characteristic gives {coupled_speed(demand, N, OMEGA_M_BENCH, TAU_S_BENCH):.2f} '
          f'rad/s against a 1.30 rad/s requirement')
    power = 0.25 * 2 * beta * eta * TAU_S_BENCH * OMEGA_M_BENCH
    print(f'  above {power/demand:.2f} rad/s the combined condition is empty at any ratio')


def table_failures():
    print('\n=== Reconstructed failure record ===')
    print(f'  {"generation":<16}{"year":>6}{"matches":>9}{"failures":>10}'
          f'{"incidence":>11}  95% CI')
    for label, year, matches, _, count in FAILURES:
        rate = count / matches
        lo, hi = poisson_ci(count, matches)
        print(f'  {label:<16}{year:6d}{matches:9d}{count:10d}{rate:11.2f}  '
              f'[{lo:.2f}, {hi:.2f}]')
    c1, e1 = FAILURES[0][4], FAILURES[0][2]
    c3, e3 = FAILURES[2][4], FAILURES[2][2]
    rr, wald, exact = rate_ratio_intervals(c1, e1, c3, e3)
    print(f'\n  rate ratio, first against third: {rr:.2f}')
    print(f'    log-Wald [{wald[0]:.2f}, {wald[1]:.2f}]   '
          f'exact [{exact[0]:.2f}, {exact[1]:.2f}]')

    print('\n  over main-tournament play alone, where all nine incidents fall:')
    for label, year, _, main, count in FAILURES:
        print(f'    {label:<16}{count}/{main} = {count/main:.2f} per match')
    c1m, e1m = FAILURES[0][4], FAILURES[0][3]
    c3m, e3m = FAILURES[2][4], FAILURES[2][3]
    rr_m, wald_m, exact_m = rate_ratio_intervals(c1m, e1m, c3m, e3m)
    print(f'    rate ratio {rr_m:.2f}   log-Wald [{wald_m[0]:.2f}, {wald_m[1]:.2f}]   '
          f'exact [{exact_m[0]:.2f}, {exact_m[1]:.2f}]')
    # The hip-yaw pattern was noticed first and examined afterwards, so this is
    # exploratory. Conditioning on the five incidents, how do they fall between
    # the exposures before and after the idler bearing was fitted?
    print('\n  hip-yaw incidents, five before the idler bearing and none after:')
    for label, before, after in (('all matches', 14, 12), ('main tournament only', 9, 6)):
        n, k = before + after, 5
        p0 = before / n
        probs = [binom.pmf(i, k, p0) for i in range(k + 1)]
        two = sum(x for x in probs if x <= probs[k] + 1e-12)
        print(f'    {label:<22} {before}/{after}   one-sided {p0**k:.3f}   two-sided {two:.3f}')
    print('    exploratory: the joint was examined because the pattern was noticed')

    print('\n  restricting exposure to main-tournament play halves the rate ratio.')
    print('  That is arithmetic, not an estimate of what the change contributed,')
    print('  and neither interval excludes unity.')


# =============================================================================
# Figures
# =============================================================================
def figure_frontal(out):
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))
    for (label, _, mass, z0, W), colour in zip(
            [GENERATIONS[0], GENERATIONS[2]], ['#c0392b', '#1f4e79']):
        w = math.sqrt(g / z0)
        half = W / 2.0
        v0 = half * w * math.tanh(0.5 * w * Ts)
        t = np.linspace(0, Ts, 200)
        # Start the step at the midline heading toward the stance foot.
        x = half * np.cosh(w * (t - Ts / 2)) - (v0 / w) * 0.0
        x = np.sqrt(np.maximum(half ** 2 - (v0 / w) ** 2, 0)) * np.cosh(w * (t - Ts / 2))
        left.plot(t, -x * 100, color=colour, lw=1.6,
                  label=f'{label}: W={W*100:.1f} cm, $z_0$={z0:.3f} m')
        amp = (half - math.sqrt(max(half ** 2 - (v0 / w) ** 2, 0))) * 100
        left.annotate('', xy=(Ts / 2, -x.min() * 100), xytext=(Ts / 2, -half * 100),
                      arrowprops=dict(arrowstyle='<->', color=colour, lw=1.1))
        left.annotate(f'{amp:.2f} cm', (Ts / 2 + 0.012, -(half * 100 + x.min() * 100) / 2),
                      color=colour, fontsize=8.5)
    left.axhline(0, color='k', ls='--', lw=1)
    left.annotate('stance foot', (0.005, 0.6), fontsize=8)
    left.set_xlabel('time within single support (s)')
    left.set_ylabel('lateral CoM position\nrelative to stance foot (cm)')
    left.grid(alpha=.25)
    left.legend(fontsize=7.6, loc='lower right', framealpha=.95)
    left.set_title('(a) frontal-plane trajectory, measured $z_0$ per generation',
                   fontsize=9.5)

    widths = np.linspace(0.10, 0.30, 200)
    for mass, colour, label in ((12.1, '#c0392b', 'Gen. 1'), (13.2, '#1f4e79', 'Gen. 3')):
        right.plot(widths * 100, [tau_ref(mass, W) for W in widths], color=colour,
                   lw=1.6, ls=':' if mass == 12.1 else '-',
                   label=rf'$\tau_{{\rm ref}}$ at m = {mass} kg ({label})')
    right.plot(26.0, tau_ref(12.1, 0.260), 'ks', ms=7)
    right.annotate(f'Gen. 1\n{tau_ref(12.1, 0.260):.2f} N$\\cdot$m', (26.0, tau_ref(12.1, 0.260)),
                   textcoords='offset points', xytext=(-6, 8), fontsize=8, ha='right')
    right.plot(17.9, tau_ref(13.2, 0.179), 'o', color='#e67e22', ms=8)
    right.annotate(f'Gen. 3\n{tau_ref(13.2, 0.179):.2f} N$\\cdot$m', (17.9, tau_ref(13.2, 0.179)),
                   textcoords='offset points', xytext=(8, -14), fontsize=8)
    right.annotate('width $-$31%,  mass $+$9%\n$\\Rightarrow$ moment $-$24.9%',
                   (12.5, 16.5), fontsize=8.5, color='#1f4e79')
    right.set_xlabel('stance width $W$ (cm)')
    right.set_ylabel(r'reference moment $\tau_{\rm ref}$ (N$\cdot$m)')
    right.grid(alpha=.25)
    right.legend(fontsize=7.8, loc='upper left', framealpha=.95)
    right.set_title('(b) the mass gain offsets part of the geometric gain', fontsize=9.5)
    fig.tight_layout()
    fig.savefig(f'{out}/fig_frontal.png', dpi=300)
    plt.close(fig)


def figure_designspace(out):
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))
    W = np.linspace(0.08, 0.30, 300)
    w_d = 0.374   # stance-sway estimate, Equation (11)
    torque_bound = [SF * tau_ref(13.2, w) / tau_out(2, 1.0, mesh=eta) for w in W]
    left.fill_between(W, torque_bound, 6, color='#d6e4f0', alpha=.6,
                      label='meets target ratio of 2')
    left.plot(W, torque_bound, color='k', lw=1.6, label='torque condition, Eq. (7)')
    upper = [admissible_ratios(SF * tau_ref(13.2, w), w_d)[1] for w in W]
    left.plot(W, upper, color='#c0392b', ls=':', lw=1.3,
              label=r'torque--speed upper root, $\omega_d$ = 0.374 rad/s')
    left.axvspan(0.08, 0.179, color='0.85', alpha=.5)
    left.axvline(0.179, color='0.35', ls='--', lw=1.1)
    left.annotate('$W_{\\min}$ lies somewhere in here\n(clearance limit not determined)',
                  (0.086, 0.55), fontsize=7.4, color='0.3')
    left.plot(0.179, N_ROLL, 'o', color='#e67e22', ms=8)
    left.annotate('Gen. 3', (0.179, N_ROLL), textcoords='offset points',
                  xytext=(8, -4), fontsize=8.5)
    left.plot(0.260, 1.0, 'ks', ms=7)
    left.annotate('Gen. 1 (as built)', (0.260, 1.0), textcoords='offset points',
                  xytext=(-6, -14), fontsize=8.5, ha='right')
    left.set_xlim(0.08, 0.30)
    left.set_ylim(0, 6)
    left.set_xlabel('stance width $W$ (m)')
    left.set_ylabel('external reduction ratio $N$')
    left.grid(alpha=.25)
    left.legend(fontsize=7.2, loc='upper right', framealpha=.95)
    left.set_title("(a) design space at the third generation's parameters", fontsize=9.5)

    C = tau_out(2, N_ROLL) / tau_ref(13.2, 0.179)
    adverse = np.linspace(0, 6, 200)
    right.plot(adverse, C * (1 - adverse / 100), color='#1f4e79', lw=1.7,
               label=r'$\beta$, $\eta$ or $\tau_u$ reduced by x%')
    right.plot(adverse, C / (1 + adverse / 100), color='#2e86c1', ls='--', lw=1.4,
               label=r'$\kappa\lambda$ or $W$ increased by x%')
    right.axhline(SF, color='#c0392b', ls='--', lw=1.3)
    right.annotate(f'target ratio = {SF:.0f}', (4.0, SF + 0.006), fontsize=8,
                   color='#c0392b')
    margin = 100 * (C / SF - 1)
    right.plot(margin, SF, '*', color='k', ms=13)
    right.annotate(f'{margin:.1f}% is the whole margin', (margin, SF),
                   textcoords='offset points', xytext=(10, 8), fontsize=8.5)
    right.annotate(f'nominal (Gen. 3) = {C:.3f}', (0.1, C), textcoords='offset points',
                   xytext=(4, -12), fontsize=8)
    right.set_xlabel('adverse change in one input (%)')
    right.set_ylabel('capacity ratio $C$')
    right.set_xlim(0, 6)
    right.grid(alpha=.25)
    right.legend(fontsize=7.8, loc='lower left', framealpha=.95)
    right.set_title('(b) one margin, five equivalent expressions', fontsize=9.5)
    fig.tight_layout()
    fig.savefig(f'{out}/fig_designspace.png', dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='outputs', help='directory for the figures')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    table_capacity()
    table_allocation()
    table_frontal()
    table_binding()
    table_sensitivity()
    table_divergence()
    table_failures()

    figure_frontal(args.out)
    figure_designspace(args.out)
    print(f'\nwrote fig_frontal.png and fig_designspace.png to {args.out}/')


if __name__ == '__main__':
    main()
