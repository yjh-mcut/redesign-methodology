#!/usr/bin/env python3
"""
Reproduces every analytical quantity and both analysis figures reported in

    "Upgrading Humanoid Robot Mechatronics Without Replacing Actuators:
     A Field-Failure-Driven Redesign Methodology"

Running this script regenerates:
    Table 6  - per-axis transmission allocation
    Table 8  - hip-roll torque budget across Cycle 1
    Table 10 - frontal-plane measures before and after the stance change
    Table 11 - failure incidence with exact Poisson confidence intervals
    Figure 9  - fig_sensitivity.png
    Figure 10 - fig12_sim.png

All model parameters are collected in the PARAMETERS block below and correspond
to Table 7 of the manuscript. No measurement is performed here; the script
recomputes the predictions that Stage S4 required before each redeployment.

Usage:  python reproduce_analysis.py
Requires: numpy, scipy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import chi2

# =============================================================================
# PARAMETERS  (Table 7 of the manuscript)
# =============================================================================
g          = 9.81       # m/s^2
m          = 13.2       # kg,   platform mass including battery
z0         = 1.011      # m,    CoM height, from CAD
Ts         = 0.40       # s,    single-support period, gait controller setting
W_gen1     = 0.260      # m,    inter-leg distance, generation 1
W_gen2     = 0.179      # m,    inter-leg distance, generations 2-3
tau_stall  = 10.0       # N.m,  MX-106R stall torque at 14.8 V (manufacturer)
tau_usable = 5.4        # N.m,  maximum characterised in the performance graph
                        #       at 12 V; used unscaled, which is conservative
omega_m    = 5.76       # rad/s, no-load speed at 14.8 V (manufacturer)
eta        = 0.95       # -,    assumed spur-mesh efficiency
ETA_LO, ETA_HI = 0.80, 0.98   # sensitivity band

AXES = [  # name,            teeth in:out,  n_actuators, governing requirement
    ("Hip / ankle roll",     24, 55, 2, "Quasi-static lateral moment"),
    ("Hip / ankle pitch",    24, 40, 2, "Swing and kicking"),
    ("Hip yaw",              24, 36, 1, "Turning rate"),
]

# Archival failure record (Table 11). Replace with the per-match log if released.
FAILURES = [  # generation, year, matches, failures
    ("Generation 1", 2017,  5, 4),
    ("Generation 2", 2018,  9, 3),
    ("Generation 3", 2019, 12, 2),
]

OUT = "outputs"   # directory for the generated figures


# =============================================================================
# MODELS  (Equations 1 and 2 of the manuscript)
# =============================================================================
def tau_out(n_a, N, eff=eta, tau=tau_usable):
    """Equation (1): torque available at a joint output."""
    return n_a * N * eff * tau


def omega_out(N):
    """Equation (1): angular velocity available at a joint output."""
    return omega_m / N


def tau_req(W):
    """Equation (2): quasi-static hip-roll moment with the CoM at the midline."""
    return 0.5 * m * g * W


def lipm(W):
    """Equation (2) and its consequences, for stance width W.

    Returns peak hip-roll torque, lateral sway amplitude, CoM velocity at
    support exchange, redirected energy per step, and the minimum lateral
    CoM-to-foot margin.
    """
    w = np.sqrt(g / z0)
    half = W / 2.0
    v0 = half * w * np.tanh(0.5 * w * Ts)
    E = 0.5 * m * v0 ** 2
    margin = half / np.cosh(0.5 * w * Ts)
    sway = half - margin
    return tau_req(W), sway, v0, E, margin


def poisson_ci(k, exposure, alpha=0.05):
    """Exact (Garwood) Poisson confidence interval on a rate."""
    lo = chi2.ppf(alpha / 2, 2 * k) / 2 if k > 0 else 0.0
    hi = chi2.ppf(1 - alpha / 2, 2 * (k + 1)) / 2
    return lo / exposure, hi / exposure


# =============================================================================
# TABLE 6 - per-axis transmission allocation
# =============================================================================
def table6():
    print("\nTABLE 6  Per-axis transmission allocation")
    print(f"{'Axis':22}{'Gear':>8}{'N':>7}{'n_a':>5}"
          f"{'tau_out':>10}{'w_out':>9}   Governing requirement")
    for name, zin, zout, n_a, req in AXES:
        N = zout / zin
        print(f"{name:22}{f'{zin}:{zout}':>8}{N:7.2f}{n_a:5d}"
              f"{tau_out(n_a, N):10.1f}{omega_out(N):9.2f}   {req}")
    print("  tau_out in N.m, w_out in rad/s, computed with the usable torque "
          f"of {tau_usable} N.m per actuator.")


# =============================================================================
# TABLE 8 - hip-roll torque budget across Cycle 1
# =============================================================================
def table8():
    N_roll = 55 / 24
    configs = [
        ("Gen. 1: as deployed",       W_gen1, 2, 1.0,    1.0),
        ("Transmission change only",  W_gen1, 2, N_roll, eta),
        ("Geometry change only",      W_gen2, 2, 1.0,    1.0),
        ("Gen. 2: as deployed",       W_gen2, 2, N_roll, eta),
    ]
    print("\nTABLE 8  Hip-roll torque budget across Cycle 1")
    print(f"{'Configuration':28}{'W (cm)':>9}{'tau_req':>10}"
          f"{'tau_out':>10}{'SF':>8}")
    for label, W, n_a, N, eff in configs:
        avail, need = tau_out(n_a, N, eff), tau_req(W)
        print(f"{label:28}{W*100:9.1f}{need:10.2f}{avail:10.2f}{avail/need:8.2f}")

    # widest stance an ungeared joint can hold: tau_out = tau_req with N = 1
    W_crit = 2 * 2 * tau_usable / (m * g)
    print(f"\n  Widest stance an ungeared joint can hold (SF = 1): "
          f"{W_crit*100:.1f} cm")
    print(f"  Swing clearance permits no less than {W_gen2*100:.1f} cm, so no "
          "feasible geometry closes the deficit without the gear stage.")


# =============================================================================
# TABLE 10 - frontal-plane measures
# =============================================================================
def table10():
    a, b = lipm(W_gen1), lipm(W_gen2)
    names = ["Peak hip-roll torque (N.m)",
             "Lateral CoM sway amplitude (cm)",
             "CoM velocity at support exchange (m/s)",
             "Step-to-step redirection energy (J/step)",
             "Minimum CoM-foot lateral margin (cm)"]
    scale = [1, 100, 1, 1, 100]
    print("\nTABLE 10  Frontal-plane measures before and after the stance change")
    print(f"{'Measure':42}{'Gen. 1':>10}{'Gen. 2-3':>11}{'Change':>9}")
    for nm, x, y, s in zip(names, a, b, scale):
        print(f"{nm:42}{x*s:10.3f}{y*s:11.3f}{(y-x)/x*100:8.1f}%")
    print(f"  omega = sqrt(g/z0) = {np.sqrt(g/z0):.4f} rad/s")


# =============================================================================
# TABLE 11 - failure incidence with exact Poisson intervals
# =============================================================================
def table11():
    print("\nTABLE 11  Failure incidence with exact Poisson 95% intervals")
    print(f"{'Generation':16}{'Year':>6}{'Matches':>9}{'Failures':>10}"
          f"{'Incidence':>11}{'95% CI':>20}")
    for gen, yr, n, k in FAILURES:
        lo, hi = poisson_ci(k, n)
        print(f"{gen:16}{yr:6d}{n:9d}{k:10d}{k/n:11.2f}"
              f"     [{lo:.2f}, {hi:.2f}]")
    (_, _, n1, k1), (_, _, n3, k3) = FAILURES[0], FAILURES[-1]
    rr = (k1 / n1) / (k3 / n3)
    se = np.sqrt(1 / k1 + 1 / k3)
    print(f"\n  Rate ratio (Gen. 1 / Gen. 3) = {rr:.2f}, "
          f"95% CI [{rr*np.exp(-1.96*se):.2f}, {rr*np.exp(1.96*se):.2f}]")
    print("  The interval includes unity, so the trend is descriptive rather "
          "than a statistically established difference.")


# =============================================================================
# FIGURE 9 - sensitivity of the hip-roll budget
# =============================================================================
def figure9():
    N_roll = 55 / 24
    frac = tau_usable / tau_stall
    k = np.linspace(0.30, 1.0, 400)
    sf = lambda W, n_a, N, eff: (n_a * N * eff * tau_stall * k) / tau_req(W)

    fig, ax = plt.subplots(figsize=(7.4, 4.1))
    ax.plot(k * 100, sf(W_gen1, 2, 1.0, 1.0), color="#B23A3A", lw=2.0,
            label="Gen. 1: no gearing, $W$ = 26.0 cm")
    ax.fill_between(k * 100, sf(W_gen2, 2, N_roll, ETA_LO),
                    sf(W_gen2, 2, N_roll, ETA_HI),
                    color="#2C5F8A", alpha=0.18,
                    label=r"Gen. 2--3: $\eta$ = 0.80--0.98")
    ax.plot(k * 100, sf(W_gen2, 2, N_roll, eta), color="#2C5F8A", lw=2.0,
            label=r"Gen. 2--3: 24:55 gearing, $W$ = 17.9 cm ($\eta$ = 0.95)")

    ax.axvline(frac * 100, color="#1a7a3a", lw=1.6, ls="-.")
    ax.text(frac * 100 + 1.5, 0.30,
            f"performance-graph\nmaximum ({frac*100:.0f}%)",
            fontsize=8.5, color="#1a7a3a")
    ax.axhline(2.0, color="#444", ls="--", lw=1.0)
    ax.axhline(1.0, color="#444", ls=":", lw=1.0)
    ax.text(31, 2.07, "safety factor 2", fontsize=8, color="#444")
    ax.text(31, 1.07, "safety factor 1", fontsize=8, color="#444")
    ax.set_xlabel("usable torque as a percentage of rated stall torque (%)")
    ax.set_ylabel("hip-roll safety factor")
    ax.set_xlim(30, 100); ax.set_ylim(0, 4.3)
    ax.grid(alpha=0.25); ax.legend(fontsize=8, loc="upper left")
    plt.tight_layout()
    plt.savefig(f"{OUT}/fig_sensitivity.png", dpi=200)
    plt.close()
    print("\nwrote fig_sensitivity.png")


# =============================================================================
# FIGURE 10 - frontal-plane analysis
# =============================================================================
def figure10():
    w = np.sqrt(g / z0)

    def traj(W, n_steps=4):
        half, ts, ys, feet, t0 = W / 2, [], [], [], 0.0
        v0 = half * w * np.tanh(0.5 * w * Ts)
        for i in range(n_steps):
            foot = half if i % 2 == 0 else -half
            eta0, etad0 = -foot, (v0 if foot > 0 else -v0)
            t = np.linspace(0, Ts, 200)
            e = eta0 * np.cosh(w * t) + (etad0 / w) * np.sinh(w * t)
            ts.append(t + t0); ys.append(e + foot); feet.append((t0, foot))
            t0 += Ts
        return np.concatenate(ts), np.concatenate(ys), feet

    fig, ax = plt.subplots(1, 2, figsize=(9.5, 3.4))
    for W, c, lb in [(W_gen1, "#B23A3A", "Generation 1 ($W$ = 26.0 cm)"),
                     (W_gen2, "#2C5F8A", "Generation 2--3 ($W$ = 17.9 cm)")]:
        t, y, feet = traj(W)
        ax[0].plot(t, y * 100, color=c, lw=1.8, label=lb)
        for t0, f in feet:
            ax[0].hlines(f * 100, t0, t0 + Ts, color=c, lw=0.9, ls=":", alpha=0.6)
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("lateral position (cm)")
    ax[0].set_title("(a) Lateral CoM trajectory (dotted: stance foot)", fontsize=10)
    ax[0].legend(fontsize=8, loc="upper right"); ax[0].grid(alpha=0.25)

    Ws = np.linspace(0.12, 0.32, 200)
    ax[1].plot(Ws * 100, [lipm(W)[0] for W in Ws], color="#444", lw=1.8,
               label="peak hip-roll torque (N$\\cdot$m)")
    ax2 = ax[1].twinx()
    ax2.plot(Ws * 100, [lipm(W)[3] for W in Ws], color="#7A5CA8", lw=1.8,
             ls="--", label="redirection energy (J/step)")
    for W, c in [(W_gen1, "#B23A3A"), (W_gen2, "#2C5F8A")]:
        ax[1].axvline(W * 100, color=c, lw=1.1, ls=":")
    ax[1].set_xlabel("inter-leg distance $W$ (cm)")
    ax[1].set_ylabel("torque (N$\\cdot$m)")
    ax2.set_ylabel("energy (J/step)")
    ax[1].set_title("(b) Torque and energy vs. stance width", fontsize=10)
    lines = ax[1].get_lines()[:1] + ax2.get_lines()[:1]
    ax[1].legend(lines, [l.get_label() for l in lines], fontsize=8, loc="upper left")
    ax[1].grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(f"{OUT}/fig12_sim.png", dpi=200)
    plt.close()
    print("wrote fig12_sim.png")


if __name__ == "__main__":
    table6()
    table8()
    table10()
    table11()
    figure9()
    figure10()
    print("\nDone.")
