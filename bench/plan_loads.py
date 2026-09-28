"""Work out payload counts and test speeds for a given load.

    python plan_loads.py --m-empty 0.40 --n 33
    python plan_loads.py --m-empty 0.40 --target 11.59     # how many masses
    python plan_loads.py --m-empty 0.40 --n 33 --volt 12.0

What it computes:

    joint torque    tau = (M_empty + n * WEIGHT_MASS) * g * R_LEVER
    loaded bound    w_c = w_m (N - tau/(n_a beta eta tau_s)) / N^2
    no-load bound   w_s = w_m / N         -- carries no load term

The loaded bound falls as the load rises and the no-load bound does not, so the
two screens only give different answers for w_c < w < w_s.  Choosing a test
speed inside that interval is the whole point of planning the loads.

Stall torque and no-load speed both depend strongly on supply voltage, so
nothing can be predicted until the voltage is fixed; the figures come from
config.MX106_SPEC, selected by SUPPLY_V_NOMINAL.
"""
import argparse, math
import config as C

PI = math.pi


def spec(volt):
    if volt is None:
        raise SystemExit('set config.SUPPLY_V_NOMINAL or pass --volt (e.g. 14.8)')
    v = min(C.MX106_SPEC, key=lambda k: abs(k - volt))
    if abs(v - volt) > 0.6:
        print(f'  [note] {volt} V is not in the spec table ({sorted(C.MX106_SPEC)}); '
              f'using the {v} V figures. Measure the real no-load speed to check')
    ts, rpm = C.MX106_SPEC[v]
    return ts, rpm * 2 * PI / 60, v


def torque(m_empty, n):
    return (m_empty + n * C.WEIGHT_MASS) * C.G * C.R_LEVER


def bounds(tau, tau_s, w_m):
    a = tau / (C.NA_MOTORS * C.BETA * C.ETA_GEAR * tau_s)
    return w_m * (C.N_GEAR - a) / C.N_GEAR ** 2, w_m / C.N_GEAR


def travel(w, t_acc, t_const):
    return w * (t_acc + t_const) * 180 / PI, C.R_LEVER * (w / t_acc) / C.G


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--m-empty', type=float, default=C.M_LEVER,
                    help='equivalent mass of the bare lever [kg], referred to the box centre')
    ap.add_argument('--n', type=int, help='number of calibrated masses in the box')
    ap.add_argument('--target', type=float, help='target torque [N m]; prints how many masses that needs')
    ap.add_argument('--volt', type=float, default=C.SUPPLY_V_NOMINAL)
    ap.add_argument('--t-acc', type=float, default=C.T_ACC)
    ap.add_argument('--t-const', type=float, default=C.T_CONST)
    a = ap.parse_args()

    if a.m_empty is None:
        raise SystemExit('pass --m-empty or fill in config.M_LEVER')
    ts, wm, v = spec(a.volt)
    per = C.WEIGHT_MASS * C.G * C.R_LEVER
    print(f'\nat {v} V: stall {ts} N m, no-load {wm*60/2/PI:.0f} rpm ({wm:.3f} rad/s)')
    print(f'one mass = {per:.4f} N m,  bare lever = {a.m_empty*C.G*C.R_LEVER:.3f} N m')
    print(f'available output n_a*beta*N*eta*tau_u = '
          f'{C.NA_MOTORS*C.BETA*C.N_GEAR*C.ETA_GEAR*C.TAU_U:.2f} N m '
          f'(tau_u = {C.TAU_U} N m -- check which voltage that figure is for)')

    if a.target:
        need = (a.target - a.m_empty * C.G * C.R_LEVER) / per
        print(f'\ntarget {a.target} N m -> {need:.2f} masses -> '
              f'{math.ceil(need)} gives {torque(a.m_empty, math.ceil(need)):.2f} N m '
              f'({(torque(a.m_empty, math.ceil(need))/a.target-1)*100:+.1f} %)')
        return

    if not a.n:
        raise SystemExit('need either --n or --target')

    tau = torque(a.m_empty, a.n)
    wc, ws = bounds(tau, ts, wm)
    print(f'\n{a.n} masses -> joint torque {tau:.3f} N m  '
          f'({tau/C.TAU_REF:.2f} x tau_ref = {C.TAU_REF})')
    print(f'  loaded bound {wc:.3f} rad/s   no-load bound {ws:.3f} rad/s   '
          f'interval width {ws-wc:.3f}')
    if wc <= 0:
        print('  [warning] the loaded characteristic predicts stall at this load')
        return
    if ws - wc < 0.15:
        print('  [warning] the interval is too narrow to be useful; add load')

    print(f'\n  {"point":>10} {"prediction":>26} {"travel":>8} {"r*a/g":>7} {"MovSpd":>7}')
    cands = [('low', C.SPEED_LOW), ('midpoint', (wc + ws) / 2),
             ('bound +14%', wc * 1.14), ('bound +25%', wc * 1.25)]
    for lbl, w in cands:
        if w >= ws:
            verdict = 'both screens reject -- no use'
        elif w > wc:
            verdict = 'no-load admits, loaded rejects *'
        else:
            verdict = 'both admit'
        deg, inert = travel(w, a.t_acc, a.t_const)
        reg = min(1023, round(w * C.N_GEAR * 60 / (2 * PI) / 0.114))
        print(f'  {lbl:>10} {w:5.3f} {verdict:>26} {deg:7.1f}d {inert*100:6.0f}% {reg:7d}')
    print('\n  * check the travel fits the range of motion with exp2_dynamic.py --dry')
    print('  * r*alpha/g applies to the acceleration phase only; the verdict comes')
    print('    from the constant-speed segment')
    print('  * testing the starred point at three different loads tests the loaded')
    print('    model itself, not just one operating point')


if __name__ == '__main__':
    main()
