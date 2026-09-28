"""Estimate the no-load speed w_m at the supply voltage in use.

    python measure_noload.py --dry         # plan only, nothing moves
    python measure_noload.py               # measure, bare lever
  python3 measure_noload.py --amp 30 --sweeps 3

Why
  Every bound scales with w_m: the no-load bound is w_m/N and the loaded one
  is w_m (N - tau/(n_a beta eta tau_s)) / N^2.  If the real figure differs
  from the specification (55 rpm at 14.8 V) then 1.844, 1.510 and 1.175 all
  move with it.

How
  Sweep at maximum commandable speed within the range of motion and take the
  encoder slope over the plateau, with the ramps excluded.  The bare lever
  weighs about 1.5 N m at the joint, which opposes one direction and assists
  the other, so the two directions are averaged to cancel it.

  What is left is the motor characteristic together with the external mesh
  and bearing friction.  That is the w_m of this drive train, not of the
  motor on its own, and it cannot be separated here.

Cautions
  - take the payload out; with it in, this is not a no-load measurement
  - motion stays inside the EEPROM angle limits, so run set_limits.py first
  - the figure comes from the encoder; Present Speed is printed for reference
"""
import argparse, math, sys, time
import config as C
import dxl_common as D

PI = math.pi


def fit_speed(ts, qs):
    """Least-squares slope of (t, q[deg]), in deg/s."""
    n = len(ts)
    mt = sum(ts) / n
    mq = sum(qs) / n
    num = sum((t - mt) * (q - mq) for t, q in zip(ts, qs))
    den = sum((t - mt) ** 2 for t in ts)
    return num / den if den else 0.0


def plateau(ts, qs, frac=0.60):
    """Slope over the plateau only.  Returns (speed, samples, duration[s])."""
    n = len(ts)
    if n < 20:
        return None
    h = max(2, n // 50)
    v = [0.0] * n
    for k in range(h, n - h):
        v[k] = (qs[k + h] - qs[k - h]) / (ts[k + h] - ts[k - h])
    core = sorted(abs(x) for x in v[h:n - h])
    peak = core[int(len(core) * 0.95)]
    if peak <= 0:
        return None
    # longest run with |v| >= 0.9 * peak
    best = cur = None
    for k in range(h, n - h):
        if abs(v[k]) >= 0.9 * peak:
            cur = (cur or k)
            if best is None or k - cur > best[1] - best[0]:
                best = (cur, k)
        else:
            cur = None
    if best is None or best[1] - best[0] < 10:
        return None
    a, b = best
    m = (a + b) / 2.0
    w = (b - a) * frac / 2.0
    a2, b2 = int(m - w), int(m + w) + 1
    if b2 - a2 < 8:
        a2, b2 = a, b + 1
    return (fit_speed(ts[a2:b2], qs[a2:b2]), b2 - a2, ts[b2 - 1] - ts[a2])


ADDR_CURRENT = 68          # MX-106 only, 2B. 4.5 mA/LSB, 2048 = 0 A


def sweep(bus, q_to, speed_reg, settle, read_current=False):
    """Move to q_to at maximum speed, logging [(t, q_deg, present_rpm, amps)]."""
    bus.sync_goal(D.joint_goals(q_to, bus.ids), speed_reg=speed_reg)
    rec = []
    t0 = time.time()
    while time.time() - t0 < settle:
        s = bus.sample()
        qs = [D.servo_to_joint_deg(i, s[i]['pos']) for i in bus.ids if s[i]]
        sp = [abs(s[i]['speed_rpm']) for i in bus.ids if s[i]]
        amps = []
        if read_current:
            for i in bus.ids:
                c = bus.r2(i, ADDR_CURRENT)
                if c is not None:
                    amps.append(abs((c - 2048) * 0.0045))
        if qs:
            rec.append((s['t_read'], sum(qs) / len(qs),
                        sum(sp) / len(sp) if sp else 0.0,
                        sum(amps) / len(amps) if amps else 0.0))
    return rec


def calibrate(bus, a):
    """Back out rpm per register count from several commanded speeds.

    Present Speed disagreeing with the encoder by 9.3%, and the joint running
    faster than commanded (exp2, 2026-09-20), are the same problem: the
    conversion constant is wrong.  Gravity makes the two directions differ,
    so both are measured and averaged.
    """
    print('\n>>> Check the payload is out and the lever is bare.')
    input('    Enter to start the calibration > ')
    bus.set_common(torque_limit=a.torque_limit, accel_reg=a.accel_reg,
                   return_delay=C.RETURN_DELAY)
    s = bus.sample()
    bus.sync_goal({i: s[i]['pos'] for i in bus.ids}, speed_reg=50)
    bus.torque(True)
    bus.sync_goal(D.joint_goals(-a.amp, bus.ids), speed_reg=200)
    time.sleep(2.0)

    print(f'\n  {"reg":>5} {"commanded":>12} {"meas +":>9} {"meas -":>9} '
          f'{"mean":>9} {"rpm/LSB":>9}')
    ks = []
    for reg in (100, 200, 300, 400):
        got = {}
        for sgn, goal in ((+1, a.amp), (-1, -a.amp)):
            rec = sweep(bus, goal, reg, a.settle)
            r = plateau([x[0] for x in rec], [x[1] for x in rec])
            got[sgn] = abs(math.radians(r[0])) if r else None
            time.sleep(0.3)
        if None in got.values():
            print(f'  {reg:5d}   no plateau found'); continue
        w = (got[+1] + got[-1]) / 2
        cmd = reg * D.RPM_PER_SPEED * 2 * PI / 60 / C.N_GEAR
        k = w * C.N_GEAR * 60 / (2 * PI) / reg        # rpm per LSB
        ks.append(k)
        print(f'  {reg:5d} {cmd:13.3f} {got[+1]:9.3f} {got[-1]:9.3f} '
              f'{w:9.3f} {k:9.4f}')
    if ks:
        k = sum(ks) / len(ks)
        print(f'\n  RPM_PER_SPEED  currently {D.RPM_PER_SPEED}  ->  measured {k:.4f} '
              f'({(k/D.RPM_PER_SPEED-1)*100:+.1f} %)')
        print(f'  spread {min(ks):.4f} to {max(ks):.4f}')
        print('  set dxl_common.RPM_PER_SPEED to this and commanded speed will')
        print('  match what the joint actually does')
    bus.sync_goal(D.joint_goals(0.0, bus.ids), speed_reg=200)
    time.sleep(1.5)
    bus.torque(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--amp', type=float, default=30.0, help='sweep amplitude about horizontal [deg]')
    ap.add_argument('--sweeps', type=int, default=3, help='number of sweeps')
    ap.add_argument('--accel-reg', type=int, default=150, help='Goal Acceleration(73)')
    ap.add_argument('--torque-limit', type=int, default=1023)
    ap.add_argument('--current', action='store_true',
                    help='also read current (68) to estimate friction torque; '
                         'this roughly halves the sample rate')
    ap.add_argument('--settle', type=float, default=1.2, help='logging time per direction [s]')
    ap.add_argument('--cal-speed', action='store_true',
                    help='calibrate the Moving Speed conversion: measure the speed '
                         'achieved at several register values and back out '
                         'RPM_PER_SPEED. Bare lever')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()

    D.require_config('JOINT_MIN_DEG', 'JOINT_MAX_DEG')

    ts_spec, rpm_spec = C.MX106_SPEC[min(C.MX106_SPEC,
                                         key=lambda k: abs(k - C.SUPPLY_V_NOMINAL))]
    wm_spec = rpm_spec * 2 * PI / 60
    aj = a.accel_reg * D.DEG_S2_PER_ACC / C.N_GEAR          # joint acceleration [deg/s^2]
    w_exp = wm_spec / C.N_GEAR                              # expected joint speed [rad/s]
    d_acc = math.degrees(w_exp) ** 2 / (2 * aj)             # angle used up by the ramp

    print(f'\nprespecified supply {C.SUPPLY_V_NOMINAL} V -> spec {rpm_spec} rpm '
          f'({wm_spec:.3f} rad/s), expected joint speed {w_exp:.3f} rad/s')
    print(f'amplitude +-{a.amp} deg ({2*a.amp} total), joint acceleration '
          f'{aj:.0f} deg/s^2, ramps use {d_acc:.1f} deg each')
    const_deg = 2 * a.amp - 2 * d_acc
    print(f'expected plateau {const_deg:.1f} deg = '
          f'{const_deg/math.degrees(w_exp):.3f} s -> unthrottled logging at about '
          f'1000 Hz gives {const_deg/math.degrees(w_exp)*1000:.0f} samples')
    if const_deg < 5:
        sys.exit('  the plateau is too short; raise --amp or --accel-reg')
    if a.amp > min(abs(C.JOINT_MIN_DEG), abs(C.JOINT_MAX_DEG)) - 2:
        sys.exit(f'  --amp is too large for the range '
                 f'({C.JOINT_MIN_DEG} to {C.JOINT_MAX_DEG} deg)')
    if a.dry:
        print('\n--dry, stopping here.')
        return

    bus = D.Bus()
    try:
        s = bus.sample()
        if any(s[i] is None for i in bus.ids):
            sys.exit('  could not read the servos')
        if a.cal_speed:
            return calibrate(bus, a)
        v = [s[i]['volt'] for i in bus.ids]
        print(f'\nstart voltage {v} V   temperature {[s[i]["temp"] for i in bus.ids]} C')
        if abs(min(v) - C.SUPPLY_V_NOMINAL) > C.SUPPLY_V_TOL:
            sys.exit(f'  voltage is outside the prespecified '
                     f'{C.SUPPLY_V_NOMINAL} +-{C.SUPPLY_V_TOL} V')

        print('\n>>> Check the payload is out and the lever is bare.')
        print('>>> It sweeps fast. Clear hands and objects.')
        input('    Enter to start > ')

        bus.set_common(torque_limit=a.torque_limit, accel_reg=a.accel_reg,
                       return_delay=C.RETURN_DELAY)
        bus.sync_goal({i: s[i]['pos'] for i in bus.ids}, speed_reg=50)
        bus.torque(True)
        bus.sync_goal(D.joint_goals(-a.amp, bus.ids), speed_reg=200)
        time.sleep(2.0)

        log = D.Logger(f'{C.OUT_DIR}/noload_{D.ts()}.csv',
                       ['sweep', 'dir', 't', 'q_deg', 'present_rpm', 'amp_mean'])
        res = {+1: [], -1: []}
        cur = {+1: [], -1: []}
        for k in range(a.sweeps):
            for sgn, goal in ((+1, a.amp), (-1, -a.amp)):
                rec = sweep(bus, goal, 1023, a.settle, a.current)
                for t, q, rpm, am in rec:
                    log.write(dict(sweep=k, dir=sgn, t=f'{t:.6f}',
                                   q_deg=f'{q:.4f}', present_rpm=f'{rpm:.3f}',
                                   amp_mean=f'{am:.4f}'))
                r = plateau([x[0] for x in rec], [x[1] for x in rec])
                if r is None:
                    print(f'  {k+1}/{a.sweeps} {"+" if sgn>0 else "-"}: no plateau found')
                    continue
                dps, n, dur = r
                w = abs(math.radians(dps))
                res[sgn].append(w)
                if a.current:
                    mid = [x[3] for x in rec[len(rec)//4: 3*len(rec)//4] if x[3] > 0]
                    if mid:
                        cur[sgn].append(sum(mid) / len(mid))
                print(f'  {k+1}/{a.sweeps} {"+" if sgn>0 else "-"}: '
                      f'{w:.4f} rad/s  ({abs(dps):6.1f} deg/s, n={n}, {dur*1000:.0f} ms, '
                      f'{len(rec)/a.settle:.0f} Hz)')
        log.close()

        print('\nreturning to q = 0')
        bus.sync_goal(D.joint_goals(0.0, bus.ids), speed_reg=200)
        time.sleep(1.5)
        bus.torque(False)

        if not res[+1] or not res[-1]:
            sys.exit('\nboth directions are needed and one is missing')
        up = sum(res[+1]) / len(res[+1])
        dn = sum(res[-1]) / len(res[-1])
        w_joint = (up + dn) / 2
        wm = w_joint * C.N_GEAR
        print(f'\n=== result ===')
        print(f'  + direction {up:.4f} rad/s   - direction {dn:.4f} rad/s   '
              f'difference {abs(up-dn)/w_joint*100:.1f} % (gravity and friction)')
        print(f'  mean joint speed {w_joint:.4f} rad/s')
        print(f'  implied w_m      {wm:.4f} rad/s = {wm*60/2/PI:.1f} rpm')
        print(f'  specification    {wm_spec:.4f} rad/s = {rpm_spec} rpm '
              f'({(wm/wm_spec-1)*100:+.1f} %)')
        if cur[+1] and cur[-1]:
            iu = sum(cur[+1]) / len(cur[+1]); idn = sum(cur[-1]) / len(cur[-1])
            i_avg = (iu + idn) / 2
            tau_m = i_avg * C.TORQUE_PER_AMP
            tau_j = tau_m * C.NA_MOTORS * C.N_GEAR * C.ETA_GEAR
            print(f'\n  --- friction, from current ---')
            print(f'  mean current per motor {i_avg:.3f} A  '
                  f'(+ {iu:.3f} / - {idn:.3f})')
            print(f'  motor torque {tau_m:.3f} N m -> {tau_j:.2f} N m at the joint')
            print(f'  torque implied by the speed shortfall '
                  f'{(1-wm/wm_spec)*C.STALL_TORQUE_NM:.3f} N m per motor')
            print('  if those two agree, friction explains the shortfall -- the mesh,')
            print('  the bearings, or the pair fighting each other')
            print('  if the current is much smaller, the motors are simply weaker')
            print('  than their specification')

        print(f'\n=== bounds recomputed with the measured w_m ===')
        print(f'  {"load":>10} {"loaded spec":>12} {"loaded meas":>12} '
              f'{"no-load spec":>13} {"no-load meas":>13}')
        for tau in (C.TAU_REF, C.TAU_150, C.TAU_SF2):
            al = tau / (C.NA_MOTORS * C.BETA * C.ETA_GEAR * ts_spec)
            f = lambda w: w * (C.N_GEAR - al) / C.N_GEAR ** 2
            print(f'  {tau:8.2f}   {f(wm_spec):11.3f} {f(wm):11.3f} '
                  f'{wm_spec/C.N_GEAR:11.3f} {wm/C.N_GEAR:11.3f}')
        print('\n  * stall torque is not measured here; the specification value is used')
        print('  * if the difference exceeds 5 %, re-derive the test speeds from the')
        print('    measured figure rather than config.MX106_SPEC (plan_loads.py)')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
