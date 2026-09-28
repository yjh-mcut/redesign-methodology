"""exp2b -- maximum joint speed under load.

  python3 exp2b_maxspeed.py --box A --dry
  python3 exp2b_maxspeed.py --box A --run C-A-1
  python3 exp2b_maxspeed.py --box C --run C-C-1 --amp 30

Why this and not exp2
  exp2 commands a speed and asks whether the joint holds it to +-3%.  With
  only 68 degrees of travel the constant-speed segment lasts under 0.2 s and
  deceleration starts before the servo has settled; values 8 through 128 at
  address 28 were all tried on 2026-09-21 and none met the criterion.  That
  measures how well the servo tracks, not which description of the drive
  train is right.

  What the two descriptions actually disagree about is the maximum speed
  available under load:

    no-load bound   w_max = w_m / N        -- no load term, always 2.618
    loaded          w_max = w_m (N - tau/(n_a beta eta tau_s)) / N^2

  Over the three loads the predictions differ by factors of 1.37, 1.68 and
  2.16, so +-3% precision is beside the point.  Moving Speed is commanded at
  its maximum, which saturates the controller and removes the tracking
  question entirely.  Only the loaded description predicts that the maximum
  falls as the load rises, so measuring all three loads tests it.

How
  One swing through horizontal, in the direction that lifts the payload, with
  the encoder slope taken only over |q| <= --qwin.  There cos q is close to 1
  and its slope is zero, so the load matches the target and barely varies.

Cautions
  - the lifting direction is not optional.  Driving downhill lets gravity do
    the work, which measures nothing, and pushes the bus to 24.1 V (seen on
    2026-09-20)
  - a heavy payload moves quickly here; keep the padding and the clearance
  - the verdict comes from the encoder.  Present Speed is for reference
"""
import argparse, math, sys, time
import config as C
import dxl_common as D

PI = math.pi


def predict(tau):
    """(loaded bound, no-load bound) in rad/s, from the measured w_m."""
    a = tau / (C.NA_MOTORS * C.BETA * C.ETA_GEAR * C.STALL_TORQUE_NM)
    return C.WM_MEAS * (C.N_GEAR - a) / C.N_GEAR ** 2, C.WM_MEAS / C.N_GEAR


def fit(ts, qs):
    """Least-squares slope, deg/s."""
    n = len(ts)
    mt, mq = sum(ts) / n, sum(qs) / n
    den = sum((t - mt) ** 2 for t in ts)
    return (sum((t - mt) * (q - mq) for t, q in zip(ts, qs)) / den) if den else 0.0


def evaluate(rec, qwin):
    """Speed over the window near q = 0.  rec = [(t, q, rpm, v1, v2)]."""
    w = [r for r in rec if abs(r[1]) <= qwin]
    if len(w) < 15:
        return None
    ts = [r[0] for r in w]; qs = [r[1] for r in w]
    dps = fit(ts, qs)
    # If the speed is still changing across the window it is not a maximum.
    mid = len(w) // 2
    f = abs(fit(ts[:mid], qs[:mid])); b = abs(fit(ts[mid:], qs[mid:]))
    return dict(n=len(w), w=abs(math.radians(dps)),
                drift=abs(f - b) / ((f + b) / 2) * 100 if f + b else 0.0,
                dur=ts[-1] - ts[0],
                rpm=sum(r[2] for r in w) / len(w),
                vmin=min(min(r[3], r[4]) for r in w),
                vmax=max(max(r[3], r[4]) for r in w))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--box', choices=('A', 'B', 'C'),
                    help='take the load and payload mass from config.BOXES')
    ap.add_argument('--load', type=float, help='joint load [N m], instead of --box')
    ap.add_argument('--payload', type=float, help='total payload mass [kg]')
    ap.add_argument('--run', default='X')
    ap.add_argument('--dir', type=int, choices=(1, -1), default=-1,
                    help='-1 lifts the payload on this rig (checked 2026-09-20)')
    ap.add_argument('--amp', type=float, default=30.0, help='swing amplitude about horizontal [deg]')
    ap.add_argument('--qwin', type=float, default=8.0,
                    help='evaluation window |q| [deg]; 8 deg varies the load by 1 %%')
    ap.add_argument('--accel-reg', type=int, default=254,
                    help='register 73. Top speed has to be reached before the '
                         'window, or this measures the ramp and not the limit')
    ap.add_argument('--settle-tol', type=float, default=0.05,
                    help='settle band [deg]; 0.3 s inside it counts as stopped')
    ap.add_argument('--repeats', type=int, default=C.REPEATS)
    ap.add_argument('--torque-limit', type=int, default=C.TORQUE_LIMIT)
    ap.add_argument('--note', default='')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()

    D.require_config('M_LEVER', 'R_LEVER_CM', 'JOINT_MIN_DEG', 'JOINT_MAX_DEG')
    if a.box:
        a.load = a.load or C.LOAD_ACTUAL[a.box]
        a.payload = a.payload or C.PAYLOAD_KG[a.box]
        if abs(C.M_LEVER - C.BOXES[a.box]['M_empty']) > 1e-6:
            sys.exit(f'  config.M_LEVER ({C.M_LEVER}) does not match box {a.box} '
                     f'({C.BOXES[a.box]["M_empty"]}); fix that first')
    if a.load is None or a.payload is None:
        sys.exit('  need --box, or both --load and --payload')

    wc, ws = predict(a.load)
    print(f'\nload {a.load:.4f} N m (payload {a.payload} kg)   dir {a.dir:+d} '
          f'({"lifting" if a.dir < 0 else "lowering -- wrong direction!"})')
    print(f'swing {-a.dir*a.amp:+.1f} -> {a.dir*a.amp:+.1f} deg   '
          f'window |q| <= {a.qwin} deg (load varies '
          f'{(1-math.cos(math.radians(a.qwin)))*100:.1f} %)')
    print(f'\n  predictions')
    print(f'    no-load bound   {ws:.3f} rad/s   -- carries no load term')
    print(f'    loaded          {wc:.3f} rad/s   -- at this load')
    print(f'    difference      {ws-wc:.3f} rad/s ({ws/wc:.2f} x)')
    aj = a.accel_reg * D.DEG_S2_PER_ACC / C.N_GEAR
    d_acc = math.degrees(ws) ** 2 / (2 * aj)
    t_acc = math.degrees(ws) / aj
    print(f'\n  at {aj:.0f} deg/s^2 the no-load speed needs {d_acc:.1f} deg / '
          f'{t_acc:.3f} s (from {a.amp:.0f} deg, reached at q={a.amp-d_acc:+.1f})')
    print(f'  spare before the window {a.amp - d_acc - a.qwin:+.1f} deg '
          f'(negative means still accelerating inside it)')
    if a.amp - d_acc < a.qwin:
        print(f'  [note] acceleration is not finished before the window; '
              f'raise --amp or --accel-reg')
    if a.amp > min(abs(C.JOINT_MIN_DEG), abs(C.JOINT_MAX_DEG)) - 2:
        sys.exit(f'  --amp is too large for the range '
                 f'({C.JOINT_MIN_DEG} to {C.JOINT_MAX_DEG} deg)')
    if a.dir > 0:
        print('\n  [warning] dir=+1 lowers the payload. Gravity does the work, the '
              'measurement means nothing, and the bus will overvolt.')
    if a.dry:
        print('\n--dry, stopping here.')
        return

    bus = D.Bus()
    log = D.Logger(f'{C.OUT_DIR}/exp2b_{a.run}_{D.ts()}.csv',
                   ['run_id', 'rep', 't_s', 'q_deg', 'tau_grav_Nm',
                    'pos1', 'spd1_rpm', 'load1_pct', 'volt1', 'temp1_C',
                    'pos2', 'spd2_rpm', 'load2_pct', 'volt2', 'temp2_C',
                    'dq12_deg', 'comm_fail', 'note'])
    try:
        s = bus.sample()
        if any(s[i] is None for i in bus.ids):
            sys.exit('  could not read the servos')
        v0 = [s[i]['volt'] for i in bus.ids]
        print(f'\nstart temperature {[s[i]["temp"] for i in bus.ids]} C   voltage {v0} V')
        if max(s[i]['temp'] for i in bus.ids) > C.TEMP_START_C:
            sys.exit(f'  let them cool below {C.TEMP_START_C} C first')
        if abs(min(v0) - C.SUPPLY_V_NOMINAL) > C.SUPPLY_V_TOL:
            sys.exit(f'  voltage is outside {C.SUPPLY_V_NOMINAL} +-{C.SUPPLY_V_TOL} V')

        print('\n>>> Check the load is applied and there is padding under the payload.')
        print('>>> It moves fast and it is heavy. Clear hands and objects.')
        input('    Enter to start > ')

        bus.set_common(torque_limit=a.torque_limit, accel_reg=10,
                       return_delay=C.RETURN_DELAY)
        bus.sync_goal({i: s[i]['pos'] for i in bus.ids}, speed_reg=50)
        bus.torque(True)

        res = []
        print(f'\n  {"rep":>4}{"speed":>9}{"n":>6}{"window":>8}{"change":>9}'
              f'{"PresSpd":>9}{"voltage":>13}')
        for r in range(a.repeats):
            # Wait for the joint to actually stop, not for a fixed delay. A 3 s wait
            # was not enough for a 60 degree return, and reps 2 and 3 on
            # 2026-09-21 started measuring while it was still moving.
            bus.set_common(accel_reg=10)
            bus.sync_goal(D.joint_goals(-a.dir * a.amp, bus.ids), speed_reg=60)
            t_w = time.time(); still = None; q_prev = None
            while time.time() - t_w < 12.0:
                sm = bus.sample()
                if sm[bus.ids[0]] is None:
                    continue
                qn = D.servo_to_joint_deg(bus.ids[0], sm[bus.ids[0]]['pos'])
                if q_prev is not None and abs(qn - q_prev) <= a.settle_tol:
                    still = still or time.time()
                    if time.time() - still >= 0.3:
                        break
                else:
                    still = None
                q_prev = qn
                time.sleep(0.02)
            else:
                print('  [warning] the return did not settle within 12 s')
            time.sleep(0.3)
            bus.set_common(accel_reg=a.accel_reg)

            rec = []
            bus.sync_goal(D.joint_goals(a.dir * a.amp, bus.ids), speed_reg=1023)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                smp = bus.sample()
                p1 = smp[bus.ids[0]]
                p2 = smp[bus.ids[1]] if len(bus.ids) > 1 else None
                if p1 is None:
                    continue
                q = D.servo_to_joint_deg(bus.ids[0], p1['pos'])
                q2 = D.servo_to_joint_deg(bus.ids[1], p2['pos']) if p2 else None
                t = smp['t_read'] - t0
                rec.append((t, q, abs(p1['speed_rpm']),
                            p1['volt'], p2['volt'] if p2 else p1['volt']))
                log.write(dict(
                    run_id=a.run, rep=r + 1, t_s=round(t, 5), q_deg=round(q, 4),
                    tau_grav_Nm=round(D.gravity_torque(q, m_p=a.payload), 4),
                    pos1=p1['pos'], spd1_rpm=round(p1['speed_rpm'], 2),
                    load1_pct=p1['load_pct'], volt1=p1['volt'], temp1_C=p1['temp'],
                    pos2=(p2['pos'] if p2 else ''),
                    spd2_rpm=(round(p2['speed_rpm'], 2) if p2 else ''),
                    load2_pct=(p2['load_pct'] if p2 else ''),
                    volt2=(p2['volt'] if p2 else ''),
                    temp2_C=(p2['temp'] if p2 else ''),
                    dq12_deg=(round(q - q2, 3) if p2 else ''),
                    comm_fail=bus.comm_fail, note=a.note))

            m = evaluate(rec, a.qwin)
            if m is None:
                print(f'  {r+1:4d}   too few samples in the window -- did the swing cross q=0?')
                continue
            res.append(m)
            print(f'  {r+1:4d}{m["w"]:9.4f}{m["n"]:6d}{m["dur"]*1000:7.0f}ms'
                  f'{m["drift"]:8.1f}%{m["rpm"]/C.N_GEAR*2*PI/60:9.4f}'
                  f'{m["vmin"]:7.1f}~{m["vmax"]:.1f}V')
            if m['vmax'] > C.SUPPLY_V_MAX_ABORT:
                print(f'       [stop] regenerative overvoltage {m["vmax"]:.1f} V -- check --dir')
                break

        print('\n  returning to q = 0')
        bus.set_common(accel_reg=10)
        bus.sync_goal(D.joint_goals(0.0, bus.ids), speed_reg=60)
        time.sleep(2.5)

        if not res:
            sys.exit('\n  no valid repetitions')
        w = sum(x['w'] for x in res) / len(res)
        sd = (sum((x['w'] - w) ** 2 for x in res) / len(res)) ** 0.5
        print(f'\n=== result ===')
        print(f'  measured maximum {w:.4f} rad/s   (n={len(res)}, sd {sd:.4f}, '
              f'{sd/w*100:.1f} %)')
        print(f'  no-load bound    {ws:.4f}   measured is {(w/ws-1)*100:+.1f} %')
        print(f'  loaded           {wc:.4f}   measured is {(w/wc-1)*100:+.1f} %')
        near = 'the no-load bound' if abs(w - ws) < abs(w - wc) else 'the loaded prediction'
        print(f'\n  closer to {near}')
        if max(x['drift'] for x in res) > 10:
            print(f'  [note] speed changed {max(x["drift"] for x in res):.0f} % across the '
                  'window; it may still be accelerating. Raise --amp')
        print('\n  * one load on its own says little. Measuring all three tests the')
        print('    prediction that only the loaded description makes: that the')
        print('    maximum falls as the load rises')
    finally:
        print('\n>>> Put the support back, then Enter to release torque > ', end='')
        try: input()
        except Exception: pass
        bus.close(); log.close()


if __name__ == '__main__':
    main()
