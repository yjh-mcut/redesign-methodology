"""Sweep the proportional gain, looking for one that damps the speed ripple.

  python3 tune_gains.py --dry
  python3 tune_gains.py --load 11.7275 --payload 3.6226 --speed 0.374 --dir -1
  python3 tune_gains.py --slope 8,16,32,64,128 --repeats 2

Why
  Run B-A-low-3 on 2026-09-21 oscillated at 11 Hz through the constant-speed
  segment.  The mean, 0.375, matched the 0.374 target, so this is not the
  motors running out of capability.  The evaluation window was 0.118 s, only
  1.3 times the 0.088 s oscillation period, so the ripple did not average out
  and showed up as a 21% difference between the two halves.

What is swept
  Address 28, which on this firmware (model 320, revision 41) is P Gain;
  26 and 27 are D and I Gain and are left alone.  Address 29 is written with
  the same value, though it is not documented for this model.

  An earlier version of this file claimed these units used the compliance
  table, on the grounds that address 29 read 32.  That was wrong, and this
  sweep is part of why: a larger value at 28 gives less droop and better
  tracking, which is proportional-gain behaviour.  A compliance slope acts
  the other way.

What is measured
  The same motion as exp2, repeated at each gain, evaluated over the same
  window as the prespecified criteria:

    first half vs second half   <- the criterion (UNIFORMITY_TOL_PCT)
    mean speed error            <- SPEED_TOL_PCT
    peak-to-peak ripple         <- the size of the oscillation itself
    final position              <- too high a gain and it cannot keep up

  Speed is differentiated over a window in *time* (VEL_WIN_S), not over a
  fixed number of samples: a sample-count window gives different noise at
  500 Hz and at 1000 Hz, and the two would not be comparable.

Cautions
  - run this under the real load; the ripple depends on the inertia
  - the original values are restored on the way out, including on an abort
  - whatever is chosen goes into every run and onto the setup sheet
"""
import argparse, math, sys, time
import config as C
import dxl_common as D

PI = math.pi
A_CW_MARGIN, A_CCW_MARGIN = 26, 27
A_CW_SLOPE, A_CCW_SLOPE   = 28, 29

VEL_WIN_S = 0.030      # half-width of the differentiation window [s]


def vel(ts, qs, win=VEL_WIN_S):
    if len(ts) < 5:
        return []
    dt = (ts[-1] - ts[0]) / (len(ts) - 1)
    h = max(2, int(round(win / dt)))
    return [(ts[k], math.radians((qs[k + h] - qs[k - h]) / (ts[k + h] - ts[k - h])))
            for k in range(h, len(ts) - h)]


def metrics(ts, qs, target, t_acc, t_const):
    lo = t_acc + (1 - C.EVAL_FRACTION) / 2 * t_const
    hi = t_acc + (1 + C.EVAL_FRACTION) / 2 * t_const
    v = [(t, abs(x)) for t, x in vel(ts, qs) if lo <= t <= hi]
    if len(v) < 8:
        return None
    mid = len(v) // 2
    mean = sum(x for _, x in v) / len(v)
    f = sum(x for _, x in v[:mid]) / mid
    b = sum(x for _, x in v[mid:]) / (len(v) - mid)
    return dict(n=len(v), mean=mean,
                err=(mean / target - 1) * 100,
                unif=abs(f - b) / ((f + b) / 2) * 100,
                ripple=(max(x for _, x in v) - min(x for _, x in v)) / mean * 100)


def one_run(bus, a, half, speed_reg, accel_reg):
    bus.set_common(accel_reg=10)
    bus.sync_goal(D.joint_goals(-a.dir * half, bus.ids), speed_reg=60)
    time.sleep(2.0)
    bus.set_common(accel_reg=accel_reg)

    dur = 2 * a.t_acc + a.t_const + 0.4
    ts, qs = [], []
    bus.sync_goal(D.joint_goals(a.dir * half, bus.ids), speed_reg=speed_reg)
    t0 = time.time()
    while time.time() - t0 < dur:
        s = bus.sample()
        p = [D.servo_to_joint_deg(i, s[i]['pos']) for i in bus.ids if s[i]]
        if p:
            ts.append(s['t_read'] - t0)
            qs.append(sum(p) / len(p))
    return ts, qs, (qs[-1] if qs else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--load', type=float, default=C.LOAD_ACTUAL['A'])
    ap.add_argument('--payload', type=float, default=C.PAYLOAD_KG['A'])
    ap.add_argument('--speed', type=float, default=C.SPEED_LOW)
    ap.add_argument('--dir', type=int, choices=(1, -1), default=-1,
                    help='-1 lifts the payload on this rig (checked 2026-09-20)')
    ap.add_argument('--slope', default='8,16,32,64,128',
                    help='values to try at addresses 28 and 29')
    ap.add_argument('--margin', type=int, default=0, help='value for addresses 26 and 27')
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--t-acc', type=float, default=C.T_ACC)
    ap.add_argument('--t-const', type=float, default=C.T_CONST)
    ap.add_argument('--torque-limit', type=int, default=C.TORQUE_LIMIT)
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()

    D.require_config('M_LEVER', 'R_LEVER_CM', 'JOINT_MIN_DEG', 'JOINT_MAX_DEG')
    slopes = [int(x) for x in a.slope.split(',') if x.strip()]

    half = a.speed * (a.t_acc + a.t_const) * 180 / PI / 2
    speed_reg = D.joint_rads_to_moving_speed(a.speed)
    accel_reg = D.joint_rads2_to_goal_accel(a.speed / a.t_acc)
    lo = a.t_acc + (1 - C.EVAL_FRACTION) / 2 * a.t_const
    hi = a.t_acc + (1 + C.EVAL_FRACTION) / 2 * a.t_const
    print(f'\nload {a.load} N m (payload {a.payload} kg)   speed {a.speed} rad/s   '
          f'dir {a.dir:+d}')
    print(f'swing {-a.dir*half:+.2f} -> {a.dir*half:+.2f} deg   '
          f'(about horizontal, half-amplitude {half:.2f})')
    print(f'MovingSpeed {speed_reg}   GoalAccel {accel_reg}   '
          f'window {lo:.3f} to {hi:.3f} s')
    print(f'values to try: {slopes}   margin {a.margin}')
    print(f'{len(slopes) * a.repeats} swings in total, about '
          f'{2*a.t_acc + a.t_const + 2.6:.1f} s each')
    if half > min(abs(C.JOINT_MIN_DEG), abs(C.JOINT_MAX_DEG)) - 2:
        sys.exit('  the half-amplitude is too large for the range of motion')
    if a.dry:
        print('\n--dry, stopping here.')
        return

    bus = D.Bus()
    orig = {}
    stamp = D.ts()
    log = D.Logger(f'{C.OUT_DIR}/tune_{stamp}.csv',
                   ['slope', 'margin', 'rep', 'n', 'mean_rad_s', 'err_pct',
                    'unif_pct', 'ripple_pct', 'reach_deg'])
    # Keep the raw traces too, so a change of metric does not mean another run
    raw = D.Logger(f'{C.OUT_DIR}/tune_{stamp}_raw.csv',
                   ['slope', 'rep', 't_s', 'q_deg'])
    try:
        s = bus.sample()
        if any(s[i] is None for i in bus.ids):
            sys.exit('  could not read the servos')
        for i in bus.ids:
            orig[i] = tuple(bus.r1(i, x) for x in
                            (A_CW_MARGIN, A_CCW_MARGIN, A_CW_SLOPE, A_CCW_SLOPE))
            print(f'  ID {i} now  26/27 {orig[i][0]}/{orig[i][1]}  '
                  f'28/29 {orig[i][2]}/{orig[i][3]}')
        print(f'\nstart temperature {[s[i]["temp"] for i in bus.ids]} C   '
              f'voltage {[s[i]["volt"] for i in bus.ids]} V')
        print('\n>>> Check the real payload is fitted. A bare lever tells you nothing.')
        print('>>> The lever sweeps repeatedly. Clear the area.')
        input('    Enter to start > ')

        bus.set_common(torque_limit=a.torque_limit, return_delay=C.RETURN_DELAY)
        bus.sync_goal({i: s[i]['pos'] for i in bus.ids}, speed_reg=50)
        bus.torque(True)

        print(f'\n  {"gain":>7}{"rep":>4}{"n":>6}{"mean":>8}{"err":>8}'
              f'{"halves":>8}{"ripple":>8}{"final":>8}')
        summary = {}
        for sl in slopes:
            for i in bus.ids:
                bus.w1(i, A_CW_MARGIN, a.margin)
                bus.w1(i, A_CCW_MARGIN, a.margin)
                bus.w1(i, A_CW_SLOPE, sl)
                bus.w1(i, A_CCW_SLOPE, sl)
            back = [bus.r1(i, A_CW_SLOPE) for i in bus.ids]
            if any(b != sl for b in back):
                print(f'  {sl:7d}  write failed, readback {back} -- skipping')
                continue
            rows = []
            for r in range(a.repeats):
                ts, qs, reach = one_run(bus, a, half, speed_reg, accel_reg)
                for t_, q_ in zip(ts, qs):
                    raw.write(dict(slope=sl, rep=r + 1, t_s=round(t_, 5),
                                   q_deg=round(q_, 4)))
                m = metrics(ts, qs, a.speed, a.t_acc, a.t_const)
                if m is None:
                    print(f'  {sl:7d}{r+1:4d}   no constant segment found'); continue
                rows.append(m)
                log.write(dict(slope=sl, margin=a.margin, rep=r + 1, n=m['n'],
                               mean_rad_s=round(m['mean'], 4),
                               err_pct=round(m['err'], 2),
                               unif_pct=round(m['unif'], 2),
                               ripple_pct=round(m['ripple'], 1),
                               reach_deg=round(reach, 3)))
                print(f'  {sl:7d}{r+1:4d}{m["n"]:6d}{m["mean"]:8.4f}{m["err"]:+7.1f}%'
                      f'{m["unif"]:7.1f}%{m["ripple"]:7.0f}%{reach:+7.2f}°')
            if rows:
                summary[sl] = (sum(x['unif'] for x in rows) / len(rows),
                               sum(x['ripple'] for x in rows) / len(rows))
        if summary:
            print(f'\n  {"gain":>7}{"halves":>9}{"ripple":>9}')
            for sl in slopes:
                if sl in summary:
                    u, rp = summary[sl]
                    mark = '  <-' if u <= C.UNIFORMITY_TOL_PCT else ''
                    print(f'  {sl:7d}{u:8.1f}%{rp:8.0f}%{mark}')
            print(f'  criterion: halves within {C.UNIFORMITY_TOL_PCT} %')
            print('  * a small half-to-half difference with a large ripple just means')
            print('    the phase happened to line up; look at both')
            print('  * a final angle drifting from the target means the gain is too low')
            print('    to keep up')
    finally:
        for i, g in orig.items():
            try:
                for addr, v in zip((A_CW_MARGIN, A_CCW_MARGIN,
                                    A_CW_SLOPE, A_CCW_SLOPE), g):
                    if v is not None:
                        bus.w1(i, addr, v)
            except Exception:
                pass
        if orig:
            print('\nrestored: ' + '  '.join(
                f'ID{i} 26/27 {g[0]}/{g[1]} 28/29 {g[2]}/{g[3]}' for i, g in orig.items()))
        print('>>> Put the support back, then Enter to release torque > ', end='')
        try: input()
        except Exception: pass
        bus.close(); log.close(); raw.close()


if __name__ == '__main__':
    main()
