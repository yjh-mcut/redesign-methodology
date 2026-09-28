"""
exp2 -- constant-rate traverse under load.

Goal Acceleration and Moving Speed give a trapezoidal profile, and one
SyncWrite starts both actuators together.

    python exp2_dynamic.py --load 11.73 --speed 0.374 --run X --dry
  python exp2_dynamic.py --load 11.59 --payload 3.69 --speed 0.374 --run B-1159-0374-1

Starting them with two separate writes leaves 0.5-1 ms between the two, and
for that long they push against each other.  Load sharing is one of the things
being measured, so this path has to be a SyncWrite.
"""
import argparse, math, time, sys
import config as C
import dxl_common as D

PI = math.pi
FIELDS = ['run_id', 't_s', 'wall_time', 'phase',
          'load_target_Nm', 'payload_kg', 'speed_target_rad_s',
          't_acc_s', 't_const_s', 'dir',
          'q_deg', 'q_dot_rad_s', 'tau_grav_Nm',
          'pos1', 'spd1_rpm', 'load1_pct', 'volt1', 'temp1_C',
          'pos2', 'spd2_rpm', 'load2_pct', 'volt2', 'temp2_C',
          'q2_deg', 'dq12_deg', 'comm_fail', 'note']


def plan(omega, t_acc, t_const):
    return dict(alpha=omega / t_acc,
                ramp_deg=omega * t_acc * 180 / PI,
                const_deg=omega * t_const * 180 / PI,
                total_deg=omega * (t_acc + t_const) * 180 / PI,
                inertia_frac=C.R_LEVER * (omega / t_acc) / C.G)


def show_plan(omega, t_acc, t_const):
    p = plan(omega, t_acc, t_const)
    print(f'\n  target speed   {omega:.3f} rad/s')
    print(f'  acceleration   {p["alpha"]:.2f} rad/s^2')
    print(f'  ramp angle     {p["ramp_deg"]:.1f} deg')
    print(f'  constant angle {p["const_deg"]:.1f} deg')
    print(f'  total travel   {p["total_deg"]:.1f} deg')
    print(f'  inertia share  {p["inertia_frac"]*100:.1f} %  (while accelerating, near horizontal)')
    if C.JOINT_MIN_DEG is not None and C.JOINT_MAX_DEG is not None:
        span = C.JOINT_MAX_DEG - C.JOINT_MIN_DEG
        marg = (span - p['total_deg']) / 2
        v = ('ok' if marg > C.ANGLE_TOL_DEG else
             'tight -- reduce t_const' if marg > 0 else 'does not fit')
        print(f'  range {span:.1f} deg, spare {marg:+.2f} deg at each end  ->  {v}')
    print(f'  MovingSpeed  {D.joint_rads_to_moving_speed(omega)}')
    print(f'  GoalAccel    {D.joint_rads2_to_goal_accel(p["alpha"])}')
    rpm = omega * C.N_GEAR * 60 / (2 * PI)
    print(f'  servo speed    {rpm:.1f} rpm')
    for v, (ts_, nl) in sorted(C.MX106_SPEC.items()):
        tag = '  <-- above the no-load speed: both screens reject, nothing to tell apart' if rpm > nl else ''
        print(f'    spec at {v:>5.1f} V: no-load {nl} rpm, stall {ts_} N m{tag}')
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--load', type=float, required=True)
    ap.add_argument('--speed', type=float, required=True, help='target joint speed [rad/s]')
    ap.add_argument('--run', required=True)
    ap.add_argument('--payload', type=float, default=None)
    ap.add_argument('--t-acc', type=float, default=C.T_ACC)
    ap.add_argument('--t-const', type=float, default=C.T_CONST)
    ap.add_argument('--torque-limit', type=int, default=1023)
    ap.add_argument('--dir', type=int, choices=(1, -1), default=1,
                    help='direction of travel; pick the one that lifts the payload')
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--note', default='')
    a = ap.parse_args()

    p = show_plan(a.speed, a.t_acc, a.t_const)
    if a.dry: return
    if a.payload is None: sys.exit('--payload is required')

    D.require_config('M_LEVER', 'R_LEVER_CM', 'JOINT_MIN_DEG', 'JOINT_MAX_DEG',
                     'SUPPLY_V_NOMINAL')
    span = C.JOINT_MAX_DEG - C.JOINT_MIN_DEG
    if p['total_deg'] > span:
        sys.exit('the travel does not fit the range of motion; reduce --t-const')

    speed_reg = D.joint_rads_to_moving_speed(a.speed)
    accel_reg = D.joint_rads2_to_goal_accel(p['alpha'])
    if speed_reg >= 1023:
        print('  [warning] Moving Speed is at its ceiling; the target may not be reached')

    bus = D.Bus()
    log = D.Logger(f'{C.OUT_DIR}/exp2_{a.run}_{D.ts()}.csv', FIELDS)
    try:
        for _ in range(20):
            s = bus.sample()
            if all(s[i] is not None for i in bus.ids): break
            time.sleep(0.05)
        else:
            sys.exit('could not read both servos')
        temps = [s[i]['temp'] for i in bus.ids]
        volts = [s[i]['volt'] for i in bus.ids]
        print(f'\nstart temperature {temps} C   voltage {volts} V')
        if max(temps) > C.TEMP_START_C:
            sys.exit(f'  let them cool below {C.TEMP_START_C} C')
        if abs(min(volts) - C.SUPPLY_V_NOMINAL) > C.SUPPLY_V_TOL:
            sys.exit(f'  {min(volts):.1f} V is outside the prespecified '
                     f'{C.SUPPLY_V_NOMINAL} +-{C.SUPPLY_V_TOL} V; the model '
                     f'predictions are for that voltage')

        # The swing is centred on horizontal, not on the middle of the range:
        # cos q peaks there and its slope is zero.
        half = p['total_deg'] / 2
        q_start, q_end = -a.dir * half, a.dir * half
        if min(q_start, q_end) - C.ANGLE_TOL_DEG < C.JOINT_MIN_DEG or \
           max(q_start, q_end) + C.ANGLE_TOL_DEG > C.JOINT_MAX_DEG:
            sys.exit('a swing centred on horizontal leaves the range; reduce --t-const')
        print(f'  {q_start:+.1f}° -> {q_end:+.1f}°  '
              '(check this lifts the payload; if not, use --dir -1)')

        # ---- move slowly to the start position ---------------------
        bus.set_common(torque_limit=a.torque_limit, accel_reg=10,
                       return_delay=C.RETURN_DELAY)
        print('\n>>> Check the payload is loading the joint.')
        input('    Enter to move to the start position > ')
        # Set the goal to where we are before enabling torque, so the joint
        # does not snap to a stale one.
        s2 = bus.sample()
        if any(s2[i] is None for i in bus.ids):
            sys.exit('could not read position')
        bus.sync_goal({i: s2[i]['pos'] for i in bus.ids}, speed_reg=60)
        bus.torque(True)
        time.sleep(0.3)
        bus.sync_goal(D.joint_goals(q_start, bus.ids), speed_reg=60)
        time.sleep(3.0)

        # ---- set up the motion profile -----------------------------
        bus.set_common(accel_reg=accel_reg)
        input('    Enter to start the traverse > ')

        dt = 1.0 / C.LOG_HZ_DYN
        t0 = time.time()
        bus.sync_goal(D.joint_goals(q_end, bus.ids), speed_reg=speed_reg)

        duration = 2 * a.t_acc + a.t_const + 1.0
        prev_q = prev_t = None
        v_peak = 0.0
        nxt = t0
        while True:
            now = time.time(); t = now - t0
            if t > duration: break
            if now < nxt:
                time.sleep(max(0, nxt - now)); continue
            nxt += dt

            smp = bus.sample()
            if any(smp[i] is None for i in bus.ids): continue
            now = smp['t_read']; t = now - t0     # the time the position was read
            p1 = smp[bus.ids[0]]
            p2 = smp[bus.ids[1]] if len(bus.ids) > 1 else None
            q = D.servo_to_joint_deg(bus.ids[0], p1['pos'])
            q2 = D.servo_to_joint_deg(bus.ids[1], p2['pos']) if p2 else None
            qdot = 0.0
            if prev_q is not None and now > prev_t:
                qdot = math.radians(q - prev_q) / (now - prev_t)
            prev_q, prev_t = q, now

            # Trapezoid: accelerate over 0..t_acc, hold to t_acc+t_const, then
            # decelerate. An earlier version used t_acc/2 as the boundary, which
            # put half the acceleration inside the 'const' window.
            # These labels come from the command, not the servo, so analyze.py
            # checks against the encoder when it reports.
            phase = ('accel' if t < a.t_acc else
                     'const' if t < a.t_acc + a.t_const else 'decel')

            # Watch for regenerative overvoltage. Driving the load downhill turns
            # the actuators into generators and a switched-mode supply cannot
            # sink the current, so the bus climbs. Three runs on 2026-09-20 sat
            # at 21.5-24.1 V for 0.2 s each; the servo's own limit is 16.0 V.
            vnow = max([p1['volt']] + ([p2['volt']] if p2 else []))
            if vnow > v_peak:
                v_peak = vnow

            log.write(dict(
                run_id=a.run, t_s=round(t, 5), wall_time=f'{now:.4f}',
                phase=phase, load_target_Nm=a.load, payload_kg=a.payload,
                speed_target_rad_s=a.speed,
                t_acc_s=a.t_acc, t_const_s=a.t_const, dir=a.dir,
                q_deg=round(q, 3), q_dot_rad_s=round(qdot, 4),
                tau_grav_Nm=round(D.gravity_torque(q, m_p=a.payload), 3),
                pos1=p1['pos'], spd1_rpm=round(p1['speed_rpm'], 2),
                load1_pct=p1['load_pct'], volt1=p1['volt'], temp1_C=p1['temp'],
                pos2=(p2['pos'] if p2 else ''),
                spd2_rpm=(round(p2['speed_rpm'], 2) if p2 else ''),
                load2_pct=(p2['load_pct'] if p2 else ''),
                volt2=(p2['volt'] if p2 else ''),
                temp2_C=(p2['temp'] if p2 else ''),
                q2_deg=(round(q2, 3) if p2 else ''),
                dq12_deg=(round(q - q2, 3) if p2 else ''),
                comm_fail=bus.comm_fail,
                note=a.note))

        sf = bus.sample()
        q_f = D.servo_to_joint_deg(bus.ids[0], sf[bus.ids[0]]['pos']) \
            if sf[bus.ids[0]] else float('nan')
        rate = log.n / duration
        print(f'\n  reached {q_f:+.2f} deg  (target {q_end:+.2f})  '
              f'{log.n} samples (~{rate:.0f} Hz)  failed reads {bus.comm_fail}')
        print(f'  peak bus voltage {v_peak:.1f} V')
        if v_peak > C.SUPPLY_V_MAX_ABORT:
            print(f'  [stop] regenerative overvoltage {v_peak:.1f} V > {C.SUPPLY_V_MAX_ABORT} V.')
            print('         The actuators are braking, which usually means the run is')
            print('         lowering the payload. Try flipping --dir.')
            print('         Keep the log but do not use it for model comparison, and')
            print('         do not run again until this is fixed.')
        if rate < 0.5 * C.LOG_HZ_DYN:
            print(f'  [warning] sampled at {rate:.0f} Hz, under half the configured '
                  f'{C.LOG_HZ_DYN} Hz. Set the FTDI latency timer to 1 ms')
        print('  run analyze.py to check the constant-speed segment')

        bus.set_common(accel_reg=10)
        bus.sync_goal(D.joint_goals(q_start, bus.ids), speed_reg=60)
        time.sleep(3.0)
    finally:
        print('\n>>> Put the support back, then Enter to release torque > ', end='')
        try: input()
        except Exception: pass
        bus.close(); log.close()


if __name__ == '__main__':
    main()
