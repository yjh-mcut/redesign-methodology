"""
exp1 -- hold a load at a fixed joint angle.

  python exp1_static.py --load 5.0   --payload 1.5  --seconds 20  --run PRE-1
  python exp1_static.py --load 11.59 --payload 3.69 --run A-1159-1
    python exp1_static.py --load 11.73 --payload 3.6226 --run A-A-1 --center

Starting angle
  The load falls as cos q.  Ten degrees off horizontal costs 1.52%, and the
  SF 2 margin is only 1.44%, so that alone makes the run meaningless.  Use
  --center, or level the lever with a spirit level.  Past config.Q0_TOL_DEG
  the script refuses to run.

Procedure
  1) start with the payload resting on its support
  2) the script enables torque and commands both servos in one SyncWrite
  3) lower the support slowly when prompted, so the load transfers
  4) hold and log until the time is up or an abort condition fires
  5) put the support back before torque goes off
"""
import argparse, math, time, sys
import config as C
import dxl_common as D

FIELDS = ['run_id', 't_s', 'wall_time', 'load_target_Nm', 'payload_kg',
          'q_deg', 'q_err_deg', 'tau_grav_Nm',
          'pos1', 'load1_pct', 'volt1', 'temp1_C', 'cur1_A',
          'pos2', 'load2_pct', 'volt2', 'temp2_C', 'cur2_A',
          'q2_deg', 'dq12_deg', 'comm_fail', 'slope',
          'abort_reason', 'note']


ADDR_CURRENT = 68      # MX-106 only, 2B. 4.5 mA/LSB, 2048 = 0 A
A_CW_SLOPE, A_CCW_SLOPE = 28, 29   # 28 is P Gain on this firmware; 29 is undocumented


def read_current(bus, i):
    """Signed current in amps, or None if the read failed."""
    c = bus.r2(i, ADDR_CURRENT)
    return None if c is None else (c - 2048) * 0.0045


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--load', type=float, required=True, help='target joint torque [N m]')
    ap.add_argument('--payload', type=float, required=True, help='weighed payload mass [kg]')
    ap.add_argument('--seconds', type=float, default=C.HOLD_SECONDS)
    ap.add_argument('--run', required=True)
    ap.add_argument('--torque-limit', type=int, default=1023)
    ap.add_argument('--center', action='store_true',
                     help='move to horizontal (q = 0) before starting, instead of '
                          'holding wherever the lever happens to be. Use this for '
                          'real runs; the load is then not reduced by cos q')
    ap.add_argument('--allow-tilt', action='store_true',
                     help=f'proceed even past {C.Q0_TOL_DEG} deg off horizontal. '
                          'Preliminary runs only: the load will fall short')
    ap.add_argument('--slope', type=int,
                    help='value for addresses 28 and 29. Larger values give less '
                         'droop: 32 -> 1.11 deg, 128 -> 0.28 deg on 2026-09-21. '
                         'Left alone if omitted')
    ap.add_argument('--note', default='')
    a = ap.parse_args()

    D.require_config('M_LEVER', 'R_LEVER_CM')
    if a.center:
        D.require_config('JOINT_MIN_DEG', 'JOINT_MAX_DEG')

    bus = D.Bus()
    log = D.Logger(f'{C.OUT_DIR}/exp1_{a.run}_{D.ts()}.csv', FIELDS)
    abort, t = '', 0.0
    slope_orig = {}
    try:
        # ---- start temperature -----------------------------------
        for _ in range(20):
            s = bus.sample()
            if all(s[i] is not None for i in bus.ids): break
            time.sleep(0.05)
        else:
            sys.exit('could not read both servos -- check wiring, power and IDs')
        temps = [s[i]['temp'] for i in bus.ids if s[i]]
        print(f'start temperature: {temps} C')
        if max(temps) > C.TEMP_START_C:
            sys.exit(f'  let them cool below {C.TEMP_START_C} C first')
        if len(temps) > 1 and abs(temps[0] - temps[1]) > C.TEMP_START_MATCH_C:
            sys.exit(f'  the two differ by more than {C.TEMP_START_MATCH_C} C')

        # ---- proportional gain -------------------------------------
        # Steady-state droop goes inversely with this. On 2026-09-25 box C
        # (23.39 N m) drooped 1.9 deg, past the 1.0 deg tolerance.
        for i in bus.ids:
            slope_orig[i] = (bus.r1(i, A_CW_SLOPE), bus.r1(i, A_CCW_SLOPE))
        if a.slope is not None:
            for i in bus.ids:
                bus.w1(i, A_CW_SLOPE, a.slope)
                bus.w1(i, A_CCW_SLOPE, a.slope)
            back = [(bus.r1(i, A_CW_SLOPE), bus.r1(i, A_CCW_SLOPE)) for i in bus.ids]
            if any(x != (a.slope, a.slope) for x in back):
                sys.exit(f'  write failed; readback gave {back}')
        cur_slope = [bus.r1(i, A_CW_SLOPE) for i in bus.ids]
        print(f'addr 28/29 = {cur_slope}'
              + ('' if a.slope is None else f'  (was {[x[0] for x in slope_orig.values()]})'))

        # ---- goal position, both servos in one packet ---------------
        bus.set_common(torque_limit=a.torque_limit, accel_reg=20,
                       return_delay=C.RETURN_DELAY)

        if a.center:
            # Horizontal, not the middle of the range: gravity torque peaks here
            # and d(cos q)/dq is zero, so the load matches the target and is
            # insensitive to a small angle error.
            q0 = 0.0
            goals = D.joint_goals(q0, bus.ids)
            print(f'reference joint angle q0 = {q0:+.2f} deg (horizontal)')
            print('\n>>> Check the payload is resting on its support.')
            input('    Enter to enable torque and move to horizontal > ')
            bus.torque(True)
            bus.sync_goal(goals, speed_reg=60)
            time.sleep(2.0)
        else:
            print('\n>>> Check the payload is resting on its support.')
            input('    Enter to enable torque > ')
            # Re-read the position after the prompt. If the lever moved while we
            # waited, a stale position becomes the goal and the joint snaps to
            # it the instant torque comes on -- the 2026-09-15 131641 log
            # starts 42 degrees out for exactly that reason.
            s2 = bus.sample()
            if any(s2[i] is None for i in bus.ids):
                sys.exit('could not read position -- start again')
            goals = {i: s2[i]['pos'] for i in bus.ids}
            bus.sync_goal(goals, speed_reg=50)
            q0 = D.servo_to_joint_deg(bus.ids[0], s2[bus.ids[0]]['pos'])
            print(f'reference joint angle q0 = {q0:+.2f} deg')
            bus.torque(True)
            time.sleep(0.5)

        # The load falls as cos q0. Running tilted records a 'held' result at less
        # than the target torque, and the SF 2 margin is only 1.44%.
        shortfall = (math.cos(math.radians(q0)) - 1.0) * 100
        print(f'  {q0:+.2f} deg off horizontal -> effective load '
              f'{a.load * math.cos(math.radians(q0)):.3f} '
              f'N m ({shortfall:+.2f} %)')
        if abs(q0) > C.Q0_TOL_DEG:
            msg = (f'  start angle is {abs(q0):.2f} deg off horizontal '
                   f'(tolerance {C.Q0_TOL_DEG}); the load is {shortfall:+.2f} % out.')
            if not a.allow_tilt:
                sys.exit(msg + '\n  Level it, or use --center. For a preliminary run, '
                               '--allow-tilt.')
            print('  [warning]' + msg[2:] + '  (--allow-tilt)')

        print('>>> Now lower the support slowly, so the load transfers.')
        input('    Enter once it is carrying the full load > ')
        sl = bus.sample()
        if all(sl[i] is not None for i in bus.ids):
            lj = [D.load_joint_dir(i, sl[i]['load_pct']) for i in bus.ids]
            print(f'  Present Load, joint frame: {lj} %')
            if sum(abs(x) for x in lj) < 5.0:
                print('  [warning] both Loads are near zero; the payload may still be '
                      'resting on the support. Two PRE-1 runs on 2026-09-15 were '
                      'like this')

        # ---- logging ----------------------------------------------
        dt = 1.0 / C.LOG_HZ_FAST
        t0 = time.time(); nxt = t0
        last_slow, over_since = 0.0, None
        last_ok = t0
        dq12_0 = None
        temp_c = {i: s[i]['temp'] for i in bus.ids}
        volt_c = {i: s[i]['volt'] for i in bus.ids}

        while True:
            now = time.time(); t = now - t0
            if t >= a.seconds: break
            if now < nxt:
                time.sleep(max(0, nxt - now)); continue
            nxt += dt

            smp = bus.sample()
            if any(smp[i] is None for i in bus.ids):
                # With the bus down we cannot check the angle, so bound how long
                if smp['t_read'] - last_ok >= C.COMM_ABORT_S:
                    abort = f'comm_loss>={C.COMM_ABORT_S}s'; break
                continue
            last_ok = smp['t_read']
            t = smp['t_read'] - t0          # the time the position was read, not before it
            p1 = smp[bus.ids[0]]
            p2 = smp[bus.ids[1]] if len(bus.ids) > 1 else None

            q = D.servo_to_joint_deg(bus.ids[0], p1['pos'])
            qerr = q - q0
            tau_g = D.gravity_torque(q, m_p=a.payload)
            q2 = D.servo_to_joint_deg(bus.ids[1], p2['pos']) if p2 else None
            dq12 = (q - q2) if p2 else None
            if dq12 is not None and dq12_0 is None:
                dq12_0 = dq12

            cur = {i: read_current(bus, i) for i in bus.ids}

            if t - last_slow >= 1.0 / C.LOG_HZ_SLOW:
                last_slow = t
                for i in bus.ids:
                    temp_c[i] = smp[i]['temp']; volt_c[i] = smp[i]['volt']
                print(f'\r  {t:6.1f}s  q={q:+6.2f} err={qerr:+5.2f}°  '
                      f'T={[temp_c[i] for i in bus.ids]}°C  '
                      f'V={[round(volt_c[i],1) for i in bus.ids]}  '
                      f'load={[round(smp[i]["load_pct"]) for i in bus.ids]}%  '
                      f'I={[None if cur.get(i) is None else round(cur[i],2) for i in bus.ids]}A',
                      end='')

            log.write(dict(
                run_id=a.run, t_s=round(t, 4), wall_time=f'{smp["t_read"]:.3f}',
                load_target_Nm=a.load, payload_kg=a.payload,
                q_deg=round(q, 3), q_err_deg=round(qerr, 3),
                tau_grav_Nm=round(tau_g, 3),
                pos1=p1['pos'], load1_pct=p1['load_pct'],
                volt1=p1['volt'], temp1_C=p1['temp'],
                cur1_A=('' if cur[bus.ids[0]] is None
                        else round(cur[bus.ids[0]], 4)),
                pos2=(p2['pos'] if p2 else ''),
                load2_pct=(p2['load_pct'] if p2 else ''),
                volt2=(p2['volt'] if p2 else ''),
                temp2_C=(p2['temp'] if p2 else ''),
                cur2_A=('' if len(bus.ids) < 2 or cur[bus.ids[1]] is None
                        else round(cur[bus.ids[1]], 4)),
                q2_deg=(round(q2, 3) if p2 else ''),
                dq12_deg=(round(dq12, 3) if p2 else ''),
                comm_fail=bus.comm_fail, slope=cur_slope[0],
                abort_reason='', note=a.note))

            if dq12 is not None and abs(dq12 - dq12_0) > C.INTERSERVO_WARN_DEG:
                print(f'\n  [warning] the inter-servo angle moved {dq12 - dq12_0:+.2f} deg '
                      f'from where it started -- check the mesh')

            if abs(qerr) > C.ANGLE_TOL_DEG:
                over_since = over_since if over_since is not None else t
                if t - over_since >= C.ANGLE_ABORT_DWELL_S:
                    abort = (f'angle_error>{C.ANGLE_TOL_DEG}deg '
                             f'for {C.ANGLE_ABORT_DWELL_S}s'); break
            else:
                over_since = None
            if max(temp_c.values()) >= C.TEMP_ABORT_C:
                abort = f'temp>={C.TEMP_ABORT_C}C'; break


        print(f'\n\nended: {abort or "completed"}   elapsed {t:.1f} s')
        log.write(dict(run_id=a.run, t_s=round(t, 3), comm_fail=bus.comm_fail,
                       abort_reason=abort or 'completed', note=a.note))
        if bus.comm_fail:
            print(f'  {bus.comm_fail} failed or rejected reads -- note this on the run sheet')
    finally:
        print('\n>>> Put the support back, then Enter to release torque > ', end='')
        try: input()
        except Exception: pass
        if a.slope is not None:
            for i, g in slope_orig.items():
                try:
                    if g[0] is not None: bus.w1(i, A_CW_SLOPE, g[0])
                    if g[1] is not None: bus.w1(i, A_CCW_SLOPE, g[1])
                except Exception:
                    pass
            print('restored addr 28/29 to their previous values: '
                  + '  '.join(f'ID{i} {g[0]}/{g[1]}' for i, g in slope_orig.items()))
        bus.close(); log.close()
        print('log written')


if __name__ == '__main__':
    main()
