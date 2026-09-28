"""Rock the joint about q = 0 so the sign conventions can be checked by eye.

    python test_direction.py                      # +-10 deg, 1 Hz, 5 s
    python test_direction.py --amp 10 --freq 1.0 --seconds 5

Run this with the bare lever, no payload, after setting SIGN and THETA0 in
config.py.  If a servo moves the wrong way its SIGN is wrong; if the lever is
not level at rest its THETA0 is.  Nothing is logged.
"""
import argparse, math, time
import config as C
import dxl_common as D

PI = math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--amp', type=float, default=10.0, help='amplitude [deg]')
    ap.add_argument('--freq', type=float, default=1.0, help='frequency [Hz]')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--torque-limit', type=int, default=300)
    ap.add_argument('--hz', type=float, default=100.0, help='command update rate [Hz]')
    a = ap.parse_args()

    D.require_config()  # only checks that THETA0 covers every id in SERVO_IDS

    omega = 2 * PI * a.freq
    peak_joint_speed = math.radians(a.amp) * omega  # peak of the sine
    speed_reg = D.joint_rads_to_moving_speed(peak_joint_speed * 1.8)  # 80% headroom so the servo is never the limit
    print(f'amplitude +-{a.amp:.1f} deg   frequency {a.freq:.2f} Hz   '
          f'peak joint speed {math.degrees(peak_joint_speed):.1f} deg/s   '
          f'MovingSpeed reg={speed_reg}')
    if speed_reg >= 1023:
        print('  [warning] that is at or above the servo maximum; lower --amp or --freq')

    bus = D.Bus()
    try:
        bus.set_common(torque_limit=a.torque_limit, accel_reg=0,
                       return_delay=C.RETURN_DELAY)
        print('\n>>> Check the lever is bare and the swept range is clear.')
        input('    Enter to enable torque and move to q = 0 > ')
        bus.torque(True)
        bus.sync_goal(D.joint_goals(0.0, bus.ids), speed_reg=60)
        time.sleep(2.0)

        input('    Enter to start rocking > ')
        dt = 1.0 / a.hz
        t0 = time.time(); nxt = t0
        while True:
            now = time.time(); t = now - t0
            if t > a.seconds:
                break
            if now < nxt:
                time.sleep(max(0, nxt - now)); continue
            nxt += dt
            q = a.amp * math.sin(omega * t)
            bus.sync_goal(D.joint_goals(q, bus.ids), speed_reg=speed_reg)
            smp = bus.sample()
            qs = {i: D.servo_to_joint_deg(i, smp[i]['pos'])
                  for i in bus.ids if smp[i]}
            print('\r  t={:5.2f}s  target={:+6.2f}°  actual='.format(t, q) +
                  '  '.join(f'ID{i}:{v:+6.2f}°' for i, v in qs.items()), end='')

        print('\n\nreturning to q = 0')
        bus.sync_goal(D.joint_goals(0.0, bus.ids), speed_reg=60)
        time.sleep(2.0)
    finally:
        bus.close()


if __name__ == '__main__':
    main()
