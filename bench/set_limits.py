"""Write the servo angle limits (CW at 6, CCW at 8) to EEPROM.

    python set_limits.py                # set the ends by hand, torque off
    python set_limits.py --margin 2     # keep 2 joint degrees inside each end
    python set_limits.py --show         # print the current limits
    python set_limits.py --reset        # back to 0 / 4095

With limits set, a Goal Position outside the range is refused by the servo
with an Angle Limit Error.  That is the last thing standing between a wrong
command and the frame.

Notes:
  - this writes EEPROM, so torque has to be off; the script handles that
  - CW must be below CCW, in raw terms, for each servo independently, even
    though the two have opposite SIGN
  - what is set here is the mechanically safe end.  The swing of the dynamic
    runs has to fit inside it, so setting it too tight rules out the fast tests
"""
import argparse, sys
import config as C
import dxl_common as D

ADDR_CW, ADDR_CCW = 6, 8
DEG = D.DEG_PER_TICK


def show(bus):
    print(f'\n{"ID":>5} {"CW":>6} {"CCW":>6} {"servo":>9} {"joint":>9} {"now":>6}')
    out = {}
    for i in bus.ids:
        cw, ccw, pos = bus.r2(i, ADDR_CW), bus.r2(i, ADDR_CCW), bus.r2(i, 36)
        if None in (cw, ccw, pos):
            print(f'{i:5d}   read failed -- check power and wiring')
            continue
        out[i] = (cw, ccw, pos)
        if cw == 0 and ccw == 0:
            print(f'{i:5d} {cw:6d} {ccw:6d}    wheel mode -- no position control!')
            continue
        span = (ccw - cw) * DEG
        inside = '' if cw <= pos <= ccw else '  <- currently outside the range'
        print(f'{i:5d} {cw:6d} {ccw:6d} {span:8.1f}° {span/C.N_GEAR:8.1f}° '
              f'{pos:6d}{inside}')
    return out


def capture(bus):
    bus.torque(False)
    print('\nTorque is off. Move the lever by hand.')
    print('The positions set here are the ones the servo will never pass.')
    print('Stop a little short of the frame.\n')

    pts = []
    for k, label in ((0, 'one end'), (1, 'the other end')):
        input(f'  move the lever slowly to {label}, then Enter > ')
        s = bus.sample()
        if any(s[i] is None for i in bus.ids):
            sys.exit('  could not read position -- check the bus')
        p = {i: s[i]['pos'] for i in bus.ids}
        print('    ' + '   '.join(f'ID{i} = {v} tick' for i, v in p.items()))
        pts.append(p)

    a, b = pts
    if all(abs(a[i] - b[i]) < 30 for i in bus.ids):
        sys.exit('\nThe two positions are nearly the same; the lever did not move. Stopping.')
    return a, b


def plan(bus, a, b, margin_joint_deg):
    m = int(round(margin_joint_deg * C.N_GEAR / DEG))  # joint degrees -> servo ticks
    out = {}
    print(f'\nmargin {margin_joint_deg:.1f} deg at the joint = {m} servo ticks')
    print(f'\n{"ID":>5} {"meas lo":>8} {"meas hi":>8} -> {"CW":>6} {"CCW":>6} {"joint":>9}')
    for i in bus.ids:
        lo, hi = sorted((a[i], b[i]))
        cw, ccw = lo + m, hi - m
        cw, ccw = max(0, cw), min(4095, ccw)
        if ccw - cw < 50:
            sys.exit(f'  ID {i}: the margin leaves only {ccw-cw} ticks. Lower --margin.')
        out[i] = (cw, ccw)
        print(f'{i:5d} {lo:8d} {hi:8d} -> {cw:6d} {ccw:6d} '
              f'{(ccw-cw)*DEG/C.N_GEAR:8.1f}°')

    span = min((ccw - cw) for cw, ccw in out.values()) * DEG / C.N_GEAR
    print(f'\nJoint range both servos allow: {span:.1f} deg')

    print(f'\nDoes the dynamic test fit in that range?  (T_ACC = {C.T_ACC} s)')
    print(f'  {"speed":>8} {"t_const":>8} {"travel":>8} {"spare":>9}  verdict')
    for w in (C.SPEED_LOW, C.SPEED_MID, C.SPEED_HIGH):
        for tc in (0.25, 0.20, 0.18):
            trav = w * (C.T_ACC + tc) * 180.0 / D.PI
            marg = (span - trav) / 2.0
            v = ('ok' if marg > C.ANGLE_TOL_DEG else
                 'tight' if marg > 0 else 'no')
            print(f'  {w:8.3f} {tc:8.2f} {trav:8.1f}° {marg:+8.2f}°  {v}')
    print(f'  * the spare at each end must exceed the tracking tolerance, '
          f'{C.ANGLE_TOL_DEG} deg')
    print('  * the swing is centred on q = 0, so the figures above assume q = 0 is')
    print('    the middle of the range')
    return out


def write(bus, limits):
    bus.torque(False)
    print()
    for i, (cw, ccw) in limits.items():
        bus.w2(i, ADDR_CW, cw)
        bus.w2(i, ADDR_CCW, ccw)
        rcw, rccw = bus.r2(i, ADDR_CW), bus.r2(i, ADDR_CCW)
        ok = 'OK' if (rcw, rccw) == (cw, ccw) else '<- mismatch!'
        print(f'  ID {i}: CW {rcw}  CCW {rccw}   {ok}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--margin', type=float, default=1.0,
                    help='margin inside each measured end [joint deg]')
    ap.add_argument('--show', action='store_true', help='print the current limits and exit')
    ap.add_argument('--reset', action='store_true', help='restore 0 / 4095')
    a = ap.parse_args()

    bus = D.Bus()
    try:
        print('=== current angle limits ===')
        show(bus)
        if a.show:
            return
        if a.reset:
            if input('\nRestore limits to 0 / 4095. Continue? (yes) > ').strip() != 'yes':
                print('cancelled'); return
            write(bus, {i: (0, 4095) for i in bus.ids})
            print('\n=== after reset ==='); show(bus)
            return

        print('\n=== setting new limits ===')
        print('Do this with the payload removed.')
        p1, p2 = capture(bus)
        limits = plan(bus, p1, p2, a.margin)

        print('\nThese go to EEPROM. --reset undoes it.')
        if input('type yes to continue > ').strip() != 'yes':
            print('cancelled; nothing written'); return
        write(bus, limits)
        print('\n=== after setting ===')
        show(bus)
        print('\nNext: exp0_setup.py, to fix THETA0, SIGN and the range of motion.')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
