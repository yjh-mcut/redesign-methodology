"""
exp0 -- setup and sanity checks.

  python exp0_setup.py

Six steps:

  1) dump the registers (firmware, gains, limits) for the setup sheet
  2) set Return Delay to 0, to cut bus latency
  3) find THETA0 per servo, with the lever horizontal
  4) find SIGN per servo: does raw position rise with joint angle?
  5) measure the range of motion and check the dynamic runs fit in it
  6) check the pair is not fighting itself, and print a payload table
"""
import time, sys
import config as C
import dxl_common as D


def dump_registers(bus):
    print('\n=== register dump -- copy into sheet_A_setup.csv ===')
    hdr = ['ID', 'model', 'fw', 'P', 'I', 'D', 'TqLimit',
           'MaxTq', 'TempLim', 'GoalAcc', 'V', 'T']
    print(' '.join(f'{h:>9}' for h in hdr))
    rows = []
    for i in bus.ids:
        r = dict(ID=i,
                 model=bus.r2(i, D.ADDR_MODEL_NUMBER),
                 fw=bus.r1(i, D.ADDR_FIRMWARE),
                 # firmware 41 uses the PID table: 26 D, 27 I, 28 P
                 CWslope=bus.r1(i, D.ADDR_CW_SLOPE),
                 CCWslope=bus.r1(i, D.ADDR_CCW_SLOPE),
                 CWmargin=bus.r1(i, D.ADDR_CW_MARGIN),
                 TqLimit=bus.r2(i, D.ADDR_TORQUE_LIMIT),
                 MaxTq=bus.r2(i, D.ADDR_MAX_TORQUE),
                 TempLim=bus.r1(i, D.ADDR_TEMP_LIMIT),
                 GoalAcc=bus.r1(i, D.ADDR_GOAL_ACCEL),
                 V=bus.r1(i, D.ADDR_PRESENT_VOLTAGE) / 10.0,
                 T=bus.r1(i, D.ADDR_PRESENT_TEMP))
        rows.append(r)
        print(' '.join(f'{str(r[h]):>9}' for h in hdr))

    for k in ('CWslope', 'CCWslope', 'CWmargin', 'TqLimit'):
        if len({r[k] for r in rows}) > 1:
            print(f'  [warning] {k} differs between servos; make them match')
    if any(r['GoalAcc'] is None for r in rows):
        print('  [warning] could not read Goal Acceleration (73). On older firmware '
              'the trapezoidal profile of exp2_dynamic.py is unavailable.')
    return rows


def set_return_delay(bus):
    print(f'\n=== Return Delay -> {C.RETURN_DELAY} ===')
    for i in bus.ids:
        before = bus.r1(i, D.ADDR_RETURN_DELAY)
        bus.w1(i, D.ADDR_RETURN_DELAY, C.RETURN_DELAY)
        after = bus.r1(i, D.ADDR_RETURN_DELAY)
        print(f'  ID {i}: {before} -> {after}')


def find_theta0(bus):
    print('\n=== THETA0 -- reference pose, per servo ===')
    print('  Torque goes off. Level the lever by hand, then Enter.')
    bus.torque(False)
    input('  > ')
    s = bus.sample()
    t0 = {i: s[i]['pos'] for i in bus.ids if s[i]}
    print('\n  put these in config.py:')
    print('    THETA0 = {' + ', '.join(f'{i}: {v}' for i, v in t0.items()) + '}')
    if len(t0) > 1:
        ids = list(t0)
        d = t0[ids[1]] - t0[ids[0]]
        print(f'\n  the two servos differ by {d} raw ticks -- a mounting phase offset')
        print('  commanding both from one THETA0 would put the joint in the wrong place')
    return t0


def find_sign(bus, t0):
    print('\n=== SIGN -- direction of rotation, per servo ===')
    print('  With torque off, turn the lever 10-20 degrees one way, then Enter.')
    print('  Either direction will do; just remember which one you used.')
    input('  > ')
    s = bus.sample()
    sign = {}
    for i in bus.ids:
        d = s[i]['pos'] - t0[i]
        sign[i] = +1 if d >= 0 else -1
        print(f'  ID {i}: raw moved {d:+d} ticks -> SIGN {sign[i]:+d}')
    if len(set(sign.values())) > 1:
        print('\n  [note] the two signs are opposite, which is what a facing pair gives.')
        print('         That is expected; copy them into config as they are.')
    print('\n  config.py:  SIGN = {' +
          ', '.join(f'{i}: {v:+d}' for i, v in sign.items()) + '}')
    print('\n  * these signs define the direction you just turned as increasing')
    print('    joint angle; flip both together to reverse that convention')
    return sign


def measure_range(bus, t0, sign):
    print('\n=== range of motion ===')
    i0 = bus.ids[0]
    print('  Turn the lever slowly to one end, then Enter.')
    input('  > ')
    a = bus.sample()[i0]['pos']
    print('  Now to the other end, then Enter.')
    input('  > ')
    b = bus.sample()[i0]['pos']
    f = lambda p: sign[i0] * (p - t0[i0]) * D.DEG_PER_TICK / C.N_GEAR
    lo, hi = sorted((f(a), f(b)))
    span = hi - lo
    print(f'\n  range: {lo:+.1f} to {hi:+.1f} deg   ({span:.1f} deg total)')
    print(f'  config.py:  JOINT_MIN_DEG = {lo:.1f}   JOINT_MAX_DEG = {hi:.1f}')

    print('\n  --- does the dynamic test fit? ---')
    print(f'  {"speed":>8} {"t_const":>8} {"travel":>8} {"spare":>9} {"r*a/g":>7}  verdict')
    for w, lbl in ((C.SPEED_LOW, 'LOW'), (C.SPEED_MID, 'MID'), (C.SPEED_HIGH, 'HIGH')):
        for tc in (0.25, 0.20, 0.18):
            trav = w * (C.T_ACC + tc) * 180.0 / D.PI
            marg = (span - trav) / 2.0
            inert = C.R_LEVER * (w / C.T_ACC) / C.G * 100
            v = ('ok' if marg > C.ANGLE_TOL_DEG else
                 'tight' if marg > 0 else 'no')
            print(f'  {w:8.3f} {tc:8.2f} {trav:8.1f} {marg:+9.2f} {inert:6.1f}%  {v}')
    print(f'\n  * the spare at each end must exceed the tracking tolerance, '
          f'{C.ANGLE_TOL_DEG} deg')
    return lo, hi


def fight_check(bus, t0):
    if len(bus.ids) < 2:
        return
    print('\n=== is the pair fighting itself? ===')
    print('  Remove the payload; bare lever only.')
    input('  Enter when ready > ')
    bus.set_common(torque_limit=200, accel_reg=20)
    s = bus.sample()
    if any(s[i] is None for i in bus.ids):
        print('  could not read position; skipping'); return
    bus.sync_goal({i: s[i]['pos'] for i in bus.ids}, speed_reg=50)
    bus.torque(True)
    time.sleep(1.0)
    print('\n   #   ' + '  '.join(f'ID{i} load% (joint frame)' for i in bus.ids))
    bad = 0
    for k in range(10):
        smp = bus.sample()
        if any(smp[i] is None for i in bus.ids):
            print(f'  {k:2d}   read failed'); time.sleep(0.3); continue
        # After the SIGN correction, a facing pair pushing together reads the same
        # sign; opposite signs here mean they are pushing against each other.
        loads = [D.load_joint_dir(i, smp[i]['load_pct']) for i in bus.ids]
        print(f'  {k:2d}   ' + '  '.join(f'{l:+8.1f}' for l in loads))
        if len(loads) == 2 and loads[0] * loads[1] < 0 \
           and min(abs(loads[0]), abs(loads[1])) > 15:
            bad += 1
        time.sleep(0.3)
    bus.torque(False)
    if bad >= 5:
        print('\n  [warning] opposite signs and large magnitudes: they are fighting.')
        print('            Re-do THETA0, or check the mesh.')
    else:
        print('\n  pass -- no large circulating torque')
    print('  * Present Load is not a torque measurement; direction and relative')
    print('    magnitude only')


def payload_table():
    print('\n=== payload mass for a target torque ===')
    if C.M_LEVER is None or C.R_LEVER_CM is None:
        print('  fill in M_LEVER and R_LEVER_CM in config.py first'); return
    tau_l = C.M_LEVER * C.G * C.R_LEVER_CM
    print(f'  bare lever moment {tau_l:.3f} N m   (r = {C.R_LEVER:.3f} m)')
    print(f'  {"target [N m]":>14} {"raw [kg]":>12} {"corrected [kg]":>16}')
    for tau in (C.TAU_REF, C.TAU_150, C.TAU_SF2):
        print(f'  {tau:12.2f} {tau/(C.G*C.R_LEVER):13.2f} '
              f'{D.payload_for_target(tau):13.2f}')


def main():
    bus = D.Bus()
    try:
        dump_registers(bus)
        set_return_delay(bus)
        t0 = find_theta0(bus)
        sign = find_sign(bus, t0)
        # the next two steps use what was just measured
        C.THETA0.update(t0); C.SIGN.update(sign)
        measure_range(bus, t0, sign)
        fight_check(bus, t0)
        payload_table()
        print('\n=== next ===')
        print('  1) put THETA0, SIGN, JOINT_MIN/MAX_DEG, M_LEVER and R_LEVER_CM')
        print('     into config.py')
        print('  2) copy the register dump and the measured values into')
        print('     sheet_A_setup.csv')
        print('  3) python exp1_static.py --load 5.0 --payload 1.5 '
              '--seconds 20 --run PRE-1')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
