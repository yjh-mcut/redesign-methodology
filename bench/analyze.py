"""
analyze -- summarise a log file.

    python analyze.py data/exp1_A-C-2_*.csv     # a static hold
    python analyze.py data/exp2_B-C-low-1_*.csv  # a constant-rate traverse

For a hold: duration, worst angle error, temperature rise, why it stopped.
For a traverse: mean speed over the evaluated window against +-5%, first half
against second half against +-3%, the actual load, and the supply voltage.
"""
import csv, glob, math, statistics as st, sys
import config as C

PI = math.pi


def load(path):
    with open(path, encoding='utf-8-sig') as f:
        return [r for r in csv.DictReader(f) if r.get('t_s') not in (None, '')]


def fnum(r, k, d=None):
    v = r.get(k, '')
    try: return float(v)
    except (TypeError, ValueError): return d


# ==================================================================
def _sign(i_idx):
    ids = list(C.SERVO_IDS)
    return C.SIGN.get(ids[i_idx], 1) if i_idx < len(ids) else 1


def _common_checks(dat, rows):
    """Common to both: recompute gravity torque, report Load, inter-servo angle
    difference, and failed reads."""
    # The logged tau_grav came from whatever config was in force at the time;
    # recompute it from the current one.
    tg = [fnum(r, 'tau_grav_Nm') for r in dat if fnum(r, 'tau_grav_Nm') is not None]
    if tg and max(abs(x) for x in tg) > 100:
        print(f'  [warning] logged tau_grav reaches {max(tg):.0f} N m, so the config '
              'in force was wrong (units of R_LEVER_CM, most likely). Ignore that column')
    if C.R_LEVER_CM is not None and C.M_LEVER is not None:
        tr = []
        for r in dat:
            q, mp = fnum(r, 'q_deg'), fnum(r, 'payload_kg')
            if q is None or mp is None: continue
            tr.append((mp * C.G * C.R_LEVER + C.M_LEVER * C.G * C.R_LEVER_CM)
                      * math.cos(math.radians(q)))
        if tr:
            print(f'  gravity torque, recomputed  {st.mean(tr):.3f} N m '
                  f'({min(tr):.3f} to {max(tr):.3f})   * assumes q = 0 is horizontal')
    l1 = [fnum(r, 'load1_pct') for r in dat if fnum(r, 'load1_pct') is not None]
    l2 = [fnum(r, 'load2_pct') for r in dat if fnum(r, 'load2_pct') is not None]
    if l1 and l2:
        m1, m2 = _sign(0) * st.mean(l1), _sign(1) * st.mean(l2)
        print(f'  mean Present Load, joint frame  {m1:+.1f} / {m2:+.1f} %   '
              '* not a torque')
        if abs(m1) + abs(m2) < 5:
            print('  [warning] Load is near zero; the payload may not have been applied')
    # Newer logs carry dq12_deg; older ones need it computed from the positions.
    ids = list(C.SERVO_IDS)
    dq = []
    for r in dat:
        v = fnum(r, 'dq12_deg')
        if v is None and len(ids) > 1 and fnum(r, 'pos1') is not None \
                and fnum(r, 'pos2') is not None:
            try:
                import dxl_common as D
                v = (D.servo_to_joint_deg(ids[0], int(fnum(r, 'pos1'))) -
                     D.servo_to_joint_deg(ids[1], int(fnum(r, 'pos2'))))
            except SystemExit:
                v = None
        if v is not None: dq.append(v)
    if dq:
        print(f'  inter-servo angle  {dq[0]:+.2f} -> {dq[-1]:+.2f} deg  '
              f'(range {min(dq):+.2f} to {max(dq):+.2f}, spread {max(dq)-min(dq):.2f})')
        print('    * mixes backlash, zero offset and deflection. If one servo moved')
        print('      and the other did not, the run is void')
    cf = [fnum(r, 'comm_fail') for r in rows if fnum(r, 'comm_fail') is not None]
    if cf:
        print(f'  failed or rejected reads  {int(max(cf))}')


def analyse_static(rows, path):
    dat = [r for r in rows if fnum(r, 'q_err_deg') is not None]
    if not dat:
        print('  no data'); return
    t     = [fnum(r, 't_s') for r in dat]
    err   = [abs(fnum(r, 'q_err_deg')) for r in dat]
    tau   = [fnum(r, 'tau_grav_Nm') for r in dat]
    t1    = [fnum(r, 'temp1_C') for r in dat if fnum(r, 'temp1_C')]
    t2    = [fnum(r, 'temp2_C') for r in dat if fnum(r, 'temp2_C')]
    v1    = [fnum(r, 'volt1') for r in dat if fnum(r, 'volt1')]

    abort = next((r.get('abort_reason') for r in reversed(rows)
                  if r.get('abort_reason')), 'unknown')

    print(f'\n=== static hold -- {path.split("/")[-1]} ===')
    print(f'  target load      {dat[0].get("load_target_Nm")} N m   '
          f'payload {dat[0].get("payload_kg")} kg')
    print(f'  duration         {max(t):.1f} s')
    print(f'  ended            {abort}')
    print(f'  worst angle err  {max(err):.3f} deg   (tolerance +-{C.ANGLE_TOL_DEG})')
    print(f'  gravity torque   {st.mean(tau):.3f} N m  '
          f'({min(tau):.3f} to {max(tau):.3f})')
    if t1: print(f'  servo 1 temp     {t1[0]:.0f} -> {t1[-1]:.0f} C  (peak {max(t1):.0f})')
    if t2: print(f'  servo 2 temp     {t2[0]:.0f} -> {t2[-1]:.0f} C  (peak {max(t2):.0f})')
    if t1 and t2:
        d = max(abs(a - b) for a, b in zip(t1, t2))
        print(f'  temp difference  up to {d:.0f} C')
        print('    * cooling, unit-to-unit spread and friction all produce this;')
        print('      it is not by itself evidence that beta < 1')
    if v1: print(f'  voltage          {min(v1):.1f} to {max(v1):.1f} V')
    _common_checks(dat, rows)
    print(f'\n  verdict: {"completed" if abort == "completed" else "aborted -- " + abort}')


# ==================================================================
def analyse_dynamic(rows, path):
    dat = [r for r in rows if fnum(r, 'q_deg') is not None]
    if len(dat) < 10:
        print('  not enough samples'); return
    tgt = fnum(dat[0], 'speed_target_rad_s')
    t   = [fnum(r, 't_s') for r in dat]
    q   = [math.radians(fnum(r, 'q_deg')) for r in dat]

    # Evaluation window: the central EVAL_FRACTION of the 'const' phase.
    idx = [k for k, r in enumerate(dat) if r.get('phase') == 'const']
    if len(idx) < 6:
        print('  too few samples in the constant phase; record as not reached'); return
    n = len(idx)
    cut = int(n * (1 - C.EVAL_FRACTION) / 2)
    ev = idx[cut:n - cut] if n - 2 * cut >= 6 else idx
    i0, i1 = ev[0], ev[-1]

    dt = t[i1] - t[i0]
    if dt <= 0:
        print('  zero-length window'); return
    v_mean = abs(q[i1] - q[i0]) / dt      # signed the other way for --dir -1, so take the magnitude

    # ---- first half against second half --------------------------
    mid = ev[len(ev) // 2]
    v_a = abs(q[mid] - q[i0]) / (t[mid] - t[i0]) if t[mid] > t[i0] else 0
    v_b = abs(q[i1] - q[mid]) / (t[i1] - t[mid]) if t[i1] > t[mid] else 0
    uni = abs(v_a - v_b) / tgt * 100 if tgt else 0

    err_pct = (v_mean - tgt) / tgt * 100 if tgt else 0

    tau = [fnum(r, 'tau_grav_Nm') for r in dat[i0:i1 + 1]
           if fnum(r, 'tau_grav_Nm') is not None]
    vv  = [fnum(r, 'volt1') for r in dat[i0:i1 + 1] if fnum(r, 'volt1')]
    vv2 = [fnum(r, 'volt2') for r in dat[i0:i1 + 1] if fnum(r, 'volt2')]

    # When the encoder actually reached and left 95% of target. Reported only;
    # the verdict does not use it.
    ta, tb = None, None
    vc = [None] * len(dat)
    for k in range(1, len(dat) - 1):
        if t[k + 1] > t[k - 1]:
            vc[k] = (q[k + 1] - q[k - 1]) / (t[k + 1] - t[k - 1])
    sgn = 1 if q[-1] >= q[0] else -1
    hit = [k for k, v in enumerate(vc) if v is not None and sgn * v >= 0.95 * tgt]
    if hit:
        ta, tb = t[hit[0]], t[hit[-1]]
    rate = (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0
    # Joint speed as the Present Speed register sees it, for comparison
    sp = [abs(fnum(r, 'spd1_rpm')) for r in dat[i0:i1 + 1]
          if fnum(r, 'spd1_rpm') is not None]
    v_reg = (st.mean(sp) * 2 * PI / 60 / C.N_GEAR) if sp else None
    old_label = t[idx[0]] < 0.75 * (fnum(dat[0], 't_acc_s') or C.T_ACC)

    print(f'\n=== loaded traverse -- {path.split("/")[-1]} ===')
    print(f'  target load      {dat[0].get("load_target_Nm")} N m   '
          f'payload {dat[0].get("payload_kg")} kg')
    print(f'  target speed     {tgt:.3f} rad/s')
    print(f'  window           {t[i0]:.3f} to {t[i1]:.3f} s   ({len(ev)} samples)')
    print(f'  mean speed       {v_mean:.3f} rad/s   ({err_pct:+.1f} %)')
    print(f'  first / second   {v_a:.3f} / {v_b:.3f} rad/s   difference {uni:.1f} %')
    print(f'  gravity torque   {st.mean(tau):.3f} N m  '
          f'({min(tau):.3f} to {max(tau):.3f})')
    print(f'  sampling         {rate:.0f} Hz   (configured {C.LOG_HZ_DYN})')
    if old_label:
        print('  [warning] this log was written before the phase boundary was fixed, so'
              ' the const window may include part of the acceleration')
    if ta is not None:
        inside = ta <= t[i0] and t[i1] <= tb
        print(f'  encoder at 95% from {ta:.3f} to {tb:.3f} s   '
              f'window {"inside it" if inside else "extends outside -- check"}')
    else:
        print('  the encoder never reached 95% of target')
    if v_reg is not None:
        print(f'  Present Speed    {v_reg:.3f} rad/s  (encoder {abs(v_mean):.3f}, '
              f'{(v_reg/abs(v_mean)-1)*100 if v_mean else 0:+.1f} %)  * verdict uses the encoder')
    _common_checks(dat, rows)
    if vv:  print(f'  servo 1 voltage  {min(vv):.1f} to {max(vv):.1f} V')
    if vv2: print(f'  servo 2 voltage  {min(vv2):.1f} to {max(vv2):.1f} V')

    ok_speed = abs(err_pct) <= C.SPEED_TOL_PCT
    ok_uni   = uni <= C.UNIFORMITY_TOL_PCT
    print(f'\n  speed within +-{C.SPEED_TOL_PCT}%      : {"pass" if ok_speed else "fail"}')
    print(f'  uniformity within +-{C.UNIFORMITY_TOL_PCT}% : {"pass" if ok_uni else "fail"}')
    if ok_speed and ok_uni:
        print('  => constant speed accepted; compare against the model at this load')
    elif not ok_uni:
        print('  => constant speed not held; keep the trace and record it as such')
    else:
        print('  => speed not reached; record whether it was the acceleration or the')
        print('     constant phase that failed')
    print('\n  * look at the angle-time trace as well: the two halves can average')
    print('    the same and still oscillate in between')
    print(f'  * {C.UNIFORMITY_TOL_PCT}% is an operating criterion fixed for this study')


# ==================================================================
def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    paths = []
    for p in sys.argv[1:]:
        paths += sorted(glob.glob(p))
    if not paths:
        sys.exit('no such file')
    for p in paths:
        rows = load(p)
        if 'exp1' in p:   analyse_static(rows, p)
        elif 'exp2' in p: analyse_dynamic(rows, p)
        else:
            print(f'{p}: the file name has to start with exp1_ or exp2_')


if __name__ == '__main__':
    main()
