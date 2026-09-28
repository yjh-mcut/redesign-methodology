"""Live supply voltage, for trimming the bench supply.  Torque stays off.

    python volt_monitor.py                 # target 14.8 V
    python volt_monitor.py --target 14.8 --tol 0.5
    python volt_monitor.py --hz 2

Present Voltage (address 42) is one byte in units of 0.1 V, so that is as fine
as this gets.  Ctrl-C prints the min and max seen.
"""
import argparse, sys, time
import config as C
import dxl_common as D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=float, default=C.SUPPLY_V_NOMINAL or 14.8)
    ap.add_argument('--tol', type=float, default=C.SUPPLY_V_TOL)
    ap.add_argument('--hz', type=float, default=5.0)
    a = ap.parse_args()

    bus = D.Bus()
    lo = {i: None for i in bus.ids}
    hi = {i: None for i in bus.ids}
    n_fail = 0
    print(f'target {a.target:.1f} V  (tolerance +-{a.tol:.1f} V)   Ctrl-C to stop\n')
    try:
        while True:
            s = bus.sample()
            cells = []
            for i in bus.ids:
                if s[i] is None:
                    n_fail += 1
                    cells.append(f'ID{i}  ----  ')
                    continue
                v = s[i]['volt']
                lo[i] = v if lo[i] is None else min(lo[i], v)
                hi[i] = v if hi[i] is None else max(hi[i], v)
                d = v - a.target
                mark = 'OK ' if abs(d) <= a.tol else ('LOW' if d < 0 else 'HI ')
                cells.append(f'ID{i} {v:5.1f}V {d:+.1f} {mark}')
            worst = [s[i]['volt'] for i in bus.ids if s[i]]
            tail = ''
            if worst:
                w = min(worst, key=lambda v: -abs(v - a.target))
                tail = '  >>> in tolerance' if all(abs(v - a.target) <= a.tol for v in worst) else ''
            sys.stdout.write('\r  ' + '   '.join(cells) + tail + ' ' * 8)
            sys.stdout.flush()
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        print('\n')
        for i in bus.ids:
            if lo[i] is None:
                print(f'  ID {i}: read failed')
            else:
                print(f'  ID {i}: min {lo[i]:.1f} V   max {hi[i]:.1f} V')
        if n_fail:
            print(f'  {n_fail} failed reads')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
