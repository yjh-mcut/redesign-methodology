"""Read the control registers and firmware revision from both actuators.

    python read_gains.py

No payload needed and torque is never enabled.  Address 29 is not part of the
documented control table for this model; it is read anyway because an earlier
version of this code mistook its value for evidence of a compliance table.
"""
import config as C
import dxl_common as D

REGISTERS = [
    (0, 'Model Number (2B)', 2),
    (2, 'Firmware Version', 1),
    (26, 'addr 26  (control table: D Gain)', 1),
    (27, 'addr 27  (control table: I Gain)', 1),
    (28, 'addr 28  (control table: P Gain)', 1),
    (29, 'addr 29  (not defined for this model)', 1),
    (34, 'Torque Limit (2B)', 2),
]


def main():
    bus = D.Bus()
    try:
        for i in bus.ids:
            print(f'\n=== ID {i} ===')
            for addr, name, width in REGISTERS:
                value = bus.r2(i, addr) if width == 2 else bus.r1(i, addr)
                print(f'  {name:42s} = {value}')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
