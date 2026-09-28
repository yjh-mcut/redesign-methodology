"""Read temperature and voltage a few times and exit.  Torque stays off.

    python check_temp.py

Used before a run to check the start-temperature ceiling.
"""
import config as C
import dxl_common as D

bus = D.Bus()
try:
    for _ in range(5):
        s = bus.sample()
        for i in bus.ids:
            if s[i] is None:
                print(f'  ID {i}: read failed')
            else:
                print(f'  ID {i}: temp={s[i]["temp"]} C  volt={s[i]["volt"]:.1f} V  '
                      f'pos={s[i]["pos"]}')
        print('-')
finally:
    bus.close()
