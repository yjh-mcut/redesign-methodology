# Humanoid redesign with existing actuators

Analysis code, bench control code, and the complete raw logs behind

> J. Yang, *Humanoid Robot Redesign with Existing Actuators: Torque–Speed
> Constraints, Design Margins, and a Bench Test of the Screens*, submitted to
> *Applied Sciences*.

The paper asks what a legged platform's redesign can achieve once its actuators
are held fixed, states the answer as conditions checkable before hardware is
committed, and then measures how a real actuator group behaves against those
conditions.

Two things are here, and they are independent of each other:

* **`analysis/`** — the screening arithmetic for three hardware generations of
  an adult-size humanoid contested in the RoboCup Humanoid League between 2017
  and 2019. Nothing is measured; every quantity is computed from documented
  parameters. The platform has been retired.
* **`bench/`** — the acquisition and analysis code for the bench measurements,
  together with every log those runs produced, valid and invalid.

---

## Reproducing the analysis

```bash
pip install -r requirements.txt
cd analysis
python reproduce_analysis.py
```

This prints every analytical table in the paper — capacity ratios, per-axis
allocation, frontal-plane measures, which condition binds, the sensitivity
study, the screen-divergence comparison, and the failure record with its
exposure and intervals — and writes two figures to `analysis/outputs/`.

The failure counts it works from are the per-generation totals, which are in the
`FAILURES` block. The match-by-match reconstruction behind them is Supplementary
Table S1 of the paper, not part of this repository;
`analysis/failure_log_TEMPLATE.csv` gives the schema it follows and contains
example rows only.

Every model parameter sits in one `PARAMETERS` block at the top of the file.
Change any of them and rerun. That is the point of publishing this rather than
only the numbers: the paper's own conclusion is that the design clears its
target by 1.4%, and a 1.4% change in any one input decides it.

## Reproducing the bench figures

```bash
cd bench
python make_figures.py
```

Reads the logs in `bench/data/` and writes the three bench figures to
`bench/outputs/`. The effective stall torque is fitted here, from the same four
load points the paper reports; no fitted value is hard-coded.

To summarise a single run:

```bash
python analyze.py data/exp1_A-C-2_20260927_155011.csv
python analyze.py data/exp2_B-C-low-3_20260925_160611.csv
```

---

## The bench

Two Dynamixel MX-106 actuators drive a common shaft through a 24:55 external
spur pair, N = 2.2917 — the ratio fitted at the deployed hip roll. The lever is
300 mm from axis to load centre and carries one of three weight boxes. Supply is
a 600 W switched-mode unit at 14.8 V.

The actuators are of the same model and external ratio as the deployed hip-roll
axis, but they are not the units the retired platform carried.

| Condition | Box | Lever [kg] | Payload [kg] | Moment [N·m] | Capacity ratio |
|---|---|---|---|---|---|
| Empty lever | C | 0.3892 | — | 1.1454 | — |
| Loaded 1 | A | 0.3622 | 3.6226 | 11.7275 | 1.01 |
| Loaded 2 | B | 0.3874 | 5.5896 | 17.5904 | 1.52 |
| Loaded 3 | C | 0.3892 | 7.5596 | 23.3934 | 2.02 |

Payload masses were weighed in bulk rather than computed from a per-mass
nominal, and the lever's equivalent mass was measured with the lever detached
from the drive — mounted, the gear train holds 0.61 N·m and swamps the reading.

## Running the hardware

`bench/exp*.py` and the setup scripts need `dynamixel-sdk` and a servo bus. They
are published so the protocol is inspectable, not because anyone is expected to
rebuild the rig. The order is:

```
set_limits.py      write the EEPROM angle limits
exp0_setup.py      THETA0, SIGN, range of motion, register dump
measure_noload.py  estimate w_m at the supply voltage in use
exp1_static.py     sustained hold at a fixed angle
exp2_dynamic.py    constant-rate traverse
exp2b_maxspeed.py  maximum speed under load
```

Everything reads `bench/config.py`. Change settings there, not in the scripts.

Safety is not incidental here: the heaviest payload is 7.6 kg on a 300 mm lever
moving at up to 1.4 rad/s. Each script prompts before it enables torque and
tells you what to check first.

## Reading the run log

`bench/sheet_B_runlog.csv` lists every run attempted — 49 in total, 26 valid,
23 invalid or excluded, with the reason stated for each. Invalid runs are kept
deliberately, so the ratio of attempted to reported runs is visible. The
principal exclusions are six runs driven in the wrong direction (lowering the
load, which turns the actuators into generators and pushed the bus to 24.1 V),
three at a reduced torque limit, and the box C static runs that preceded the
thermal protocol.

`bench/sheet_A_setup.csv` holds the prespecified criteria, fixed 2026-09-19
before any valid run, and the measured setup constants.

## A note on the control registers

Both units report model 320, firmware 41, and read D = 0, I = 0, P = 32 at
addresses 26, 27 and 28 — the manufacturer's PID defaults for this model.
Address 29 is undocumented for it and returns whatever 28 holds.

An earlier version of this code described the firmware as using the compliance
control table, on the grounds that address 29 read 32. That was wrong and has
been corrected. The scripts record every register value they write and restore
the previous value of anything they change, but they do not log the value that
was there before, so the gains in force on each individual run cannot be read
out of this archive. Section 5.3 of the paper sets out what does and does not
follow from that.

---

## Layout

```
analysis/
  reproduce_analysis.py      every analytical table and figure, one file
  failure_log_TEMPLATE.csv   the schema of the per-match failure record, with
                             example rows only -- the reconstruction itself is
                             Supplementary Table S1 of the paper
bench/
  config.py                  all settings
  dxl_common.py              bus helpers, angle conversion, torque model
  set_limits.py exp0_setup.py plan_loads.py     setup
  measure_noload.py read_gains.py tune_gains.py characterisation
  exp1_static.py exp2_dynamic.py exp2b_maxspeed.py   the three experiments
  check_temp.py volt_monitor.py test_direction.py    small utilities
  analyze.py make_figures.py                     analysis
  sheet_A_setup.csv sheet_B_runlog.csv           records
  data/                      48 raw logs
docs/images/                 figures used in the paper
```

## Licence

MIT, see `LICENSE`. The logs in `bench/data/` are released under the same terms.
