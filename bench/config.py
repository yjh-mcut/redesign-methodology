"""Bench configuration.  Change things here, not in the experiment scripts.

Every angle in this file, and everywhere downstream, is a *joint* angle,
measured after the external gear pair.
"""

# ---------------------------------------------------------------------------
# 1. Bus
# ---------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'      # ls -l /dev/ttyUSB* to find it
BAUDRATE = 1_000_000
PROTOCOL = 1.0             # MX-106 factory default; 2.0 has different addresses

SERVO_IDS = [9, 48]        # the hip-roll pair; use [1] for a single actuator

RETURN_DELAY = 0           # exp0_setup.py writes this, to cut bus latency

# ---------------------------------------------------------------------------
# 2. Mechanics
# ---------------------------------------------------------------------------
N_GEAR = 55 / 24           # external reduction = 2.2917

# Assembly phase and direction differ per servo; exp0_setup.py prints both.
# Leaving THETA0 as None makes require_config() refuse to run the experiments,
# which is deliberate: a wrong zero sends the lever somewhere unexpected.
SIGN = {9: +1, 48: -1}     # +1 if raw position rises with joint angle
THETA0 = {9: 1999, 48: 2082}   # raw position with the lever horizontal

R_LEVER = 0.300            # [m] joint axis to the centre of the weight box

# --- Equivalent mass of the bare lever -------------------------------------
# Measured 2026-09-20, with the lever *detached from the drive*.  Weighing it
# while mounted is useless: the gear train holds 0.61 N m (207 g equivalent),
# which swamps the reading.  An earlier figure of 0.514 kg was taken that way
# and discarded.
#   M_empty = M_total * r_cm / 0.300,  r_cm = axis hole to balance point
BOXES = {   # capacity, weighed total [kg], balance point [m], equivalent mass, count
    'A': dict(cap=32, M_total=0.5126, r_cm=0.212, M_empty=0.3622, n=33),
    'B': dict(cap=50, M_total=0.5456, r_cm=0.213, M_empty=0.3874, n=51),
    'C': dict(cap=68, M_total=0.5456, r_cm=0.214, M_empty=0.3892, n=69),
}
# Change M_LEVER whenever the box changes, or the target torque will be wrong.
M_LEVER = 0.3892           # [kg] currently fitted: box C  (A 0.3622, B 0.3874)
R_LEVER_CM = 0.300         # [m] M_LEVER is already referred to 300 mm, so this
                           # stays 0.300.  Using M_total with its own r_cm gives
                           # the same moment, as long as both are changed together.
I_LEVER_O = None           # unused; nothing calls inertia_about_joint()

WEIGHT_MASS = 0.10965      # [kg] mean of three bulk weighings, 2026-09-20:
                           # 32 -> 3513 g, 50 -> 5480 g, 68 -> 7450 g,
                           # i.e. 109.78 / 109.60 / 109.56 g, 0.32% under nominal
N_WEIGHTS = 69             # masses in the box for the run being set up.
                           # Pass --payload from PAYLOAD_KG rather than relying
                           # on this: the weighed totals are more accurate than
                           # WEIGHT_MASS * n by a couple of grams.
M_PAYLOAD = WEIGHT_MASS * N_WEIGHTS
R_PAYLOAD_DISC = None      # not a disc payload

JOINT_MIN_DEG = -36        # range of motion, measured in exp0_setup.py
JOINT_MAX_DEG = 41

# ---------------------------------------------------------------------------
# 3. Prespecified criteria
#    Fixed before the first valid run and copied into sheet_A_setup.csv.
# ---------------------------------------------------------------------------
PRESPEC_DATE = '2026-09-19'

HOLD_SECONDS = 600
ANGLE_TOL_DEG = 1.0
ANGLE_ABORT_DWELL_S = 0.5   # 0 aborts on the first sample outside tolerance
TEMP_ABORT_C = 70
TEMP_START_C = 40
TEMP_START_MATCH_C = 2

SPEED_TOL_PCT = 5.0         # mean speed over the evaluated window vs target
UNIFORMITY_TOL_PCT = 3.0    # first half vs second half of that window
EVAL_FRACTION = 0.60        # central fraction of the constant-speed phase

REPEATS = 3
Q0_TOL_DEG = 2.0            # how far from horizontal a static run may start.
                            # Load falls as cos q, and the SF 2 margin is only
                            # 1.44%, so a 10 deg tilt (-1.52%) would eat all of
                            # it.  2 deg costs 0.06%.
TORQUE_LIMIT = 1023         # address 34.  This is a PWM ceiling, so it scales
                            # speed as well as capacity: at 800 the no-load
                            # speed read 45.9 rpm against 57.3 at 1023.  Keep it
                            # at 1023 and keep the scripts' defaults matching.

SUPPLY_V_NOMINAL = 14.8     # [V] the voltage the manufacturer's figures are for
SUPPLY_V_TOL = 0.5          # [V] allowed deviation at the start of a run
SUPPLY_V_MAX_ABORT = 16.0   # [V] regenerative overvoltage alarm, same as the
                            # servo's own High Limit Voltage.  Driving the load
                            # downhill turns the actuators into generators and a
                            # switched-mode supply cannot sink the current: three
                            # runs on 2026-09-20 saw 21.5-24.1 V for 0.2 s.
COMM_ABORT_S = 0.5          # abort a static run after this long without a read
INTERSERVO_WARN_DEG = 2.0   # warn, do not abort, on divergence between servos

# ---------------------------------------------------------------------------
# 4. Logging
# ---------------------------------------------------------------------------
LOG_HZ_FAST = 50            # static holds
LOG_HZ_DYN = 500            # constant-rate traverses
LOG_HZ_SLOW = 1             # temperature
OUT_DIR = './data'
# exp2b_maxspeed.py is not throttled and logs at whatever the bus allows,
# which is about 1000 Hz for two servos read in one bulk transaction.

# ---------------------------------------------------------------------------
# 5. Test conditions
# ---------------------------------------------------------------------------
G = 9.81

TAU_REF = 11.59            # [N m] reference moment, third generation
TAU_150 = 17.38
TAU_SF2 = 23.18            # target capacity ratio of 2

# Test speeds are not fixed constants; plan_loads.py derives them from
# SUPPLY_V_NOMINAL and the actual load.  The values below are for 14.8 V and
# 11.59 N m.
SPEED_LOW = 0.374          # stance-sway rate from the frontal-plane model
SPEED_MID = 1.30           # control point, below the 11.59 N m loaded bound
SPEED_HIGH = 2.18          # for box A; 14% above its loaded bound of 1.913

# Test points, from the measured w_m = 5.9987 rad/s and the *measured* loads.
# The rule is 1.14 x the loaded bound.  Fixed 2026-09-20; 33/51/69 masses give
# the same loads for the static and the dynamic runs.
#   box  masses  torque    SF     loaded  no-load  test
#    A     33   11.7275  1.012    1.913    2.618   2.18
#    B     51   17.5904  1.518    1.560    2.618   1.78
#    C     69   23.3934  2.018    1.211    2.618   1.38
STAR_SPEED = {'A': 2.18, 'B': 1.78, 'C': 1.38}
LOAD_ACTUAL = {'A': 11.7275, 'B': 17.5904, 'C': 23.3934}
PAYLOAD_KG = {'A': 3.6226, 'B': 5.5896, 'C': 7.5596}

T_ACC = 0.20
T_CONST = 0.20             # superseded for the low-speed runs; see below

# --- Revised 2026-09-21 ----------------------------------------------------
# t_const is now the longest the range of motion allows at each speed.
# Reason: over 68 deg of travel, t_const = 0.20 s leaves an evaluation window of
# 0.12 s, only 1.3 times the servo's own speed oscillation period of 0.088 s, so
# the oscillation does not average out.  Compliance slopes 8/16/32/64/128 were
# all tried and none met the +-3% uniformity criterion (data/tune_20260921_*).
# Experiment 2 had produced no valid data at that point, so this is not a
# post-hoc adjustment.
T_CONST_BY_SPEED = {0.374: 2.60}   # 60.0 deg of travel, +-30.0 deg, 3.90 spare
#
# The high-speed points (2.18/1.78/1.38) were replaced by exp2b_maxspeed.py.
# Reason: the range of motion cannot give a constant-speed segment longer than
# 0.31 s at those speeds, and the servo takes longer than that to settle.  What
# separates the two models is the *maximum* speed, where the predictions differ
# by 37-116%, so +-3% precision is not needed.

# ---------------------------------------------------------------------------
# 6. MX-106 figures at 14.8 V, for reference
# ---------------------------------------------------------------------------
STALL_TORQUE_NM = 10.0
NO_LOAD_RPM = 55              # specification
NO_LOAD_RPM_MEAS = 57.3       # measured 2026-09-19: bare lever, both directions
                              # averaged, torque limit 1023
WM_MEAS = 5.9987              # [rad/s] the same figure; test speeds use this
TORQUE_PER_AMP = 1.587

# Per-voltage specification (ROBOTIS MX-106): {V: (stall N m, no-load rpm)}
MX106_SPEC = {11.1: (8.0, 41), 12.0: (8.4, 45), 14.8: (10.0, 55)}

# Model parameters, used by plan_loads.py
NA_MOTORS = 2                 # actuators per joint
BETA = 1.0                    # load sharing; 1.0 assumes an equal split
ETA_GEAR = 0.95               # external mesh efficiency
TAU_U = 5.4                   # [N m] usable output per actuator, from the
                              # manufacturer's performance graph at 12 V
