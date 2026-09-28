"""
Shared MX-106 helpers, Protocol 1.0.

Two things matter here:

  * both actuators get their goal in one SyncWrite packet, never one after the
    other -- see sync_goal() for why
  * each servo has its own THETA0 and SIGN, because they are mounted at
    different phases and in opposite directions

  pip install dynamixel-sdk
"""
import csv, math, os, sys, time
import config as C

# ---- Control table, Protocol 1.0 ----------------------------------
ADDR_MODEL_NUMBER    = 0    # 2B
ADDR_FIRMWARE        = 2    # 1B
ADDR_RETURN_DELAY    = 5    # 1B, units of 2 us
ADDR_TEMP_LIMIT      = 11   # 1B
ADDR_MAX_TORQUE      = 14   # 2B
ADDR_TORQUE_ENABLE   = 24   # 1B
# Corrected 2026-09-28.  These units (model 320, firmware 41) use the PID
# control table: 26 = D Gain, 27 = I Gain, 28 = P Gain.  Readback gives
# 26=0, 27=0, 28=32, which are the factory PID defaults for this model.
# Address 29 is not defined for it and returns whatever 28 holds.
# An earlier comment here claimed the compliance table on the grounds that 29
# read 32.  That was wrong: a larger value at 28 gives *less* droop, and at 8
# the joint could not track at all (0.003 rad/s), which is proportional-gain
# behaviour, not compliance-slope behaviour.
ADDR_CW_MARGIN       = 26   # 1B, dead band; left at 0
ADDR_CCW_MARGIN      = 27   # 1B
ADDR_CW_SLOPE        = 28   # 1B, = P Gain.  Old name kept so callers still work.
ADDR_CCW_SLOPE       = 29   # 1B
ADDR_D_GAIN          = ADDR_CW_MARGIN    # aliases, for readability at the call site
ADDR_I_GAIN          = ADDR_CCW_MARGIN
ADDR_P_GAIN          = ADDR_CW_SLOPE
ADDR_GOAL_POSITION   = 30   # 2B  -+ four consecutive bytes, so
ADDR_MOVING_SPEED    = 32   # 2B  -+ one SyncWrite covers both
ADDR_TORQUE_LIMIT    = 34   # 2B
ADDR_PRESENT_POSITION= 36   # 2B  ┐
ADDR_PRESENT_SPEED   = 38   # 2B  |  eight bytes in one BulkRead
ADDR_PRESENT_LOAD    = 40   # 2B  |  Load is not a torque measurement
ADDR_PRESENT_VOLTAGE = 42   # 1B  │
ADDR_PRESENT_TEMP    = 43   # 1B  ┘
ADDR_MOVING          = 46   # 1B
ADDR_GOAL_ACCEL      = 73   # 1B, units of 8.583 deg/s^2; 0 means unlimited

BULK_START, BULK_LEN = ADDR_PRESENT_POSITION, 8
SYNC_START, SYNC_LEN = ADDR_GOAL_POSITION, 4

DEG_PER_TICK   = 360.0 / 4096.0
RPM_PER_SPEED  = 0.1258   # calibrated 2026-09-20; the datasheet's
                          # nominal 0.114 is wrong for these units.  Measured
                          # with measure_noload.py --cal-speed at register
                          # values 100/200/300/400, both directions:
                          # 0.1266 / 0.1249 / 0.1257 / 0.1258, spread +-0.7%.
                          # Using 0.114 commands 10.3% slow and makes Present
                          # Speed read 9.3% below the encoder.
DEG_S2_PER_ACC = 8.583
PI = math.pi


def _patch_bulk_read_tx():
    """Work around a bug in dynamixel_sdk 4.1.0.

    GroupBulkRead.txPacket() calls bulkReadTx(port, param, length,
    fast_option) without checking the protocol, but
    Protocol1PacketHandler.bulkReadTx takes no fast_option and dies with
    "takes 4 positional arguments but 5 were given".
    """
    from dynamixel_sdk.protocol1_packet_handler import Protocol1PacketHandler
    orig = Protocol1PacketHandler.bulkReadTx
    if orig.__code__.co_argcount >= 5:
        return  # already patched, or this version is fine
    def bulkReadTx(self, port, param, param_length, fast_option=False):
        return orig(self, port, param, param_length)
    Protocol1PacketHandler.bulkReadTx = bulkReadTx


# ==================================================================
class Bus:
    def __init__(self, port=None, baud=None, ids=None):
        from dynamixel_sdk import (PortHandler, PacketHandler,
                                   GroupBulkRead, GroupSyncWrite)
        _patch_bulk_read_tx()
        self.port_h = PortHandler(port or C.PORT)
        self.pk     = PacketHandler(C.PROTOCOL)
        self.ids    = list(ids or C.SERVO_IDS)
        if not self.port_h.openPort():
            sys.exit(f'cannot open port: {port or C.PORT}')
        if not self.port_h.setBaudRate(baud or C.BAUDRATE):
            sys.exit('failed to set baudrate')

        self.bulk = GroupBulkRead(self.port_h, self.pk)
        for i in self.ids:
            if not self.bulk.addParam(i, BULK_START, BULK_LEN):
                sys.exit(f'bulk addParam failed for ID {i}')
        self.sync = GroupSyncWrite(self.port_h, self.pk, SYNC_START, SYNC_LEN)

    # ---- Single reads and writes ---------------------------------------
    # Returning v without checking the comm result records a silent 0 for a servo
    # that did not answer.  With the supply off on 2026-09-20 every register read
    # back as 0 and was briefly taken for EEPROM corruption.
    def r1(self, i, a):
        from dynamixel_sdk import COMM_SUCCESS
        v, c, e = self.pk.read1ByteTxRx(self.port_h, i, a)
        return None if c != COMM_SUCCESS else v

    def r2(self, i, a):
        from dynamixel_sdk import COMM_SUCCESS
        v, c, e = self.pk.read2ByteTxRx(self.port_h, i, a)
        return None if c != COMM_SUCCESS else v
    def w1(self, i, a, v):
        self.pk.write1ByteTxRx(self.port_h, i, a, int(v))
    def w2(self, i, a, v):
        self.pk.write2ByteTxRx(self.port_h, i, a, int(v))

    # ---- SyncWrite: one packet for both actuators ----------------------
    def sync_goal(self, goals, speed_reg):
        """goals = {id: tick}.  Writes Goal Position and Moving Speed together.

        Writing the two servos one after the other leaves 0.5-1 ms between
        them, and for that long the pair fights itself.  Anything that measures
        load sharing has to go through here.
        """
        self.sync.clearParam()
        for i in self.ids:
            g = int(goals[i]) & 0xFFFF
            s = int(speed_reg) & 0xFFFF
            param = [g & 0xFF, (g >> 8) & 0xFF, s & 0xFF, (s >> 8) & 0xFF]
            if not self.sync.addParam(i, param):
                sys.exit(f'sync addParam failed for ID {i}')
        from dynamixel_sdk import COMM_SUCCESS
        res = self.sync.txPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError(f'SyncWrite failed (comm={res})')

    # ---- BulkRead --------------------------------------------------
    def sample(self):
        """One BulkRead.  A servo that fails or returns nonsense comes back None.

        Each entry carries 't_read', the time.time() at which the read
        completed.  Use that as the timestamp: a time taken before the read is
        offset from the position by the USB latency, which is 16 ms by default
        on some hosts.
        """
        from dynamixel_sdk import COMM_SUCCESS
        res = self.bulk.txRxPacket()
        t_read = time.time()
        self.comm_fail = getattr(self, 'comm_fail', 0)
        out = {}
        if res != COMM_SUCCESS:
            self.comm_fail += 1
            for i in self.ids: out[i] = None
            out['t_read'] = t_read
            return out
        for i in self.ids:
            if not self.bulk.isAvailable(i, BULK_START, BULK_LEN):
                out[i] = None; self.comm_fail += 1; continue
            d = dict(
                pos       = self.bulk.getData(i, ADDR_PRESENT_POSITION, 2),
                speed_rpm = decode_speed(self.bulk.getData(i, ADDR_PRESENT_SPEED, 2)),
                load_pct  = decode_load(self.bulk.getData(i, ADDR_PRESENT_LOAD, 2)),
                volt      = self.bulk.getData(i, ADDR_PRESENT_VOLTAGE, 1) / 10.0,
                temp      = self.bulk.getData(i, ADDR_PRESENT_TEMP, 1))
            # A preliminary run on 2026-09-15 logged rows with V=0.0, T=0 and
            # Load=+-102.3; reject those rather than record them.
            if d['volt'] < 5.0 or d['temp'] <= 0 or not (0 <= d['pos'] <= 4095):
                out[i] = None; self.comm_fail += 1; continue
            out[i] = d
        out['t_read'] = t_read
        return out

    def torque(self, on, ids=None):
        """Write Torque Enable and check it took.  Raises on failure when enabling,
        warns when disabling -- a failed disable is worth knowing about but not
        worth crashing the cleanup path for."""
        from dynamixel_sdk import COMM_SUCCESS
        for i in (ids or self.ids):
            c, e = self.pk.write1ByteTxRx(self.port_h, i, ADDR_TORQUE_ENABLE,
                                          1 if on else 0)
            if c != COMM_SUCCESS or e != 0:
                msg = f'Torque Enable={int(on)} failed (ID {i}, comm={c}, err={e})'
                if on: raise RuntimeError(msg)
                print('  [warning] ' + msg)

    def set_common(self, torque_limit=None, accel_reg=None, return_delay=None):
        for i in self.ids:
            if return_delay is not None:
                self.w1(i, ADDR_RETURN_DELAY, return_delay)
            if torque_limit is not None:
                self.w2(i, ADDR_TORQUE_LIMIT, torque_limit)
            if accel_reg is not None:
                self.w1(i, ADDR_GOAL_ACCEL, accel_reg)

    def close(self):
        try: self.torque(False)
        except Exception: pass
        self.port_h.closePort()


# ==================================================================
def decode_speed(raw):
    mag = raw & 0x3FF
    return (-1 if (raw & 0x400) else 1) * mag * RPM_PER_SPEED

def decode_load(raw):
    """Present Load as a signed percentage.  Not a torque measurement -- useful
    for direction and relative magnitude only."""
    mag = raw & 0x3FF
    return (-1 if (raw & 0x400) else 1) * mag * 0.1


# ---- Angle conversion, per servo -----------------------------------
def _t0(i):
    v = C.THETA0.get(i)
    if v is None: sys.exit(f'fill in config.THETA0[{i}] -- run exp0_setup.py')
    return v

def _sg(i):
    return C.SIGN.get(i, +1)

def servo_to_joint_deg(i, pos_tick):
    """Raw servo position -> joint angle [deg], as the servo sees it.

    Backlash in the external mesh and frame deflection are not in this.
    """
    return _sg(i) * (pos_tick - _t0(i)) * DEG_PER_TICK / C.N_GEAR

def joint_to_servo_tick(i, q_deg):
    return int(round(_t0(i) + _sg(i) * q_deg * C.N_GEAR / DEG_PER_TICK))

def load_joint_dir(i, load_pct):
    """Present Load referred to the joint direction.  A pair mounted facing each
    other must show opposite raw signs when they are pushing together."""
    return _sg(i) * load_pct

def joint_goals(q_deg, ids=None):
    """Goal ticks for every servo, each through its own THETA0 and SIGN."""
    return {i: joint_to_servo_tick(i, q_deg) for i in (ids or C.SERVO_IDS)}


# ---- Speed and acceleration commands -------------------------------
def joint_rads_to_moving_speed(omega_joint):
    rpm = omega_joint * C.N_GEAR * 60.0 / (2 * PI)
    return max(1, min(1023, int(round(rpm / RPM_PER_SPEED))))

def joint_rads2_to_goal_accel(alpha_joint):
    a_deg = alpha_joint * C.N_GEAR * 180.0 / PI
    return max(1, min(254, int(round(a_deg / DEG_S2_PER_ACC))))


# ---- Torque model ---------------------------------------------------
def gravity_torque(q_deg, m_p=None, r=None, m_l=None, r_l=None):
    """(m_p g r + m_l g r_l) cos q   [N m]"""
    m_p = C.M_PAYLOAD  if m_p is None else m_p
    r   = C.R_LEVER    if r   is None else r
    m_l = C.M_LEVER    if m_l is None else m_l
    r_l = C.R_LEVER_CM if r_l is None else r_l
    return (m_p * C.G * r + m_l * C.G * r_l) * math.cos(math.radians(q_deg))

def inertia_about_joint(m_p=None, r=None, i_lever=None, a_disc=None):
    """m_p r^2 + I_disc,cm + I_lever,O   [kg m^2]"""
    m_p     = C.M_PAYLOAD      if m_p     is None else m_p
    r       = C.R_LEVER        if r       is None else r
    i_lever = C.I_LEVER_O      if i_lever is None else i_lever
    a_disc  = C.R_PAYLOAD_DISC if a_disc  is None else a_disc
    i = m_p * r * r + (i_lever or 0.0)
    if a_disc: i += 0.5 * m_p * a_disc * a_disc
    return i

def payload_for_target(tau_target, r=None, m_l=None, r_l=None):
    r   = C.R_LEVER    if r   is None else r
    m_l = C.M_LEVER    if m_l is None else m_l
    r_l = C.R_LEVER_CM if r_l is None else r_l
    return (tau_target - m_l * C.G * r_l) / (C.G * r)


# ---- Logging --------------------------------------------------------
class Logger:
    def __init__(self, path, fields):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.f = open(path, 'w', newline='', encoding='utf-8-sig')
        self.w = csv.DictWriter(self.f, fieldnames=fields)
        self.w.writeheader(); self.n = 0
    def write(self, row):
        self.w.writerow(row); self.n += 1
    def close(self):
        self.f.close()


def ts(): return time.strftime('%Y%m%d_%H%M%S')

def require_config(*names):
    miss = [n for n in names if getattr(C, n, None) is None]
    if miss: sys.exit('fill these in in config.py first: ' + ', '.join(miss))
    if any(C.THETA0.get(i) is None for i in C.SERVO_IDS):
        sys.exit('config.THETA0 needs an entry per servo -- run exp0_setup.py')
