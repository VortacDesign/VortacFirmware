import time
import json
import bisect
import math
import logging

# vortac_grabber V0.5
#   Phase 2 refactor: hardware-only (angle sensor, stepper, LUT, calibration,
#   closed-loop angle moves). Dock load/unload templates moved out of this
#   module; vortac_manager (Phase 5) calls engage()/disengage()/read_angle()
#   directly via the Python API exposed below.


class VortacGrabber:
    def __init__(self, config):
        try:
            import numpy  # noqa: F401  -- required by Klipper's angle sensor
        except Exception:
            raise config.error("Angle calibration requires numpy module")

        self.printer = config.get_printer()
        self.angle_sensor_name = config.get("angleSensor")
        self.name = config.get_name()

        self.steps_per_rev = config.getint('steps_per_rev', default=360)
        self.sample_count = config.getint('sample_count', default=180)
        self.speed = config.getint('speed', default=50)

        self.disengage_pos = config.getint('disengage_pos', default=0)
        self.engage_pos = config.getint('engage_pos', default=130)
        self.zero_pos_offset = config.getfloat('zero_pos_offset', default=0.0)

        # Absorb any stale params_* options left in saved-config blocks from
        # the pre-Phase-2 era (dock geometry now lives on [vortac_tool Tn]).
        # Marking them accessed prevents Klipper "unknown option" errors.
        config.get_prefix_options('params_')

        # Angle sensor must be declared before [vortac_grabber] in the config.
        self.angle_sensor = self.printer.lookup_object(
            'angle {}'.format(self.angle_sensor_name))

        cal_str = config.get('lookup_table', None)
        self.table = None
        if cal_str is not None:
            try:
                self.table = json.loads(cal_str)
            except json.JSONDecodeError as e:
                raise config.error("lookup_table parse error: {}".format(e))

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("VORTAC_CALIBRATE", self.cmd_vortac_calibrate,
                               desc=self.cmd_vortac_calibrate_help)
        gcode.register_command("VORTAC_SET_ZERO", self.cmd_vortac_set_zero)
        gcode.register_command("VORTAC_MOVE", self.cmd_simple_move)
        gcode.register_command("VORTAC_SIMPLE_READ", self.cmd_simple_read)
        gcode.register_command("VORTAC_MESURE", self.cmd_tool_messure)
        gcode.register_command("VORTAC_ENGAGE", self.cmd_engage,
                               desc="Move grabber to engage_pos "
                                    "(override with ANGLE=)")
        gcode.register_command("VORTAC_DISENGAGE", self.cmd_disengage,
                               desc="Move grabber to disengage_pos")

    # --------------------------------------------------------------------
    # Python API (called by vortac_manager and other Klipper extras).
    # gcmd is optional; when omitted, status messages go to klippy.log.
    # --------------------------------------------------------------------

    def engage(self, angle=None, gcmd=None):
        """Move grabber to engage_pos (or override `angle`). Returns final true angle."""
        target = float(angle) if angle is not None else float(self.engage_pos)
        return self._move_to(target, gcmd=gcmd)

    def disengage(self, gcmd=None):
        """Move grabber to disengage_pos. Returns final true angle."""
        return self._move_to(float(self.disengage_pos), gcmd=gcmd)

    def read_angle(self, gcmd=None):
        """Synchronous direct read; returns offset-applied true angle in [0, 360)."""
        raw = self.read_raw()
        return self._measured_to_true(raw, include_offset=True)

    # --------------------------------------------------------------------
    # gcode handlers
    # --------------------------------------------------------------------

    def cmd_engage(self, gcmd):
        angle = gcmd.get_float('ANGLE', default=None)
        return self.engage(angle=angle, gcmd=gcmd)

    def cmd_disengage(self, gcmd):
        return self.disengage(gcmd=gcmd)

    def cmd_vortac_set_zero(self, gcmd):
        raw = self.read_raw()
        true_no_off = self._measured_to_true(raw, include_offset=False)
        zero_offset = (true_no_off % 360.0 + 360.0) % 360.0
        gcmd.respond_info("Set zero offset: {:.4f}".format(zero_offset))
        cfg = self.printer.lookup_object('configfile')
        self.zero_pos_offset = zero_offset
        cfg.set(self.name, 'zero_pos_offset', zero_offset)

    def cmd_simple_read(self, gcmd):
        raw = self.read_raw()
        true_ = self._measured_to_true(raw, include_offset=True)
        gcmd.respond_info(
            "raw={:.3f}  true={:.3f}".format(raw, true_))
        return true_

    def cmd_simple_move(self, gcmd):
        target = gcmd.get_float('TARGET', default=0.0)
        kwargs = {
            'mode':         gcmd.get('MODE', default='shortest').lower(),
            'speed':        gcmd.get_float('SPEED', default=self.speed),
            'tol':          gcmd.get_float('TOL', default=2.0),
            'overshoot':    gcmd.get_float('OVERSHOOT_MARGIN', default=None),
            'guard':        gcmd.get_float('GUARD', default=None),
            'min_step':     gcmd.get_float('MIN_STEP', default=0.20),
            'max_step':     gcmd.get_float('MAX_STEP', default=6.0),
            'kP':           gcmd.get_float('K', default=1.4),
            'max_iters':    gcmd.get_int('MAX_ITERS', default=40),
            'fine_speed':   gcmd.get_float('FINE_SPEED', default=None),
            'n_reads':      gcmd.get_int('READS', default=2),
            'r_settle':     gcmd.get_float('READ_SETTLE', default=0.010),
            'near_sw':      gcmd.get_float('NEAR_SWITCH', default=None),
            'coarse_ratio': gcmd.get_float('COARSE_RATIO', default=0.93),
        }
        return self._move_to(target, gcmd=gcmd, **kwargs)

    def cmd_tool_messure(self, gcmd):
        """Sweep one full turn and report raw/lut/final at each bin (debug)."""
        samples = gcmd.get_int('SAMPLES', default=90)
        speed   = gcmd.get_float('SPEED', default=60.0)
        toolhead = self.printer.lookup_object('toolhead')
        force_mv = self.printer.lookup_object('force_move')
        stepper  = self.angle_sensor.calibration.mcu_stepper

        step = 360.0 / float(samples)
        rawPairs, lutPairs, finalPairs = [], [], []
        gcmd.respond_info(f"{samples} samples for testing")

        for i in range(samples):
            force_mv.manual_move(stepper, +step, speed)
            toolhead.dwell(0.05)
            toolhead.wait_moves()
            raw = self.read_raw()
            pos = self._measured_to_true(raw, include_offset=False)
            posoff = self._measured_to_true(raw, include_offset=True)
            rawPairs.append([i * step, raw])
            lutPairs.append([i * step, pos])
            finalPairs.append([i * step, posoff])

        gcmd.respond_info(f"rawPairs: {rawPairs}")
        gcmd.respond_info(f"lutPairs: {lutPairs}")
        gcmd.respond_info(f"finalPairs: {finalPairs}")

    # --------------------------------------------------------------------
    # Calibration (synchronous direct reads)
    # --------------------------------------------------------------------

    cmd_vortac_calibrate_help = (
        "Populate LUT via direct synchronous SPI reads (forward, multi-turn)")

    def cmd_vortac_calibrate(self, gcmd):
        """
        VORTAC_CALIBRATE [SAMPLES=<n>] [SPEED=<v>] [TURNS=<n>] [PHASE=<deg>]
                         [SETTLE=<s>] [READS=<n>] [READ_DWELL=<s>]

        Forward-only calibration. At every bin:
          1. manual_move forward by 360/SAMPLES
          2. dwell SETTLE then wait_moves so the move + dwell complete
             in wall-clock before any SPI read fires
          3. take READS direct SPI reads, circular-mean them
        Multi-turn with last-turn-wins so the saved LUT reflects the
        post-backlash steady-state reading at each bin.
        """
        def circ_mean(vals):
            sx = sum(math.cos(math.radians(v)) for v in vals)
            sy = sum(math.sin(math.radians(v)) for v in vals)
            if sx == 0.0 and sy == 0.0:
                return vals[-1]
            ang = math.degrees(math.atan2(sy, sx))
            return (ang % 360.0 + 360.0) % 360.0

        samples    = gcmd.get_int('SAMPLES', default=180, minval=4)
        speed      = gcmd.get_float('SPEED', default=40.0, above=0.0)
        turns      = gcmd.get_int('TURNS', default=2, minval=1)
        settle     = gcmd.get_float('SETTLE', default=0.10, minval=0.0)
        reads      = gcmd.get_int('READS', default=8, minval=1)
        read_dwell = gcmd.get_float('READ_DWELL', default=0.001, minval=0.0)
        step  = 360.0 / float(samples)
        phase = gcmd.get_float('PHASE', default=step / 2.0)

        toolhead   = self.printer.lookup_object('toolhead')
        force_move = self.printer.lookup_object('force_move').manual_move
        stepper    = self.angle_sensor.calibration.mcu_stepper

        # Phase pre-roll so the sensor seam falls between bins, not on a bin edge.
        if abs(phase) > 1e-9:
            force_move(stepper, phase, speed)
            toolhead.dwell(settle)
            toolhead.wait_moves()

        measured = [None] * samples
        total = samples * turns
        gcmd.respond_info(
            f"Calibrating: {samples} bins x {turns} turn(s) = {total} steps; "
            f"{reads} reads/pos, settle={settle}s")

        for k in range(total):
            force_move(stepper, +step, speed)
            toolhead.dwell(settle)
            toolhead.wait_moves()
            vals = []
            for _ in range(reads):
                vals.append(self.read_raw())
                if read_dwell > 0:
                    time.sleep(read_dwell)
            bin_idx = k % samples
            # Earlier turns (where backlash/settling skewed readings) are
            # overwritten by the final pass; last turn wins.
            measured[bin_idx] = circ_mean(vals)

        missing = [i for i, v in enumerate(measured) if v is None]
        if missing:
            raise self.printer.command_error(
                f"Incomplete data, empty bins: {missing}")

        # LUT entries are [true_deg, raw_deg]; 3 decimals = ~0.001° (well
        # below sensor LSB ~0.022° but plenty for interpolation precision).
        pairs = [[round(i * step, 3), round(measured[i], 3)]
                 for i in range(samples)]
        cfg = self.printer.lookup_object('configfile')
        cfg.set(self.name, 'lookup_table', json.dumps(pairs))
        self.table = pairs[:]

        gcmd.respond_info(
            f"Calibration OK. lookup_table saved ({samples} bins, last of "
            f"{turns} turns, {reads} reads/pos). Run SAVE_CONFIG.")

    # --------------------------------------------------------------------
    # Closed-loop move engine (drives engage/disengage and VORTAC_MOVE)
    # --------------------------------------------------------------------

    def _move_to(self, target_true, mode='shortest', speed=None, tol=2.0,
                 overshoot=None, guard=None, min_step=0.20, max_step=6.0,
                 kP=1.4, max_iters=40, fine_speed=None,
                 n_reads=2, r_settle=0.010, near_sw=None,
                 coarse_ratio=0.93, gcmd=None):
        def wrap180(a):
            return ((a + 180.0) % 360.0) - 180.0

        def err_by_mode(cur, tgt, m):
            if m in ('cw', 'clockwise'):
                return (tgt - cur) % 360.0
            if m in ('ccw', 'counterclockwise', 'cclockwise'):
                return -((cur - tgt) % 360.0)
            return wrap180(tgt - cur)

        def circ_median(vals):
            vals = [((v % 360.0) + 360.0) % 360.0 for v in vals]
            vals.sort()
            n = len(vals)
            if n == 1:
                return vals[0]
            best_span, best_med = 1e9, vals[0]
            for i in range(n):
                j = (i + n // 2) % n
                a, b = vals[i], vals[j]
                span = (b - a) if b >= a else (b + 360.0 - a)
                if span < best_span:
                    best_span = span
                    mid_idx = i + (n // 2)
                    best_med = vals[mid_idx] if mid_idx < n else vals[mid_idx - n]
            return best_med

        def read_true_quiet(n_, settle_):
            toolhead.dwell(settle_)
            toolhead.wait_moves()
            vs = []
            for _ in range(n_):
                vs.append(self._measured_to_true(self.read_raw(),
                                                  include_offset=True))
            return float(circ_median(vs))

        def respond(msg):
            if gcmd is not None:
                gcmd.respond_info(msg)
            else:
                logging.info("vortac_grabber: %s", msg)

        target_true = float(target_true)
        mode = (mode or 'shortest').lower()
        speed = float(speed) if speed is not None else float(self.speed)
        tol = float(tol)
        overshoot = float(overshoot) if overshoot is not None else max(0.5 * tol, 0.5)
        guard = float(guard) if guard is not None else max(1.0, tol)
        fine_speed = (float(fine_speed) if fine_speed is not None
                      else max(40.0, min(speed, 80.0)))
        near_sw = float(near_sw) if near_sw is not None else max(2.0 * tol, 2.0)

        toolhead = self.printer.lookup_object('toolhead')
        force_mv = self.printer.lookup_object('force_move').manual_move
        stepper = self.angle_sensor.calibration.mcu_stepper

        # Stage A: bold coarse hop with safety buffer (don't enter the band).
        cur_true = read_true_quiet(n_reads, r_settle)
        err = err_by_mode(cur_true, target_true, mode)

        if abs(err) > tol:
            proposed = err * coarse_ratio
            buf = max(guard, overshoot)
            max_allow = max(0.0, abs(err) - buf)
            step_mag = min(abs(proposed), max_allow)
            sgn = 1.0 if err >= 0.0 else -1.0
            coarse = sgn * step_mag
            if mode in ('cw', 'clockwise') and coarse < 0:
                coarse = 0.0
            if mode in ('ccw', 'counterclockwise', 'cclockwise') and coarse > 0:
                coarse = 0.0
            if abs(coarse) > 0.01:
                coarse = max(-10 * max_step, min(10 * max_step, coarse))
                force_mv(stepper, coarse, speed)
                toolhead.wait_moves()

        # Stage B: fast proportional → halving near target.
        stable = 0
        last_err = None
        last_step_mag = max_step

        for i in range(max_iters):
            cur_true = read_true_quiet(n_reads, r_settle)
            err = err_by_mode(cur_true, target_true, mode)

            if abs(err) <= tol:
                stable += 1
                if stable >= 2:
                    respond(f"Reached {cur_true:.2f}° (target {target_true:.2f}°, "
                            f"tol {tol}) in {i + 1} iters")
                    return cur_true
                toolhead.dwell(0.008)
                continue
            stable = 0

            max_safe = max(0.0, abs(err) - 0.5 * tol)

            if abs(err) > near_sw:
                proposal = kP * err
                proposal = max(-max_step, min(max_step, proposal))
                if abs(proposal) < min_step and max_safe >= min_step:
                    proposal = math.copysign(min_step, err)
            else:
                last_step_mag = max(min_step, 0.5 * last_step_mag)
                proposal = math.copysign(last_step_mag, err)

            if mode in ('cw', 'clockwise') and proposal < 0:
                proposal = abs(proposal)
            if mode in ('ccw', 'counterclockwise', 'cclockwise') and proposal > 0:
                proposal = -abs(proposal)

            step_ = proposal
            if abs(step_) > max_safe:
                step_ = math.copysign(max_safe, step_)
            if abs(step_) < 1e-3:
                toolhead.dwell(0.006)
                continue

            force_mv(stepper, step_, fine_speed)
            toolhead.wait_moves()

            if last_err is not None and abs(err) > abs(last_err) + 0.05:
                kP *= 0.75
            last_err = err

        respond(f"Stopped after {max_iters} iters at {cur_true:.2f}° "
                f"(target {target_true:.2f}°, err {err:.2f}°)")
        return cur_true

    # --------------------------------------------------------------------
    # LUT lookup: raw sensor degrees -> true motor angle (with optional offset).
    # Wrap-aware linear interpolation across the seam.
    # --------------------------------------------------------------------

    def _measured_to_true(self, raw_deg, include_offset=True):
        def wrap360(v):
            return (float(v) % 360.0 + 360.0) % 360.0

        table = self.table
        if isinstance(table, str):
            try:
                table = json.loads(table)
            except Exception:
                table = None

        if not table or len(table) < 2:
            return wrap360(float(raw_deg)
                           - (self.zero_pos_offset if include_offset else 0.0))

        norm = []
        for entry in table:
            try:
                a, r = entry[0], entry[1]  # (true, raw)
                norm.append([wrap360(a), wrap360(r)])
            except Exception:
                pass
        if len(norm) < 2:
            return wrap360(float(raw_deg)
                           - (self.zero_pos_offset if include_offset else 0.0))

        norm.sort(key=lambda x: x[1])
        m, t = [], []
        for a, r in norm:
            if not m or abs(r - m[-1]) > 1e-9:
                m.append(r)
                t.append(a)
        n = len(m)
        if n < 2:
            return wrap360(float(raw_deg)
                           - (self.zero_pos_offset if include_offset else 0.0))

        x = wrap360(float(raw_deg))
        i = bisect.bisect_right(m, x)
        i0, i1 = (i - 1) % n, i % n
        m0, m1 = m[i0], m[i1]
        t0, t1 = t[i0], t[i1]

        # Unwrap segment across the 360° seam for linear interpolation.
        xm = x
        if m1 <= m0:
            m1 += 360.0
            if xm < m0:
                xm += 360.0
        if t1 <= t0:
            t1 += 360.0

        if abs(m1 - m0) < 1e-12:
            out = t0
        else:
            frac = (xm - m0) / (m1 - m0)
            out = t0 + frac * (t1 - t0)

        if include_offset:
            out -= self.zero_pos_offset
        return wrap360(out)

    # --------------------------------------------------------------------
    # Status / introspection
    # --------------------------------------------------------------------

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'engage_pos': self.engage_pos,
            'disengage_pos': self.disengage_pos,
            'zero_pos_offset': self.zero_pos_offset,
            'has_lookup_table': bool(self.table),
        }

    # --------------------------------------------------------------------
    # AS5047D SPI helpers
    # --------------------------------------------------------------------

    def _build_read_command(self, addr):
        cmd = (1 << 14) | (addr & 0x3FFF)  # R/W=1 + address
        if bin(cmd).count("1") % 2 == 1:
            cmd |= (1 << 15)               # even parity
        return [(cmd >> 8) & 0xFF, cmd & 0xFF]

    def _build_write_command(self, addr):
        cmd = (0 << 14) | (addr & 0x3FFF)  # R/W=0 -> Write
        if bin(cmd).count("1") % 2 == 1:
            cmd |= (1 << 15)
        return [(cmd >> 8) & 0xFF, cmd & 0xFF]

    def _build_write_data(self, value):
        data = value & 0x3FFF
        if bin(data).count("1") % 2 == 1:
            data |= (1 << 15)
        return [(data >> 8) & 0xFF, data & 0xFF]

    def read_raw(self):
        """Synchronous SPI read of ANGLECOM (0x3FFE). Returns degrees in [0, 360)."""
        cmd = self._build_read_command(0x3FFE)
        self.angle_sensor.spi.spi_transfer(cmd)
        data = self.angle_sensor.spi.spi_transfer([0x00, 0x00])['response']
        word = (data[0] << 8) | data[1]
        # parity = (word >> 15) & 0x1
        # error_flag = (word >> 14) & 0x1
        angle_raw = word & 0x3FFF  # bits 13..0
        return angle_raw * 360.0 / 16384.0


def load_config(config):
    return VortacGrabber(config)


def load_config_prefix(config):
    return VortacGrabber(config)
