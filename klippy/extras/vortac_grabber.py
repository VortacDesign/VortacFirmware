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
        # Angle tolerance that counts as "arrived" for engage()/disengage()
        # (and the default TOL of VORTAC_MOVE). Missing it is a hard error,
        # not a warning: a key that did not turn rams the tool at the dock.
        self.move_tol = config.getfloat(
            'move_tol', default=2.0, above=0.0, maxval=30.0)
        self.zero_pos_offset = config.getfloat('zero_pos_offset', default=0.0)

        # Direction constraint for engage/disengage moves. 'cw' means
        # increasing true angle (same convention as VORTAC_MOVE MODE=cw).
        _modes = {'shortest': 'shortest', 'cw': 'cw', 'ccw': 'ccw'}
        self.engage_mode = config.getchoice(
            'engage_mode', _modes, default='shortest')
        self.disengage_mode = config.getchoice(
            'disengage_mode', _modes, default='shortest')

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

    def engage(self, angle=None, gcmd=None, require=True):
        """Move grabber to engage_pos (or override `angle`).

        Returns the final true angle. With require=True (default) a move
        that does not land within move_tol raises command_error — callers
        get success/failure directly instead of having to re-check the
        returned angle. require=False downgrades that to a report and is
        for manual/debug use only."""
        target = float(angle) if angle is not None else float(self.engage_pos)
        return self._move_to(target, mode=self.engage_mode, gcmd=gcmd,
                             tol=self.move_tol, require=require,
                             what='engage')

    def disengage(self, gcmd=None, require=True):
        """Move grabber to disengage_pos. See engage() for `require`."""
        return self._move_to(float(self.disengage_pos),
                             mode=self.disengage_mode, gcmd=gcmd,
                             tol=self.move_tol, require=require,
                             what='disengage')

    def read_angle(self, gcmd=None):
        """Synchronous direct read; returns offset-applied true angle in [0, 360)."""
        raw = self.read_raw()
        return self._measured_to_true(raw, include_offset=True)

    # --------------------------------------------------------------------
    # gcode handlers
    # --------------------------------------------------------------------

    def cmd_engage(self, gcmd):
        angle = gcmd.get_float('ANGLE', default=None)
        require = bool(gcmd.get_int('CHECK', 1, minval=0, maxval=1))
        return self.engage(angle=angle, gcmd=gcmd, require=require)

    def cmd_disengage(self, gcmd):
        require = bool(gcmd.get_int('CHECK', 1, minval=0, maxval=1))
        return self.disengage(gcmd=gcmd, require=require)

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
            'mode':      gcmd.get('MODE', default='shortest').lower(),
            'speed':     gcmd.get_float('SPEED', default=self.speed),
            'tol':       gcmd.get_float('TOL', default=self.move_tol),
            'backoff':   gcmd.get_float('BACKOFF', default=None),
            'guard_frac': gcmd.get_float('GUARD_FRAC', default=0.05),
            'max_iters': gcmd.get_int('MAX_ITERS', default=10),
            'n_reads':   gcmd.get_int('READS', default=2),
            'r_settle':  gcmd.get_float('READ_SETTLE', default=0.010),
            'require':   bool(gcmd.get_int('CHECK', 1, minval=0, maxval=1)),
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
        "Populate LUT via direct synchronous SPI reads "
        "(DIR=cw|ccw|both, default both)")

    def cmd_vortac_calibrate(self, gcmd):
        """
        VORTAC_CALIBRATE [SAMPLES=<n>] [SPEED=<v>] [TURNS=<n>] [PHASE=<deg>]
                         [SETTLE=<s>] [READS=<n>] [READ_DWELL=<s>]
                         [DIR=cw|ccw|both]

        At every bin:
          1. manual_move by ±360/SAMPLES
          2. dwell SETTLE then wait_moves so the move + dwell complete
             in wall-clock before any SPI read fires
          3. take READS direct SPI reads, circular-mean them
        TURNS passes per direction with last-turn-wins so each sweep
        reflects the post-backlash steady-state reading at each bin.

        DIR=cw sweeps with increasing true angle only (same convention as
        VORTAC_MOVE MODE=cw), ccw with decreasing only. DIR=both (default)
        sweeps cw then ccw, stores the per-bin circular midpoint, and
        reports the cw/ccw spread — a direct backlash measurement.
        """
        def circ_mean(vals):
            sx = sum(math.cos(math.radians(v)) for v in vals)
            sy = sum(math.sin(math.radians(v)) for v in vals)
            if sx == 0.0 and sy == 0.0:
                return vals[-1]
            ang = math.degrees(math.atan2(sy, sx))
            return (ang % 360.0 + 360.0) % 360.0

        def wrap180(a):
            return ((a + 180.0) % 360.0) - 180.0

        samples    = gcmd.get_int('SAMPLES', default=180, minval=4)
        speed      = gcmd.get_float('SPEED', default=40.0, above=0.0)
        turns      = gcmd.get_int('TURNS', default=2, minval=1)
        settle     = gcmd.get_float('SETTLE', default=0.10, minval=0.0)
        reads      = gcmd.get_int('READS', default=8, minval=1)
        read_dwell = gcmd.get_float('READ_DWELL', default=0.001, minval=0.0)
        dir_       = gcmd.get('DIR', default='both').lower()
        if dir_ not in ('cw', 'ccw', 'both'):
            raise gcmd.error("DIR must be cw, ccw or both")
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

        # Bin bookkeeping: after the pre-roll we sit at "bin -1"; every
        # +step lands on the next bin, every -step on the previous one, so
        # cw and ccw sweeps label the same physical positions identically.
        cur_bin = [-1]

        def sweep(direction):
            measured = [None] * samples
            for _k in range(samples * turns):
                force_move(stepper, direction * step, speed)
                toolhead.dwell(settle)
                toolhead.wait_moves()
                vals = []
                for _ in range(reads):
                    vals.append(self.read_raw())
                    if read_dwell > 0:
                        time.sleep(read_dwell)
                cur_bin[0] += direction
                # Earlier turns (where backlash/settling skewed readings)
                # are overwritten by the final pass; last turn wins.
                measured[cur_bin[0] % samples] = circ_mean(vals)
            missing = [i for i, v in enumerate(measured) if v is None]
            if missing:
                raise self.printer.command_error(
                    f"Incomplete data, empty bins: {missing}")
            return measured

        n_dirs = 2 if dir_ == 'both' else 1
        gcmd.respond_info(
            f"Calibrating: {samples} bins x {turns} turn(s) x {n_dirs} "
            f"direction(s); {reads} reads/pos, settle={settle}s")

        if dir_ == 'ccw':
            final = sweep(-1)
        elif dir_ == 'cw':
            final = sweep(+1)
        else:
            fw = sweep(+1)
            gcmd.respond_info("cw sweep done, starting ccw sweep")
            bw = sweep(-1)
            final = [circ_mean([fw[i], bw[i]]) for i in range(samples)]
            spread = [abs(wrap180(fw[i] - bw[i])) for i in range(samples)]
            gcmd.respond_info(
                "Backlash (cw vs ccw raw reading): mean {:.3f} deg, "
                "max {:.3f} deg".format(sum(spread) / samples, max(spread)))

        # LUT entries are [true_deg, raw_deg]; 3 decimals = ~0.001° (well
        # below sensor LSB ~0.022° but plenty for interpolation precision).
        pairs = [[round(i * step, 3), round(final[i], 3)]
                 for i in range(samples)]
        cfg = self.printer.lookup_object('configfile')
        cfg.set(self.name, 'lookup_table', json.dumps(pairs))
        self.table = pairs[:]

        gcmd.respond_info(
            f"Calibration OK. lookup_table saved ({samples} bins, "
            f"DIR={dir_}, {turns} turn(s)/direction, {reads} reads/pos). "
            f"Run SAVE_CONFIG.")

    # --------------------------------------------------------------------
    # Closed-loop move engine (drives engage/disengage and VORTAC_MOVE)
    # --------------------------------------------------------------------

    def _move_to(self, target_true, mode='shortest', speed=None, tol=None,
                 backoff=None, guard_frac=0.05, max_iters=10,
                 n_reads=2, r_settle=0.010, gcmd=None, require=True,
                 what='move'):
        """
        Closed-loop absolute move. The angle sensor is absolute ground
        truth, so every pass commands the FULL remaining error in a single
        manual_move and re-measures; lost steps or backlash simply show up
        in the next reading and are corrected by the next full pass.
        Typically converges in 1-2 moves.

        Direction-constrained modes (cw/ccw) stop short of the target on
        every pass by max(backoff, guard_frac * distance) — backoff
        defaults to 0.5*tol (clamped below tol), guard_frac to 5% of the
        remaining distance: mechanically overshooting the target there
        could not be corrected without a full extra revolution. A tiny
        overshoot that still lands inside tol is accepted as reached.
        Large constrained moves thus take one long pass plus one short
        finishing pass.

        Success/failure is explicit: reaching the target within `tol`
        returns the measured angle, and with require=True (default) NOT
        reaching it raises command_error instead of quietly returning the
        angle it ended up at. Everything mechanical downstream of a grabber
        move (entering a dock, letting go of a tool) depends on the key
        actually being where it was told to go, so the caller must not have
        to remember to re-check. `what` only names the move in that error.
        """
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
        tol = float(tol) if tol is not None else float(self.move_tol)
        constrained = mode in ('cw', 'clockwise',
                               'ccw', 'counterclockwise', 'cclockwise')
        backoff = float(backoff) if backoff is not None else 0.5 * tol
        backoff = min(backoff, 0.8 * tol)

        toolhead = self.printer.lookup_object('toolhead')
        force_mv = self.printer.lookup_object('force_move').manual_move
        stepper = self.angle_sensor.calibration.mcu_stepper

        stable = 0
        moves = 0
        cur_true = read_true_quiet(n_reads, r_settle)
        for i in range(max_iters):
            # Accept by shortest distance even in constrained modes: a tiny
            # mechanical overshoot inside tol is not worth going all the
            # way around again.
            if abs(wrap180(target_true - cur_true)) <= tol:
                stable += 1
                if stable >= 2:
                    respond(f"Reached {cur_true:.2f}° (target "
                            f"{target_true:.2f}°, tol {tol}) "
                            f"in {moves} move(s)")
                    return cur_true
                cur_true = read_true_quiet(n_reads, r_settle)
                continue
            stable = 0

            err = err_by_mode(cur_true, target_true, mode)
            step_ = err
            if constrained:
                back = max(backoff, guard_frac * abs(err))
                step_ = math.copysign(max(0.0, abs(err) - back), err)
            if abs(step_) >= 1e-3:
                force_mv(stepper, step_, speed)
                toolhead.wait_moves()
                moves += 1
            cur_true = read_true_quiet(n_reads, r_settle)

        err = wrap180(target_true - cur_true)
        msg = (f"Stopped after {max_iters} passes ({moves} moves) at "
               f"{cur_true:.2f}° (target {target_true:.2f}°, err "
               f"{err:.2f}°, tol {tol})")
        if require:
            raise self.printer.command_error(
                f"vortac_grabber: {what} failed — {msg}. Check the grabber "
                f"stepper and the angle sensor; VORTAC_SIMPLE_READ shows "
                f"the current angle.")
        respond(msg)
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

        if abs(m1 - m0) < 1e-12:
            out = t0
        else:
            frac = (xm - m0) / (m1 - m0)
            # Raw-adjacent LUT entries are one bin apart on the true circle in
            # either direction (sensor may run reversed vs. the stepper) —
            # take the shortest signed delta instead of assuming true
            # increases with raw.
            dt = ((t1 - t0 + 180.0) % 360.0) - 180.0
            out = t0 + frac * dt

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
