import time
import json
import bisect
import statistics
import ast
import math

#vortac_grabber V0.4

class VortacGrabber:
    def __init__(self, config):
        try:
            import numpy
        except:
            raise config.error("Angle calibration requires numpy module")

        self.printer = config.get_printer()
        self.angle_sensor_name = config.get("angleSensor")
        self.name = config.get_name()
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')

        self.steps_per_rev = config.getint('steps_per_rev', default=360)
        self.sample_count = config.getint('sample_count', default=180)
        self.speed = config.getint('speed', default=50)

        self.disengage_pos = config.getint('disengage_pos', default=0)
        self.engage_pos = config.getint('engage_pos', default=130)
        self.zero_pos_offset = config.getfloat('zero_pos_offset', default=0.0)

        self.params = get_params_dict(config)
        config.get_prefix_options('params_')

        self.tool_doc_test_code = self.gcode_macro.load_template(
            config, 'tool_doc_test_code', '')

        self.tool_doc_load = self.gcode_macro.load_template(
            config, 'tool_doc_load', '')

        self.tool_doc_unload = self.gcode_macro.load_template(
            config, 'tool_doc_unload', '')



        #TODO better way to grab angle sensor because now it has to be declared before vortac grabber
        try:
             self.angle_sensor = self.printer.lookup_object('angle {}'.format(self.angle_sensor_name))
        except Exception as e:
             self.angle_sensor = None
        self.angle_sensor = self.printer.lookup_object('angle {}'.format(self.angle_sensor_name))

        cal_str = config.get('lookup_table', None)
        if cal_str is not None:
            try:
                # convert JSON-String  in List von [trueAngle, measuredAngle]
                self.table = json.loads(cal_str)
            except json.JSONDecodeError as e:
                raise  config.error("Load Config Error: {}".format(e))

        gcode = self.printer.lookup_object('gcode')

        gcode.register_command("VORTAC_CALIBRATE", self.cmd_vortac_calibrate,
                               desc=self.cmd_vortac_calibrate_help)

        gcode.register_command("VORTAC_SET_ZERO", self.cmd_vortac_set_zero)


        gcode.register_command("VORTAC_MOVE", self.cmd_simple_move,
                               desc=None)

        gcode.register_command("VORTAC_SIMPLE_READ", self.cmd_simple_read,
                               desc=None)

        gcode.register_command("VORTAC_SET_SAVE_DOC_POS", self.cmd_set_save_doc_pos,
                               desc=None)

        gcode.register_command("VORTAC_TEST_DOC_POS", self.cmd_test_tool_doc_pos,
                               desc=None)

        gcode.register_command("VORTAC_TEST_LOAD", self.cmd_tool_doc_load,
                               desc=None)

        gcode.register_command("VORTAC_TEST_UNLOAD", self.cmd_tool_doc_unload,
                               desc=None)

        gcode.register_command("VORTAC_MESURE", self.cmd_tool_messure,
                               desc=None)

    cmd_vortac_calibrate_help = "Populates reference Value Table"

    def cmd_vortac_set_zero(self,gcmd):
        angle_result = self.cmd_simple_read(gcmd, include_offset=False)
        zero_offset = (angle_result % 360.0 + 360.0) % 360.0
        gcmd.respond_info("Set Z Offset: {}".format(zero_offset))
        cfg = self.printer.lookup_object('configfile')
        self.zero_pos_offset = zero_offset
        cfg.set(self.name, 'zero_pos_offset', zero_offset)

    def cmd_simple_read(self, gcmd, include_offset = True):
        angle_result = self.read_raw(gcmd)
        current_angle = self._measured_to_true(gcmd, angle_result, include_offset= include_offset)
        gcmd.respond_info("Current Raw Angle={} and looked up actual angle {}".format(angle_result, current_angle))
        return current_angle

    def cmd_tool_messure(self, gcmd):
        # Args & handles

        samples = gcmd.get_int('SAMPLES', default=90)
        speed = gcmd.get_float('SPEED', default=60.0)
        toolhead = self.printer.lookup_object('toolhead')
        force_mv = self.printer.lookup_object('force_move')
        stepper = self.angle_sensor.calibration.mcu_stepper

        step = 360.0 / float(samples)

        rawPairs = []
        lutPairs = []
        finalPairs = []

        gcmd.respond_info(f"{samples} Samples for Testing")

        for i in range(samples):
            force_mv.manual_move(stepper, +step, speed)
            toolhead.dwell(0.05)
            angle_result = self.read_raw(gcmd)
            pos = self._measured_to_true(gcmd, angle_result, include_offset= False)
            posoff = self._measured_to_true(gcmd, angle_result, include_offset= True)

            rawPairs.append((i*step, angle_result))
            lutPairs.append((i*step, pos))
            finalPairs.append((i * step, posoff))
            toolhead.wait_moves()

        gcmd.respond_info(f"rawPairs: {rawPairs}")
        gcmd.respond_info(f"lutPairs: {lutPairs}")
        gcmd.respond_info(f"finalPairs: {finalPairs}")
        return

    def _measured_to_true(self, gcmd, raw_deg, include_offset=True):
        """
        Map sensor reading (raw_deg) -> actual (linear) motor angle using LUT
        stored as [(actual_deg, raw_deg)]. Wrap-aware interpolation + zero offset.
        """
        import bisect, json

        def wrap360(v):  # robust wrap [0,360)
            return (float(v) % 360.0 + 360.0) % 360.0

        # 1) Fetch/parse LUT
        table = self.table

        if isinstance(table, str):  # if it came from config as JSON text
            try:
                table = json.loads(table)
            except Exception:
                table = None

        if not table or len(table) < 2:
            # No LUT -> just zero-adjust the raw reading
            out = wrap360(float(raw_deg) - (getattr(self, 'zero_pos_offset', 0.0) if include_offset else 0.0))
            return out

        # 2) Normalize to floats and wrap into 0..360
        norm = []
        for entry in table:
            try:
                a, r = entry[0], entry[1]  # (actual, raw)
                norm.append([wrap360(a), wrap360(r)])
            except Exception:
                pass
        if len(norm) < 2:
            out = wrap360(float(raw_deg) - (getattr(self, 'zero_pos_offset', 0.0) if include_offset else 0.0))
            return out

        # 3) Sort by measured/raw angle and dedupe identical bins
        norm.sort(key=lambda x: x[1])
        m, t = [], []
        for a, r in norm:
            if not m or abs(r - m[-1]) > 1e-9:
                m.append(r);
                t.append(a)
        n = len(m)
        if n < 2:
            out = wrap360(float(raw_deg) - (getattr(self, 'zero_pos_offset', 0.0) if include_offset else 0.0))
            return out

        # 4) Locate neighbors on circular domain
        x = wrap360(float(raw_deg))
        i = bisect.bisect_right(m, x)
        i0, i1 = (i - 1) % n, i % n
        m0, m1 = m[i0], m[i1]
        t0, t1 = t[i0], t[i1]

        # 5) Unwrap the segment across 360 for linear interpolation
        xm = x
        if m1 <= m0:
            m1 += 360.0
            if xm < m0:
                xm += 360.0
        if t1 <= t0:
            t1 += 360.0

        # 6) Interpolate (guard zero-length)
        if abs(m1 - m0) < 1e-12:
            out = t0
        else:
            frac = (xm - m0) / (m1 - m0)
            out = t0 + frac * (t1 - t0)

        # 7) Apply zero offset and wrap
        if include_offset:
            out -= getattr(self, 'zero_pos_offset', 0.0)
        return wrap360(out)

    def cmd_simple_move(self, gcmd, target_true=0.0, mode='shortest', threshold=8.0):
        """
        G-Code: VORTAC_MOVE TARGET=<Angle> [MODE=shortest|cw|ccw] [SPEED=<v>]
                [TOL=<deg>] [OVERSHOOT_MARGIN=<deg>] [GUARD=<deg>]
                [MIN_STEP=<deg>] [MAX_STEP=<deg>] [K=<gain>]
                [MAX_ITERS=<n>] [FINE_SPEED=<v>]
        """
        import math

        # --- helpers ---
        def wrap180(a):  # -> [-180, 180)
            return ((a + 180.0) % 360.0) - 180.0

        def err_by_mode(cur, tgt, m):
            if m in ('cw', 'clockwise'):
                return (tgt - cur) % 360.0  # [0..360)
            if m in ('ccw', 'counterclockwise', 'cclockwise'):
                return -((cur - tgt) % 360.0)  # (-360..0]
            return wrap180(tgt - cur)  # shortest

        # --- args ---
        tgt = gcmd.get_float('TARGET', default=target_true)
        mode = gcmd.get('MODE', default=mode).lower()
        speed = gcmd.get_float('SPEED', default=self.speed)
        tol = gcmd.get_float('TOL', default=threshold)

        # Tunables
        overshoot = gcmd.get_float('OVERSHOOT_MARGIN', default=2.0 * tol)
        guard = gcmd.get_float('GUARD', default=max(1.0, 1.5 * tol))
        min_step = gcmd.get_float('MIN_STEP', default=1.0)
        max_step = gcmd.get_float('MAX_STEP', default=3.0)
        kP = gcmd.get_float('K', default=0.8)
        max_iters = gcmd.get_int('MAX_ITERS', default=25)
        fine_speed = gcmd.get_float('FINE_SPEED', default=min(speed, 60.0))

        toolhead = self.printer.lookup_object('toolhead')
        force_move = self.printer.lookup_object('force_move').manual_move
        stepper = self.angle_sensor.calibration.mcu_stepper

        # --- read current TRUE angle (already LUT+offset corrected) ---
        cur_true = float(self.cmd_simple_read(gcmd))

        # --- coarse move: stop safely short of target ---
        err = err_by_mode(cur_true, tgt, mode)
        if abs(err) > tol:
            buf = max(guard, overshoot)
            sign = 1.0 if err >= 0.0 else -1.0
            coarse = err - sign * buf
            # enforce direction constraints
            if mode in ('cw', 'clockwise') and coarse < 0:   coarse = 0.0
            if mode in ('ccw', 'counterclockwise', 'cclockwise') and coarse > 0: coarse = 0.0
            if abs(coarse) > 0.01:
                force_move(stepper, coarse, speed)
                toolhead.wait_moves()

        # --- fine loop: never overshoot into the tolerance band ---
        stable = 0
        last_err = None
        for i in range(max_iters):
            cur_true = float(self.cmd_simple_read(gcmd))
            err = err_by_mode(cur_true, tgt, mode)

            # done? require two stable reads to debounce
            if abs(err) <= tol:
                stable += 1
                if stable >= 2:
                    gcmd.respond_info(f"Reached {cur_true:.2f}° (target {tgt:.2f}°, tol {tol}) in {i + 1} iters")
                    return
                toolhead.dwell(0.02)
                continue
            stable = 0

            # Maximum safe step that cannot cross into the tolerance band.
            # Leaves a 0.5*TOL cushion so we don't flirt with the boundary.
            max_safe = max(0.0, abs(err) - 0.5 * tol)

            # Proportional proposal, capped by MAX_STEP
            proposal = max(-max_step, min(max_step, kP * err))

            # Enforce direction constraint
            if mode in ('cw', 'clockwise') and proposal < 0:   proposal = abs(proposal)
            if mode in ('ccw', 'counterclockwise', 'cclockwise') and proposal > 0: proposal = -abs(proposal)

            # Clip to never exceed the safe step
            step = proposal
            if abs(step) > max_safe:
                step = math.copysign(max_safe, step)

            # Respect a minimum step when far away, but NEVER exceed max_safe
            if abs(step) < min_step and max_safe >= min_step:
                step = math.copysign(min_step, step)
            # If max_safe < min_step, we keep step ≤ max_safe (might be tiny) to avoid overshoot.

            if abs(step) < 0.01:  # too tiny to matter; wait for the next read
                toolhead.dwell(0.02)
                continue

            force_move(stepper, step, fine_speed)
            toolhead.wait_moves()

            # If error grew, soften gain a bit to damp oscillation
            if last_err is not None and abs(err) > abs(last_err) + 0.1:
                kP *= 0.7
            last_err = err

        gcmd.respond_info(f"Stopped after {max_iters} iters at {cur_true:.2f}° (target {tgt:.2f}°, err {err:.2f}°)")

    def cmd_vortac_calibrate(self, gcmd):
        """
        VORTAC_CALIBRATE [SAMPLES=<count>] [SPEED=<units/s>]
        Assumes msg['data'] == [(time, angle_rad_x1e4), ...] (Klipper scale).
        Saves one LUT: pairs_deg = [[true_deg, measured_deg], ...]
        """
        import math, json
        RAD2DEG = 180.0 / math.pi

        def cmean(vals):
            # circular mean in degrees (handles wrap at 360)
            sx = sum(math.cos(math.radians(v)) for v in vals)
            sy = sum(math.sin(math.radians(v)) for v in vals)
            if sx == 0 and sy == 0:
                return vals[-1]
            return (math.degrees(math.atan2(sy, sx)) % 360.0 + 360.0) % 360.0

        samples = gcmd.get_int('SAMPLES', default=90)
        speed = gcmd.get_float('SPEED', default=50.0)

        toolhead = self.printer.lookup_object('toolhead')
        force_mv = self.printer.lookup_object('force_move')
        stepper = self.angle_sensor.calibration.mcu_stepper

        step = 360.0 / float(samples)

        # --- collect bulk angle messages ---
        msgs, done = [], False

        def cb(msg):
            if done: return False
            if 'data' in msg: msgs.append(msg)
            return True

        try:
            try:
                cid = self.angle_sensor.add_client(cb)
            except TypeError:
                cid = cb;
                self.angle_sensor.add_client(cb)

            # Build time windows + bin map (forward then reverse)
            times, idx_map = [], []
            toolhead.dwell(0.05)

            # forward pass
            for i in range(samples):
                force_mv.manual_move(stepper, +step, speed)
                toolhead.dwell(0.10)
                end = toolhead.get_last_move_time()
                times.append((end - 0.09, end));
                idx_map.append(i)

            # reverse pass (mapped onto the same angle bins)
            for j in range(samples):
                force_mv.manual_move(stepper, -step, speed)
                toolhead.dwell(0.10)
                end = toolhead.get_last_move_time()
                times.append((end - 0.09, end));
                idx_map.append(samples - 1 - j)

            toolhead.wait_moves()
        finally:
            done = True
            for r in ('remove_client', 'del_client'):
                if hasattr(self.angle_sensor, r):
                    try:
                        getattr(self.angle_sensor, r)(cid)
                    except Exception:
                        pass
                    break

        # --- convert to degrees, flatten, sort by time ---
        pts = []
        for msg in msgs:
            for t, raw in msg['data']:  # (time, angle_rad_x1e4)
                ang_deg = (float(raw) / 10000.0) * RAD2DEG
                pts.append((float(t), ang_deg % 360.0))
        if not pts:
            raise self.printer.command_error("No valid samples from angle stream.")
        pts.sort(key=lambda x: x[0])

        # --- bin samples into angle bins across both passes ---
        bins = [[] for _ in range(samples)]
        w = 0
        for t, deg in pts:
            while w < len(times) and t > times[w][1]:
                w += 1
            if w >= len(times): break
            a, b = times[w]
            if a <= t <= b:
                bins[idx_map[w]].append(deg)

        missing = [i for i, b in enumerate(bins) if not b]
        if missing:
            raise self.printer.command_error(f"Incomplete data, empty bins: {missing}")

        measured = [cmean(b) for b in bins]  # averaged across fwd+rev

        # Build single LUT (true = commanded sample angle; measured = sensor)
        pairs_deg = [[round(i * step, 2), round(measured[i], 2)] for i in range(samples)]

        # Save
        cfg = self.printer.lookup_object('configfile')
        try:
            cfg.remove_section(self.name)
        except Exception:
            pass
        cfg.set(self.name, 'lookup_table', json.dumps(pairs_deg))

        gcmd.respond_info("Calibration OK. Saved single LUT 'pairs_deg' (true→measured).")

    def cmd_set_save_doc_pos(self, gcmd):
        # 1) Zugriff auf ToolHead
        toolhead = self.printer.lookup_object('toolhead')
        # 2) Aktuelle Positionen abfragen (X, Y, Z, E)
        pos = toolhead.get_position()
        # 3) configfile holen
        configfile = self.printer.lookup_object('configfile')

        # 4) locally save to make testing easier
        self.params['params_x_doc_pos'] = pos[0]
        self.params['params_y_doc_pos'] = pos[1]
        self.params['params_z_doc_pos'] = pos[2]

        #node test if saveing multiple different values if the last Value gets stored on saveConfig
        # 5) In die Config schreiben
        configfile.set(self.name, 'params_x_doc_pos', pos[0])
        configfile.set(self.name, 'params_y_doc_pos', pos[1])
        configfile.set(self.name, 'params_z_doc_pos', pos[2])
        # Optional: Save Config, z.B. mit
        # SAVE_CONFIG RESTART=0

    def cmd_test_tool_doc_pos(self, gcmd):
        self.run_gcode('vortac.tool_doc_test_code',self.tool_doc_test_code, {}, gcmd)

    def cmd_tool_doc_load(self, gcmd):
            self.run_gcode('vortac.tool_doc_load', self.tool_doc_load, {}, gcmd)

    def cmd_tool_doc_unload(self, gcmd):
        self.run_gcode('vortac.tool_doc_unload', self.tool_doc_unload, {}, gcmd)

    def run_gcode(self, name, template, extra_context, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        context = {
            **template.create_template_context(), #generates default context
            'vortac_grabber': self.get_status(curtime),
        }
        gcmd.respond_info(f"context: {context}")
        template.run_gcode_from_command(context)

    def get_status(self, eventtime):
        return {**self.params,
                'name': self.name,
                }

    def _build_read_command(self, addr):
        """
        addr: 14-Bit-Adresse (z.B. 0x3FFE oder 0x3FFF).
        Gibt eine Liste von zwei Bytes [MSB, LSB] zurück mit gesetztem R/W=1 und Paritätsbit.
        """
        cmd = (1 << 14) | (addr & 0x3FFF)  # R/W=1 + Adresse
        # Berechne Parität über Bits14..0 (cmd). Wenn Anzahl Einsen ungerade, setze Bit15.
        if bin(cmd).count("1") % 2 == 1:
            cmd |= (1 << 15)
        # Sonst Parität 0 lassen
        return [(cmd >> 8) & 0xFF, cmd & 0xFF]

    def _build_write_command(self, addr):
        """
        Baut den 16-Bit-SPI-Befehl zum Schreiben in ein 14-Bit-Register mit gerader Parität.
        addr: 14-Bit Register-Adresse (z.B. 0x0016 für ZPOSM, 0x0017 für ZPOSL).
        """
        cmd = (0 << 14) | (addr & 0x3FFF)  # R/W=0 → Write
        # Parität über Bits14..0
        if bin(cmd).count("1") % 2 == 1:
            cmd |= (1 << 15)
        return [(cmd >> 8) & 0xFF, cmd & 0xFF]

    def _build_write_data(self, value):
        """
        Baut die 16-Bit-Daten-Wort mit Parität vor dem Schreiben.
        value: 14-Bit Daten (0..0x3FFF)
        """
        data = value & 0x3FFF
        # Parität über Bits14..0
        if bin(data).count("1") % 2 == 1:
            data |= (1 << 15)
        return [(data >> 8) & 0xFF, data & 0xFF]

    def read_raw(self,gcmd):
        """
        Lese den Rohwert vom Sensor. Passe hier deinen SPI-Transfer oder sonstige Logik an.
        """

        #note converting commands
        #ANGLECOM = 0x3FFE
        #ANGLEUNC = 0x3FFF


        #self.angle_sensor.spi.spi_transfer([0xBF, 0xFE])  # Read ANGLEUNC
        # self.angle_sensor.spi.spi_transfer([0xC0, 0x18])  # Read Settings1


        cmd = self._build_read_command(0x3FFE)

        #NOTE this starts the read process
        self.angle_sensor.spi.spi_transfer(cmd)  # Read ANGLECOM

        data = self.angle_sensor.spi.spi_transfer([0x00, 0x00])['response']
        word = (data[0] << 8) | data[1]
        parity = (word >> 15) & 0x1
        error_flag = (word >> 14) & 0x1
        angle_raw = word & 0x3FFF  # Bits 13..0
        angle_deg = angle_raw * 360.0 / 16384.0

        return angle_deg

def get_params_dict(config):
    result = {}
    for option in config.get_prefix_options('params_'):
        try:
            result[option] = ast.literal_eval(config.get(option))
        except ValueError as e:
            raise config.error(
                "Option '%s' in section '%s' is not a valid literal" % (
                    option, config.get_name()))
    return result

def load_config(config):
    return VortacGrabber(config)

def load_config_prefix(config):
    return VortacGrabber(config)
