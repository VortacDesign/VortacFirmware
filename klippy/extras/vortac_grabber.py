import time
import json
import bisect
import ast

#vortac_grabber V0.3

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
        self.zero_pos_offset = config.getint('zero_pos_offset', default=0)

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

        gcode.register_command("VORTAC_CALIBRATE_BULK", self.cmd_bulk_vortac_calibrate,
                               desc=self.cmd_vortac_calibrate_help)

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

    cmd_vortac_calibrate_help = "Populates reference Value Table"

    def cmd_simple_read(self,gcmd):
        angle_result = self.read_raw(gcmd)
        current_angle = self._measured_to_true(gcmd,angle_result)
        gcmd.respond_info("Current Raw Angle={} and looked up actual angle {}".format(angle_result, current_angle))
        return angle_result

    def _measured_to_true(self, gcmd, measured):
        """
        Interpoliert aus self.table den trueAngle zum gegebenen measuredAngle,
        berücksichtigt den Wrap-Around bei 360°.
        """
        # sortiere nach measuredAngle
        table = sorted(self.table, key=lambda x: x[1])
        measured_list = [m for _, m in table]
        true_list = [t for t, _ in table]

        measured = measured % 360.0

        # clamp (optional)
        if measured <= measured_list[0]:
            return true_list[0]
        if measured >= measured_list[-1]:
            return true_list[-1]

        # finde Index
        idx = bisect.bisect_left(measured_list, measured)
        i0 = (idx - 1) % len(measured_list)
        i1 = idx % len(measured_list)

        m0, m1 = measured_list[i0], measured_list[i1]
        t0, t1 = true_list[i0], true_list[i1]

        # Wrap-Around im measured-Bereich
        if m1 < m0:
            m1 += 360.0
            if measured < m0:
                measured += 360.0

        # Wrap-Around im true-Bereich
        if t1 < t0:
            t1 += 360.0

        # lineare Interpolation
        frac = (measured - m0) / (m1 - m0)
        true_interp = t0 + frac * (t1 - t0)

        # Ergebnis zurück in [0,360)
        return ((true_interp % 360.0) + self.zero_pos_offset)

    def cmd_simple_move(self, gcmd, target_true= 0.0, mode = 'shortest', threshold= 2.0):
        """
        G-Code: VORTAC_MOVE TARGET=<Winkel> [MODE=shortest|cw|ccw] [SPEED=<v>]
        Fährt nur im gewünschten Modus:
          - shortest  (Standard): kürzester Weg (±max 180°)
          - cw        : nur im Uhrzeigersinn (0→360, immer +)
          - ccw       : nur gegen den Uhrzeigersinn (360→0, immer -)
        """
        try:
            # 1) Argumente
            target_true = gcmd.get_float('TARGET', default=target_true)
            speed = gcmd.get_float('SPEED', default=self.speed)
            mode = gcmd.get('MODE', default=mode).lower()
            threshold = gcmd.get_float('THRESHOLD', default=threshold)

            # 2) Aktuellen wahren Winkel ermitteln
            measured = self.cmd_simple_read(gcmd)
            current_true = self._measured_to_true(gcmd, measured)
            toolhead = self.printer.lookup_object('toolhead')

            # 3) Delta berechnen je Modus
            if mode in ('cw', 'clockwise'):
                # nur vorwärts: modulo 360, immer positiv
                delta = (target_true - current_true) % 360.0
            elif mode in ('ccw', 'counterclockwise'):
                # nur rückwärts: modulo 360, immer negativ
                delta = -((current_true - target_true) % 360.0)
            else:
                # kürzester Weg: ± maximal 180°
                # (target - current + 180) mod 360 - 180 → in [-180,180]
                delta = ((target_true - current_true + 180.0) % 360.0) - 180.0

            reduced_move = delta*0.8

            gcmd.respond_info(
                f"MODE={mode}  Target={target_true:.2f}°  threshold={threshold}"
                f"Current={current_true:.2f}°  ΔMove={delta:.2f}°"
                f"actual move will be {reduced_move:.2f}°"
            )

            gcmd.respond_info(f"move exeeds threshold")
            move = self.angle_sensor.printer.lookup_object('force_move').manual_move
            mcu_stepper = self.angle_sensor.calibration.mcu_stepper
            move(mcu_stepper, reduced_move, speed)
            toolhead.wait_moves()

            if abs(reduced_move) > threshold:
                self.cmd_simple_move(gcmd, target_true=target_true, mode=mode, threshold=threshold)
            else:
                gcmd.respond_info(f"finished Move Commands!")


        except Exception as e:
            gcmd.respond_error(f"Error in simple_move: {e}")

    def cmd_bulk_vortac_calibrate(self, gcmd):
        """
              G-Code-Kommando: VORTAC_CALIBRATE [SAMPLES=<Anzahl>] [SPEED=<Wert>]
              Starte Kalibrierung. Führt auf einem eigenen Thread aus, damit Klippy nicht blockiert.
        """
        # Parameter auslesen
        sample_count = gcmd.get_int('SAMPLES', default=self.sample_count)
        speed = gcmd.get_int('SPEED', default=self.speed)
        # Start data collection

        """
            Starting Bulk Sensor Reading for Current Angle Sensor
            Collecting Results in msgs [] Bulk Reading continues untill handle_batch returns false
            this is the case when thin functions sets the is_finished flag to true
        """
        msgs = []
        is_finished = False


        def handle_batch(msg):
            if is_finished:
                return False
            msgs.append(msg)
            return True

        gcmd.respond_info("Startingh Bulk Mesorments for AngleSenseor")

        self.angle_sensor.add_client(handle_batch)
        steps_per_sample = float(self.steps_per_rev) / float(sample_count)
        toolhead = self.printer.lookup_object('toolhead')
        times = []

        #Queue move commands
        for i in range(sample_count):
            start_query_time = toolhead.get_last_move_time() + 0.050
            end_query_time = start_query_time + 0.050
            times.append((start_query_time, end_query_time))
            angle = (i * self.steps_per_rev / sample_count) % self.steps_per_rev
            gcmd.respond_info("i={}, angle={:.3f}".format(i, angle))
            move = self.angle_sensor.printer.lookup_object('force_move').manual_move
            self.angle_stepper = self.angle_sensor.calibration.mcu_stepper
            move(self.angle_stepper, steps_per_sample, speed)
            toolhead.dwell(0.150)

        toolhead.wait_moves()
        # Finish data collection
        is_finished = True
        gcmd.respond_info("Finished mesurements for AngleSenseor")
        gcmd.respond_info(f"Got {len(msgs)} and sample times {len(times)}")
        cal = {}
        step = 0

        gcmd.respond_info(f"Sample Times: {times}")

        for msg in msgs:
            for query_time, pos in msg['data']:
                # Add to step tracking
                while step < len(times) and query_time > times[step][1]:
                    step += 1
                if step < len(times) and query_time >= times[step][0]:
                    cal.setdefault(step, []).append(pos)

        gcmd.respond_info("Checked messages")
        gcmd.respond_info(f"cal dictionary: {cal}")

        if len(cal) != len(times):
            raise self.printer.command_error(
                "Failed calibration - incomplete sensor data")
        # fcal = {i: cal[i] for i in range(sample_count)}
        # rcal = {sample_count - i - 1: cal[i + sample_count] for i in range(sample_count)}
        #
        # gcmd.respond_info(f"should get fcal und rcal")
        # gcmd.respond_info(f"fcal: {fcal}")
        # gcmd.respond_info(f"rcal: {rcal}")

        return
        """
                Saves Results into config file as lookup_table
        """

        configfile = self.printer.lookup_object('configfile')
        configfile.remove_section(self.name)
        table_json = json.dumps(self.table)
        configfile.set(self.name, 'lookup_table', table_json)

    def cmd_vortac_calibrate(self, gcmd):
        """
              G-Code-Kommando: VORTAC_CALIBRATE [SAMPLES=<Anzahl>] [SPEED=<Wert>]
              Starte Kalibrierung. Führt auf einem eigenen Thread aus, damit Klippy nicht blockiert.
        """
        # Parameter auslesen
        sample_count = gcmd.get_int('SAMPLES', default=self.sample_count)
        speed = gcmd.get_int('SPEED', default=self.speed)
        # Evtl. Warnung, wenn Datei schon existiert?

        gcmd.respond_info(
            "Starte Winkelsensor-Kalibrierung: {} Samples, Speed {}".format(sample_count, speed
                                                                            ))
        # Starte Thread, damit Klippy-Scheduler nicht blockiert:

        self._do_calibration(gcmd, sample_count, speed)

        # Speichere in CSV: angle,raw

        configfile = self.printer.lookup_object('configfile')
        configfile.remove_section(self.name)
        table_json = json.dumps(self.table)
        configfile.set(self.name, 'lookup_table', table_json)

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

    def _do_calibration(self, gcmd, sample_count, speed):
        """
        Führt die eigentliche Kalibrierung aus:
        - Dreht in sample_count Schritten (jeweils relative Bewegungen) insgesamt 360°
        - Liest nach jedem Schritt den Sensorwert
        - Speichert angle,raw in self.table und in Datei
        """
        try:
            # Cleanup alte Tabelle
            self.table = []
            # Berechne Schrittzahl pro Sample:
            # steps_per_rev ist total steps für 360°, sample_count Punkte → steps_per_sample evtl. float
            steps_per_sample = float(self.steps_per_rev) / float(sample_count)
            #estimated_move_duration = float(speed / sample_count) * 0.2
            toolhead = self.printer.lookup_object('toolhead')
            gcmd.respond_info(f"started calibration with {sample_count} Samples")
            # Hauptloop:
            for i in range(sample_count):

                # Fahre relativen Schritt:
                # `manual_move(self.mcu, step, speed)`: Die Doku sagt: manual_move(mcuname, step_count, speed)
                # Übergebe hier den Stepperschrittnamen und step count. Der Typ (float) sollte funktionieren, Klipper rundet intern?
                raw = self.read_raw(gcmd)
                angle = (i * self.steps_per_rev / sample_count) % self.steps_per_rev
                self.table.append((angle, raw))
                gcmd.respond_info("i={}, angle={:.3f}, raw={}".format(i, angle, raw))

                # Move-Befehl:
                # force_move.manual_move: signature: manual_move(self, mcu_name, step_count, speed)
                move = self.angle_sensor.printer.lookup_object('force_move').manual_move
                self.angle_stepper = self.angle_sensor.calibration.mcu_stepper
                move(self.angle_stepper, steps_per_sample, speed)

                # Gibt Klipper kurz Zeit, um den move command zu beenden:

                toolhead.wait_moves()


            gcmd.respond_info("{}".format(self.table))
            gcmd.respond_info("Kalibrierung komplett: {} Einträge".format(len(self.table)))



        except Exception as e:
            # Fehlerbehandlung
            gcmd.respond_info("Fehler in Kalibrierung: {}".format(e))

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

    def _is_agc_stable(self,gcmd , n=5, tol=2):
        """
            Liest den sensor n mal aus um zu checken ob die werte constant sind
            als kleine "warmup" routine
        """
        vals = []
        for _ in range(n):
            cmd = self._build_read_command(0x3FFC)
            self.angle_sensor.spi.spi_transfer(cmd)
            resp = self.angle_sensor.spi.spi_transfer([0x00, 0x00])['response']
            word = (resp[0] << 8) | resp[1]
            # 4) Bits auslesen
            magl = bool(word & (1 << 11))
            magh = bool(word & (1 << 10))
            cof = bool(word & (1 << 9))
            lf = bool(word & (1 << 8))
            agc = word & 0xFF
            vals.append(agc)
            gcmd.respond_info(f"magnet low?: {magl}, magnet high ?:{magh}, cof: {cof}, lf:{lf}, bootsequence test Value {agc}")
            time.sleep(0.05)

        gcmd.respond_info(f"bootsequence test max distance: {max(vals) - min(vals)}")
        return max(vals) - min(vals) <= tol


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
