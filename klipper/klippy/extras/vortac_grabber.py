from . import angle
import time
import threading


class VortacGrabber:
    def __init__(self, config):
        try:
            import numpy
        except:
            raise config.error("Angle calibration requires numpy module")

        self.printer = config.get_printer()
        self.angle_sensor_name = config.get("angleSensor")

        # Beispielhafte Keys: steps_per_rev, sample_count, speed, output_path, mcu_stepper
        self.steps_per_rev = config.getint('steps_per_rev', default=720)
        self.sample_count = config.getint('sample_count', default=72)
        self.speed = config.getint('speed', default=50)

        #TODO better way to grab angle sensor because now it has to be declared before vortac grabber
        self.angle_sensor = self.printer.lookup_object('angle {}'.format(self.angle_sensor_name))


        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("VORTAC_CALIBRATE", self.cmd_vortac_calibrate,
                               desc=self.cmd_vortac_calibrate_help)
        gcode.register_command("VORTAC_SET_ZERO", self.set_zero_position_volatile,
                               desc=None)
        gcode.register_command("VORTAC_SIMPLE_READ", self.cmd_simple_read,
                               desc=None)



        #NOTE Angle Sensor in mendentory and can just be set as nessesary Object
        # try:
        #     self.angle_sensor = self.printer.lookup_object('angle {}'.format(self.angle_sensor_name))
        # except Exception as e:
        #     self.angle_sensor = None
        #     #TODO if logging available give feedback don't raise error because Printer boot would fail
        #     #raise config.error(f"Failed to load angle sensor {self.angle_sensor_name}: {e}")

    cmd_vortac_calibrate_help = "Populates reference Value Table"

    def cmd_simple_read(self,gcmd):
        angle_result = self.read_raw(gcmd)
        gcmd.respond_info("angle={}".format(angle_result))

    def cmd_vortac_calibrate(self, gcmd):
        """
              G-Code-Kommando: ANGLE_CALIBRATE [START=<Grad>] [SAMPLES=<Anzahl>] [SPEED=<Wert>] [OUTPUT=<Pfad>]
              Starte Kalibrierung. Führt auf einem eigenen Thread aus, damit Klippy nicht blockiert.
              """
        # Parameter auslesen
        start_angle = gcmd.get_float('START', default=0.0)  # Falls du von einer definierten Anfangsposition startest
        sample_count = gcmd.get_int('SAMPLES', default=self.sample_count)
        speed = gcmd.get_int('SPEED', default=self.speed)
        # Evtl. Warnung, wenn Datei schon existiert?

        gcmd.respond_info(
            "Starte Winkelsensor-Kalibrierung: {} Samples, Speed {}".format(sample_count, speed
                                                                                       ))
        # Starte Thread, damit Klippy-Scheduler nicht blockiert:

        self._do_calibration(gcmd, start_angle, sample_count, speed)


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
            time.sleep(0.002)

        gcmd.respond_info(f"bootsequence test max distance: {max(vals) - min(vals)}")
        return max(vals) - min(vals) <= tol

    def _do_calibration(self, gcmd, start_angle, sample_count, speed):
        """
        Führt die eigentliche Kalibrierung aus:
        - Fährt start_angle an (falls nötig)
        - Dreht in sample_count Schritten (jeweils relative Bewegungen) insgesamt 360°
        - Liest nach jedem Schritt den Sensorwert
        - Speichert angle,raw in self.table und in Datei
        """

        max_attempts = 50
        for attempt in range(1, max_attempts + 1):
            if self._is_agc_stable(gcmd, n=5, tol=2):
                gcmd.respond_info(f"Warm-Up abgeschlossen nach {attempt} Versuchen.")
                break
            gcmd.respond_info(f"AGC noch instabil (Versuch {attempt}/{max_attempts}), warte...")
            time.sleep(0.01)
        else:
            # kommt hierher, wenn die Schleife nicht per break verlassen wurde
            gcmd.respond_error("Warm-Up nicht innerhalb der erlaubten Versuche stabil. Abbruch!")
            return  # oder raise RuntimeError("Warm-Up failed")

        gcmd.respond_info("started internal thread and funktion")
        try:
            # Cleanup alte Tabelle
            self.table = []
            # Berechne Schrittzahl pro Sample:
            # steps_per_rev ist total steps für 360°, sample_count Punkte → steps_per_sample evtl. float
            steps_per_sample = float(self.steps_per_rev) / float(sample_count)
            # Falls Startwinkel != 0: optional initialisieren. Hier angenommen: aktuell bei sensor-Winkel = start_angle.
            # Falls nicht, musst du Achse zuerst auf bekannten Referenzpunkt fahren. Das ist spezifisch und hier nicht implementiert.
            # Hauptloop:
            for i in range(sample_count):
                    # Fahre relativen Schritt:
                    # `manual_move(self.mcu, step, speed)`: Die Doku sagt: manual_move(mcuname, step_count, speed)
                    # Übergebe hier den Stepperschrittnamen und step count. Der Typ (float) sollte funktionieren, Klipper rundet intern?

                    # Move-Befehl:
                    # force_move.manual_move: signature: manual_move(self, mcu_name, step_count, speed)
                    move = self.angle_sensor.printer.lookup_object('force_move').manual_move
                    self.angle_stepper = self.angle_sensor.calibration.mcu_stepper
                    move(self.angle_stepper, steps_per_sample, speed)
                    # Gib Klipper kurz Zeit, um ruhig zu stehen:
                    # Man kann optional time.sleep abhängig von speed und Mechanikzeit
                    time.sleep(0.05)
                    raw = self.read_raw(gcmd)
                    angle = (start_angle + i * self.steps_per_rev / sample_count) % self.steps_per_rev
                    self.table.append((angle, raw))
                    gcmd.respond_info("i={}, angle={:.3f}, raw={}".format(i, angle, raw))
                # Hier evtl. optionale Averaging-Schleife, Retry bei Ausreißer etc.
            # Nach Loop: evtl. zurück zur Startposition drehen (optional).
            gcmd.respond_info("{}".format(self.table))
            gcmd.respond_info("Kalibrierung komplett: {} Einträge".format(len(self.table)))

            # Speichere in CSV: angle,raw

            #TODO SAVE VALUES
            # try:
            #     # Stelle sicher, dass Verzeichnis existiert
            #     dirname = os.path.dirname(output_path)
            #     if dirname and not os.path.isdir(dirname):
            #         os.makedirs(dirname, exist_ok=True)
            #     with open(output_path, 'w') as f:
            #         f.write("angle_deg,raw_value\n")
            #         for angle, raw in self.table:
            #             f.write("{:.6f},{}\n".format(angle, raw))
            #     gcmd.respond_info("Kalibriertabelle geschrieben nach: {}".format(output_path))
            # except Exception as e:
            #     gcmd.respond_info("Fehler beim Schreiben der Datei: {}".format(e))
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

    def set_zero_position_volatile(self, gcmd):
        # 1) Roh-Winkel (14 bit) lesen
        cmd = self._build_read_command(0x3FFE)  # CORDICANG
        self.angle_sensor.spi.spi_transfer(cmd)
        resp = self.angle_sensor.spi.spi_transfer([0,0])['response']
        raw14 = ((resp[0]<<8)|resp[1]) & 0x3FFF

        # 2) In ZPOS-Register soft schreiben
        zpos_m = (raw14 >> 6) & 0xFF
        zpos_l =  raw14       & 0x3F

        # ZPOSM (Registeradresse 0x0016)
        cmd_m  = self._build_write_command(0x0016)
        data_m = self._build_write_data(zpos_m)
        self.angle_sensor.spi.spi_transfer(cmd_m)
        self.angle_sensor.spi.spi_transfer(data_m)

        # ZPOSL (Registeradresse 0x0017)
        cmd_l  = self._build_write_command(0x0017)
        data_l = self._build_write_data(zpos_l)
        self.angle_sensor.spi.spi_transfer(cmd_l)
        self.angle_sensor.spi.spi_transfer(data_l)

        gcmd.respond_info(f"Soft-Zero-Offset gesetzt: raw={raw14}")

def load_config(config):
    return VortacGrabber(config)

#push script via
#right klick on klippy folder open in terminal
#pase