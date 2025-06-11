class SensorOld:
    def __init__(self, config):
        try:
            import numpy
        except:
            raise config.error("Angle calibration requires numpy module")

        self.angle_sensor_name = config.get('angle_sensor', "my_angle_sensor")

        #TODO set or use calibrated values to set lock and open position ?
        pos_lock = config.get('pos_lock', None)
        pos_open = config.get('pos_open', None)

        # Register commands

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("VORTAC_SET_OPEN", self.cmd_set_open_pos,
                               desc=self.cmd_VORTAC_SET_OPEN_help)
        gcode.register_command("VORTAC_SET_CLOSE", self.cmd_set_close_pos,
                               desc=self.cmd_VORTAC_SET_CLOSE_help)

    cmd_VORTAC_SET_OPEN_help = "Set the Grabber position to open"
    cmd_VORTAC_SET_CLOSE_help = "Set the Grabber position to close"



    def angle_data_callback(self, data):
        if 'data' in data:
            latest_sample = data['data'][-1]  # Get the latest sample
            self.current_angle = latest_sample[1]
            self.log_info("sample length: {}".format(len(data['data'])))
            self.log_info(f"Current angle updated: {self.current_angle}")

    def cmd_set_open_pos(self, gcmd):        # Retrieve angle sensor instance if available
        if self.angle_sensor_name:
            try:
                self.angle_sensor = self.printer.lookup_object('angle {}'.format(self.angle_sensor_name))
                self.log_info(f"Angle sensor {self.angle_sensor_name} successfully loaded.")
            except Exception as e:
                self.angle_sensor = None
                self.log_info(f"Failed to load angle sensor {self.angle_sensor_name}: {e}")
        else:
            self.angle_sensor = None
            self.log_info("No angle sensor name provided.")

        if self.angle_sensor:
            gcmd.respond_info("should calibrate open Position from {}".format(self.angle_sensor))

            import numpy as np
            try:
                msgs = []
                is_finished = False

                def handle_batch(msg):
                    if is_finished:
                        return False
                    msgs.append(msg)
                    return True

                move = self.angle_sensor.printer.lookup_object('force_move').manual_move
                mcu_stepper = self.angle_sensor.calibration.mcu_stepper


                self.angle_sensor.spi.spi_transfer([0xFF, 0xFF])  # Read DIAAGC
                #self.angle_sensor.spi.spi_transfer([0xC0, 0x18])  # Read Settings1
                raw_samples =  self.angle_sensor.spi.spi_transfer([0x00, 0x00])
                data = raw_samples['response']
                data_msg = "".join(["%02x" % (x,) for x in data])
                gcmd.respond_info("result: {}".format(data_msg))
                combinedHex =  (data[0] << 8 | data[1]) & 0x7FFF
                combinedDecimal = int(combinedHex)
                gcmd.respond_info("result: {}".format(combinedDecimal))


                move(mcu_stepper, 360, 50)


                self.angle_sensor.spi.spi_transfer([0xFF, 0xFF])  # Read DIAAGC
                # self.angle_sensor.spi.spi_transfer([0xC0, 0x18])  # Read Settings1
                raw_samples = self.angle_sensor.spi.spi_transfer([0x00, 0x00])
                data = raw_samples['response']
                data_msg = "".join(["%02x" % (x,) for x in data])
                gcmd.respond_info("result: {}".format(data_msg))
                combinedHex = (data[0] << 8 | data[1]) & 0x7FFF
                combinedDecimal = int(combinedHex)
                gcmd.respond_info("result: {}".format(combinedDecimal))

                #return

                is_finished = True
                raw_samples = self.angle_sensor.bulk_queue.pull_queue()
                if not raw_samples:
                    return {}
                samples, error_count = self.angle_sensor._extract_samples(raw_samples)
                if not samples:
                    return {}


                # Extrahiere die sample-Werte
                samplespoints = [sample for time, sample in samples]

                # Berechne den Durchschnitt der sample-Werte
                average_sample = sum(samplespoints) / len(samplespoints)
                # Messwert
                messwert = average_sample

                gcmd.respond_info("average sample point: {}".format(average_sample))

                calibrate = self.angle_sensor.calibration.calibration

                # gcmd.respond_info("Loaded Calibration List: {}".format(calibrate))


                interp_bits = ANGLE_BITS - CALIBRATION_BITS
                interp_mask = (1 << interp_bits) - 1
                interp_round = 1 << (interp_bits - 1)

                for i, (samp_time, angle) in enumerate(samples):
                    bucket = (angle & 0xffff) >> interp_bits
                    cal1 = calibrate[bucket]
                    cal2 = calibrate[bucket + 1]
                    adj = (angle & interp_mask) * (cal2 - cal1)
                    adj = cal1 + ((adj + interp_round) >> interp_bits)
                    angle_diff = (adj - angle) & 0xffff
                    angle_diff -= (angle_diff & 0x8000) << 1
                    new_angle = angle + angle_diff

                gcmd.respond_info("Current Angle should be {}".format(new_angle %360))

            except Exception as e:
                gcmd.respond_info(f"Error processing batch: {e}")
        else:
            gcmd.respond_info("No angle sensor found.")

    def cmd_set_close_pos(self, gcmd):
        if self.angle_sensor:
            gcmd.respond_info("should calibrate open Position from {}".format(self.angle_sensor))
            try:
                result = self.angle_sensor._process_batch("asd")
                gcmd.respond_info("result: {}".format(result))
            except Exception as e:
                gcmd.respond_info(f"Error processing batch: {e}")
        else:
            gcmd.respond_info("No angle sensor found.")

    def log_info(self, message):
        # Function to log information
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info(message)

def load_config_prefix(config):
    return SensorOld(config)