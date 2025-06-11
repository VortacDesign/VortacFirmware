from . import angle
import time


class VortacGrabber:
    def __init__(self, config):
        try:
            import numpy
        except:
            raise config.error("Angle calibration requires numpy module")

        gcode.register_command("VORTAC_CALIBRATE", self.cmd_set_close_pos,
                               desc=self.cmd_VORTAC_SET_CLOSE_help)

    cmd_VORTAC_CALIBRATE_help = "Populates reference Value Table"



def load_config_prefix(config):
    return VortacGrabber(config)
