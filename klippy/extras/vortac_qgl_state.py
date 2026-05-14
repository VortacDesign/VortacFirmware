# Vortac QGL State — tracks the gantry's two valid orientations and lets you
# toggle between them on demand.
#
# A 4-Z-stepper gantry has two physically meaningful states once QGL has run:
#   * "flat"   — gantry parallel to the frame (the top-home reference)
#   * "tilted" — gantry parallel to the bed (QGL-applied state)
#
# Top-mounted docks are referenced to the frame, so dock approach must run in
# `flat`. Printing must run in `tilted` so first layers track the bed. This
# module captures the per-stepper deltas QGL applies and replays them forward
# (tilt) or inverse (flatten) so you can switch between states without
# re-probing.
#
# Mechanic:
#   * z_helper.adjust_steppers(adjustments, speed) is wrapped to capture each
#     incremental delta vector QGL passes; deltas accumulate into stored_deltas
#     across QGL's internal retries (typical 1-5) and across consecutive
#     QUAD_GANTRY_LEVEL invocations (refinement).
#   * The QUAD_GANTRY_LEVEL gcode command is wrapped: if invoked while the
#     gantry is currently flat, stored_deltas are zeroed before the underlying
#     QGL runs, so a fresh measurement isn't double-counted onto stale data.
#   * VORTAC_GANTRY_FLAT replays adjust_steppers(-stored) via the original
#     (un-wrapped) helper so stored_deltas are NOT modified.
#   * VORTAC_GANTRY_TILT replays adjust_steppers(+stored), same way.
#   * homing:home_rails_end on a Z rail clears stored_deltas (state is now
#     determined by the new home reference, any prior captures are invalid).
#
# State lives in RAM only. Power cycle, mid-print loss, or any fresh Z home
# invalidates it; user must re-run QGL.

class VortacQglState:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.flatten_speed = config.getfloat(
            'flatten_speed', 5.0, above=0.0)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'VORTAC_GANTRY_FLAT', self.cmd_VORTAC_GANTRY_FLAT,
            desc=self.cmd_VORTAC_GANTRY_FLAT_help)
        self.gcode.register_command(
            'VORTAC_GANTRY_TILT', self.cmd_VORTAC_GANTRY_TILT,
            desc=self.cmd_VORTAC_GANTRY_TILT_help)
        self.gcode.register_command(
            'VORTAC_QGL_STATUS', self.cmd_VORTAC_QGL_STATUS,
            desc=self.cmd_VORTAC_QGL_STATUS_help)

        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'homing:home_rails_end', self._handle_home_rails_end)

        # Populated at klippy:ready (QGL must be loaded by then)
        self.qgl = None
        self.z_helper = None
        self._original_adjust = None
        self._original_qgl_cmd = None
        self.stored_deltas = []
        self.state = "flat"

    def _handle_ready(self):
        try:
            self.qgl = self.printer.lookup_object('quad_gantry_level')
        except Exception:
            raise self.printer.config_error(
                "[vortac_qgl_state] requires [quad_gantry_level] "
                "to be configured")

        self.z_helper = self.qgl.z_helper
        n = len(self.z_helper.z_steppers)
        if n == 0:
            raise self.printer.config_error(
                "[vortac_qgl_state] z_helper.z_steppers is empty; "
                "klippy:connect did not populate it (check load order)")
        self.stored_deltas = [0.0] * n

        # Wrap z_helper.adjust_steppers — accumulate deltas, mark tilted
        original_adjust = self.z_helper.adjust_steppers
        self._original_adjust = original_adjust
        owner = self

        def wrapped_adjust(adjustments, speed):
            adj = list(adjustments)
            if len(adj) != len(owner.stored_deltas):
                raise owner.printer.command_error(
                    "vortac_qgl_state: adjust_steppers got %d items, "
                    "expected %d" % (len(adj), len(owner.stored_deltas)))
            for i, a in enumerate(adj):
                owner.stored_deltas[i] += float(a)
            owner.state = "tilted"
            return original_adjust(adjustments, speed)

        self.z_helper.adjust_steppers = wrapped_adjust

        # Wrap QUAD_GANTRY_LEVEL gcode command — zero stored deltas if a fresh
        # QGL is starting from flat state (so the new measurement starts clean)
        original_cmd = self.gcode.register_command('QUAD_GANTRY_LEVEL', None)
        if original_cmd is None:
            raise self.printer.config_error(
                "[vortac_qgl_state] QUAD_GANTRY_LEVEL was not registered "
                "before klippy:ready (load order issue)")
        self._original_qgl_cmd = original_cmd

        original_help = getattr(
            self.qgl, 'cmd_QUAD_GANTRY_LEVEL_help',
            "Quad gantry level (vortac_qgl_state tracked)")

        def wrapped_cmd(gcmd):
            if owner.state == "flat":
                # A fresh QGL run starting from frame-flat: any stored deltas
                # are about to be re-measured. Zero them so accumulation in
                # wrapped_adjust tracks the new measurement only.
                owner.stored_deltas = [0.0] * len(owner.stored_deltas)
            return original_cmd(gcmd)

        self.gcode.register_command(
            'QUAD_GANTRY_LEVEL', wrapped_cmd, desc=original_help)

    def _handle_home_rails_end(self, homing_state, rails):
        # Reset deltas and state when a Z rail finishes homing. Match by
        # primary stepper name prefix to avoid catching manual_stepper or
        # X/Y rails.
        for rail in rails:
            steppers = rail.get_steppers()
            if not steppers:
                continue
            name = steppers[0].get_name()
            if name.startswith('stepper_z'):
                self.stored_deltas = [0.0] * len(self.stored_deltas)
                self.state = "flat"
                return

    cmd_VORTAC_GANTRY_FLAT_help = (
        "Switch gantry to flat state (parallel to frame, top-home reference)")

    def cmd_VORTAC_GANTRY_FLAT(self, gcmd):
        if self._original_adjust is None:
            raise gcmd.error(
                "vortac_qgl_state not ready (klippy:ready not fired yet)")
        if self.state == "flat":
            gcmd.respond_info("vortac_qgl_state: already flat")
            return
        if not any(d != 0.0 for d in self.stored_deltas):
            gcmd.respond_info(
                "vortac_qgl_state: stored_deltas all zero, "
                "marking state=flat (no move)")
            self.state = "flat"
            return
        speed = gcmd.get_float('SPEED', self.flatten_speed, above=0.0)
        inverse = [-d for d in self.stored_deltas]
        # Bypass the wrapper so stored_deltas are not modified
        self._original_adjust(inverse, speed)
        self.state = "flat"
        gcmd.respond_info(
            "vortac_qgl_state: flattened (applied %s mm at %.2f mm/s)"
            % (["%.4f" % v for v in inverse], speed))

    cmd_VORTAC_GANTRY_TILT_help = (
        "Switch gantry to tilted state (parallel to bed, QGL-applied)")

    def cmd_VORTAC_GANTRY_TILT(self, gcmd):
        if self._original_adjust is None:
            raise gcmd.error(
                "vortac_qgl_state not ready (klippy:ready not fired yet)")
        if self.state == "tilted":
            gcmd.respond_info("vortac_qgl_state: already tilted")
            return
        if not any(d != 0.0 for d in self.stored_deltas):
            gcmd.respond_info(
                "vortac_qgl_state: stored_deltas all zero, "
                "marking state=tilted (no move)")
            self.state = "tilted"
            return
        speed = gcmd.get_float('SPEED', self.flatten_speed, above=0.0)
        forward = list(self.stored_deltas)
        # Bypass the wrapper so stored_deltas are not modified
        self._original_adjust(forward, speed)
        self.state = "tilted"
        gcmd.respond_info(
            "vortac_qgl_state: tilted (applied %s mm at %.2f mm/s)"
            % (["%.4f" % v for v in forward], speed))

    cmd_VORTAC_QGL_STATUS_help = (
        "Report current vortac_qgl_state: hooked? state, stored_deltas, speed")

    def cmd_VORTAC_QGL_STATUS(self, gcmd):
        hooked = self._original_adjust is not None
        deltas_str = ", ".join("%+.4f" % d for d in self.stored_deltas) \
            if self.stored_deltas else "(empty)"
        all_zero = (not self.stored_deltas) or \
            (not any(d != 0.0 for d in self.stored_deltas))
        gcmd.respond_info(
            "vortac_qgl_state:\n"
            "  hooked into QGL: %s\n"
            "  state:           %s\n"
            "  stored_deltas:   [%s] mm  %s\n"
            "  flatten_speed:   %.2f mm/s"
            % (hooked, self.state, deltas_str,
               "(all zero)" if all_zero else "",
               self.flatten_speed))

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'stored_deltas': list(self.stored_deltas),
            'flatten_speed': self.flatten_speed,
            'hooked': self._original_adjust is not None,
        }


def load_config(config):
    return VortacQglState(config)
