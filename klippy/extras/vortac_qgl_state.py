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
# TWO COORDINATE SYSTEMS (manage_coordinate_system, default on)
# The gantry orientation is only half of the frame change. Everything derived
# from probing the BED — the bed mesh and the gcode offset (tool offsets plus
# live babystepping) — is valid in the tilted, bed-referenced frame and wrong
# in the flat, frame-referenced one where the docks live:
#   * the bed mesh is a move transform on every G1, so it shifts the dock Z by
#     the (edge-clamped) mesh value at the dock XY, and a mesh probed
#     bed-tilted is meaningless once the gantry is frame-flat anyway;
#   * dock positions are taught in kinematic coordinates
#     (toolhead.get_position) but replayed as G1 moves, which the gcode offset
#     shifts — so no tool offset, hand-set SET_GCODE_OFFSET or babystep value
#     may reach a dock move.
# Hence FLAT enters the "dock" frame (mesh suspended, gcode offset captured
# and zeroed) and TILT restores the "print" frame (mesh back, offset back).
# This makes the toggle a complete frame switch for MANUAL use too — dock
# teaching via VORTAC_DOCK_CAL_SAVE right after a hand-triggered
# VORTAC_GANTRY_FLAT records clean kinematic positions.
#
# The probe's own z_offset is NOT part of this: it is not a move transform,
# it only applies while probing, and probing is refused frame-flat-with-a-tool
# by the manager's probe guard anyway. Nothing to switch there.
#
# vortac_manager delegates the frame switch to this module and takes the
# captured offset over with take_saved_offset() — a tool change must not
# restore the LEAVING tool's offsets on TILT, it applies the arriving tool's
# offsets (plus the carried babystep) itself in its final step.
#
# State lives in RAM only. Power cycle, mid-print loss, or any fresh Z home
# invalidates it; user must re-run QGL.

class VortacQglState:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.flatten_speed = config.getfloat(
            'flatten_speed', 5.0, above=0.0)
        # Switch bed mesh + gcode offset along with the gantry orientation.
        self.manage_frame = config.getboolean(
            'manage_coordinate_system', True)

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
        # Coordinate-system state: "print" (mesh + offsets live) or "dock"
        # (mesh suspended, offsets zeroed). Held on self so an aborted dock
        # operation can still be recovered by the next TILT.
        self.frame = "print"
        self.saved_offset = None    # (x, y, z) captured on entering "dock"
        self.saved_mesh = None      # bed mesh suspended on entering "dock"

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
            result = original_cmd(gcmd)
            # QGL ends bed-tilted by definition, so the frame follows: a QGL
            # run out of a hand-triggered FLAT leaves the print coordinate
            # system live again instead of a silently suspended mesh.
            owner.restore_print_frame(gcmd)
            return result

        self.gcode.register_command(
            'QUAD_GANTRY_LEVEL', wrapped_cmd, desc=original_help)

    def _handle_home_rails_end(self, homing_state, rails):
        # Reset deltas and state when a Z rail finishes homing. Match by
        # primary stepper name prefix to avoid catching manual_stepper or
        # X/Y rails.
        # The coordinate frame is deliberately NOT touched here: dropping a
        # suspended mesh or a captured offset would lose them for good, and
        # the next TILT hands them back regardless of how Z was homed.
        for rail in rails:
            steppers = rail.get_steppers()
            if not steppers:
                continue
            name = steppers[0].get_name()
            if name.startswith('stepper_z'):
                self.stored_deltas = [0.0] * len(self.stored_deltas)
                self.state = "flat"
                return

    # --------------------------------------------------------------------
    # Coordinate system (dock frame <-> print frame)
    # --------------------------------------------------------------------

    def enter_dock_frame(self, gcmd=None):
        """Suspend the bed mesh and zero the gcode offset, remembering both.
        Idempotent: a second call while already in the dock frame does not
        overwrite what was captured on the first."""
        if not self.manage_frame or self.frame == "dock":
            return
        gcode_move = self.printer.lookup_object('gcode_move')
        origin = gcode_move.get_status()['homing_origin']
        self.saved_offset = (origin[0], origin[1], origin[2])
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is not None:
            mesh = bed_mesh.get_mesh()
            if mesh is not None:
                # set_mesh() caches the gcode position around the transform
                # change, so dropping the mesh here is jump-free.
                self.saved_mesh = mesh
                bed_mesh.set_mesh(None)
        # Unconditional: whatever the offset was, dock moves run on zero.
        self.gcode.run_script_from_command(
            'SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0')
        self.frame = "dock"
        if gcmd is not None:
            gcmd.respond_info(
                "vortac_qgl_state: dock frame — mesh %s, gcode offset "
                "X=%.4f Y=%.4f Z=%.4f parked"
                % ("suspended" if self.saved_mesh is not None else "none",
                   self.saved_offset[0], self.saved_offset[1],
                   self.saved_offset[2]))

    def restore_print_frame(self, gcmd=None, apply_offset=True):
        """Hand the bed mesh back and (unless the caller owns the offset)
        re-apply the captured gcode offset."""
        if self.frame != "dock":
            return
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if self.saved_mesh is not None and bed_mesh is not None:
            # Never clobber a mesh that was loaded or probed while we were
            # in the dock frame — the newer one is what the user wants.
            if bed_mesh.get_mesh() is None:
                bed_mesh.set_mesh(self.saved_mesh)
        self.saved_mesh = None
        offset = self.saved_offset
        self.saved_offset = None
        self.frame = "print"
        applied = None
        if apply_offset and offset is not None:
            # ADD it back on top of whatever is live now instead of
            # overwriting: an offset taken out of the way must always come
            # back, and babystepping done inside the dock frame is a real
            # adjustment that has to survive too.
            gcode_move = self.printer.lookup_object('gcode_move')
            origin = gcode_move.get_status()['homing_origin']
            applied = (offset[0] + origin[0], offset[1] + origin[1],
                       offset[2] + origin[2])
            self.gcode.run_script_from_command(
                'SET_GCODE_OFFSET X=%.6f Y=%.6f Z=%.6f MOVE=0'
                % applied)
        if gcmd is not None:
            gcmd.respond_info(
                "vortac_qgl_state: print frame — mesh restored, gcode "
                "offset %s"
                % ("X=%.4f Y=%.4f Z=%.4f" % applied
                   if applied is not None
                   else "left to the caller"))

    def take_saved_offset(self):
        """Give the captured offset to a caller that will restore the gcode
        offset itself (vortac_manager applies the ARRIVING tool's offsets, so
        the leaving tool's must not come back on TILT). Returns (x, y, z) or
        None, and drops it here."""
        offset = self.saved_offset
        self.saved_offset = None
        return offset

    def in_dock_frame(self):
        return self.frame == "dock"

    cmd_VORTAC_GANTRY_FLAT_help = (
        "Switch gantry to flat state (parallel to frame, top-home reference)")

    def cmd_VORTAC_GANTRY_FLAT(self, gcmd):
        if self._original_adjust is None:
            raise gcmd.error(
                "vortac_qgl_state not ready (klippy:ready not fired yet)")
        # Frame first: an aborted move must not leave mesh/offsets live on a
        # half-flattened gantry, and the switch has to happen even when there
        # is no motion to do (already flat, or no deltas stored).
        self.enter_dock_frame(gcmd)
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
        elif not any(d != 0.0 for d in self.stored_deltas):
            gcmd.respond_info(
                "vortac_qgl_state: stored_deltas all zero, "
                "marking state=tilted (no move)")
            self.state = "tilted"
        else:
            speed = gcmd.get_float('SPEED', self.flatten_speed, above=0.0)
            forward = list(self.stored_deltas)
            # Bypass the wrapper so stored_deltas are not modified
            self._original_adjust(forward, speed)
            self.state = "tilted"
            gcmd.respond_info(
                "vortac_qgl_state: tilted (applied %s mm at %.2f mm/s)"
                % (["%.4f" % v for v in forward], speed))
        # Frame last: mesh and offsets only become valid once the gantry is
        # actually back on the bed reference. Runs on every path so a toggle
        # out of an aborted dock operation still recovers the print frame.
        self.restore_print_frame(gcmd)

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
            "  flatten_speed:   %.2f mm/s\n"
            "  frame:           %s%s\n"
            "  parked offset:   %s\n"
            "  suspended mesh:  %s"
            % (hooked, self.state, deltas_str,
               "(all zero)" if all_zero else "",
               self.flatten_speed,
               self.frame,
               "" if self.manage_frame else " (management disabled)",
               "X=%.4f Y=%.4f Z=%.4f" % self.saved_offset
               if self.saved_offset is not None else "none",
               "yes" if self.saved_mesh is not None else "no"))

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'stored_deltas': list(self.stored_deltas),
            'flatten_speed': self.flatten_speed,
            'hooked': self._original_adjust is not None,
            'frame': self.frame,
            'manages_frame': self.manage_frame,
            'saved_offset': list(self.saved_offset)
                            if self.saved_offset is not None else None,
            'mesh_suspended': self.saved_mesh is not None,
        }


def load_config(config):
    return VortacQglState(config)
