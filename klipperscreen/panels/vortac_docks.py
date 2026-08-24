# vortac_docks.py — KlipperScreen custom panel for the Vortac toolchanger.
#
# One screen for daily operation: the dock map (which tool sits in which
# dock, what the carriage holds, is detection fresh) plus the handful of
# actions that matter — tool change (tap an occupied dock tile), park,
# detect, engage/disengage, and saving a dock-calibration position.
# Commissioning tasks (grabber LUT, set zero, gantry flat/tilt) stay in the
# gcode menu / console on purpose.
#
# Deployment: install.sh symlinks this file into ~/KlipperScreen/panels/.
# Menu entry in ~/printer_data/config/KlipperScreen.conf:
#
#     [menu __main vortac]
#     name: Vortac
#     icon: extrude
#     panel: vortac_docks
#
# KlipperScreen shows its standard title bar with the back arrow for panels
# opened from a menu — no extra code needed to return to the main menu.
#
# Data source: vortac_manager / vortac_tool / vortac_qgl_state get_status()
# polled once per second via Moonraker "printer.objects.query" while the
# panel is visible. Deliberately NO reliance on KlipperScreen's internal
# object-subscription set, so the panel survives KlipperScreen updates that
# reshuffle subscriptions. Sense semantics (see vortac_tool.py): False =
# active-low pin asserted (docked / grabbed), True = idle, None = no reading.
#
# Dock supply (vortac_manager's dock_power): shown as a bolt glyph in each
# tile's header row, lit while the dock feeds its tool board. The 1 Hz poll
# only reliably shows the STEADY state — the handover window during a tool
# change lasts a second or two and will usually be missed, by design. What
# matters is the combination "dock holds a tool AND supply off": that strands
# the tool board (WS2812 keep their state across a klipper restart, so the
# board would be missing at the next mcu identify), so it gets a red tile and
# a tap action to switch the supply back on. Switching a dock OFF is never
# offered here — it costs an mcu and thus a klipper shutdown.

import logging
import os

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib, Pango

from ks_includes.screen_panel import ScreenPanel

# The panel file is symlinked into ~/KlipperScreen/panels/; realpath
# resolves back into the repo, where ../icons/ holds the custom SVGs.
ICON_DIR = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "icons"))

POLL_INTERVAL_MS = 1000
MAX_COLS = 4

CSS = b"""
button.vortac-dock { border-width: 2px; border-style: solid; border-radius: 8px; }
button.vortac-dock-occupied { border-color: #3d6ea5; }
button.vortac-dock-held     { border-color: #f0764f; }
button.vortac-dock-empty    { border-color: #4a5568; }
button.vortac-dock-unknown  { border-color: #a06a1f; }
button.vortac-dock-uncal    { border-color: #c9a75a; }
button.vortac-dock-fault    { border-color: #e5484d; border-width: 3px; }
"""


class Panel(ScreenPanel):
    def __init__(self, screen, title, **kwargs):
        title = title or "Vortac"
        super().__init__(screen, title)

        self.dock_count = None      # from [vortac_manager] via configfile
        self.tool_ids = []          # vortac_tool section names, from manager
        self.manager = {}           # last vortac_manager status
        self.qgl = {}               # last vortac_qgl_state status
        self.tool_status = {}       # tool_id -> last vortac_tool status
        self.tiles = {}             # dock_name -> widget dict
        self.poll_source = None

        provider = Gtk.CssProvider()
        try:
            provider.load_from_data(CSS)
            Gtk.StyleContext.add_provider_for_screen(
                Gdk.Screen.get_default(), provider,
                Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        except Exception:
            logging.exception("vortac_docks: css load failed (cosmetic only)")

        # --- header: held tool | gantry | detection freshness -------------
        self.lbl_holding = Gtk.Label(hexpand=True, halign=Gtk.Align.START)
        self.lbl_gantry = Gtk.Label(halign=Gtk.Align.CENTER)
        self.lbl_detect = Gtk.Label(halign=Gtk.Align.END)
        for lbl in (self.lbl_holding, self.lbl_gantry, self.lbl_detect):
            lbl.set_ellipsize(Pango.EllipsizeMode.END)
        header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        header.add(self.lbl_holding)
        header.add(self.lbl_gantry)
        header.add(self.lbl_detect)

        # --- dock grid (tiles are built once dock_count is known) ---------
        self.grid = Gtk.Grid(column_homogeneous=True, row_homogeneous=True,
                             column_spacing=8, row_spacing=8,
                             hexpand=True, vexpand=True)
        self.lbl_waiting = Gtk.Label(label="Waiting for vortac_manager status…")
        self.grid.attach(self.lbl_waiting, 0, 0, 1, 1)

        # --- action bar ----------------------------------------------------
        actions = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL,
                          spacing=8, homogeneous=True)
        # Contextual: park icon while holding, load icon when the carriage
        # is empty (insensitive/greyed when the target is ambiguous — then
        # the dock tiles are the way to load). Icon-only on purpose; the
        # confirm dialog names the concrete tool.
        self.pix_park = self._load_pixbuf("vortac_park.svg")
        self.pix_load = self._load_pixbuf("vortac_load.svg")
        self.btn_primary = Gtk.Button()
        self.btn_primary.set_can_focus(False)
        if self.pix_park is not None and self.pix_load is not None:
            self.img_primary = Gtk.Image.new_from_pixbuf(self.pix_park)
            self.lbl_primary = Gtk.Label(label="Park")
            pbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2,
                           halign=Gtk.Align.CENTER, valign=Gtk.Align.CENTER)
            pbox.add(self.img_primary)
            pbox.add(self.lbl_primary)
            self.btn_primary.add(pbox)
        else:
            self.img_primary = None
            self.lbl_primary = None
            self.btn_primary.set_label("Park / Load Tool")
        self.btn_primary.connect("clicked", self._primary_clicked)
        actions.add(self.btn_primary)
        actions.add(self._action_button(
            "refresh", "Detect", None, "VORTAC_DETECT"))
        actions.add(self._icon_button(
            "vortac_engage.svg", "Engage",
            "Rotate the grabber to the engage position?", "VORTAC_ENGAGE"))
        actions.add(self._icon_button(
            "vortac_disengage.svg", "Disengage",
            "Rotate the grabber to the disengage position?",
            "VORTAC_DISENGAGE"))
        actions.add(self._icon_button(
            "vortac_dock_save.svg", "Save Dock",
            "Save the current XYZ as the selected dock's hooked position?\n"
            "(Select a dock first by tapping an occupied tile's tool while "
            "it is detected, or via VORTAC_SELECT_DOCK.)",
            "VORTAC_DOCK_CAL_SAVE"))

        self.content.add(header)
        self.content.add(self.grid)
        self.content.add(actions)
        self.content.show_all()

    # ------------------------------------------------------------------
    # lifecycle: poll only while visible
    # ------------------------------------------------------------------

    def activate(self):
        self._request_config()
        self._request_status()
        if self.poll_source is None:
            self.poll_source = GLib.timeout_add(
                POLL_INTERVAL_MS, self._request_status)

    def deactivate(self):
        if self.poll_source is not None:
            GLib.source_remove(self.poll_source)
            self.poll_source = None

    # ------------------------------------------------------------------
    # moonraker queries
    # ------------------------------------------------------------------

    def _request_config(self):
        # one-shot: dock_count from the parsed config
        self._screen._ws.send_method(
            "printer.objects.query",
            {"objects": {"configfile": ["settings"]}},
            self._config_cb)

    def _request_status(self):
        objects = {"vortac_manager": None, "vortac_qgl_state": None}
        for tid in self.tool_ids:
            objects["vortac_tool %s" % tid] = None
        self._screen._ws.send_method(
            "printer.objects.query", {"objects": objects}, self._status_cb)
        return True     # keep the GLib timeout alive

    @staticmethod
    def _extract_status(response):
        # send_method callbacks may receive the raw response or the
        # unwrapped result depending on KlipperScreen version — accept both.
        if not isinstance(response, dict):
            return None
        result = response.get("result", response)
        if not isinstance(result, dict):
            return None
        return result.get("status")

    def _config_cb(self, response, *args):
        status = self._extract_status(response)
        if not status:
            return
        try:
            settings = status.get("configfile", {}).get("settings", {})
            count = settings.get("vortac_manager", {}).get("dock_count")
            if count:
                self.dock_count = int(count)
        except Exception:
            logging.exception("vortac_docks: configfile parse failed")

    def _status_cb(self, response, *args):
        status = self._extract_status(response)
        if not status or "vortac_manager" not in status:
            return
        GLib.idle_add(self._apply_status, status)

    # ------------------------------------------------------------------
    # rendering
    # ------------------------------------------------------------------

    def _apply_status(self, status):
        try:
            self.manager = status.get("vortac_manager") or {}
            self.qgl = status.get("vortac_qgl_state") or self.qgl
            self.tool_ids = list(self.manager.get("tools") or [])
            for tid in self.tool_ids:
                st = status.get("vortac_tool %s" % tid)
                if st:
                    self.tool_status[tid] = st
            self._render_header()
            self._render_docks()
        except Exception:
            logging.exception("vortac_docks: render failed")
        return False    # idle_add: run once

    def _render_header(self):
        held = self.manager.get("current_tool")
        if held:
            idx = (self.tool_status.get(held) or {}).get("tool_index")
            tn = " (T%s)" % idx if idx is not None else ""
            self.lbl_holding.set_markup(
                "Holding: <b><span foreground='#f0764f'>%s%s</span></b>"
                % (GLib.markup_escape_text(str(held)), tn))
        else:
            self.lbl_holding.set_markup("Carriage <b>empty</b>")

        self._update_primary_button(held)

        qgl = self.manager.get("qgl_state") or self.qgl.get("state")
        if qgl == "tilted":
            self.lbl_gantry.set_markup(
                "Gantry: <span foreground='#7fd6a0'>TILTED</span>")
        elif qgl:
            self.lbl_gantry.set_markup(
                "Gantry: <span foreground='#e8b45a'>%s</span>"
                % GLib.markup_escape_text(str(qgl).upper()))
        else:
            self.lbl_gantry.set_label("Gantry: –")

        if self.manager.get("dock_detection_valid"):
            detect = "Detection: <span foreground='#7fd6a0'>OK</span>"
        else:
            detect = ("Detection: <span foreground='#e8b45a'>stale — run "
                      "Detect</span>")
        # Dock power is only reported when the manager actually manages it,
        # so an older manager on the pi simply shows nothing extra here.
        power = self.manager.get("dock_power") or {}
        if power:
            live = sum(1 for on in power.values() if on)
            if self._power_faults():
                detect += (" · <span foreground='#e5484d'><b>Power: "
                           "check dock</b></span>")
            else:
                detect += " · Power: %d/%d" % (live, len(power))
        self.lbl_detect.set_markup(detect)

    def _power_faults(self):
        """Docks that hold a tool but are switched off. That combination
        strands the tool board: WS2812 keep their state across a klipper
        restart, so the board would be missing from the CAN bus at the next
        mcu identify and klipper would refuse to start."""
        power = self.manager.get("dock_power") or {}
        occupancy = self.manager.get("dock_occupancy") or {}
        return sorted(d for d, on in power.items()
                      if not on and occupancy.get(d))

    def _dock_calibrated(self, dock, tool_id):
        # A dock counts as calibrated for a tool once all three axes of its
        # hooked position (params_<dock>_x|y|z) exist in the tool's status.
        pos = ((self.tool_status.get(tool_id) or {})
               .get("dock_positions") or {}).get(dock) or {}
        return all(axis in pos for axis in ("x", "y", "z"))

    def _loadable_tools(self):
        # Only tools whose dock has a calibrated hooked position — loading
        # an uncalibrated one just errors out in the manager.
        occupancy = self.manager.get("dock_occupancy") or {}
        return sorted({t for d, t in occupancy.items()
                       if t and self._dock_calibrated(d, t)})

    def _load_pixbuf(self, svg_name, size=40):
        try:
            from gi.repository import GdkPixbuf
            return GdkPixbuf.Pixbuf.new_from_file_at_size(
                os.path.join(ICON_DIR, svg_name), size, size)
        except Exception:
            logging.exception("vortac_docks: pixbuf %s failed", svg_name)
            return None

    def _update_primary_button(self, held):
        loadable = self._loadable_tools()
        if held:
            pix, sensitive, label = self.pix_park, True, "Park"
        elif len(loadable) == 1:
            pix, sensitive, label = self.pix_load, True, "Load"
        elif loadable:
            pix, sensitive, label = self.pix_load, False, "Load"
        else:
            pix, sensitive, label = self.pix_load, False, "Load"
        if self.img_primary is not None:
            self.img_primary.set_from_pixbuf(pix)
            self.lbl_primary.set_label(label)
        else:
            self.btn_primary.set_label(label)
        self.btn_primary.set_sensitive(sensitive)

    def _primary_clicked(self, widget):
        held = self.manager.get("current_tool")
        if held:
            self._confirm_script(
                widget, "Park %s at its dock?" % held, "VORTAC_UNLOAD")
            return
        loadable = self._loadable_tools()
        if len(loadable) == 1:
            tid = loadable[0]
            idx = (self.tool_status.get(tid) or {}).get("tool_index")
            if idx is not None:
                self._confirm_script(
                    widget, "Load %s (T%s)?" % (tid, idx), "T%s" % idx)

    def _dock_names(self):
        occupancy = self.manager.get("dock_occupancy") or {}
        names = set(occupancy)
        count = self.dock_count or 0
        for i in range(count):
            names.add("dock%d" % i)
        return sorted(names, key=lambda n: (len(n), n))

    def _render_docks(self):
        docks = self._dock_names()
        if not docks:
            return
        if set(self.tiles) != set(docks):
            self._build_tiles(docks)
        occupancy = self.manager.get("dock_occupancy") or {}
        held = self.manager.get("current_tool")
        # park_dock: tool_id -> dock it returns to; invert for tile lookup
        reserved = {d: t for t, d in
                    (self.manager.get("park_dock") or {}).items()
                    if t == held}
        for dock in docks:
            self._update_tile(dock, occupancy.get(dock), held,
                              reserved.get(dock))

    def _build_tiles(self, docks):
        for child in list(self.grid.get_children()):
            self.grid.remove(child)
        self.tiles = {}
        cols = min(MAX_COLS, max(1, len(docks)))
        for i, dock in enumerate(docks):
            name = Gtk.Label(halign=Gtk.Align.START, hexpand=True)
            power = Gtk.Label(halign=Gtk.Align.END)
            tool = Gtk.Label(halign=Gtk.Align.START)
            sense = Gtk.Label(halign=Gtk.Align.START)
            hint = Gtk.Label(halign=Gtk.Align.START)
            for lbl in (name, tool, sense, hint):
                lbl.set_ellipsize(Pango.EllipsizeMode.END)
            # Dock name left, supply glyph right — the border colour stays
            # reserved for occupancy, which is an orthogonal dimension.
            top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
            top.add(name)
            top.pack_end(power, False, False, 0)
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            box.add(top)
            box.add(tool)
            box.add(sense)
            box.pack_end(hint, False, False, 0)
            btn = Gtk.Button(hexpand=True, vexpand=True)
            btn.set_can_focus(False)
            btn.add(box)
            btn.get_style_context().add_class("vortac-dock")
            btn.connect("clicked", self._tile_clicked, dock)
            self.grid.attach(btn, i % cols, i // cols, 1, 1)
            self.tiles[dock] = {"btn": btn, "name": name, "power": power,
                                "tool": tool, "sense": sense, "hint": hint}
        self.grid.show_all()

    @staticmethod
    def _sense_char(state):
        # False = active-low asserted, True = idle, None = no reading yet
        if state is False:
            return "●"
        if state is True:
            return "○"
        return "–"

    def _power_markup(self, dock):
        """Supply glyph for one dock: lit bolt = the dock feeds its tool
        board, dimmed = contacts dead. Empty string when the manager does not
        manage the supply at all (mode off, or an older manager on the pi),
        so the tile silently degrades instead of lying."""
        power = self.manager.get("dock_power") or {}
        if dock not in power:
            return ""
        if power[dock]:
            return "<small><span foreground='#7fd6a0'>⚡</span></small>"
        return "<small><span foreground='#4a5568'>⚡</span></small>"

    def _update_tile(self, dock, tool_id, held, reserved_for=None):
        tile = self.tiles.get(dock)
        if tile is None:
            return
        st = self.tool_status.get(tool_id or reserved_for) or {}
        ctx = tile["btn"].get_style_context()
        for cls in ("vortac-dock-occupied", "vortac-dock-held",
                    "vortac-dock-empty", "vortac-dock-unknown",
                    "vortac-dock-uncal", "vortac-dock-fault"):
            ctx.remove_class(cls)

        tile["name"].set_markup(
            "<small>%s</small>" % GLib.markup_escape_text(dock.upper()))
        tile["power"].set_markup(self._power_markup(dock))

        # A dock holding a tool with its supply off strands that tool board:
        # the LED keeps its state across a klipper restart, so the board would
        # be missing at the next mcu identify. Loud, and tappable to fix.
        if dock in self._power_faults():
            ctx.add_class("vortac-dock-fault")
            tile["tool"].set_markup(
                "<b><span foreground='#e5484d'>%s</span></b> "
                "<small>· NO POWER</small>"
                % GLib.markup_escape_text(str(tool_id or "?")))
            tile["sense"].set_markup(
                "<small>dock %s   grab %s</small>"
                % (self._sense_char(st.get("dock_sense_state")),
                   self._sense_char(st.get("grab_sense_state"))))
            tile["hint"].set_markup("<small>tap: switch power on</small>")
            return

        if tool_id:
            idx = st.get("tool_index")
            tn = " · T%s" % idx if idx is not None else ""
            calibrated = self._dock_calibrated(dock, tool_id)
            if tool_id == held:
                color, cls = "#f0764f", "vortac-dock-held"
            elif calibrated:
                color, cls = "#8fc1f2", "vortac-dock-occupied"
            else:
                # occupied but no hooked position saved for this dock —
                # loading would fail; offer calibration instead
                color, cls = "#c9a75a", "vortac-dock-uncal"
            ctx.add_class(cls)
            note = "" if calibrated else " · not calibrated"
            tile["tool"].set_markup(
                "<b><span foreground='%s'>%s</span></b><small>%s%s</small>"
                % (color, GLib.markup_escape_text(str(tool_id)), tn, note))
            tile["sense"].set_markup(
                "<small>dock %s   grab %s</small>"
                % (self._sense_char(st.get("dock_sense_state")),
                   self._sense_char(st.get("grab_sense_state"))))
            if calibrated:
                hint = "tap: change" if held else "tap: load"
            else:
                hint = "tap: select for calibration"
            tile["hint"].set_markup("<small>%s</small>" % hint)
        elif reserved_for:
            # the held tool's return dock — occupancy is None while carried
            ctx.add_class("vortac-dock-held")
            tile["tool"].set_markup(
                "<b><span foreground='#f0764f'>%s</span></b> "
                "<small>on carriage</small>"
                % GLib.markup_escape_text(str(reserved_for)))
            tile["sense"].set_markup(
                "<small>grab %s</small>"
                % self._sense_char(st.get("grab_sense_state")))
            tile["hint"].set_markup("<small>tap: park here</small>")
        else:
            calibrated = any(
                dock in (ts.get("dock_positions") or {})
                for ts in self.tool_status.values())
            if calibrated or self.manager.get("dock_detection_valid"):
                ctx.add_class("vortac-dock-empty")
                tile["tool"].set_markup("<i>empty</i>")
            else:
                ctx.add_class("vortac-dock-unknown")
                tile["tool"].set_markup(
                    "<span foreground='#c9a75a'>unknown</span>")
            tile["sense"].set_label("")
            tile["hint"].set_markup(
                "<small>tap: select for calibration</small>")

    # ------------------------------------------------------------------
    # actions
    # ------------------------------------------------------------------

    def _action_button(self, icon, label, confirm_text, script):
        try:
            btn = self._gtk.Button(icon, label, "color1")
        except Exception:
            btn = Gtk.Button(label=label)
        btn.set_can_focus(False)
        if confirm_text is None:
            btn.connect("clicked", self._run_script, script)
        else:
            btn.connect("clicked", self._confirm_script, confirm_text, script)
        return btn

    def _icon_button(self, svg_name, label, confirm_text, script):
        # Button with one of our custom SVGs from klipperscreen/icons/.
        # Falls back to a stock-icon button if the SVG cannot be loaded
        # (missing librsvg, moved repo, ...).
        try:
            from gi.repository import GdkPixbuf
            path = os.path.join(ICON_DIR, svg_name)
            pix = GdkPixbuf.Pixbuf.new_from_file_at_size(path, 40, 40)
            img = Gtk.Image.new_from_pixbuf(pix)
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2,
                          halign=Gtk.Align.CENTER, valign=Gtk.Align.CENTER)
            box.add(img)
            box.add(Gtk.Label(label=label))
            btn = Gtk.Button()
            btn.add(box)
            btn.set_can_focus(False)
            btn.connect("clicked", self._confirm_script, confirm_text, script)
            return btn
        except Exception:
            logging.exception("vortac_docks: icon %s failed, using fallback",
                              svg_name)
            return self._action_button("extrude", label, confirm_text, script)

    def _run_script(self, widget, script):
        self._screen._send_action(
            widget, "printer.gcode.script", {"script": script})

    def _confirm_script(self, widget, text, script):
        self._screen._confirm_send_action(
            widget, text, "printer.gcode.script", {"script": script})

    def _tile_clicked(self, widget, dock):
        occupancy = self.manager.get("dock_occupancy") or {}
        tool_id = occupancy.get(dock)
        held = self.manager.get("current_tool")
        park_map = self.manager.get("park_dock") or {}
        if dock in self._power_faults():
            # Only ever offer switching a dock ON from here. Switching an
            # occupied dock OFF drops its board off the CAN bus and takes
            # klipper down — that stays in the console behind FORCE=1.
            self._confirm_script(
                widget,
                "%s holds %s but its supply is switched off — that board "
                "will be missing after the next restart.\nSwitch the dock "
                "supply back on?" % (dock, tool_id or "a tool"),
                "VORTAC_DOCK_POWER DOCK=%s VALUE=1" % dock)
            return
        if held and park_map.get(held) == dock and not tool_id:
            self._confirm_script(
                widget, "Park %s at %s?" % (held, dock), "VORTAC_UNLOAD")
        elif tool_id and not self._dock_calibrated(dock, tool_id):
            # Loading would fail (no hooked position for this dock) — offer
            # to select it as the calibration target instead.
            self._confirm_script(
                widget,
                "%s in %s has no saved hooked position — select it as the "
                "calibration target?\n(Then jog the toolhead into the hooked "
                "position via the Move panel and press Save Dock.)"
                % (tool_id, dock),
                "VORTAC_SELECT_DOCK DOCK=%s" % dock)
        elif tool_id:
            idx = (self.tool_status.get(tool_id) or {}).get("tool_index")
            if idx is None:
                self._screen.show_popup_message(
                    "No tool_index for %s yet — wait for status." % tool_id)
                return
            self._confirm_script(
                widget, "Tool change to %s (T%s)?" % (tool_id, idx),
                "T%s" % idx)
        else:
            self._confirm_script(
                widget,
                "Select %s for dock calibration?\n(Runs VORTAC_DETECT; the "
                "dock must hold a detectable tool.)" % dock,
                "VORTAC_SELECT_DOCK DOCK=%s" % dock)
