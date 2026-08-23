# include_with.py — load a Klipper template under a remapped MCU namespace
#
# Phase 3: enables a single tool-board template (e.g. SB2209.cfg) to be
# instantiated multiple times under different MCU UUIDs without "duplicate
# section" collisions. Tool configs use this to wire per-tool hardware from
# one shared template after declaring their static [mcu toolN] section.
#
# Two entry points:
#   1. [include_with <namespace> <filename>]  -- config-section wrapper
#        tool_index: <int>          (default: assigned by LOAD ORDER when the
#                                    template contains a [vortac_tool ...]
#                                    section — the first such include gets 0,
#                                    the next 1, ... Klipper expands [include]
#                                    inline and loads sections in file order,
#                                    so the include order in tools.cfg IS the
#                                    tool numbering. Commenting a tool out
#                                    renumbers the ones after it, which keeps
#                                    Klipper's extruder/extruder<N> contract
#                                    intact. Fallback for non-tool templates:
#                                    trailing number of <namespace>, e.g.
#                                    "tool1" -> 1. Mixing explicit tool_index
#                                    with auto-numbered includes is not
#                                    supported — the counter ignores explicit
#                                    values.)
#        mcu_from:   <name>         (auto-detected from [mcu <name>] otherwise)
#        overrides:  (optional) per-section option overrides, one per line:
#                        overrides:
#                          extruder.sensor_type: MAX31865
#                          extruder.sensor_pin: EBBCan: PA4
#                    Format: <orig section>.<key>: <value>. Section names use
#                    ORIGINAL (pre-rename) template names and may contain
#                    spaces ("heater_fan hotend_fan.pin: ..."). Values run
#                    through the same MCU/value rewriter as template values,
#                    so pins may be written template-relative ("EBBCan: PA4").
#                    Keys missing from the template are added; an EMPTY value
#                    removes the option from the injected section.
#                    A JSON dict ({"section": {"key": "val"}}) is also
#                    accepted; there, null removes an option.
#        skip_sections: comma/newline list of original template sections to skip
#   2. include_with_remap(...)      -- programmatic API / config generator helper
#
# Section-rename rules:
#
#   Any tool_index — EVERY named section gets the NAMESPACE as prefix, which
#   is also what dashboards display (namespace "tool1" -> tool1_logo_rgb,
#   namespace "miniPink" -> miniPink_logo_rgb):
#
#   [mcu <mcu_from>]     -> [mcu <namespace>]
#   [vortac_tool <any>]  -> [vortac_tool <namespace>]     (name REPLACED, not
#                            prefixed — the namespace IS the tool's logical
#                            name; tool_index, mcu_name, canbus_uuid and
#                            extruder_name are auto-injected, see below)
#   [<head> name]        -> [<head> <namespace>_name]     (generic rule:
#                            neopixel, heater_fan, temperature_sensor,
#                            output_pin, filament_*_sensor, gcode_macro, ...)
#   [adxl345]            -> [adxl345 <namespace>]
#   [tmcXXXX <target>]   -> follows <target>'s rename
#   [verify_heater <t>]  -> follows <t>'s rename
#
#   tool_index >= 1 only (Klipper needs a primary [extruder] and the [fan]
#   singleton for M106, so tool 0 keeps those unrenamed; multi-extruder
#   naming is hardwired to extruder<N> by Klipper — [extruder miniPink]
#   is not a valid Klipper section):
#
#   [extruder]           -> [extruder<N>]
#   [fan]                -> [fan_generic <namespace>_fan]   (type change!)
#
#   Bare singletons with no name part (input_shaper, firmware_retraction,
#   ...) cannot be namespaced — they pass through unchanged; skip them via
#   skip_sections if they collide across tools.
#
# Singletons that can't be safely renamed (resonance_tester, input_shaper,
# shaketune) are skipped for tool_index >= 1 by default. Caller can override
# via skip_sections=.

import os
import re
import json


DEFAULT_SINGLETON_SKIP = ('resonance_tester', 'input_shaper', 'shaketune')

_TMC_HEADS = ('tmc2209', 'tmc2240', 'tmc2130', 'tmc5160')


# Section heads whose "name" part is a REFERENCE to another section — they
# must follow that section's rename instead of getting a namespace prefix.
_REFERENCE_HEADS = _TMC_HEADS + ('verify_heater',)


def _rename_section(orig, idx, mcu_from, mcu_to):
    parts = orig.split()
    head = parts[0]
    rest = parts[1:]

    # MCU swap is index-independent — that IS the namespace remap.
    if head == 'mcu' and rest and rest[0] == mcu_from:
        return f"mcu {mcu_to}"

    # The template's [vortac_tool <placeholder>] becomes THE logical tool
    # section of this instantiation: the namespace replaces the placeholder
    # name entirely (the generic prefix rule would yield "miniGrey_TN",
    # which is not a usable tool name).
    if head == 'vortac_tool':
        return f"vortac_tool {mcu_to}"

    # Reference sections track their target's rename:
    # [tmc2209 extruder] -> [tmc2209 extruder1], [verify_heater extruder]
    # likewise; [tmc2209 manual_stepper foo] follows the renamed stepper.
    if head in _REFERENCE_HEADS and rest:
        target = _rename_section(' '.join(rest), idx, mcu_from, mcu_to)
        return f"{head} {target}"

    # Klipper hardwires these singletons: the primary [extruder] and the
    # M106 [fan] belong to tool 0; extras are extruder<N> / fan_generic.
    if orig == 'extruder':
        return orig if idx == 0 else f"extruder{idx}"
    if orig == 'fan':
        return orig if idx == 0 else f"fan_generic {mcu_to}_fan"
    if orig == 'adxl345':
        return f"adxl345 {mcu_to}"

    # Generic rule, every tool index: ANY named section gets the NAMESPACE
    # as prefix — the namespace doubles as the display name in Mainsail/
    # Fluidd/KlipperScreen (namespace "miniPink" -> miniPink_logo_rgb,
    # miniPink_hotend_fan, miniPink_filament_sensor, ...).
    if rest:
        return f"{head} {mcu_to}_{'_'.join(rest)}"

    # Unhandled bare singleton ([input_shaper], [firmware_retraction], ...):
    # cannot be namespaced — leave as-is; use skip_sections if it collides.
    return orig


class _ToolIndexCounter:
    """Per-printer sequence for order-based tool numbering.

    Must live on the Printer object, NOT at module level: Klipper's RESTART
    rebuilds the Printer in the same process while imported modules stay
    loaded — a module-global counter would keep counting across restarts.
    """
    def __init__(self):
        self.next_index = 0

    def take(self):
        idx = self.next_index
        self.next_index += 1
        return idx


def _next_auto_tool_index(printer):
    counter = printer.lookup_object('include_with_tool_counter', None)
    if counter is None:
        counter = _ToolIndexCounter()
        printer.add_object('include_with_tool_counter', counter)
    return counter.take()


def _load_template(printer, filepath):
    printer_config = printer.lookup_object('configfile')
    template_path = _resolve_filepath(printer, filepath)
    return printer_config.read_config(template_path)


def _template_tool_section(template, skip_set):
    """Return the template's [vortac_tool ...] section name, unless the
    caller skips it — a skipped tool section must not consume an index."""
    for sect in template.fileconfig.sections():
        if sect.split()[0] == 'vortac_tool' and sect not in skip_set:
            return sect
    return None


def _autodetect_mcu_from(template_config):
    for sect in template_config.fileconfig.sections():
        parts = sect.split()
        if parts and parts[0] == 'mcu' and len(parts) >= 2:
            return parts[1]
    return None


def _resolve_filepath(printer, filepath):
    if os.path.isabs(filepath):
        return filepath
    cfg_file = printer.get_start_args().get('config_file', '')
    if not cfg_file:
        return filepath
    return os.path.join(os.path.dirname(cfg_file), filepath)


def _parse_overrides(config, raw):
    """Parse the `overrides` option.

    Primary format, one override per line:
        <orig section>.<key>: <value>
    Keys never contain dots, so the section/key split is the LAST dot —
    section names with spaces ("heater_fan hotend_fan") work unquoted.
    An empty value marks the option for removal (stored as None).

    A JSON dict ({"section": {"key": "val"}}) is accepted as fallback.
    """
    raw = raw.strip()
    if raw.startswith('{'):
        try:
            overrides = json.loads(raw)
        except json.JSONDecodeError as e:
            raise config.error(
                f"include_with overrides: invalid JSON ({e})")
        if not isinstance(overrides, dict):
            raise config.error(
                "include_with overrides must be a JSON object of the "
                "form {\"section\": {\"key\": \"value\"}}")
        return overrides
    overrides = {}
    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        left, sep, value = line.partition(':')
        if not sep or '.' not in left:
            raise config.error(
                f"include_with overrides: expected "
                f"'<section>.<key>: <value>', got {line!r}")
        section, key = left.rsplit('.', 1)
        section, key, value = section.strip(), key.strip(), value.strip()
        if not section or not key:
            raise config.error(
                f"include_with overrides: empty section or key in {line!r}")
        overrides.setdefault(section, {})[key] = value if value else None
    return overrides


def _parse_section_list(value):
    if value is None:
        return None
    sections = []
    for line in value.replace(',', '\n').splitlines():
        item = line.strip()
        if item:
            sections.append(item)
    return sections


def _make_value_rewriter(mcu_from, mcu_to, rename_map):
    # `<mcu_from>:` (with optional whitespace before colon) is the pin-token
    # signature. The colon disambiguates it from arbitrary substring matches —
    # no Klipper option uses a bare MCU name without a following colon.
    mcu_pat = re.compile(r'\b' + re.escape(mcu_from) + r'(\s*:)')
    repl_mcu = mcu_to + r'\1'

    section_renames = {orig: new for orig, new in rename_map.items()
                       if orig != new}

    def rewrite(value):
        if not isinstance(value, str):
            return value
        new_val = mcu_pat.sub(repl_mcu, value)
        # Trim-equal cross-section ref: e.g., `heater: extruder` whose
        # value is exactly "extruder", which matches a renamed section.
        # Multi-token references (lists, expressions) need the overrides
        # mechanism — we don't try to parse them.
        stripped = new_val.strip()
        if stripped in section_renames:
            new_val = new_val.replace(stripped, section_renames[stripped])
        return new_val

    return rewrite


def include_with_remap(printer, parent_config, filepath, namespace,
                       mcu_from=None, tool_index=0, overrides=None,
                       skip_sections=None, template=None):
    """Inject sections from `filepath` into `parent_config` under MCU
    namespace `namespace`, with per-tool-index renaming and value rewrites.

    Args:
      printer:        Klipper printer object.
      parent_config:  ConfigWrapper from caller's __init__ (the destination).
      filepath:       Path to template; resolved against printer config dir
                      if relative.
      namespace:      Destination MCU name (e.g., 'tool0').
      mcu_from:       Original MCU name in template; auto-detected from the
                      first [mcu <name>] section if None.
      tool_index:     0..N. Index 0 is namespace-swap only; >=1 triggers
                      section renames per the module docstring.
      overrides:      Optional dict {orig_section: {key: value}}. Keys use
                      the ORIGINAL (pre-rename) section names. Values pass
                      through the MCU/value rewriter (template-relative pin
                      names allowed); keys absent from the template are
                      added; a None/null value removes the option.
      skip_sections:  Optional iterable of orig-section names to skip.
                      Defaults to DEFAULT_SINGLETON_SKIP for tool_index >= 1.
      template:       Already-loaded template ConfigWrapper (from
                      _load_template); loaded from filepath if None.

    Returns: dict {orig_section: renamed_section}.
    """
    overrides = overrides or {}
    if skip_sections is None:
        skip_sections = (DEFAULT_SINGLETON_SKIP
                         if tool_index >= 1 else ())
    skip_set = set(skip_sections)

    if template is None:
        template = _load_template(printer, filepath)

    if mcu_from is None:
        mcu_from = _autodetect_mcu_from(template)
        if mcu_from is None:
            raise parent_config.error(
                f"include_with: cannot auto-detect MCU name from "
                f"'{filepath}' (no [mcu <name>] section). "
                f"Pass mcu_from= explicitly.")

    rename_map = {
        sect: _rename_section(sect, tool_index, mcu_from, namespace)
        for sect in template.fileconfig.sections()
    }
    rewrite = _make_value_rewriter(mcu_from, namespace, rename_map)

    for orig_sect in template.fileconfig.sections():
        if orig_sect in skip_set:
            continue
        new_sect = rename_map[orig_sect]
        items = template.fileconfig.items(section=orig_sect)
        # Copy: we pop consumed keys to find the ones the template lacks.
        section_overrides = dict(overrides.get(orig_sect, {}))
        new_items = {}
        for k, v in items:
            if k in section_overrides:
                ov = section_overrides.pop(k)
                if ov is None:
                    continue  # JSON null removes the option entirely
                new_items[k] = rewrite(str(ov))
            else:
                new_items[k] = rewrite(v)
        # Override keys absent from the template are ADDED, not dropped —
        # needed e.g. to switch a sensor_type that brings new options along
        # (MAX31865: spi_bus, rtd_nominal_r, rtd_reference_r, ...).
        for k, ov in section_overrides.items():
            if ov is not None:
                new_items[k] = rewrite(str(ov))
        # The logical tool section gets its identity injected — the
        # namespace is the single source of truth for the tool's name.
        # Template values / overrides win over the auto-injection.
        if orig_sect.split()[0] == 'vortac_tool':
            new_items.setdefault('tool_index', str(tool_index))
            new_items.setdefault('mcu_name', namespace)
            pc = parent_config.fileconfig
            mcu_sect = f"mcu {namespace}"
            if ('canbus_uuid' not in new_items
                    and pc.has_section(mcu_sect)
                    and pc.has_option(mcu_sect, 'canbus_uuid')):
                new_items['canbus_uuid'] = pc.get(mcu_sect, 'canbus_uuid')
            # The renamed extruder ('extruder' for index 0, 'extruder<N>'
            # otherwise) — vortac_manager runs ACTIVATE_EXTRUDER with it on
            # tool change. Injected here because the name shifts with
            # order-based renumbering.
            if 'extruder' in rename_map:
                new_items.setdefault('extruder_name', rename_map['extruder'])
        # read_dict MERGES into a pre-existing section instead of raising —
        # that is load-bearing: SAVE_CONFIG's autosave block may already
        # have created [vortac_tool <namespace>] holding the persisted
        # params_* dock positions (autosave is merged into the config
        # before extras run). Invariant: the autosave block for vortac_tool
        # sections only ever holds params_* keys, which no template
        # provides, so nothing here overwrites persisted data.
        parent_config.fileconfig.read_dict({new_sect: new_items})
        printer.load_object(parent_config, new_sect, default=None)

    return rename_map


class IncludeWith:
    """Config-section form: [include_with <namespace> <filename>]."""
    def __init__(self, config):
        parts = config.get_name().split()
        if len(parts) < 3:
            raise config.error(
                "[include_with] requires: "
                "[include_with <namespace> <filename>]")
        self.namespace = parts[1]
        # Support filenames with spaces or path separators.
        filename = ' '.join(parts[2:])

        printer = config.get_printer()
        mcu_from = config.get('mcu_from', default=None)

        overrides_str = config.get('overrides', default=None)
        overrides = None
        if overrides_str:
            overrides = _parse_overrides(config, overrides_str)

        skip_sections = _parse_section_list(
            config.get('skip_sections', default=None))

        template = _load_template(printer, filename)

        # tool_index resolution:
        #   1. explicit option,
        #   2. templates carrying a (non-skipped) [vortac_tool ...] section
        #      are numbered by LOAD ORDER — include order in tools.cfg is
        #      the tool numbering (first include -> 0),
        #   3. legacy fallback: trailing number of the namespace
        #      ("tool1" -> 1) for non-tool templates.
        tool_index = config.getint('tool_index', default=None, minval=0)
        if tool_index is None:
            if _template_tool_section(template, set(skip_sections or ())):
                tool_index = _next_auto_tool_index(printer)
            else:
                m = re.search(r'(\d+)$', self.namespace)
                if m is None:
                    raise config.error(
                        f"include_with {self.namespace}: cannot derive "
                        f"tool_index (template has no [vortac_tool] section "
                        f"and namespace has no trailing number); "
                        f"set tool_index explicitly")
                tool_index = int(m.group(1))

        self.rename_map = include_with_remap(
            printer=printer,
            parent_config=config,
            filepath=filename,
            namespace=self.namespace,
            mcu_from=mcu_from,
            tool_index=tool_index,
            overrides=overrides,
            skip_sections=skip_sections,
            template=template,
        )


def load_config_prefix(config):
    return IncludeWith(config)
