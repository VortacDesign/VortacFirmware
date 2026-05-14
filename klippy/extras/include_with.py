# include_with.py — load a Klipper template under a remapped MCU namespace
#
# Phase 3: enables a single tool-board template (e.g. SB2209.cfg) to be
# instantiated multiple times under different MCU UUIDs without "duplicate
# section" collisions. Tool configs use this to wire per-tool hardware from
# one shared template after declaring their static [mcu toolN] section.
#
# Two entry points:
#   1. [include_with <namespace> <filename>]  -- config-section wrapper
#        tool_index: <int>          (default 0; >=1 triggers section renames)
#        mcu_from:   <name>         (auto-detected from [mcu <name>] otherwise)
#        overrides:  <json-dict>    (optional; {"orig section": {"key": "val"}})
#        skip_sections: comma/newline list of original template sections to skip
#   2. include_with_remap(...)      -- programmatic API / config generator helper
#
# Section-rename table (tool_index >= 1; tool_index 0 is namespace-swap only):
#
#   [mcu <mcu_from>]     -> [mcu <namespace>]      (any tool_index)
#   [extruder]           -> [extruder<N>]
#   [tmcXXXX extruder]   -> [tmcXXXX extruder<N>]
#   [fan]                -> [fan_generic tool<N>_fan]   (type change!)
#   [heater_fan name]    -> [heater_fan tool<N>_name]
#   [neopixel name]      -> [neopixel tool<N>_name]
#   [adxl345]            -> [adxl345 tool<N>]
#   [adxl345 name]       -> [adxl345 tool<N>_name]
#
# Singletons that can't be safely renamed (resonance_tester, input_shaper,
# shaketune) are skipped for tool_index >= 1 by default. Caller can override
# via skip_sections=.

import os
import re
import json


DEFAULT_SINGLETON_SKIP = ('resonance_tester', 'input_shaper', 'shaketune')

_TMC_HEADS = ('tmc2209', 'tmc2240', 'tmc2130', 'tmc5160')


def _rename_section(orig, idx, mcu_from, mcu_to):
    parts = orig.split()
    head = parts[0]
    rest = parts[1:]

    # MCU swap is index-independent — that IS the namespace remap.
    if head == 'mcu' and rest and rest[0] == mcu_from:
        return f"mcu {mcu_to}"

    if idx == 0:
        return orig

    sfx = str(idx)

    if orig == 'extruder':
        return f"extruder{sfx}"
    if head in _TMC_HEADS and rest == ['extruder']:
        return f"{head} extruder{sfx}"
    if orig == 'fan':
        # [fan] is a Klipper singleton; promote to fan_generic for tools >= 1.
        return f"fan_generic tool{sfx}_fan"
    if head == 'heater_fan' and rest:
        return f"heater_fan tool{sfx}_{'_'.join(rest)}"
    if head == 'neopixel' and rest:
        return f"neopixel tool{sfx}_{'_'.join(rest)}"
    if orig == 'adxl345':
        return f"adxl345 tool{sfx}"
    if head == 'adxl345' and rest:
        return f"adxl345 tool{sfx}_{'_'.join(rest)}"

    return orig


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
                       skip_sections=None):
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
                      the ORIGINAL (pre-rename) section names.
      skip_sections:  Optional iterable of orig-section names to skip.
                      Defaults to DEFAULT_SINGLETON_SKIP for tool_index >= 1.

    Returns: dict {orig_section: renamed_section}.
    """
    overrides = overrides or {}
    if skip_sections is None:
        skip_sections = (DEFAULT_SINGLETON_SKIP
                         if tool_index >= 1 else ())
    skip_set = set(skip_sections)

    printer_config = printer.lookup_object('configfile')
    template_path = _resolve_filepath(printer, filepath)
    template = printer_config.read_config(template_path)

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
        section_overrides = overrides.get(orig_sect, {})
        new_items = {}
        for k, v in items:
            if k in section_overrides:
                new_items[k] = str(section_overrides[k])
            else:
                new_items[k] = rewrite(v)
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
        tool_index = config.getint('tool_index', default=0, minval=0)
        mcu_from = config.get('mcu_from', default=None)

        overrides_str = config.get('overrides', default=None)
        overrides = None
        if overrides_str:
            try:
                overrides = json.loads(overrides_str)
            except json.JSONDecodeError as e:
                raise config.error(
                    f"include_with overrides: invalid JSON ({e})")
            if not isinstance(overrides, dict):
                raise config.error(
                    "include_with overrides must be a JSON object of the "
                    "form {\"section\": {\"key\": \"value\"}}")

        skip_sections = _parse_section_list(
            config.get('skip_sections', default=None))

        self.rename_map = include_with_remap(
            printer=printer,
            parent_config=config,
            filepath=filename,
            namespace=self.namespace,
            mcu_from=mcu_from,
            tool_index=tool_index,
            overrides=overrides,
            skip_sections=skip_sections,
        )


def load_config_prefix(config):
    return IncludeWith(config)
