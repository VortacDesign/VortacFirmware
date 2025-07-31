import pkgutil
import importlib

# Discover and import all submodules in this package directory
_modules = {}
for finder, name, is_pkg in pkgutil.iter_modules(__path__):
    module = importlib.import_module(f"{__name__}.{name}")
    _modules[name] = module

def _dispatch(config, fn_name):
    """
    Dispatches the load_config or load_config_prefix call to the appropriate submodule.

    Priority:
    1. Section name prefix: [package_name submodule_name]
    2. 'module' option in section.
    """
    section_parts = config.get_name().split(None, 1)
    # Case 1: [package_name submodule_name]
    if len(section_parts) == 2 and section_parts[1] in _modules:
        module = _modules[section_parts[1]]
    # Case 2: module option in section
    elif config.has_option('module'):
        module_name = config.get('module')
        if module_name in _modules:
            module = _modules[module_name]
        else:
            raise config.error(f"Unknown module '{module_name}' specified in section {config.get_name()}")
    else:
        raise config.error(f"No submodule found for section {config.get_name()}")

    fn = getattr(module, fn_name, None)
    if fn is None:
        raise config.error(f"Module '{module.__name__}' does not implement a '{fn_name}()' function")
    return fn(config)

def load_config(config):
    """
    Called by Klipper to load configuration for this package.
    """
    return _dispatch(config, 'load_config')

def load_config_prefix(config):
    """
    Called by Klipper to load prefixed configuration sections.
    """
    return _dispatch(config, 'load_config_prefix')
