class Tool:
    def __init__(self, config):
        self.printer = config.get_printer()

        #NOTE you can split section name to extract the suffix
        #name = config.section.split()[-1]

        #NOTE All Config vars need to always be grabbed used to avoid var not valid in section issues
        self.pin = config.get("pin")
        self.max_power = config.get("max_power", None)
        self.cycle_time = config.get("cycle_time", None)
        self.kick_start_time = config.get("kick_start_time", None)


        #NOTE read in a section dict with the old optionTuple under a new name
        optionsTuple = config.fileconfig.items('tool tool0')
        sectionDict = {'fan_generic tool0': dict(optionsTuple)}
        config.fileconfig.read_dict(sectionDict)

        #NOTE new added section needs to be loaded manually
        self.fan = self.printer.load_object(config, 'fan_generic tool0')


def load_config_prefix(config):
    return Tool(config)

'''
config is a klippy.configfile.ConfigWrapper object
config.fileconfig is a python.lib.RawConfigParser
'''