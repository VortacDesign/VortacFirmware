class Tool:
    def __init__(self, config):
        self.printer = config.get_printer()
        #self.gcode = self.printer.lookup_object('gcode')
        #self.config = config

        name = config.section.split()[-1]

        #TODO All Config vars need to always be grabbed used to avoid var not valid in section issues
        #self.pin = '{}:{}'.format(name, config.get("pin"))
        self.pin = config.get("pin")
        self.max_power = config.get("max_power", None)
        self.cycle_time = config.get("cycle_time", None)
        self.kick_start_time = config.get("kick_start_time", None)

        #config.fileconfig.remove_section(config)

        #config.section = 'fan_generic tool0'
        itemtouple = config.fileconfig.items('tool tool0')

        result = {'fan_generic tool0': dict(itemtouple)}

        config.fileconfig.read_dict(result)

        #config.fileconfig.set('fan_generic tool0', 'Pin', self.pin)
       # config.fileconfig.set('gg gg1', 'Pin', self.pin)

        self.fan = self.printer.load_object(config, 'fan_generic tool0')


def load_config_prefix(config):
    return Tool(config)