#TODO configPATH shouldnt be absolute
configPATH = "/home/pi/printer_data/config/"

class IncludeWith:
    def __init__(self, config):
        self.namespace, filename = config.section.split()[1:]

        self.printer = config.get_printer()
        self.printerConfig = self.printer.lookup_object('configfile')

        includeConfig = self.printerConfig.read_config(f"{configPATH}{filename}")

        for section in includeConfig.fileconfig.sections():
            options = includeConfig.fileconfig.items(section=section)
            namespacesOptions = [(k, self.namespacePin(v)) if 'pin' in k else (k, v) for k, v in options]
            #includeConfig.fileconfig.read_dict({section : dict(namespacesOptions)})
            config.fileconfig.read_dict({section : dict(namespacesOptions)})
            self.printer.load_object(config, section, default=None)


            #raise config.error(f"{namespacesOptions}")

        #raise config.error(f"{includeConfig.fileconfig.sections()}")
        #raise config.error("imported file >> {}".format(includeConfig.fileconfig.sections()))
        #raise config.error("started include_with with namespace: {} and filepath: {}".format(self.namespace, self.filepath))

    def namespacePin(self, val):
        return f"{self.namespace}:{val}"



def load_config_prefix(config):
    return IncludeWith(config)





'''
upload
scp ./include_with.py pi@192.168.0.188:/home/pi/klipper/klippy/extras/
scp D:/git/VortacToolchanger/VortacFirmware/klipper/klippy/extras/include_with.py pi@192.168.0.188:/home/pi/klipper/klippy/extras/
'''