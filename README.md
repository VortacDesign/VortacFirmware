# Vortac Firmware
This Repository holds Config files and Python scripts that enables you to 
use and Setup the Vortac Toolchanger 

# Installation Guide
Add this to moonraker.conf (adjust path/branch if needed):
```
[update_manager vortac]
type: git_repo
path: ~/vortac
origin: https://github.com/VortacDesign/VortacFirmware.git
primary_branch: development
managed_services: klipper
```
On your pi run the Install.sh once. (adjust path/branch if needed)
```
cd ~/vortac && ./install.sh
```
The installer will mount the Config files and Python scripts to your Klipper directory.
It will set up a System path trigger to rerun itself when the Repository Head changes. 
So you should only need To Run this Once for setup. 