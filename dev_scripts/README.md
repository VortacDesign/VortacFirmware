# Developer Notes
Push windows files to Raspberry via ssh
references:
scp [windows path] pi@[local adress]:[pi path target]

# Klipper Learnings 
init config equals section with touple of object and value 
Section = []
Object: Value

Angle BulkSensor information from Klipper is in “radians × 10 000”
So to read true Angle Data you have to Convert the Values