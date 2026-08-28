import subprocess
import os

extrasPath = '../klipper/klippy/extras/'
files = ['tool.py', 'vortac_grabber.py']
piAdress = '192.168.0.188'
piName = 'pi'
targetPath = '~/klipper/klippy/extras/'
password = 'raspberry'

for filename in files:
    local_path = os.path.join(extrasPath, filename)
    # Korrigierte Argumentliste:
    args = [
        "sshpass", "-p", password,
        "scp", local_path,
        f"{piName}@{piAdress}:{targetPath}"
    ]
    try:
        result = subprocess.run(
            args,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print(f"Erfolgreich kopiert: {local_path}")
    except subprocess.CalledProcessError as e:
        print(f"Fehler beim Kopieren von {local_path}: Return-Code {e.returncode}")
        print("stderr:", e.stderr)
    except FileNotFoundError:
        print("Programm sshpass oder scp nicht gefunden. Stelle sicher, dass es installiert ist und im PATH liegt.")
