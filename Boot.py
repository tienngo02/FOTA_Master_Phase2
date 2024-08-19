import os
import subprocess
import sys
import time
import json

print()
print("================================")
print("Boot is running...")

PYTHON = 'python3.12'

APP = 'App.py'
BOOT = 'Boot.py'
CLIENT = 'FOTA_Client.py'

NEW_BOOT = 'FOTA_Master_Boot_new.py'
NEW_APP = 'FOTA_Master_App_new.py'
NEW_CLIENT = 'FOTA_Client_new.py'

BACKUP_BOOT = 'FOTA_Master_Boot_backup.py'
BACKUP_APP = 'FOTA_Master_App_backup.py'
BACKUP_CLIENT = 'FOTA_Client_backup.py'

JSONFILE = 'Version_information_file.json'


# subprocess.run(['python3.12', '-V'])

def flashClient():
    try:
        print("Stop.........")
        # process = subprocess.run([PYTHON, 'run_command.py', 'cp', CLIENT,'0'],timeout=15)
        process = subprocess.run([PYTHON, 'run_command.py', 'stop'], timeout=15)
    except subprocess.TimeoutExpired as e:
        print('Process timeout')
        process = subprocess.run([PYTHON, 'run_command.py', 'stop'], timeout=15)

    print("Flash...........")
    process = subprocess.run([PYTHON, 'run_command.py', 'cp', CLIENT, '5'])
    time.sleep(1)
    print("Start..........")
    process = subprocess.run([PYTHON, 'run_command.py', 'start', '5'])
    time.sleep(3)
    # subprocess.run([PYTHON, 'run_command.py', 'cp', 'Client_Test/client_Phase1_GoRight.py', '1'])
    # time.sleep(3)
    # subprocess.run([PYTHON, 'run_command.py', 'start', '1'])


def update_running_version(file_name):
    with open(JSONFILE, 'r') as file:
        data = json.load(file)

    temp = data[file_name]['running']
    data[file_name]['running'] = data[file_name]['non-running']
    data[file_name]['non-running'] = temp

    data[file_name]['activate'] = False

    with open(JSONFILE, 'w') as file:
        json.dump(data, file, indent=4)


def main_run():
    user_input = sys.argv[1]

    if user_input == "run_App":
        print("Run App")
        subprocess.Popen([PYTHON, APP])
        exit()

    elif user_input == "activate_Boot":
        print("Activate new boot")
        if os.path.exists(NEW_BOOT):
            os.rename(BOOT, BACKUP_BOOT)
            os.rename(NEW_BOOT, BOOT)
            update_running_version('FOTA_Master_Boot')
            subprocess.Popen([PYTHON, BOOT, 'run_App'])
            exit()
        else:
            print("New boot file does not exist.")

    elif user_input == "rollback_Boot":
        print("Rollback boot")
        if os.path.exists(BACKUP_BOOT):
            os.rename(BOOT, NEW_BOOT)
            os.rename(BACKUP_BOOT, BOOT)
            update_running_version('FOTA_Master_Boot')
            subprocess.Popen([PYTHON, BOOT, 'run_App'])
            exit()
        else:
            print("Backup boot file does not exist.")

    elif user_input == "activate_App":
        print("Activate App")
        if os.path.exists(NEW_APP):
            os.rename(APP, BACKUP_APP)
            os.rename(NEW_APP, APP)
            update_running_version('FOTA_Master_App')
            subprocess.Popen([PYTHON, APP])
            exit()
        else:
            print("New app file does not exist.")

    elif user_input == "rollback_App":
        print("Rollback app")
        if os.path.exists(BACKUP_APP):
            os.rename(APP, NEW_APP)
            os.rename(BACKUP_APP, APP)
            update_running_version('FOTA_Master_App')
            subprocess.Popen([PYTHON, APP])
            exit()
        else:
            print("Backup app file does not exist.")

    elif user_input == 'activate_Client':
        print("Activate client")
        if os.path.exists(NEW_CLIENT):
            os.rename(CLIENT, BACKUP_CLIENT)
            os.rename(NEW_CLIENT, CLIENT)
            flashClient()
            update_running_version('FOTA_Client')
            # subprocess.Popen([PYTHON, APP])
            exit()
        else:
            print("New client file does not exist.")

    elif user_input == "rollback_Client":
        print("Rollback client")
        if os.path.exists(BACKUP_CLIENT):
            os.rename(CLIENT, NEW_CLIENT)
            os.rename(BACKUP_CLIENT, CLIENT)
            flashClient()
            update_running_version('FOTA_Client')
            subprocess.Popen([PYTHON, APP])
            exit()
        else:
            print("Backup client file does not exist.")

    else:
        print("Invalid argument")


if __name__ == '__main__':
    try:
        print("New bootloader ne!")
        main_run()
        print("Bootloader finished.")
        exit()
    except Exception as e:
        # print('Boot is error, rollback boot')
        print(e)
        # subprocess.Popen(['python3.12', 'FOTA_Master_Boot_backup.py', 'rollback_Boot'])
        exit()