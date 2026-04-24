Python script for peronal computer that uses SSH to get logs from GulliView computer and can visualize them in box plots.

Run in main.py in optimized mode to not enter debug mode, this is done by run.bat as well.

Needs a config.ini file with following format:

[SSH]
host = 192.168.XX.XX
user = username
password = password
folder = ssh_folder_path

[General]
log_filename = general.log
data_folder = data
