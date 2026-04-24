# Copyright © 2025 Emil Nylander

# This file is part of GulliView logs.

# GulliView logs is free software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License 
# as published by the Free Software Foundation, either version 3 
# of the License, or (at your option) any later version.

# GulliView logs is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# General Public License for more details.

# You should have received a copy of the GNU General Public License 
# along with GulliView logs. If not, see <https://www.gnu.org/licenses/>.

#%% Standard modules
import pickle
import paramiko
import os
from tqdm import tqdm
import subprocess
from tabulate import tabulate
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import configparser

#%% Custom modules
from functions import user_acknowledge
from functions import input_int
from functions import input_str

#%% Classes
from Log_class import Log

#%% Class def
# Init properties
init_properties = {"keys": []}

class Data:

    def __init__(self):
        if __debug__:
            print("\nDebug mode\n")

        # Init variables
        config = configparser.ConfigParser()
        config.read('config.ini')

        self.ssh_host   = config['SSH']['host']
        self.ssh_user   = config['SSH']['user']
        self.ssh_pwd    = config['SSH']['password']
        self.ssh_folder = config['SSH']['folder']

        self.general_log_filename = config['General']['log_filename']
        self.data_folder = config['General']['data_folder']
        # if __debug__:
        #     self.data_folder = "debug_" + self.data_folder
        
        filepath = os.path.realpath(__file__)
        folderpath = os.path.dirname(filepath)
        self.data_folder = os.path.join(folderpath, self.data_folder)

        self.archive_folder = os.path.join(self.data_folder, "archive")
        self.data_filepath  = os.path.join(self.data_folder, "logs")
        self.backup_folder  = os.path.join(self.data_folder, "backup")
        self.input_folder   = os.path.join(self.archive_folder, "input")

        # Command lists
        self.commands = {"fetch"    : self.fetch_new_logs,
                         "list"     : self.display_data,
                         "reimport" : self.reimport_all}

        debug_commands = {"ssh"     : self.copy_files_to_local,
                          "import"  : self.read_data}
        if __debug__:
            self.commands["debug"] = debug_commands

        print("Working directory:", os.path.abspath(self.data_folder), "\n")
        # Open file, this also creates a backup
        self.openFile()
            
        print(f"{len(self.logs)} logs imported\n")

    def returnCommands(self):
        return self.commands

#%% Data handling
    # Open and close file
    def openFile(self):
        try:
            with open(self.data_filepath, 'rb') as file:
                data = pickle.load(file)
                self.logs = data[0]
                self.properties = data[1]
        except FileNotFoundError:
            # If file could not be read, create new
            if input("Could not read file, create new? (y/n): ") == "y":
                self.logs = []
                self.properties = init_properties
                self.saveFile()
                print("\n")
            else: 
                raise SystemExit()
    def saveFile(self):
        data = [self.logs, self.properties]
        with open(self.data_filepath, 'wb') as file:
            pickle.dump(data, file)

    # Copy files from ssh to local folder
    def copy_files_to_local(self):
        
        # Checks if connected to right network
        wifi = subprocess.check_output(['netsh', 'WLAN', 'show', 'interfaces'])
        wifi_data = wifi.decode('utf-8')
        
        if not "ROStig" in wifi_data:
            user_acknowledge("Not connected to ROStig WiFi!")
            return True # Return True to exit callin function as well
        
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            print("Connecting...")
            ssh.connect(self.ssh_host, username=self.ssh_user, password=self.ssh_pwd)
            print("Success!")

            sftp = ssh.open_sftp()

            # Ensure local folder exists
            os.makedirs(self.input_folder, exist_ok=True)

            # List files in the remote directory
            remote_files = sftp.listdir(self.ssh_folder)

            if not remote_files:
                print("No log files found")

            for log_file in tqdm(remote_files, desc="Downloading Logs"):
                if "exclude" in log_file:
                    continue
                remote_path = os.path.join(self.ssh_folder, log_file)
                local_path = os.path.join(self.input_folder, log_file)
                sftp.get(remote_path, local_path)

            sftp.close()
        
        except paramiko.AuthenticationException:
            print("Authentication failed! Check credentials and SSH config.")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            ssh.close()

    # Import log data
    def read_data(self, other_input_folder = None):
        
        # other input folder only used in reimport_all, so we check archive folder
        if other_input_folder:
            input_folder = other_input_folder
            new_log = Log(input_folder, self.general_log_filename, False)
        else:
            input_folder = self.input_folder

            if not os.path.exists(self.input_folder):
                user_acknowledge("Input folder not found, this message should only be present in debug mode")
                return
            new_log = Log(input_folder, self.general_log_filename)

        # Get new folder name, it's path and it's archive path to be moved to
        new_name = new_log.return_folder_name()
        new_path = os.path.join(self.archive_folder, new_name)

        # other input folder only used in reimport_all, so all files are already there så ignore this
        if not other_input_folder:
            # Check if log exists in archive
            if os.path.exists(new_path):
                user_acknowledge("Logs already imported, please delete input folder as this cannot be done by the program")
                return

        # Rename and move if not called by reimport_all
        if not other_input_folder:
            os.rename(self.input_folder, new_path)

        # Add to file and save
        self.logs.append(new_log)
        if not other_input_folder:
            self.saveFile()

    # copies files to local, creates new log object, archives logs
    def fetch_new_logs(self):
        if self.copy_files_to_local():
            return
        self.read_data()

    # imports all data from archive folder, if something has been updated
    def reimport_all(self):
        self.logs = []
        self.properties = init_properties
        filenames = os.listdir(self.archive_folder)
        for folder_name in tqdm(filenames, desc="Reimporting files"):
            filepath = os.path.join(self.archive_folder, folder_name)
            self.read_data(filepath)
        self.saveFile()
        print(f"\n\n{len(self.logs)} logs reimported\n")

#%% Print data
    # Prints all imported logs with some attributes displayed
    def print_all(self, exclude = []):
        grid = []
        headers = ["ID", "TIME", "VERSION", "COMMENT", "LIVE_FEED", "RECORDING_FOLDER"]
        for i,log_object in enumerate(self.logs):

            general_log = log_object.return_attributes()

            # Add id to first index in output list if it should not be excluded
            if i+1 in exclude:
                line = [""]
            else:
                line = [i+1]

            # Adds data to list except ID
            for attribute in headers[1:]:

                # If attribute exists and is in string, concatinate, elif just add it, else add empty string
                if attribute in general_log.keys() and type(general_log[attribute]) == type(""):
                    line.append(general_log[attribute][:30])
                elif attribute in general_log.keys():
                    line.append(general_log[attribute])
                else:
                    line.append("")
                    
            grid.append(line)
        
        print(tabulate(grid, headers, tablefmt='rounded_grid'))

    # Can get more info from a specific log
    def display_data(self):

        # If there are no logs return
        if len(self.logs) == 0:
            return
        
        # Get ID for log to view
        comp = []       # List for all Log objects
        comp_ID = []    # List of IDs added

        while True:
            self.print_all(comp_ID)
            prompt = "Input ID to add to comparison (leave empty when done): "
            ID = input_int(len(self.logs), prompt)
            if not ID:
                break

            if ID in comp_ID:
                print("ID already added!")
            
            else:
                comp_ID.append(ID)
                comp.append(self.logs[ID-1])
                print(ID, "added to comparision!")
        
        # if no logs added to comparison, return
        if comp == []:
            return

        available_keys = []
        for i in comp:
            for key in i.return_keys():
                if not key in available_keys:
                    available_keys.append(key)
            available_keys.sort()

        keys = []
        while True:
            key = input_str(available_keys, keys)
            if not key:
                break
            
            keys.append(key)
            print(key, "added to comparision!")
        
        if keys == []:
            return
        
        modes = ["Box-plot", "Timeline"]
        mode = input_str(modes)
        
        if mode == modes[0]:
            self.display_combined(comp, keys)
        elif mode == modes[1]:
            self.display_timeline(comp, keys)

    # Combine data from all cameras and display box diagram
    def display_combined(self, comp, keys):

        if __debug__: # Change colors and language for report
            units = {
                "Time" : "Tid",
                "Frequency" : "Frekvens"
            }
            colors = {
                "median" : "black",
                "box" : "gray",
                "outliers" : "gray"
            }
        else:
            units = {
                "Time" : "Time",
                "Frequency" : "Frequency"
            }
            colors = {
                "median" : "red",
                "box" : "blue",
                "outliers" : "orange"
            }
        
        # Prepare data for plotting
        data = {key: [] for key in keys}
        outliers = {key: [] for key in keys}

        labels = []  # Store log version labels
        for log in comp:
            # Get the version label for x axis
            if __debug__: # Remove some information for the report
                labels.append(log.return_short_identifier())
            else:
                labels.append(log.return_identifier())
        
        for key in tqdm(keys, "Reformatting data for plots"):
            for log in comp:
                try:
                    # Get aggregated data and outliers for the key
                    data[key].append(log.return_all_agg_data(key))
                    outliers[key].append(log.return_all_outliers(key))
                except KeyError:
                    # Handle missing data for the key
                    data[key].append([None, None, None, None, None])
                    outliers[key].append([])
            

        plot_count = len(keys)

        columns = plot_count
        rows = 1

        width = 6 * plot_count
        height = 5
        fig, axes = plt.subplots(rows, columns, figsize=(width, height))

        if len(keys) == 1:  
            axes = [axes]  # Ensure iterable axes for single key

        for i, (ax, key) in tqdm(enumerate(zip(axes, keys)), "Creating plots"):

            # Extract aggregated statistics
            aggregated_data = np.array(data[key])
            mins = aggregated_data[:, 0]
            q1s = aggregated_data[:, 1]
            medians = aggregated_data[:, 2]
            q3s = aggregated_data[:, 3]
            maxs = aggregated_data[:, 4]

            plot_data = zip(labels, mins, q1s, medians, q3s, maxs, outliers[key])

            # Plot box for each key
            for j, (label, min_val, q1, median, q3, max_val, log_outliers) in enumerate(plot_data):

                if any(x is None for x in [min_val, q1, median, q3, max_val]):
                    continue

                # Draw the box
                box = Rectangle((j - 0.3, q1), 0.3 * 2, q3 - q1, color=colors["box"])
                ax.add_patch(box)
                # Draw the median
                ax.plot([j - 0.45, j + 0.45], [median, median], color=colors["median"], linewidth=2)
                # Draw the lower whisker
                ax.plot([j, j], [min_val, q1], color="black", linestyle="-")
                ax.plot([j - 0.3, j + 0.3], [min_val, min_val], color="black", linestyle="-")
                # Draw the upper whisker
                ax.plot([j, j], [q3, max_val], color="black", linestyle="-")
                ax.plot([j - 0.3, j + 0.3], [max_val, max_val], color="black", linestyle="-")

                # Plot outliers as individual points
                ax.scatter([j] * len(log_outliers), log_outliers, color=colors["outliers"], label="Outliers" if i == 0 and j == 0 else "")

                if __debug__:
                    print(label, min_val, q1, median, q3, max_val)
            
            # Tidying plots
            if not __debug__: # Remove titles for report
                if "(" in key:
                    title = key.split("(")[0].strip()
                    ax.set_title(title)
                else:
                    ax.set_title(key)

            ax.set_xticks(range(len(labels)))  # Set tick positions correctly
            ax.set_xticklabels(labels, rotation=30, ha="right")  # Set log version labels
            ax.grid(True)

            try:
                label = key.split("(")[-1][:-1]
            except IndexError:
                continue

            if label[-1] == "s":
                label = f"{units["Time"]} ({label})"
            elif label == "Hz":
                label = f"{units["Frequency"]} ({label})"
            ax.set_ylabel(label)

        # Show the plot
        plt.tight_layout()
        plt.show()

    def display_timeline(self, comp, keys):
        filenames = []
        for log in comp:
            data = log.return_filenames()
            for filename in data:
                if not filename in filenames:
                    filenames.append(filename)
        filenames.sort()
        
        while True:
            print("Choose file to plot")
            filename = input_str(filenames)
            if not filename:
                break

            data = []
            for i in comp:
                data.append(i.return_timeline(keys, filename))
            
            plot_count = len(comp)

            columns = plot_count
            rows = 1

            width = 6 * plot_count
            height = 5
            fig, axes = plt.subplots(rows, columns, figsize=(width, height))

            if plot_count == 1:  
                axes = [axes]  # Ensure iterable axes for single key

            for i, (ax, log, dict) in tqdm(enumerate(zip(axes, comp, data)), "Creating plots"):
                for j, key in enumerate(dict):
                    ax.plot(range(len(dict[key])), dict[key], label=key)
                
                ax.set_title(log.return_identifier())
                ax.set_xlabel("Log entry timeline (not same as elapsed time)")
                ax.legend()

            # Show the plot
            plt.tight_layout()
            plt.show()