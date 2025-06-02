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
from tqdm import tqdm
import os
import datetime as dt
import numpy as np
import bisect

#%% Custom modules
from functions import try_int_float_convert

class Log:
    def __init__(self, folderpath, general_filename, show_progress = True):

        # List of filepaths for files
        self.folderpath = folderpath
        self.general_filename = general_filename
        self.filenames = os.listdir(folderpath)

        # Import general
        self.general_data = {}  # Data denoting version, timestamp, etc
        if self.general_filename in self.filenames:
            filepath = os.path.join(folderpath, self.general_filename)
            self.import_file(filepath, True)
            self.format_general()
            self.filenames.remove(self.general_filename)
        else:
            self.general_data["TIME"] = dt.datetime.now().strftime("%Y-%m-%d %H;%M;%S")

        # If we want progress bar or not for importing data
        if show_progress:
            iterator = tqdm(self.filenames, "Importing data")
        else:
            iterator = self.filenames

        # Go through all files in folder and format
        values = {}
        self.agg_data = {} # Aggregated data for plotting
        self.outliers = {} # Outliers not > 1.5 * IQR
        for filename in iterator:
            filepath = os.path.join(folderpath, filename)
            data, _ = self.import_file(filepath)
            values[filename] = self.format_data(data)
            self.agg_data[filename], self.outliers[filename] = self.aggregate(values[filename])
        
        # Save all available keys
        self.keys = []
        for filename in self.agg_data.keys():
            self.keys += list(self.agg_data[filename].keys())
        self.keys = list(dict.fromkeys(self.keys))

        # Combine all data into one dict and aggregate all
        combined_values = {}
        for key in self.keys:
            combined_values[key] = []
            for filename in values:
                if key in values[filename].keys():
                    combined_values[key] += values[filename][key]
        self.all_agg_data, self.all_outliers = self.aggregate(combined_values)

#%% return data
    # name of folder with archived logs, it is the timestamp of the log
    def return_folder_name(self):
        return str(self.general_data["TIME"]).replace(":",";")
    
    # all filenames in folder
    def return_filenames(self):
        return self.filenames

    # version of code as identifier
    def return_identifier(self):
        try:
            if "RECORDING_FOLDER" in self.general_data.keys():
                return self.general_data["VERSION"] + "\n" + str(self.general_data["TIME"]) + "\n" + self.general_data["COMMENT"] + "\n" + self.general_data["RECORDING_FOLDER"]
            else:
                return self.general_data["VERSION"] + "\n" + str(self.general_data["TIME"]) + "\n" + self.general_data["COMMENT"]
        except KeyError:
            return "Version data missing" + str(self.general_data["TIME"])
    
    def return_short_identifier(self):
        try:
            return self.general_data["COMMENT"]
        except KeyError:
            return "Version data missing" + str(self.general_data["TIME"])

    # general log attributes
    def return_attributes(self):
        return self.general_data
    
    # keys for time data
    def return_keys(self):
        return self.keys

    # data for plotting from individual files
    def return_agg_data(self, filename, key):
        return self.agg_data[filename][key]
    def return_outliers(self, filename, key):
        return self.outliers[filename][key]
    
    # data for plotting from all files
    def return_all_agg_data(self, key):
        return self.all_agg_data[key]
    def return_all_outliers(self, key):
        return self.all_outliers[key]

#%% used for __init__
    # Convert text in files to dict
    def import_file(self, filepath, general = False):
        other = []
        data = {}

        with open(filepath, "r", encoding="utf-8") as file:
            for i,line in enumerate(file):

                # If it is general log file we import a little differently
                if general:
                    key, value = line.strip().split(":", 1)
                    self.general_data[key] = value.strip()
                
                # ':' represents data, otherwise it is just info and is put in 'other'
                elif ':' in line:
                    # Group data by key, the key is the string before the first ':'
                    # For every key there is a list with the values
                    key, value = line.strip().split(":", 1)

                    data.setdefault(key, []).append(value.strip())

                # If log data does not follow expected fomrat it is put in "other"
                # Used for debugging
                elif not line in other and filter == None:
                    other.append(line)
        
        return data, other

    def format_general(self):
        
        for key in self.general_data:

            # If 0/1 change to True/False, else convert to int or float
            if self.general_data[key] == "0":
                self.general_data[key] = False
            elif self.general_data[key] == "1":
                self.general_data[key] = True
            else:
                self.general_data[key] = try_int_float_convert(self.general_data[key])

        # Change TIME to datetime object
        self.general_data["TIME"] = dt.datetime.strptime(self.general_data["TIME"], "%Y-%m-%d %H:%M:%S")

    def convert_units_to_float(self, array, factor = 1):
        time_list = []
        for value in array:
            split = value.split(" ")
            try:
                time = float(split[0])
                time_list.append(time/factor)
            except ValueError:
                # Used to debug formatting, should never be printed in real world use case
                print("Could not convert to float:", split[0], "in", value)

        return time_list

    # formats data into dicts for plotting
    def format_data(self, data):

        # If data is {"key" : ["aaa=bb, ccc=dd"]} split into {"key, aaa" : "bb", "key, ccc" : "dd"}
        keys = list(data.keys())
        for key in keys:
            if "=" in data[key][0]:
                split_lists = list(map(lambda item: item.split(","), data[key]))
                for sublist in split_lists:
                    for item in sublist:
                        subkey, value = item.split("=")
                        subkey = subkey.strip()
                        data.setdefault(key + ", " + subkey, []).append(value)
                del data[key]
        
        # Convert elements to usable values for plotting
        values = {}
        keys = list(data.keys())
        for key in keys:
            # Assume all values follow same format
            first_split_space = data[key][0].split(" ")
            first_split_space_value = try_int_float_convert(first_split_space[0])
            is_int_or_float = isinstance(first_split_space_value, (int, float))

            # Value cannot be split so we try to convert to int or float
            if len(first_split_space) == 1:
                for i in data[key]:
                    values.setdefault(key, []).append(try_int_float_convert(i))

            # Assume the second is a unit
            elif len(first_split_space) == 2 and is_int_or_float:
                unit = first_split_space[1]
                factor = 1
                if unit == "us":
                    factor = 1000
                    unit = "ms"
                if unit == "ns":
                    factor = 1000000
                    unit = "ms"
                values[f"{key} ({unit})"] = self.convert_units_to_float(data[key], factor)
        
        # Calculate framerate with loop time
        if "Loop, Duration (ms)" in values.keys():
            for i in values["Loop, Duration (ms)"]:
                framerate = 1000 / i
                values.setdefault("Framerate (Hz)", []).append(framerate)

        return values

    # Aggregate data (calculate percentiles)
    def aggregate(self, data):
        aggregated_values = {}
        outliers = {}
        for key in data:
            try:
                # Throws an exception for TypeError if data in wrong format, just skip these datapoints then
                aggregated_values[key] = list(np.percentile(data[key], [0, 25, 50, 75, 100]))
            except TypeError as e:
                # Only important when debugging
                if __debug__:
                    print(f"Error processing key '{key}' with example value '{data[key][0]}': {type(e).__name__}: {e}")
                continue

            data[key].sort()
            outliers[key] = []

            min = aggregated_values[key][0]
            Q1 = aggregated_values[key][1]
            Q3 = aggregated_values[key][3]
            max = aggregated_values[key][4]

            IQR = Q3 - Q1

            # Outliers as defined by seaborn library for python
            min_limit = Q1 - 1.5 * IQR
            if min < min_limit:
                aggregated_values[key][0] = min_limit

                # Add outliers to dict
                lower_index = bisect.bisect_left(data[key], min_limit)
                outliers[key].extend(data[key][:lower_index])
            
            max_limit = Q3 + 1.5 * IQR
            if max > max_limit:
                aggregated_values[key][4] = max_limit

                # Add outliers to dict
                upper_index = bisect.bisect_right(data[key], max_limit)
                outliers[key].extend(data[key][upper_index:])

        return aggregated_values, outliers
    
    def return_timeline(self, filter, filename):

        filepath = os.path.join(self.folderpath, filename)
        try:
            data, _ = self.import_file(filepath)
        except FileNotFoundError as e:
            return {}
        data = self.format_data(data)

        filtered_data = {}
        for key in data:
            if key in filter:
                filtered_data[key] = data[key]
        
        return filtered_data
