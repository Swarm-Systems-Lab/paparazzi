import ast
import csv
import os
import re
import sys
import time
from os import path, getenv

import numpy as np

DEF_PPRZ_HOME = path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../'))

sys.path.append(getenv("PAPARAZZI_HOME", DEF_PPRZ_HOME) + "/var/lib/python")
sys.path.append(getenv("PAPARAZZI_SRC", DEF_PPRZ_HOME) + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class SSReader:
    def __init__(self, dirpath: str):
        self.dirpath = dirpath
        self.data_dict = {}
        self.min_len = 9999999
        self.load_all_csv_in_folder(dirpath)

    def parse_value(self, value):
        # Check if the value contains a comma, indicating a point in space
        if ',' in value:
            # Use ast.literal_eval to safely evaluate the string as a tuple, then convert to a NumPy array
            parsed_value = np.array(ast.literal_eval(f'({value})'))
            return parsed_value
        else:
            # If not a point, return the value as a float
            return float(value)

    def load_csv_to_dict(self, filename):
        """Load a CSV file into a dictionary with lists of parsed values."""
        data_dict = {}
        
        # Open the CSV file with tab delimiter
        with open(filename, mode='r') as csvfile:
            csvreader = csv.DictReader(csvfile, delimiter='\t')
            
            # Fill the dictionary with parsed data
            for row in csvreader:
                for header in csvreader.fieldnames:
                    if header in ("DWM1001_DATA:enu_pos", "DWM1001_DATA:sigma", "DWM1001_DATA:centroid", "DWM1001_DATA:asc_dirc"): # TODO
                        if header not in data_dict:
                            data_dict[header] = []
                        data_dict[header].append(self.parse_value(row[header]))
        
        return data_dict

    def convert_dict_to_numpy_arrays(self, data_dict):
        """Convert lists in the dictionary to appropriate NumPy arrays or keep them as lists if necessary."""
        numpy_arrays = {}
        for key, value in data_dict.items():
            # Check if the list contains only scalars
            if all(isinstance(v, (float, int)) for v in value):
                numpy_arrays[key] = np.array(value)
            else:
                numpy_arrays[key] = value  # Keep the list as is if it contains arrays or mixed types
                
            if len(numpy_arrays[key]) < self.min_len:
                self.min_len = len(numpy_arrays[key])
        
        return numpy_arrays

    def load_all_csv_in_folder(self, folder_path):
        # List all CSV files in the given folder
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.csv'):
                # Get the full file path
                file_path = os.path.join(folder_path, file_name)
                
                # Load CSV data into a dictionary
                data_dict = self.load_csv_to_dict(file_path)
                
                # Convert the dictionary to appropriate NumPy arrays or lists
                numpy_arrays = self.convert_dict_to_numpy_arrays(data_dict)
                
                # Store the result in the self.data_dict dictionary, using the file name (without extension) as the key
                
                file_date, file_id = self.extract_date_and_id_from_filename(file_name)
                    
                if file_date not in self.data_dict:
                    self.data_dict[file_date] = {}
                
                if file_id not in self.data_dict[file_date]:
                    self.data_dict[file_date][file_id] = {}
                    
                self.data_dict[file_date][file_id] = numpy_arrays
                
    def extract_date_and_id_from_filename(self, filename):
        """Extract the date and ID from the filename in the format [date]_[time]_[id].csv."""
        match = re.match(r"(\d{2}_\d{2}_\d{2}__\d{2}_\d{2}_\d{2})_(\d+)", filename)
        if match:
            date = match.group(1)
            file_id = match.group(2)
            return date, file_id
        return None, None

class SSfakeIvyproducer:
    def __init__(self, dirpath: str, date: str):
        self.data = SSReader(dirpath)
        self.date = date
        self.ivy_interface = IvyMessagesInterface("SS")
        self.debug_msg = PprzMessage("telemetry", "DWM1001_DEBUG")
        self.debug_msg.set_value_by_name("nei_addresses", [55602,7090,5923])
        self.data_msg = PprzMessage("telemetry", "DWM1001_DATA")
    
    def send_debug(self):
        for key in self.data.data_dict[self.date]:
            # print("debug")
            self.ivy_interface.send(self.debug_msg, sender_id=int(key))
        
    def send_data(self, line_n):
        for ac_id, file_data in self.data.data_dict[self.date].items():
            for data_key, data_values in file_data.items():
                self.data_msg.set_value_by_name(data_key.split(':')[1], data_values[line_n])
            # print(f'data_msg ac_id {ac_id} line_n {line_n}')
            self.ivy_interface.send(self.data_msg, sender_id=int(ac_id))
        
    def send_data_loop(self):
        # print(self.data.data_dict)
        for i in range(self.data.min_len):
            time.sleep(0.02)
            self.send_data(i)
            if i % 10 == 0:
                self.send_debug()

def main():
    fakeProducer = SSfakeIvyproducer("./data_dir", "24_09_13__12_09_10")
    fakeProducer.send_data_loop()

if __name__ == '__main__':
    main()
