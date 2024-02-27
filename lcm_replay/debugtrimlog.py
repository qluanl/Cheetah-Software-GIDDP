#!/usr/bin/python
# This python script takes two args: raw lcm-log file and target lcm-log file path/name
# This script would read through the log file, and remap the timestamp of each event
# based on the simulation data event. In other words, it would align the event 
# timestamp to the simulation time logged in simulation.
# 
# Note: Do not move the directory of this script, it relay on its relative path 
# in the project to find the lcm-types.
# 
# Author: Shenggao Li

from __future__ import print_function
import sys
import os
import argparse
import copy
from lcm import *

# Compute absolute path to dir "lcm-types/python"
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
types_path = os.path.join(project_root, 'lcm-types/python')

# Add dir "lcm-types/python" to sys.path
if types_path not in sys.path:
    sys.path.append(types_path)

from simulator_lcmt import *
from cheetah_visualization_lcmt import *


debug_mode = False

data_count = 0


def ProcessFile(input_path, output_path, overwrite=False):
    try:
        log_raw = EventLog(input_path, "r")
        log_gen = EventLog(output_path, mode='w', overwrite=overwrite)
    except IOError, e:
        print(e);
        print("Please check your log file path.\n Or use -h option to see the guide");
        return
    except ValueError, e:
        if str(e) == "Refusing to overwrite existing log file unless overwrite is set to True":
            print("[Error]: you were trying to overwrite a existing file. \n\nIf you are sure, please use --overwrite flag when you call this script\n")
            return
        else:
            raise e


    counter = 0
    
    for event in log_raw:
        WriteEvent(log_gen, event)
        counter += 1
        if counter > 100:
            return



def WriteEvent(logfile, event):
    global data_count
    data_count += 1
    logfile.write_event(event.timestamp, event.channel, event.data)



def debugprint(*args, **kwargs):
    if debug_mode:
        print(*args, **kwargs)



if __name__ == "__main__":
    try:
        # Create parser
        parser = argparse.ArgumentParser(description='Read a lcm log file with simulator state, and generate a new file with visualization data.')
        # Add args
        parser.add_argument('input_file_path', type=str, help='Path of the lcm log file to read from')
        parser.add_argument('output_file_path', type=str, help='Path and name of the lcm log file to write to')

        parser.add_argument('--debug', action='store_true', help='Enable debug mode')
        parser.add_argument('-f','--force', action='store_true', help='Overwrite existing log file')

        # Parse args
        args = parser.parse_args()
        debug_mode = args.debug
        
        # 
        ProcessFile(args.input_file_path, args.output_file_path, overwrite=args.force)
    except KeyboardInterrupt:
        pass
