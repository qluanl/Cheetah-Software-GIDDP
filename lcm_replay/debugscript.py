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

import hashlib
import csv

# Compute absolute path to dir "lcm-types/python"
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
types_path = os.path.join(project_root, 'lcm-types/python')

# Add dir "lcm-types/python" to sys.path
if types_path not in sys.path:
    sys.path.append(types_path)

from simulator_lcmt import *


debug_mode = False

data_count = 0


def ProcessFile(input_path, output_path, overwrite=False):
    start_time = None
    last_timestamp = 0
    log_raw = EventLog(input_path, "r")
    with open(output_path, 'wb') as csvfile:
        writer = csv.writer(csvfile)

        writer.writerow(['number','timestamp','channel','datahash'])
        for event in log_raw:
            if start_time is None:
                start_time = event.timestamp

            hashdata = hashlib.md5(event.data).hexdigest()
            print("%d || %d || %s || %s" % (event.eventnum,event.timestamp,event.channel,hashdata))
            writer.writerow([str(event.eventnum),str(event.timestamp-start_time),event.channel,hashdata])
            assert (last_timestamp <= event.timestamp), "InternalError: increasing timestamp"
            last_timestamp = event.timestamp

    
    

def debugprint(*args, **kwargs):
    if debug_mode:
        print(*args, **kwargs)


if __name__ == "__main__":
    try:
        # Create parser
        parser = argparse.ArgumentParser(description='Read a lcm log file and print the events.')
        # Add args
        parser.add_argument('input_file_path', type=str, help='Path of the lcm log file to read from')
        parser.add_argument('output_file_path', type=str, help='CSV file to write to')

        parser.add_argument('--debug', action='store_true', help='Enable debug mode')
        parser.add_argument('-f','--force', action='store_true', help='Overwrite existing log file')

        # Parse args
        args = parser.parse_args()
        debug_mode = args.debug
        
        # 
        ProcessFile(args.input_file_path, args.output_file_path, overwrite=args.force)
    except KeyboardInterrupt:
        pass