#!/usr/bin/python
# This python script takes two args: raw lcm-log file and target lcm-log file path/name
# This script would read through the log file, then cast and insert a 
# cheetah_visualization_lcmt data everytime it reaches a simulator_lcmt data. 
# Please see readme for more information.
# 
# Type `thisScript --help` or 'thisScript -h' to get help of the script
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
            print("[Error]: you were trying to overwrite a existing file. \n\nIf you are sure, please use -f or --force flag when you call this script\n")
            return
        else:
            raise e

    sim_event = None
    sim_msg = None
    last_timestamp = 0
    buffer = []
    
    for event in log_raw:
        
        if event.channel == 'simulator_state':
            debugprint("Found sim data, process...")
            sim_event = event
            sim_msg   = simulator_lcmt.decode(event.data)

            # Cast visulation data
            vis_msg = cheetah_visualization_lcmt()
            vis_msg.x = sim_msg.p
            vis_msg.quat = sim_msg.quat
            for leg in range(4):
                for joint in range(3):
                    vis_msg.q[leg * 3 + joint] = sim_msg.q[leg][joint]
            # vis_msg.rgba = [0,0,0,0] # Color is not used
            vis_event = Event(sim_event.eventnum, sim_event.timestamp, 'main_cheetah_visualization', vis_msg.encode())
            
            debugprint("Write sim_event and vis_event")
            WriteEvent(log_gen, event)
            WriteEvent(log_gen, vis_event)
            
        else:
            debugprint("Other data: [%s], writing to new file." % event.channel)
            WriteEvent(log_gen, event)
            last_timestamp = event.timestamp # Set the last_timestamp to the most recent event
            

    print("Finished alignment, %d events were wrotten to %s" % (data_count, output_path))



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
