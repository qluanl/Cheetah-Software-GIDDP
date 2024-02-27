#!/usr/bin/python
# This python script takes two args: raw lcm-log file and target lcm-log file path/name
# This script would read through the log file, and remap the timestamp of each event
# based on the simulation data event. In other words, it would align the event 
# timestamp to the simulation time logged in simulation.
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

# Vars for debug and statistics
debug_mode = False
data_count = 0

# Main process function
def ProcessFile(input_path, output_path, overwrite=False):
    # Try to open input and output files and handle errors
    try:
        log_raw = EventLog(input_path, "r")
        log_alg = EventLog(output_path, mode='w', overwrite=overwrite)
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

    header = []
    buffer = []
    sim_event_a = None
    sim_event_b = None
    sim_msg_a = None
    sim_msg_b = None
    start_shift = 0

    header_len = 0
    posterior_len = 0
    
    # Enumerate through logged events
    for event in log_raw:
        
        if event.channel == 'simulator_state':
            debugprint("Found sim data, pushing [new_event -> event_b -> event_a -> discard]")
            sim_event_a = sim_event_b
            sim_event_b = event
            sim_msg_a   = sim_msg_b
            sim_msg_b   = simulator_lcmt.decode(event.data)
            if sim_event_a is None: # AKA. reaching the first sim_data event
                # Push buffer to header for later process
                debugprint("Push buffer to header");
                header = buffer
                buffer = []
                # Store the sim_start and real_start
                sim_start = Sec2MicroSec(sim_msg_b.time);
                real_start = sim_event_b.timestamp; # in microsecond
                # Store time shift between first sim time and first sim event
                # to ensure the first sim still happens at its timestamp in real time
                # (This is not important, but I prefer to have meaningful real time)
                start_shift = real_start - sim_start;
                header_len = len(header)
            else:
                # debugprint("SIMA: timestamp: %d, simtime: %d" % (sim_event_a.timestamp, Sec2MicroSec(sim_msg_a.time)))
                # debugprint("SIMB: timestamp: %d, simtime: %d" % (sim_event_b.timestamp, Sec2MicroSec(sim_msg_b.time)))
                # Re construct data
                if len(header):
                    # Align events in header and write to file
                    debugprint("Map and write header")
                    for head_e in header:
                        # debugprint("HEAD: timestamp: %d, simtime: %d" % (head_e.timestamp, Align_Time(sim_event_a, sim_msg_a, sim_event_b, sim_msg_b, head_e.timestamp)))
                        head_e.timestamp = Align_Time(sim_event_a, sim_msg_a, sim_event_b, sim_msg_b, head_e.timestamp) + start_shift
                        # Write head_e to file
                        WriteEvent(log_alg, head_e)
                    header = []
                
                debugprint("Write sim_event_a")
                s_e_a = copy.deepcopy(sim_event_a)
                s_e_a.timestamp =      Sec2MicroSec(sim_msg_a.time) + start_shift
                # Write s_e_a to file
                WriteEvent(log_alg, s_e_a)

                if sim_msg_b.time <= sim_msg_a.time:
                    print("Detacted a new simulation, dropping other events between two different simulations");
                    buffer = []
                    # Reset the time shift to the head sim event of new simulation
                    sim_start = Sec2MicroSec(sim_msg_b.time);
                    real_start = sim_event_b.timestamp; # in microsecond
                    start_shift = real_start - sim_start;

                
                # Align events in buffer and write to file
                debugprint("Map and write buffer")
                for buffer_e in buffer:
                    # debugprint("BUFF: timestamp: %d, simtime: %d" % (buffer_e.timestamp, Align_Time(sim_event_a, sim_msg_a, sim_event_b, sim_msg_b, buffer_e.timestamp)))
                    buffer_e.timestamp = Align_Time(sim_event_a, sim_msg_a, sim_event_b, sim_msg_b, buffer_e.timestamp) + start_shift
                    # Write buffer_e to file
                    WriteEvent(log_alg, buffer_e)
                buffer = []
            
        else:
            debugprint("Other data: [%s], pushing to buffer" % event.channel)
            buffer.append(event)
            
    # We have read through the file, but left a sim data and buffer haven't been wrotten to log file
    debugprint("Write sim_event_b")
    s_e_b = copy.deepcopy(sim_event_b)
    s_e_b.timestamp =      Sec2MicroSec(sim_msg_b.time) + start_shift
    # Write s_e_b to file
    WriteEvent(log_alg, s_e_b)

    debugprint("Map and write buffer as posterior")
    # Align events in buffer and write to file
    for buffer_e in buffer:
        buffer_e.timestamp = Align_Time(sim_event_a, sim_msg_a, sim_event_b, sim_msg_b, buffer_e.timestamp) + start_shift
        # Write buffer_e to file
        WriteEvent(log_alg, buffer_e)
    posterior_len = len(buffer)
    buffer = [];

    print("Finished alignment, %d events were wrotten to %s" % (data_count, output_path))

    debugprint("Finished alignment with %d headers, %d posteriors, and %d total data" % (header_len, posterior_len, data_count))
        
def Sec2MicroSec(time_sec):
    return time_sec * 1e6
    
def WriteEvent(logfile, event):
    global data_count
    data_count += 1
    logfile.write_event(event.timestamp, event.channel, event.data)
    
# Linear interpolate the timestamp based on the sim time and event timestamp of two simulation data
def Align_Time(event_a, msg_a, event_b, msg_b, timestamp):
    # Field value is the sim_time
    assert (event_b.timestamp - event_a.timestamp != 0), "Error: event timestamp must be different!"
    slope = float(Sec2MicroSec(msg_b.time - msg_a.time)) / float(event_b.timestamp - event_a.timestamp);
    aligned_time = slope * (timestamp - event_a.timestamp) + Sec2MicroSec(msg_a.time);
    return aligned_time


def debugprint(*args, **kwargs):
    if debug_mode:
        print(*args, **kwargs)



if __name__ == "__main__":
    try:
        # Create parser
        parser = argparse.ArgumentParser(description='Read a lcm log file with simulator state, and generate a new file with timestamp aligned with simulation time.')
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
