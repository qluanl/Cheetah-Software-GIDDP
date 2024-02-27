#!/usr/bin/python
# This python script would echo the control_parameter_request_lcmt 
# message to the control_parameter_respones_lcmt back to the simulation
# so that the simulation would think there is a robot communicating with it
# 
# Note: Do not move the directory of this script, it relay on its relative path 
# in the project to find the lcm-types.
# 
# Author: Shenggao Li

import sys
import os
import lcm

# Compute absolute path to dir "lcm-types/python"
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
types_path = os.path.join(project_root, 'lcm-types/python')

# Add dir "lcm-types/python" to sys.path
if types_path not in sys.path:
    sys.path.append(types_path)

from control_parameter_request_lcmt import *
from control_parameter_respones_lcmt import *



lc = lcm.LCM()


def echo_handler(channel, data):
    request = control_parameter_request_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   requestNumber   = %s" % str(request.requestNumber))
    print("")


    # respond:
    response = control_parameter_respones_lcmt()

    response.requestNumber = request.requestNumber  # acknowledge that the set has happened
    response.parameterKind = request.parameterKind  # just for debugging print statements
    response.requestKind   = request.requestKind    # just for debugging print statements
    response.value         = list(request.value)    # just for debugging print statements
    response.name          = list(request.name)     # just for debugging print statements

    # send response
    lc.publish("interface_response", response.encode())

def test_send():
    response = control_parameter_respones_lcmt()

    response.requestNumber = 1  # acknowledge that the set has happened
    response.parameterKind = 2  # just for debugging print statements
    response.requestKind   = 3  # just for debugging print statements

    # send response
    lc.publish("interface_response", response.encode())


if __name__ == "__main__":
    # test_send()
    subscription = lc.subscribe("interface_request", echo_handler)
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass
