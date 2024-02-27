
## Dev log

`Shenggao Li`

I noticed that if we run the simulation in the robot mode, it would send the control parameters to the robot and wait for the response. 

However, in our case (record data replay), we do not have a robot that can response and we shouldn't. 

Therefore, we need a script to pretend as a robot to fool the simuation. 

That's it exactlly how the python script `interface_request_echo.py` does here. 



Now I have reviewed the lcm data from simulation and experiment. The data are different:

In simulation, simulation would send a data 'simulator_state', while in experiment, robot would send data 'main_cheetah_visualization'.

I would do the following:

1. [Done] Add a lcm publish in simulation so that it would automatically send the visualization as well. Therefore, for any feature simulation, we can directly replay it.
2. [Done] Create a python script that can converge the simulation data to the visualization data. So that any old data can be replayed after convertion
	> The script should read a lcm-log file, read through its lcm-event and write another lcm-log file with some modification: Everytime it detectes a simulation data, it would create a new visualization data event right after the simulation event. Then the script should increment the event number count and change all subsequent events to make sure the event number is valid.
3. [Done] For my case: I have a very slow simulation and I want to log the data and replay. I need a script that read through a lcm-log file, and modifiy the time stamp.

Improved the lcm-type find script based on the root directory of the entire project. As a result, one can run my python scripts from **any directory**. 

Improved the command line options and help

Improved the exception handling

Bug fix: bug of the timestamp computation (missing calling Sec2Microsec function). This bug would resulting horribly not monotonically increasing timestamp, which would cause the replay stuck when we drag the time bar.

Notice that the lcm-log file can take same timestamp for two adjacent events, therefore, in the `generateVisualizationData.py` script, we just log the visualization event with same timestamp as the simulation state data. 

Also notice that there is a small bug with lcm-logger: sometimes when two events come in at the same time, the logger might have the later event logged first and ealier event logged secondly, as a result, the timestamp would be not monotonically increasing. However, it seems OK for replay.
