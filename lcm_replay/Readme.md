## Guide of lcm_replay
`Shenggao Li`

This folder contains some helper scripts that helps us replay the logged lcm data from either a real robot experiment or a simulation using Cheetah software.

> The python scripts were programmed and tested in python 2.7 and should be easy to transfer to python 3 if necessary.

### Replay a lcm-log data from a real robot experiment
Replay from a real experiment should be easy:
- Run the python script `control_param_responder` in any directory. This script would response to the set control parameters request from the GUI and pretend as a robot.
>The script would automatically find the lcm-types as long as the script and the types are still in their default folders.
- Open simulation via `./sim/sim` in build folder, and select robot mode, click 'run'.
- Use `lcm-logplayer-gui <YOUR_LOG_FILE>` or `lcm-logplayer <YOUR_LOG_FILE>` to stream to data you logged.

Now you shall see the visualization of your experiment

### Replay a lcm-log data from a simulation
This could be tricky since there was one type of data is missing in simulation. I have added such data streaming with this commit.
- If you are replaying the lcm stream logged with code after this commit, you can just run the exact same procedure above as a real robot experiment.


- If you are replaying an old lcm stream logged before this commit of lcm-replay, you have to process your logged file by the python script `inject_visual_data_from_sim_log`.
  - Run the python script `inject_visual_data_from_sim_log <YOUR_LOG_FILE> <TARGET_FILE>` to convert. The script would read the simulation state data then cast and insert a visualization data right after the simulation event. 
  - Now follow the same steps of replay an experiment, but use the modified log file by the script.

### Simulation time alignment

Your lcm-log file is recorded in the real time when the lcm event happens in the network. However, your simulation time might not be aligned with real time:
- In some cases, you might run the simulation in a fast or slow speed via change the sim_speed parameter, for example 10x fastforward to save your time or 0.1x slow motion. In those cases, it is not a big problem since the sim time is still linear about the real time, you can just change the replay speed to have any speed you want.
- We are more worried about the following case: if you have a very slow MPC solver and you are using the synchronize feature of the simulation (i.e., the simulation would wait the MPC solver to complete). Then since your MPC solve time is not a constant, and (even it is constant), MPC solve only happens occasionally within your simulation data. You cannot align to the sim time simply by changing the replay speed. 

Python script `sim_time_aligner` would align the lcm timestamp to the simulation time. After a non real speed simulation, you can use this script to remap the timestamp so that the lcm-logplay can play the animation in real time (and you can do slow motion/fast motion using the slow/fastforward functionality of lcm-logplayer-gui). And even if you are running your simulation in real time, it is still recommanded to process your file via this aligner script since simulation is **trying** to reach the target sim speed, but it is actually slightly slower than real time (about 0.98x speed). After alignment, you can ensure the replay is under 1.0x speed of the simulation time.

#### Run script
Run command `sim_time_aligner <YOUR_LOG_FILE> <TARGET_FILE>` to remap the timestamp.
