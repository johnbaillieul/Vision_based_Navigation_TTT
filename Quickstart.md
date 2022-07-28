### Quickstart Guide For Simulation

* NOTE: This assumes that the vision package is installed with the environment setup already

1. Open Terminator and split into 5 windows 
2. In one of the terminals run the jackal gazebo simulation command `roslaunch vision_based_navigation_ttt jackal_world.launch config:=front_flea3`
3. In the second terminal window run the optical flow node command `rosrun vision_based_navigation_ttt optical_flow.py`
4. In the third terminal window run the tau computation node command `rosrun vision_based_navigation_ttt tau_computation.py`
5. In the fourth terminal window run the controller node command `rosrun vision_based_navigation_ttt controller.py`
6. If you want to restart the simulation (respawn the Jackal to the beginning) run this command in the fifth window: `rosservice call /gazebo/reset_simulation "{}"`
