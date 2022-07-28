### Quickstart Guide For Simulation

* NOTE: This assumes that the vision package is installed with the environment setup already

1. Open Terminator and split into 5 windows 
2. In one of the terminals run the jackal gazebo simulation command `roslaunch vision_based_navigation_ttt jackal_world.launch config:=front_flea3`
   or for example you could run another command `roslaunch vision_based_navigation_ttt realsense_jackal_realistic.launch`
3. In the second terminal window run the optical flow node command `rosrun vision_based_navigation_ttt optical_flow.py`
4. In the third terminal window run the tau computation node command `rosrun vision_based_navigation_ttt tau_computation.py`
5. In the fourth terminal window run the controller node command `rosrun vision_based_navigation_ttt controller.py`
6. If you want to restart the simulation (respawn the Jackal to the beginning) run this command in the fifth window: `rosservice call /gazebo/reset_simulation "{}"`


### Quickstart Guide For Real Robot

* NOTE: This assumes that the vision package is installed with the environment setup already

1. Open Terminator and split into 4 windows all of which are `ssh -X` into the robot
2. In first terminal window run the realsense camera `roslaunch realsense2_camera rs_rgbd.launch`
3. In the second terminal window run the optical flow node command `rosrun vision_based_navigation_ttt optical_flow.py 1`
4. In the third terminal window run the tau computation node command `rosrun vision_based_navigation_ttt tau_computation.py`
5. In the fourth terminal window run the controller node command `rosrun vision_based_navigation_ttt controller.py`


### ~/.bashrc requirements
* NOTE: some quick commands to enable your terminal instance work with the rights paths. This should ideally be at the end of the ~/.bashrc file
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/jackal_ws/src/Vision_based_Navigation_TTT/models:/usr/share/gazebo-11
export GAZEBO_RESOURCE_PATH=~/jackal_ws/src/Vision_based_Navigation_TTT/GazeboWorlds:${GAZEBO_RESOURCE_PATH}
export JACKAL_URDF_EXTRAS=~/jackal_ws/src/Vision_based_Navigation_TTT/urdf/realsense.urdf.xacro
