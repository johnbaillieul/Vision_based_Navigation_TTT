# URDF Models for the Various Jackals Tested

## General Simulation
For general simulation, the default Jackal model with a tilted flea3 camera was used. Please refer to the top
directory in this repo for more information on using this model.

## BU Jackal
The BU Jackal has a different payload, which includes a tower and small stand. On those structures are mounted a ptz camera & realsense d435 and a VLP16 lidar respectively.

This depends on the apt packages ros-melodic-realsense2-camera and ros-melodic-realsense2-description ros-melodic-gazebo-plugins

To use, enter this command before launching view_model.launch
` export JACKAL_URDF_EXTRAS=$HOME/jackal_ws/src/vision_based_navigation_ttt/urdf/BU_Jackal.urdf.xacro `

![BU Jackal Picture](assets/BU_Jackal.jpg)
