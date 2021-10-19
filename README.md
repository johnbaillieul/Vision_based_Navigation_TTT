# Visual Navigation Using Sparse Optical Flow and Time-to-Transit

This repository contains code that allows navigation in unknown environments using only a monocular camera. The  work  takes  inspiration  from a  classic  paper  by  Lee  and  Reddish  (Nature,  1981, https://doi.org/10.1038/293293a0) in which they outline a behavioral strategy pursued by diving sea birds based on a visual cue called time-to-contact. The algorithm we introduce here is based on the estimation of a closely related quantity called time-to-transit (TTT).

The main code is represented by three nodes: the `/OpticalFlow node` is  responsible  for  the  Optical Flow (OF) estimation.  It  acquires  a  sequence  of  images  from the  camera  mounted  on  the  robot  and  it  extracts the  relevant  features  to  finally  compute  the  OF vectors. The `/TauComputation node` analyzes the array of keypoints with their velocities packed in the OF message, it computes tau values and it creates input signals for the controller. The `/Controller` selects the right control action to be used depending on the distribution of TTT values and it implements the Sense-Act cycle. For full details see our ICRA2022 paper, available on [arXiv](https://www.baillieul.org/Robotics/ICRA22_0387_Submission.pdf).

![Architecture Overview](assets/ICRArch2022.gif)

## License and attribution

If you use the code in this repository, please cite [our paper](https://www.baillieul.org/Robotics/ICRA22_0387_Submission.pdf). The code is available under the [BSD-2-Clause License](./LICENSE).

```bibtex
@inproceedings{bbzbaillieul2021,
      title={Visual Navigation Using Sparse Optical Flow and Time-to-Transit},
      author={Boretti, Chiara and Bich, Philippe and Zhang, Yanyu and Baillieul, John},
      booktitle={IEEE International Conference on Robotics and Automation},
      year={2022}
}
```

## Setup and use

Our navigation strategy can be used with any mobile robot equipped with a monocular camera.

First of all, run `optical_flow.py` to obtain the optical flow vectors for a certain number of features in the image (which is divided in three regions, and for every region the most robust features are tracked). The node generates `OpticaFlow.msg` containing position, velocity (px/s) of the features and the time delta (s) between the frames considered (to make real-time computation feasible on different platforms, a choosable but fixed number of frames can be skipped). Run the .py file with an the additional integer parameter 1 (e.g. `rosrun <package> optical_flow.py 1`) if you want to enable a real-time visual representation of the node's output.

![OF Node Output](assets/OFNode.gif)

Then, it's time to run `tau_computation.py` to obtain the average time-to-transit values for every Region-Of-Interest (ROI). The dimension and the position of those regions can be selected and they can be adapated to the environment in which the robot moves. The number of ROIs in the code presented here is fixed but it in principle it can be modified. In this case the controller must be notified of the change and different control laws must be used (stability for Tau Balancing control law with values coming from *n* different ROIs is guaranteed and simple to demonstrate, see [our paper](https://www.baillieul.org/Robotics/ICRA22_0387_Submission.pdf)). The node outputs `TauComputation.msg` used by the controller to select the proper control action. If the number of features in a ROI is not sufficiently high to guarantee a robust TTT estimation, a -1.0 is assigned to the specific region.

![Tau Computation Node Output](assets/tttnode.gif)

Finally, run `controller.py` which will make your robot move at a constant forward speed (by default: 1m/s). The proper steering command will be sent to the robot to align it to the center of the environment and to avoid obstacles. The controller implements the Sense-Act cycle and it is able to choose the right control law depending on the distribution of the time-to-transit values in the image. Tau Balancing based on 2 and 4 ROIs is implemented together with the Single Wall strategy which is a new control action that enables navigation in environments with few and localized features.

![Tests](assets/tests.gif)

Essential parameters for these three nodes are shown below. Other parameters exist and their default values are good for a large amount of environments, but better performances can be achieved by fine-tuning them to adapt the algorithm to the particular environment in which the robot has to move.

### Parameters for `optical_flow.py`

| Parameter            | Description                                                                                                   |    Example Value  |
| -------------------- | ------------------------------------------------------------------------------------------------------------- | :---------------: |
| ~image_sub_name      | name of the Image topic to subscribe.                                                                         | "front/image_raw" |
| ~num_ext_features    | max number of features to be detected in the two outer parts of the image.                                    |        250        |
| ~num_cen_features    | max number of features to be detected in the central part of the image.                                       |        150        |
| ~min_feat_threshold  | minimum % of tracked features that must still be in the image to avoid the reusage of the detector. Parameter must stay in range (0.0-1.0].      |        0.7        |

### Parameters for `tau_computation.py`

| Parameter            | Description                                                                                                   |    Example Value  |
| -------------------- | ------------------------------------------------------------------------------------------------------------- | :---------------: |
| ~x_init_l            | name of the Image topic to subscribe.                                                                         | "front/image_raw" |
| ~y_init_l            | max number of features to be detected in the two outer parts of the image.                                    |        250        |
| ~x_end_l             | max number of features to be detected in the central part of the image.                                       |        150        |
| ~x_end_l             | minimum % of tracked features that must still be in the detector. Parameter must stay in range (0.0-1.0].     |        0.7        |




## Virtual environments
To simulate the behavior of the algorithm in artificial and realistic environments, many scenarios are created in Gazebo. In this repository you will find the code to recreate them in the **GazeboWorlds** folder.  

![Gazebo Environment](assets/Environments.jpg)

We also developed patterns to be put on the walls with fixed feature density (Bernoulli distributions of features). The pattern can be created using the `Bernoulli_Textures.py` python scripts in which the density of the features can be set manually. Then the new pattern must be added to the Gazebo materials, the *bernoulli-x-M.png* generated by the python script has to be saved in `/usr/share/gazebo-x/media/materials/textures`. To use the new material in Gazebo, a *bernoulli-x-M.material* file must be created and saved in `/usr/share/gazebo-x/media/materials/scripts`. An example of *bernoulli-x-M.material* is:
```
material BernoulliMix/X
 {
    technique
    {
       pass
        {
          texture_unit
          {
            texture bernoulli-x-M.png
          }
       }  
    }
 }
 ```
 ![Features density](assets/FeaturesDensity.png)
