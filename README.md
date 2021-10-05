# Visual Navigation Using Sparse Optical Flow and Time-to-Transit

This repository contains code that allows navigation in unknown environments using only a monocular camera. The  work  takes  inspiration  from a  classic  paper  by  Lee  and  Reddish  (Nature,  1981, https://doi.org/10.1038/293293a0) in which they outline a behavioral strategy pursued by diving sea birds based on a visual cue called time-to-contact. The algorithm we introduce here is based on the estimation of a closely related quantity called time-to-transit (TTT).

The main code is represented by three nodes: the `/OpticalFlow node` is  responsible  for  the  Optical Flow (OF) estimation.  It  acquires  a  sequence  of  images  from the  camera  mounted  on  the  robot  and  it  extracts the  relevant  features  to  finally  compute  the  OF vectors. The `/TauComputation node` analyzes the array of keypoints with their velocities packed in the OF message, it computes tau values and it creates input signals for the controller. The `/Controller` selects the right control action to be used depending on the distribution of TTT values and it implements the Sense-Act cycle. For full details see our ICRA2022 paper, available on [arXiv](https://arxiv.org/abs/2010.11326).

![Architecture Overview](assets/ICRArch2022.gif)

## License and attribution

If you use the code in this repository, please cite [our paper](https://arxiv.org/abs/2010.11326). The code is available under the [BSD-2-Clause License](./LICENSE).

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

First of all, run `optical_flow.py` to obtain the optical flow vectors for a certain number of features in the images (the image is divided in three regions, and for every region the most robust features are tracked). The node generates an `OpticaFlow` message containing position, velocity (px/s) of the features and the time delta (s) between the the frames considered (to make real-time computation feasible on different platforms, a choosable but fixed number of frames can be skipped).

Then, it's time to run `tau_computation.py` to obtain the average time-to-transit values for every Region-Of-Interest (ROI). The dimension and the position of those regions can be selected and it can be adapated to the environment in which the robot moves. The number of the ROIs in the code presented here is fixed but it in principle it can be modified. In this case the controller must be notified of the change and different control laws must be used (stability for Tau Balancing control laws with values coming from *n* ROIs is guaranteed and simple to demonstrate, see [our paper](https://arxiv.org/abs/2010.11326)). The node outputs a `TauComputation` message used by the controller to select the proper control action.

Essential parameters for these three nodes are shown below. Other parameters exist and their default values are good for a large number of environments, but better performances can be achieved by changing those values to adapt the algorithm to the particular environment in which the robot has to move.

