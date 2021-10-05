# Visual Navigation Using Sparse Optical Flow and Time-to-Transit

This repository contains code that allows navigation in unknown environments using only a monocular camera. The  work  takes  inspiration  from a  classic  paper  by  Lee  and  Reddish  (Nature,  1981, https://doi.org/10.1038/293293a0) in which they outline a behavioral strategy pursued by diving sea birds based on a visual cue called time-to-contact. The algorithm we introduce here is based on the estimation of a closely related quantity called time-to-transit (TTT).

The main code is represented by three nodes: the `/OpticalFlow node` is  responsible  for  the  Optical Flow (OF) estimation.  It  acquires  a  sequence  of  images  from the  camera  mounted  on  the  robot  and  it  extracts the  relevant  features  to  finally  compute  the  OF vectors. The `/TauComputation node` analyzes the array of keypoints with their velocities packed in the OF message, it computes tau values and it creates input signals for the controller. The `/Controller` selects the right control action to be used depending on the distribution of TTT values and it implements the Sense-Act cycle. For full details see our ICRA2022 paper, available on [arXiv](https://arxiv.org/abs/2010.11326).

![Overview of approach](assets/ICRArch2022.gif)

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

First of all, run `optical_flow.py` to obtain the optical flow vectors for a certain number of features in the images (the image is divided in three regions, and for every region the most robust features are tracked). The node generates an `OpticaFlow` message with the following structure:
| **OpticalFlow.msg  **          | 
| ------------------------------ |
| **Header** header              | 
| **uint32** height              |
| **uint32** width               | 
| **float32** dt                 |
| **float32[]** x                |
| **float32**[] y                |
| **float32[]** vx               |
| **float32[]** vy               |

Essential parameters for these three nodes are shown below. Other parameters exist and their default values are good for a large number of environments, but better performances can be achieved by changing those values to adapt the algorithm to the particular environment in which the robot has to move.
