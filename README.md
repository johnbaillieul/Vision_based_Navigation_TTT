# Visual Navigation Using Sparse Optical Flow and Time-to-Transit

[![License: BSD-2-Clause](https://img.shields.io/github/license/QVPR/teach-repeat.svg?style=flat-square)](./LICENSE)
[![stars](https://img.shields.io/github/stars/QVPR/teach-repeat.svg?style=flat-square)](https://github.com/Tobias-Fischer/ensemble-event-vpr/stargazers)
[![GitHub issues](https://img.shields.io/github/issues/QVPR/teach-repeat?style=flat-square)](https://github.com/QVPR/teach-repeat/issues)
[![GitHub repo size](https://img.shields.io/github/repo-size/QVPR/teach-repeat.svg?style=flat-square)](./README.md)
[![QUT Centre for Robotics Open Source](https://img.shields.io/badge/collection-QUT%20Robotics-%23043d71?style=flat-square)](https://qcr.github.io)

This repository contains code that allows navigation in unknown environments using only a monocular camera. The  work  takes  inspiration  from a  classic  paper  by  Lee  and  Reddish  (Nature,  1981, https://doi.org/10.1038/293293a0) in which they outline a behavioral strategy pursued by diving sea birds basedon a visual cue called time-to-contact. The algorithm we introduce here is based on the estimation of a closely related concept called time-to-transit. For full details see our ICRA2022 paper, available on [arXiv](https://arxiv.org/abs/2010.11326).

The main code is represented by three nodes: the `/OpticalFlow node` is  responsible  for  the  Optical FLow (OF) estimation.  It  acquires  a  sequence  of  images  from the  camera  mounted  on  the  robot  and  it  extracts the  relevant  features  to  finally  compute  the  OF vectors. The `/TauComputation node` analyzes the array of keypoints with their velocities packed in the OF message, it computes tau values and it creates input signals for the controller.

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
