# ORB_SLAM3 Python Bindings

This repository provides Python bindings for the [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) system.

To enable a smooth installation process, the original ORB_SLAM3 package has been lightly modified.
These changes are managed using CMake and a patch file (cmakefix.patch) that are applied to the original ORB_SLAM3 repository at compile time.

## Dependencies
```sh
sudo apt install libglew-dev libboost-serialization-dev libssl-dev libopencv-dev libeigen3-dev
```
## Installation
```sh
git clone git@github.com:PRBonn/ORB-SLAM-Python.git
cd ORB-SLAM-Python
make
```
Note: if you are running out of memory during installation, reduce the number of threads for compilation using `export CMAKE_BUILD_PARALLEL_LEVEL=4`. Or increase your /swap memory. This repository is only tested in ubuntu24 and ubuntu22

## Examples
Check out the examples to try it out on TUM and EuroC datasets. The examples automatically download the ORB slam vocabulary.
If you don't prefer this you can also download your own vocabulary and pass this as a parameter to the examples.

For tum dataset run:
```sh
python3 examples/tum_pipeline.py <path to tum dataset> --visualize

```

For EuroC dataset run:
```sh
python3 examples/euroc_pipeline.py <path to euroc dataset> --visualize

```

## Docker
The repository also provides docker image files to build in ubuntu24 and ubuntu22. To build and run the project in Ubuntu24 docker:
```sh
make docker && make run
```
For ubuntu22 docker:
```sh
make ubuntu22 && make run22
```

## Citation
If you use this library for any academic work, please cite the original [paper](https://arxiv.org/pdf/2007.11898).

```bibtex
@article{campos2021tro,
    title   = {{ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM}},
    author  = {Campos, Carlos and Elvira, Richard and G\´omez, Juan J. and Montiel, Jos\'e M. M. and Tard\'os, Juan D.},
    journal = {IEEE Transactions on Robotics},
    volume  = {37},
    number  = {6},
    pages   = {1874--1890},
    year    = {2021}
 }
```

## Acknowledgements:
This work was as part of [Sumanth's](https://github.com/sumanthrao1997) master thesis at [Institue of Photogrammetry and Robotics Bonn](https://www.ipb.uni-bonn.de/). Special thanks to [Ignacio Vizzo](https://github.com/nachovizzo) and [Tiziano Gudagnino](https://github.com/tizianoGuadagnino) for their support and invaluable guidance. And a big thank you to [Saurabh Gupta](https://github.com/saurabh1002) for carefully reviewing and contributing to the codebase.
