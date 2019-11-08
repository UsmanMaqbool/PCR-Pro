# PCR-Pro: 3D Sparse and Different Scale Point Clouds Registration and Robust Estimation of Information Matrix For Pose Graph SLAM

If you use PCR-Pro, we appreciate it if you cite our paper:
```
@inproceedings{bhutta2018pcr,
    title = {{PCR-Pro}: 3D Sparse and Different Scale Point Clouds Registration and Robust Estimation of Information Matrix For Pose Graph {SLAM}},
    author={Bhutta, M Usman Maqbool and Liu, Ming},
    booktitle={2018 IEEE 8th Annual International Conference on CYBER Technology in Automation, Control, and Intelligent Systems ({CYBER})},
    pages={354--359},
    year={2018},
organization={IEEE}
}
```
#### Video Demonstration 

[![PCR-Pro](https://img.youtube.com/vi/jVjiV6BOH10/0.jpg)](https://www.youtube.com/watch?v=jVjiV6BOH10 "PCR-Pro")


#### Prerequisites:
PCR-Pro has been tested on OpenCV 2.8.9 and Eigen 3. 

Mandatory Tools:
* [Libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
* [OpenGV](https://github.com/laurentkneip/opengv)
* Written in C++ 11
  
Optional:

* [Paraview](https://www.paraview.org/), for better visualization of point cloud, Please use the paraview, it will take VTK file as input.

### To Run the Code

Change **Camera Intrinsic Parameters** in the main file code before using this, replace this with your camera calibration results.
```
float KF_fx = 467.87;
float KF_fy = 514.317;
float KF_cx = 271.828;
float KF_cy = 246.906;
```

#### Build and Run
```
mkdir build && cd build
cmake ../
make

./main <img1> <img2> <pointcloud2> <pointcloud1>
# Example
./build/main data/Result_1/robot1TestImg36.jpg data/Result_1/robot2TestImg16.jpg data/Result_1/robot2pcd16.pcd data/Result_1/robot1pcd36.pcd 
```

#### View PCD Results in paraview
Paraview accepts **VTK** file, PCD / PLY can be converted to VTK using the PCL Library. Example has been shown below.
```sh
pcl_pcd2vtk <pointcloud.pcd> <pointcloud.vtk>
```

#### Author
[**M Usman Maqbool Bhutta**](usmanmaqbool.github.io) | usmanmaqbool@outlook.com