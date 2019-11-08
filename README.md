# PCR-Pro: 3D Sparse and Different Scale Point Clouds Registration and Robust Estimation of Information Matrix For Pose Graph SLAM
Accepted in IEEE Int. Conf. on CYBER Technology in Automation, Control, and Intelligent Systems 2018.

#### Features
* Pass through Filter Added
* Scale detection
* For both indoor and outdoor using the libpointmatcher and opengv and covariance calculated from SHOT descriptors

#### Prerequisites:
PCR-Pro has been tested on OpenCV 2.8.9 and Eigen 3. 

Mandatory Tools:
* [Libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
* [OpenGV](https://github.com/laurentkneip/opengv)

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
#### To Run
```
./main <img1> <img2> <pointcloud2> <pointcloud1>
```

```sh
 ./build/main data/Result_1/robot1TestImg36.jpg data/Result_1/robot2TestImg16.jpg data/Result_1/robot2pcd16.pcd data/Result_1/robot1pcd36.pcd 
```

#### View PCD Results in paraview
Paraview accepts **VTK** file, PCD / PLY can be converted to VTK using the PCL Library. Example has been shown below.
```sh
pcl_pcd2vtk <pointcloud.pcd> <pointcloud.vtk>
```
