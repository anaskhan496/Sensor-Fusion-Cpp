# SFND 3D Object Tracking

## Code Architecture
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/Code-Architecture.PNG" width="1100" height="400" />

## Keypoint detectors, descriptors, and methods to match them between successive images 
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/keypoint_matching.png" width="1300" height="200" />

## Estimating Time-to-Collision Using Camera Sensor
- Idea: Compute Keypoint matches between successive images and observe the relative distances between them over time.
- Problem: Keypoints need to be isolated on the objects. i.e exclude road surface and static objects.

## Estimating Time-to-Collision Using Lidar Sensor
- Compute distance to objects from successive lidar measurements
- Problem: Crop lidar points to only include necessary objects. Cropping not reliable as objects may not directly be in front of the sensor

## Load Lidar Point Cloud
- Removed ground plane from lidar point cloud
- Changed the color of the Lidar points such that X=0.0m corresponds to red while X=20.0m is shown as green with a gradual transition in between
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/Lidar_top_view.png" width="800" height="550" />

## YOLO V3 Object Detection
- Initialize the parameters (nmsThreshold and confThreshold)
- Prepare the model (weights, cfg file, and load neural net)
- Generate 4D blob from image (number N x channel C x height H x width W)
- Run forward pass through the network
- Scan boxes and filter based on confThreshold
- Post process neural net to perform Non-Maximum Suppression
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/Yolo1.png" width="1300" height="200" />
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/Yolo.png" width="700" height="275" />

## Cluster Lidar Points with 2D Detection Region-of-Interest
- Conversion of Lidar points from Euclidean to Homogeneous co-ordinates.
- P_rect_00 – Intrinsic Camera Matrix. Camera-to-Image Co-ordinates
- R_rect_00 – Rectifying rotation to make images co-planar (Required for stereo camera setup used in KITTI dataset)
- (R|T)_velo_to_cam - Extrinsic camera matrix. Transform world frame/lidar frame to camera co-ordinate frame
- Transformation Equation - Y = P_rect_xx * R_rect_00 * (R|T)_velo_to_cam * X, X is the lidar points in homogeneous co-ordinates

## Cluster Lidar Points with 2D Detection Region-of-Interest : Issues
- Object detection returns ROI that are too large and thus overlap into parts of the scene that are not a part of the enclosed object.
- Strictly rectangular shape of bounding boxes. Rarely fits the physical outline of the enclosed objects and can overlap with other objects in the scene. Poses a problem during Lidar point cloud association.

## Match 3D Objects
- Extract keypoints between current and previous frame and associate keypoint matches to regions of interest
- Associate a given bounding box with the keypoints it contains
- Find by which bounding boxes keypoints are enclosed in these frames. Store the matched candidates box id.
- Associate bounding boxes with highest occurrences
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/3d_objects_info.png" width="989" height="600" />

## Estimate Time-to-Collision Using Camera and Lidar Sensors
<img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/ttc_final.png" width="1200" height="300" />



## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
