# Sensor-Fusion-Cpp
Lidar, Camera, Radar Sensor Projects in C++

## Lidar Obstacle Detection
- [x] Able to stream KITTI Dataset PCD across multiple frames
- [x] Bounding boxes enclose vehicles, and the pole on the right side of the vehicle. There is one box per detected object.
- [x] Most bounding boxes can be followed through the lidar stream, and major objects don't lose or gain bounding boxes in the middle of the lidar stream.
- [x] The code used for segmentation uses the 3D RANSAC algorithm 
- [x] The code used for clustering uses the Euclidean clustering algorithm along with the KD-Tree
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/raw_lidar_data_1.gif" width="400" />
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/final_lidar_data_1.gif" width="400" />
</p>

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/anaskhan496/Sensor-Fusion-Cpp.git
$> cd Lidar-Obstacle-Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)

