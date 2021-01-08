# Obstacle Detection Using Lidar 

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Import Raw Lidar Data
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/raw_lidar_data_1.gif" width="400" />
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/raw_lidar_data_2.gif" width="400" />
</p>

## Downsampling and Filtering - Voxel Grid and Region of Interest 
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/filter_lidar_data_2.gif" width="400" />
</p>

## RANSAC Algorithm for Planar Segmentation
- Purple colored point cloud : Ground plane 
- Blue colored point cloud : Obstacles 
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/ransac_lidar_data_2.gif" width="400" />
</p>

## Final Data After KD-Tree, Euclidean Clustering, and Bounding Box Generation
- Apart from the obstacles, the traffic pole on the right also gets a bounding box around it
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/final_lidar_data_1.gif" width="400" />
</p>

- The algorithm is able to track the bicyclist's movement right in front of the ego car in every frame and also the pedestrians in the frame. 
<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/Lidar-Obstacle-Detection/videos/final_lidar_data_2.gif" width="400" />
</p>

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
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
