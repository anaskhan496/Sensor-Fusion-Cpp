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
