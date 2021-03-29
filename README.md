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

## Sensor Fusion | 3D Object Tracking
- [x] Implemented a method called "matchBoundingBoxes", which takes as input both the previous and the current data frames and outputs the ids of the matched regions of interest (i.e. the boxID property). Each bounding box was assigned the matches with the highest number of keypoint correspondences.
- [x] Computed the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.
- [x] Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.
- [x] Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

<p float="left">
  <img src="https://github.com/anaskhan496/Sensor-Fusion-Cpp/blob/main/3D-Object-Tracking/images/3D_tracking.gif" width="1100" />
</p>
