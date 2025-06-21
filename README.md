
![Visitor Badge](https://visitor-badge.laobi.icu/badge?page_id=Hannibal730.3D-Lidar-wall-detection_ws_ws)

# Real-Time Wall Detection from 3D LiDAR Data in ROS

This repository is for real-time wall detection using 3D LiDAR data on autonomous mobile robots.  It subscribes to a Velodyne VLP-16 LiDAR point cloud (`/velodyne_points`), extracts vertical planar surfaces (walls) via PCL filters and RANSAC segmentation, and publishes wall point clouds for navigation.  Planar fitting is a common technique to detect surfaces like walls and floors in point clouds. In practice the Velodyne VLP-16 driver publishes a `sensor_msgs/PointCloud2` on `/velodyne_points`, which `wall_detect_node.cpp` processes to find walls. The complementary `wall_compress_node.cpp` then projects these walls onto the ground plane and downsamples them for compact mapping.


* ## Raw pointcloud
    ![Image](https://github.com/user-attachments/assets/a13de885-5d0b-4c22-9a26-8bcbd6c83b6e)



* ## Raw & Detected walls & Compressed walls
    ![Image](https://github.com/user-attachments/assets/d5885917-0a70-46a8-8bc6-dbf568cefc94)


---
<br>

## Features

* **Downsample raw LiDAR data:** Uses a PCL `VoxelGrid` filter to reduce point cloud density by partitioning space into 3D voxels and replacing points in each voxel with their centroid.
* **Vertical planar segmentation:** Fits plane models to the point cloud using RANSAC, isolating large vertical planes (walls) as inlier points.
* **Height filtering:** Applies a PCL `PassThrough` filter on the z-axis to keep points within a specified vertical range (e.g. ground to ceiling).
* **Wall compression:** Projects detected wall points onto the z=0 plane using PCL’s ProjectInliers (plane) filter, and then applies a fine VoxelGrid downsampling (3 cm leaf size) to produce a 2D representation of the walls.
* **Real-time processing:** Designed to run online; VoxelGrid downsampling greatly reduces data size, enabling faster RANSAC segmentation and lower computational load.
* **ROS-native:** Built on ROS Noetic with standard messages (`PointCloud2`), compatible with existing Velodyne drivers and PCL-ROS tooling.

---
<br>

## ROS Topics

* **Subscribed Topics:**

  * `/velodyne_points` (`sensor_msgs/PointCloud2`): Raw 3D scan from the Velodyne VLP-16 LiDAR.
* **Published Topics:**

  * `/detected_wall` (`sensor_msgs/PointCloud2`): Point cloud of extracted wall (vertical plane) points.
  * `/compressed_wall` (`sensor_msgs/PointCloud2`): Planar-projected, downsampled wall points (all z=0).

---
<br>

## Pipeline

### (1) wall\_detect\_node.cpp

![Image](https://github.com/user-attachments/assets/eb5d5b71-2520-4a21-a707-2b4c15f6c24e)

1. **Subscribe to `/velodyne_points`:** Receives raw `PointCloud2` messages from the LiDAR.
2. **VoxelGrid Filter:** Apply a PCL `VoxelGrid` filter to downsample the cloud. A 3D grid of tiny boxes is overlaid on the points and each occupied voxel is replaced by the centroid of its points.
3. **Height (PassThrough) Filter:** Use a PCL `PassThrough` filter on the z-axis to remove points outside a set height range (e.g. keep 0 m < z < 2 m). This focuses the segmentation on plausible wall heights.
4. **Planar Segmentation (RANSAC):** Run `pcl::SACSegmentation` with `SACMODEL_PLANE` and `SAC_RANSAC` to find the dominant planar surface in the filtered cloud.  Points within a distance threshold of the fitted plane are marked as inliers. In effect, this extracts points belonging to the largest vertical plane (i.e., a wall) in the scene.
5. **Publish `/detected_wall`:** The inlier points (the segmented wall) are published as a new `PointCloud2`. These points represent the detected wall surfaces in the LiDAR view.

### (2) wall\_compress\_node.cpp
![Image](https://github.com/user-attachments/assets/0f73d6c2-ddbe-4ac9-88de-fb0066dea0c8)

1. **Subscribe to `/detected_wall`:** Receives the wall point cloud from the first node.
2. **Project to Ground Plane:** Use PCL’s `ProjectInliers` filter with a plane model of `ax+by+cz+d=0` (with a=b=d=0, c=1) to project all points onto the z=0 (XY) plane. This “flattens” the wall into 2D by setting all z-coordinates to 0.
3. **VoxelGrid Downsampling:** Apply another `VoxelGrid` filter (with a leaf size of 0.03 m) to the projected points. This coarsely downsampled grid significantly reduces the number of points while preserving the wall outline.
4. **Publish `/compressed_wall`:** The result is a sparse 2D representation of the wall, published as `PointCloud2`. This compressed wall map is useful for real-time navigation or mapping (e.g., as a costmap or occupancy grid).

Each step uses standard PCL/ROS filters (e.g. `pcl_ros::VoxelGrid`, `pcl_ros::PassThrough`, `pcl_ros::SACSegmentation`, `pcl_ros::ProjectInliers`).

---
<br>

## Dependencies

* **ROS Noetic** (tested on Ubuntu 20.04) with `roscpp` and `sensor_msgs` – for the ROS framework and message types.

* **Point Cloud Library (PCL)** 1.9+ with `pcl_ros` and `pcl_conversions` – for point cloud filtering and segmentation. *PCL-ROS is the preferred bridge for 3D point cloud processing in ROS.*
  
* **Velodyne drivers** (e.g. `velodyne_pointcloud`, `velodyne_driver`) – to produce `/velodyne_points` from the hardware.

---
<br>

## Installation

Clone the package into your ROS workspace and build it with catkin:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Hannibal730/3D-Lidar-wall-detection_ws.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

This will compile the `wall_detect_node.cpp` and `wall_compress_node.cpp` into executables. Make sure your Velodyne LiDAR is configured and drivers are running (producing `/velodyne_points`) before launching this package.

---
<br>

## Launch Instructions

A convenient launch file `wall_detection.launch` starts both nodes. For example:

```bash
roslaunch wall_detection_pkg wall_detection.launch
```

This launch file should handle node startup and any necessary topic remappings. Ensure that a running `roscore` and the Velodyne point cloud publisher are active before running the above command. After launching, you will see `/detected_wall` and `/compressed_wall` topics being published.

---
<br>

## Potential Applications

* **Autonomous Vehicles & HD Mapping:** Detecting vertical structures (walls, building facades, road barriers) from LiDAR is critical for detailed environment mapping.  High-density LiDAR scans with vertical surface information enable the creation of precise 3D urban models and high-definition maps for self-driving navigation.
* **Indoor Mobile Robotics:** Wall detection aids navigation and SLAM in corridors or rooms (e.g. wall-following robots, indoor mapping). Identifying walls simplifies planning by providing clear boundaries.
* **Surveying & Construction:** Automated extraction of planar building surfaces from scans can be used for creating as-built building models or checking structural layouts.
* **Augmented Reality & Simulation:** Fast wall extraction helps reconstruct 3D scenes from LiDAR scans, useful in AR applications or simulated environments where knowing vertical boundaries is important.

---
<br>

## Questions or Issues
If you have any questions or issues, feel free to ask them in the Issues section.

---
<br>

## Lisence
This project is released under the Apache License 2.0
