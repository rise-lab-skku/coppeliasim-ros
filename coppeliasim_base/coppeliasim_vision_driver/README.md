# CoppeliaSim Vision Sensor ROS Interface
# 1. Overview
The `coppeliasim_vision_dirver` package probide a interface for vision sensor on the CoppeliaSim.

# 2. Nodes
## 2.1 coppeliasim_vision_sensor_driver
### 2.1.1 Subcribed Topics
* ~image_raw ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) 
    * Depending on the `~vision_sensor_mode` ros paramter, it contains a color image or a depth image.
* ~camera_info ([sensor_msgs/Camera_info](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html))
    * Valid information of this topic is camera intrinsic `K`, image height `height`, image width `widht`.
### 2.1.2 Paramters
* ~vision_sensor_object_name (str)
    * Object name on the CoppeliaSim.
* ~vision_sensor_mode (str)
    * Vision sensor mode option. Supporting options are `"color"` and `"depth"`.

# 3. Usage
## 3.1 Coppeliasim settings
### 3.1.1 Color camera
* object name: "color_camera"
### 3.1.2 Depth camera
* object name: "depth_camera"

## 3.2 Launch ROS node
### 3.2.1 Color camera
```bash
roslaunch coppeliasim_vision_driver coppeliasim_color_camera.launch
```
### 3.2.2 Depth camera
```bash
roslaunch coppeliasim_vision_driver coppeliasim_depth_camera.launch
```
### 3.2.3 RGB-D camera
```bash
roslaunch coppeliasim_vision_driver coppeliasim_rgbd_camera.launch
```