# CoppeliaSim Manipulator ROS Interface
# 1. Overview
The `coppeliasim_manipulator_driver` package probides a interface for controlling industrial manipulators on the CoppeliaSim. This package follows [ROS-Indutrial](http://wiki.ros.org/Industrial) program. See more datails on [industrial_robot_client](http://wiki.ros.org/industrial_robot_client) and [joint_trajectory_action](http://wiki.ros.org/joint_trajectory_action)

# 2. Nodes
## 2.1 coppeliasim_manipulator_driver
### 2.1.1 Published Topics
* joint_states ([sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)) 
    * Joint state for all joints of the connected robot.
* feedback_states([control_msgs/FollowJointTrajectory](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html))
    * Provide feedback of current vs. desired joint position (and velocity/acceleration).
### 2.1.2 Subsribed Topics
* joint_path_command ([trajectory_msgs/JointTrajectory Message](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html))
    * Execute a new motion trajectory on the robot.
### 2.1.3 Prameters
* controller_joint_names ([str, str, str, ..])
    * A list containing all joints the driver should assume control over. The joint name must be the same as the name defined in urdf and the name in Coppeliasim.

# 3. Usage
## 3.1 CoppeliaSim settings
### 3.1.1 Improt URDF
First, prepare your robot description file as [URDF](http://wiki.ros.org/urdf) format. If your robot description file is written in [xacro](http://wiki.ros.org/xacro) format, you can easily convert it to `URDF` format as follows. **Note that joint names on the robot description is very important.**
```bash
xacro robot.urdf > robot.xacro
```
Click `[Plugins]-[Import URDF]` button on the top-side of CoppeliaSim to import URDF.

### 3.1.2 Link properties
Check all respondable links are imported with proper option as follows.
* Shape: Should not be [Simple random shape](https://www.coppeliarobotics.com/helpFiles/en/shapes.htm)
* Body is repondable: Enable
* Body is dynamic: Enable

### 3.1.3 Joint properties
Enable control loop for all joints that you want control.
* Control loop enabled: Enable

## 3.2 Connect to MoveIt!
[MoveIt!](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) includes motion planning algorithms that can create complex movement paths. [Instructions on how to connect the manipulator driver with MoveIt! are given here](./../../../moveit_config/README.md).