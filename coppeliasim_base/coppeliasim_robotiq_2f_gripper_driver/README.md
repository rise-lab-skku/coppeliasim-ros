# Robotiq 2F 85 Gripper CoppeliaSim ROS Interface
# 1. Overview
The `coppeliasim_robotiq_2f_gripper_dirver` package probide a interface for controlling Robotiq 2F gripper on the CoppeliaSim.

# 2. Nodes
## 2.1 coppeliasim_robotiq_2f_gripper_driver
### 2.1.1 Published Topics
* Robotiq2FGripperRobotInput([Robotiq2FGripper_robot_input](http://docs.ros.org/en/kinetic/api/robotiq_2f_gripper_control/html/msg/Robotiq2FGripper_robot_input.html))
    * Robot input registers. See more details on [here](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf).

### 2.1.2 Subscribed Topics
* Robotiq2FGripperRobotOutput([Robotiq2FGripper_robot_output](http://docs.ros.org/en/kinetic/api/robotiq_2f_gripper_control/html/msg/Robotiq2FGripper_robot_output.html))
    * Robot output registers. See more details on [here](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf).
* ~grasp ([std_msgs/Bool](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) 
    * `true`: open gripper, `false`: close gripper.

# 3. Usage
## 3.1 CoppeliaSim settings
### 3.1.1 Dynamic properties of all **repondable** links of the girpper
1. Click `[Show dynamics properties dialog]`.
2. Enable `[Body is reponsable]` and `[Body is dynamic]`.
3. Click `[Compute mass & inertia...]` and put proper density property (ex 1000.0).

### 3.1.2 Joint properties
1. Physics Engine: `Vortex`.
2. Set joint properties as follow.

> Note: Robotiq 2F gripper is based on four bar linkage kinemtics. But implementing four bar linkage kinematic(closed kinematic) on the CoppeliaSim is impossible. So here we mimick the four bar linkage kinematics as follows. The left outer knuckle joint(`figer_joint`) controls all other joints, which means all other joint states depends on the left outer knuckle joint.

|joint name|Pos. min|Pos. range|Mode|Motor enabled|Maximum torque|Control loop enabled|Depened joint|Multiplication joint|
|---|---|---|---|---|---|---|---|---|
|finger_joint|0|45.84|Tourque/force mode|Enabled|1000|Enabled|None|1|
|left_inner_knuckle_joint|0|50.17|Tourque/force mode|Enabled|0|Disabled|finger_joint|1|
|left_inner_finger_joint|-50.17|50.17|Tourque/force mode|Enabled|0|Disabled|finger_joint|-1|
|right_outer_kucckle_joint|0|50.17|Tourque/force mode|Enabled|0|Disabled|finger_joint|1|
|right_inner_knuckle_joint|0|50.17|Tourque/force mode|Enabled|0|Disabled|finger_joint|1|
|right_inner_finger_joint|-50.17|50.17|Tourque/force mode|Enabled|0|Disabled|finger_joint|-1|

|Items|Location|
|---|---|
|Pos. min|`[Scene Ojbect Properties]`-`[Configuration]`|
|Pos. range|`[Scene Ojbect Properties]`-`[Configuration]`|
|Mode|`[Scene Ojbect Properties]`-`[Mode]`|
|Moter enabled|`[Scene Ojbect Properties]`-`[Motor properties]`|
|Maximum torque|`[Scene Ojbect Properties]`-`[Motor properties]`|
|Control loop enabled|`[Scene Ojbect Properties]`-`[Control properties]`|
|Depened joint|`[Scene Ojbect Properties]`-`[Motor properties]`-`[Edit engine sepci..]`-`[Vortex properties]`-`[Joint dependency]`|
|Multiplication joint|`[Scene Ojbect Properties]`-`[Motor properties]`-`[Edit engine sepci..]`-`[Vortex properties]`-`[Joint dependency]`|

### 3.2 Launch ROS node
```bash
roslaunch coppeliasim_robotiq_2f_gripper_driver coppeliasim_robotiq_2f_gripper.launch
```