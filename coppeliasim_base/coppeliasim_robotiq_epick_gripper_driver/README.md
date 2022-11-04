# Robotiq EPick Gripper CoppeliaSim ROS Interface
# 1. Overview
The `coppeliasim_robotiq_epick_gripper_driver` package provides a interface for controlling Robotiq EPick gripper on the CoppeliaSim.

# 2. Nodes
## 2.1 coppeliasim_robotiq_epick_gripper
### 2.1.1 Subcribed Topics
* /coppeliasim_synchronous ([CoppeliaSimSynchronous])
* ~grasp ([std_msgs/Bool](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) 
    * `true`: turn on vacuum, `false`: turn off vacuum.

# 3. Usage
## 3.1 CoppeliaSim settings
### 3.1.1 Object Properties
All objects that we want to pick up with the suction gripper must be set as:
* Detectable
* Collidable
### 3.1.2 Flexibility of suction cup
1. Select `[joint_x]` or `[joint_y]` in EPick suction gripper tree
2. Click `[Joint Dynamic Properties]`
3. Click `[Spring-damper mode]`
4. Set the value of spring as desired (recommended: > r^2*k_def/2 (~500))

### 3.1.2 Simulation properties
1. Physics Engine: `Vortex`.
2. dt: ` <= 25ms`

## 3.2 Python settings:
The gripper has more modes of operation:
* **Complex mode (everything)**: Force will be exerted on objects when contacting them. The seal is evaluated with the new model as well as the external forces when we turn on the vacuum.
* **Complex mode without exerting force**: The objects will not move when we contact them until the vacuum is turned on. The seal is evaluated with the new model as well as the external forces when we turn on the vacuum.
* **Complex mode without external force limit**: After the vacuum is turned on the object will fall off only when the vacuum is turned off.
* **Simple mode**: No analysis of surface or forces. If the gripper is contacting an object and the vacuum is turned on it will grab that object. 

To choose between various modes go into `[ros_robotic_epick_gripper_driver.py]`. In the `if __name__ == "main":` part you will see the following lines of code:

```python
# --------------------------------------------------------------------------------
# ----- HERE CHANGE THE PARAMETERS HOW THE SUCTION GRIPPER WILL BE SIMULATED -----
contact_force_simulation = True
seal_infinite_strength = False
use_complex_model = True
# --------------------------------------------------------------------------------
```
Change the parameters inside to get the desired mode. 

> Note: Some combination of parameters may not be possible. The code will still run bit the parameters will not have any effect on the simulation.


### 3.3 Launch ROS node
```bash
roslaunch coppeliasim_robotiq_epick_gripper_driver coppeliasim_robotiq_epick_gripper.launch
```