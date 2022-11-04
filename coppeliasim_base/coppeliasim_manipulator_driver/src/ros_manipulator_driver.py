#! /usr/bin/python
# -*- coding: utf-8 -*-
import string
import random

import numpy as np
from scipy import interpolate

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryFeedback

from coppeliasim_master.msg import CoppeliaSimSynchronous
from coppeliasim_remote_api.bluezero import b0RemoteApi


def id_generator():
    id = ''.join(
        random.choice(string.ascii_uppercase + string.digits)
        for _ in range(8))
    return id


class GetJointPositionCallback:
    def __init__(self, dst, name):
        self.dst = dst
        self.name = name

    def __call__(self, msg):
        self.dst[self.name] = msg[1]


class CoppeliaSimManipulatorDriver(object):
    """Coppeliasim 6-aixs manipulator ros bridge.

    It works like real manipulator ros drivers based on the ROS-Industrial
    [Specification](http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec).
    See details at [industrial_robot client](http://wiki.ros.org/industrial_robotclient)

    Attributes:
        client (`RomoteApiClient`): CoppeliaSim b0remoteapi.RemoteApiClient
          instantce.
        joint_names (`list`): Joint names. It should be corresponding with URDF,
          MoveIt, controllers and Coppeliasim joint names
    """
    def __init__(self, client, client_ID, joint_names):
        self.client = client
        self.client_ID = client_ID

        # get simulation paramters once
        self.obj_handles = {
            name:
            self.client.simxGetObjectHandle(name,
                                            self.client.simxServiceCall())[1]
            for name in joint_names
        }
        self.sim_time_step = self.client.simxGetFloatParameter(
            'sim.floatparam_simulation_time_step',
            self.client.simxServiceCall())[1]

        # set simulation mode as synchrnous mode
        self.do_next_step = True
        self.client.simxGetSimulationStepStarted(
            self.client.simxDefaultSubscriber(self.simulation_step_start_cb))

        # instanse variables for manipulator
        self.joint_position = {name: 0.0 for name in joint_names}
        self.joint_trajectory_points = []
        self.joint_trajectory_joint_names = []

        # create CoppeliaSim publishers
        self.sim_joint_state_pub = {
            name: self.client.simxDefaultPublisher()
            for name in joint_names
        }

        # create CoppeliaSim subscribers
        for name in joint_names:
            self.client.simxGetJointPosition(
                self.obj_handles[name],
                self.client.simxDefaultSubscriber(GetJointPositionCallback(
                    self.joint_position, name)))

        # create ros publishers
        self.ros_joint_states_pub = rospy.Publisher(
            "joint_states",
            JointState,
            queue_size=1,
        )
        self.ros_control_states_pub = rospy.Publisher(
            "feedback_states",
            FollowJointTrajectoryFeedback,
            queue_size=1,
        )
        self.coppeliasim_synchronous_trigger_pub = rospy.Publisher(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)

        # create ros subscribers
        rospy.Subscriber(
            "joint_path_command",
            JointTrajectory,
            self.ros_joint_path_cb,
        )
        self.coppeliasim_synchronous_trigger_sub = rospy.Subscriber(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)

        rospy.sleep(0.2)

    @property
    def joint_states_msg(self):
        """Joint states msg of the robot manipulator."""
        # make joint_states msg
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_position.keys()
        msg.position = self.joint_position.values()
        return msg

    @property
    def control_states_msg(self):
        """Control states msg of the robot manipulator."""
        # make control msg
        msg = FollowJointTrajectoryFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = self.joint_position.keys()
        msg.actual.positions = self.joint_position.values()
        msg.desired.positions = self.joint_position.values()
        msg.error.positions = [0, 0, 0, 0, 0, 0]
        return msg

    @staticmethod
    def interpolate_joint_trajectory(msg, time_step):
        # allocate the pos and time list
        trajectory_pos = []
        trajectory_time = []

        # get trajectory information
        trajectory_time = [
            p.time_from_start.secs + p.time_from_start.nsecs * 1e-9
            for p in msg.points
        ]
        trajectory_pos = [p.positions for p in msg.points]

        # make numpy
        trajectory_time = np.array(trajectory_time)
        trajectory_pos = np.array(trajectory_pos).transpose()

        # remove duplicated points on the trajectory
        _, unique_indices = np.unique(trajectory_time, return_index=True)
        trajectory_time = trajectory_time[unique_indices]
        trajectory_pos = trajectory_pos[:, unique_indices]

        # adjust the final point time to times of time_step
        new_points_num = round(trajectory_time[-1] / time_step)
        trajectory_time[-1] = (new_points_num - 1) * time_step

        # generate new trajectory points time
        new_trajectory_time = np.linspace(trajectory_time[0],
                                          trajectory_time[-1], new_points_num)
        new_trajectory_time = np.arange(0, trajectory_time[-1] + 0.001,
                                        time_step)

        # interpolate the trajectory
        if len(trajectory_time) < 3:
            f = interpolate.interp1d(trajectory_time,
                                     trajectory_pos,
                                     kind='linear')
        elif len(trajectory_time) == 3:
            f = interpolate.interp1d(trajectory_time,
                                     trajectory_pos,
                                     kind='quadratic')
        else:
            f = interpolate.interp1d(trajectory_time,
                                     trajectory_pos,
                                     kind='cubic')
        new_trajectory_pos = f(new_trajectory_time).transpose()

        return msg.joint_names, new_trajectory_pos.tolist()

    def ros_joint_path_cb(self, msg):
        """Joint path callback fuction of the `joint_path_command` ros topic.

        Interpolates joint trajectory from the planned joint trajectory msg
        regarding CoppeliaSim simulation step time. And generate the exexutable
        joint trajectory queue from the interpolated joint trajectory.

        Args:
            msg (`JointTrajectory`): Joint trajectory of the planned path.
        """
        if msg.points:
            joint_names, joint_trajectory = self.interpolate_joint_trajectory(
                msg, self.sim_time_step)
            self.joint_trajectory_joint_names = joint_names
            self.joint_trajectory_points = joint_trajectory

    def excute_trajectory(self):
        """Excute robot from a trajectory queue.

        Dequeue desired joint position from the planned trajectory queue
        and set joint target position at each simulation step.
        """
        # if the queue is not empty, set desired pos.
        if self.joint_trajectory_points:
            desired_joint_positions = self.joint_trajectory_points.pop(0)

            bundle = zip(self.joint_trajectory_joint_names,
                         desired_joint_positions)
            for name, desired_joint_position in bundle:
                self.client.simxSetJointTargetPosition(
                    self.obj_handles[name], desired_joint_position,
                    self.sim_joint_state_pub[name])

    def fake_client_activation(self):
        """Fake client activation

        If CoppeliaSim can not detect client's activation, it automately
        deativate the client. So we fool the simulation that the client
        is active.
        """
        self.client.simxSetIntSignal('a', 0, client.simxDefaultPublisher())
    
    def coppeliasim_synchronous_done(self):
        """
        Call this function when the client has finished doing all the calculations.
        To call this the object must have the attributes:
            - self.sequence_number
            - self.client_ID
        --------------
        Returns
        --------------
        None : None
        """
        if not hasattr(self, "sequence_number"):
            return 
        # Publish the trigger to all other synchrnous clients
        trigger_msg = CoppeliaSimSynchronous()
        trigger_msg.stamp = rospy.Time.now()
        trigger_msg.sequence_number = self.sequence_number
        trigger_msg.client_ID = self.client_ID
        self.coppeliasim_synchronous_trigger_pub.publish(trigger_msg)
        return

    def coppeliasim_synchronous_cb(self, msg):
        """
        Callback for synchronous operation. If message is received from "coppeliasim_master", 
        "self.do_next_step" is set to True
        and "self.sequence_number" is updated.
        --------------
        msg : CoppeliaSimSynchronous 
            ros message containing a time stamp,  sequence_number and client_ID

        Returns
        --------------
        Bool : True/False
            True if message from "coppeliasim_master"
        """
        if msg.client_ID == "coppeliasim_master":
            self.sequence_number = msg.sequence_number
            self.do_next_step = True
            return True
        else:
            return False


    def simulation_step_start_cb(self, msg):
        # publish target pose to sim
        self.excute_trajectory()
        self.fake_client_activation()

    def simulation_step_done_cb(self):
        # publish ros topics
        self.ros_joint_states_pub.publish(self.joint_states_msg)
        self.ros_control_states_pub.publish(self.control_states_msg)


    def step_simulation(self):
        """
        Perform one step of the simulation.
        --------------
        Returns
        --------------
        None : None
        """
        while not self.do_next_step:
            self.client.simxSpinOnce()
            # Check if simulation on. Makes for a clean exit
            if hasattr(self, "sequence_number"):
                if self.sequence_number == -1:
                    rospy.signal_shutdown("Stopping ROS: Manipulator Driver")
                    return
            rospy.sleep(0.002)

        self.simulation_step_done_cb()
    
        self.do_next_step = False
        self.coppeliasim_synchronous_done()



if __name__ == '__main__':
    # init ros
    rospy.init_node('coppeliasim_manipulator_driver')
    joint_names = rospy.get_param('controller_joint_names')

    # init sim
    client_ID = 'b0RemoteApi_robot_client_{}'.format(id_generator())
    client = b0RemoteApi.RemoteApiClient(client_ID,'b0RemoteApi')
    sim = CoppeliaSimManipulatorDriver(client, client_ID, joint_names)

    while not rospy.is_shutdown():
        sim.step_simulation()
