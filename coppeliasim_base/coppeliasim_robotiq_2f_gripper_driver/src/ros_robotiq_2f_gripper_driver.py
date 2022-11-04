#! /usr/bin/python
import string
import random

import numpy as np

import rospy
from std_msgs.msg import Bool

from coppeliasim_master.msg import CoppeliaSimSynchronous
from coppeliasim_remote_api.bluezero import b0RemoteApi

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as InputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as OutputMsg


def id_generator():
    id = ''.join(
        random.choice(string.ascii_uppercase + string.digits)
        for _ in range(8))
    return id


class CoppeliaSim2FGripperDriver(object):
    def __init__(self, client, client_ID, object_name):
        self.client = client
        self.client_ID = client_ID
        self.status = InputMsg()
        self.cmd = OutputMsg()
        self.prev_gPO = 0

        # get some simulation parameters once
        self.object_handle = self.client.simxGetObjectHandle(
            object_name, self.client.simxServiceCall())[1]

        # set simulation mode as synchrnous mode
        self.client.do_next_step = True
        self.client.simxGetSimulationStepStarted(
            self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(
            self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        # create CoppeliaSim publisher
        self.sim_set_target_joint_pub = self.client.simxCreatePublisher(True)

        # create CoppeliaSim subscriber
        self.client.simxGetJointPosition(
            self.object_handle,
            self.client.simxDefaultSubscriber(self.sim_get_joint_position_cb))
        
         # create ros publishers
        self.coppeliasim_synchrnous_trigger_pub = rospy.Publisher(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)
        self.ros_status_pub = rospy.Publisher('Robotiq2FGripperRobotInput', InputMsg, queue_size=2)

        # create ros subscriber
        self.coppeliasim_synchrnous_trigger_sub = rospy.Subscriber(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)
        self.ros_read_cmd_sub = rospy.Subscriber('Robotiq2FGripperRobotOutput', OutputMsg, self.ros_read_cmd_cb)
        rospy.Subscriber("~grasp", Bool, self.ros_grasp_cb)

        rospy.sleep(0.2)
        rospy.loginfo("Robotiq 2F 85 gripper ROS interface turned on!")

    def sim_get_joint_position_cb(self, msg):
        """CoppeliaSim simxGetJointPosition callback function

        Get joint position and update gPO status on each simulation step.
        """
        # read joint position and convert to degree
        joint_position = msg[1]
        joint_position = np.rad2deg(joint_position)

        # convert joint position to 8-bit
        gPO = round(joint_position/45.84*255)
        gPO = max(0, gPO)
        gPO = min(255, gPO)

        # update joint pose status
        self.status.gPO = gPO

    def verify_command(self, cmd):
        """Function to verify that the value of each variable satisfy its limits.

        Args:
            cmd (`Robotiq2FGripper_robot_output`): Command message.
        """
        #Verify that each variable is in its correct range
        cmd.rACT = max(0, cmd.rACT)
        cmd.rACT = min(1, cmd.rACT)
        
        cmd.rGTO = max(0, cmd.rGTO)
        cmd.rGTO = min(1, cmd.rGTO)

        cmd.rATR = max(0, cmd.rATR)
        cmd.rATR = min(1, cmd.rATR)
        
        cmd.rPR  = max(0, cmd.rPR)
        cmd.rPR  = min(255, cmd.rPR)

        cmd.rSP  = max(0, cmd.rSP)
        cmd.rSP  = min(255, cmd.rSP)

        cmd.rFR  = max(0, cmd.rFR)
        cmd.rFR  = min(255, cmd.rFR)

    def update_status(self):
        """Update robotiq 2f gripper status.
        """
        self.status.gFLT = 0  # not used on here
        self.status.gCU = 0  # not used on here

        if abs(self.prev_gPO - self.status.gPO) > 5:
            self.status.gOBJ = 0
        else:
            if (self.status.gPO - self.status.gPR) > 0:
                self.status.gOBJ = 1
            elif (self.status.gPR - self.status.gPO) > 0:
                self.status.gOBJ = 2
            else:
                self.status.gOBJ = 3
        
        self.prev_gPO = self.status.gPO

    def ros_read_cmd_cb(self, msg):
        """ROS callback function of the command topic.

        Args:
            msg (`Robotiq2FGripper_robot_output`): Command message.
        """
        self.cmd = msg
        self.verify_command(self.cmd)
        
        # set activation
        if self.cmd.rACT == 1:
            self.status.gACT = 1
            self.status.gSTA = 3
        elif self.cmd.rACT == 0:
            self.status.gACT = 0
            self.status.gSTA = 0

        # set action status
        self.status.gGTO = self.cmd.rGTO

        # set pose request
        self.status.gPR = self.cmd.rPR

        # Ignore 'auto-release', 'speed', 'force' requests
        if self.cmd.rATR == 1:
            self.cmd.rATR = 0
            rospy.logwarn("rATR is not implemented. Ignore the signal.")
        if self.cmd.rSP > 0:
            self.cmd.rSP = 0
            rospy.logwarn('rSP is not implemented. Ignore the signal')
        if self.cmd.rFR > 0:
            self.cmd.rFR = 0
            rospy.logwarn('rFR is not implemented. Ingore the signal.')

    def excute(self):
        """Excute grasp based on the target position(gPR).
        """
        # if activation is not set or if GOTO bit is not set, do nothing.
        if self.status.gACT == 0:
            return
        if self.status.gSTA == 0:
            return
        if self.status.gGTO == 0:
            return

        # go to target pose
        target_angle = 45.84/255*self.status.gPR
        self.client.simxSetJointTargetPosition(
                self.object_handle, np.deg2rad(target_angle),
                self.sim_set_target_joint_pub)

    def ros_grasp_cb(self, msg):
        """Callback function for the legacy grasp topic.

        Args:
            msg (`std_msgs.Bool`): If `True`, close gripper.
        """
        if msg.data is True:
            self.client.simxSetJointTargetPosition(
                self.object_handle, np.deg2rad(48),
                self.sim_set_target_joint_pub)
        elif msg.data is False:
            self.client.simxSetJointTargetPosition(
                self.object_handle, np.deg2rad(17.5),
                self.sim_set_target_joint_pub)

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
        self.coppeliasim_synchrnous_trigger_pub.publish(trigger_msg)
        return

    def coppeliasim_synchronous_cb(self, msg):
        """
        Callback for synchronous operation. If message is received from "coppeliasim_master", 
        "self.client.do_next_step" is set to True
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
            self.client.do_next_step=True
            return True
        else:
            return False

    def simulation_step_start_cb(self, msg):
        self.fake_client_activation()
        self.excute()

    def simulation_step_done_cb(self, msg):
        # change do next step status
        self.client.do_next_step = True
        self.update_status()
        self.ros_status_pub.publish(self.status)

    def step_simulation(self):
        while not self.client.do_next_step:
            self.client.simxSpinOnce()
            # Check if simulation on. Makes for a clean exit
            if hasattr(self, "sequence_number"):
                if self.sequence_number == -1:
                    rospy.signal_shutdown("Stoping ROS: 2F_Gripper")
                    return
            rospy.sleep(0.002)
        
        self.client.do_next_step = False
        # Signal to master that we finished all calculations
        self.coppeliasim_synchronous_done()


if __name__ == '__main__':
    # init ros
    rospy.init_node('coppeliasim_robotiq_2f_gripper')
    finger_joint_name = "finger_joint"

    # init sim
    client_ID = 'b0RemoteApi_epick_gripper_client_{}'.format(id_generator())
    client = b0RemoteApi.RemoteApiClient(client_ID,'b0RemoteApi')
    sim = CoppeliaSim2FGripperDriver(client, client_ID, finger_joint_name)

    while not rospy.is_shutdown():
        sim.step_simulation()
