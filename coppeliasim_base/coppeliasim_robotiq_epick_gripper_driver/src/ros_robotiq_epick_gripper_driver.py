#!/usr/bin/python2

import rospy 
import string
import random
import numpy as np
import epick_logic as epl
import epick_suction as eps

from coppeliasim_remote_api.bluezero import b0RemoteApi
from coppeliasim_master.msg import CoppeliaSimSynchronous
from robotiq_epick_control.msg import RobotiqEPick_robot_input
from robotiq_epick_control.msg import RobotiqEPick_robot_output



def id_generator():
    id = ''.join(
        random.choice(string.ascii_uppercase + string.digits)
        for _ in range(8))
    return id

class CoppeliaSimEpickGripperDriver(object):
    def __init__(self, client, client_ID, contact, model, simData):
        self.client = client
        self.client_ID = client_ID

        self.client.do_next_step=False
        self.client.runInSynchronousMode=True

        # Initialise
        self.model = model
        self.contact = contact
        self.simData = simData
        self.gripper_robot_output = None
        self.gripper_robot_input = None


        self.client.simxGetSimulationStepStarted(
            self.client.simxDefaultSubscriber(self.simulation_step_start_cb))

        # set ros publishers
        self.coppeliasim_synchrnous_trigger_pub = rospy.Publisher(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)
        self.robotiq_epick_gripper_status_pub = rospy.Publisher('RobotiqEPickRobotInput',
                                                                RobotiqEPick_robot_input, queue_size=10)

        # set ros subscribers
        self.coppeliasim_synchrnous_trigger_sub = rospy.Subscriber(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)
        self.robotiq_epick_gripper_control_sub = rospy.Subscriber(
            "RobotiqEPickRobotOutput", RobotiqEPick_robot_output, callback=self.robotiq_epick_gripper_control_cb)

        rospy.sleep(0.2)
        rospy.loginfo("Robotiq EPick gripper ROS interface turned on!")

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
    
    def robotiq_epick_gripper_control_cb(self, msg):
        """
        Callback for vacuum control (rostopic: "/robotiq_epick_gripper/grasp")
        --------------
        msg : Bool
            True for vacuum on and False for vacuum off
        Returns
        --------------
        None : None
        """
        if msg.rGTO == 1:
            self.simData.gACT = 1
            self.simData.vacuum_on = [True, 1]
        else:
            self.simData.gACT = 1
            self.simData.vacuum_on = [True, 0] 
        return None

    def simulation_step_start_cb(self, msg):
        self.fake_client_activation()

    def update_gripper_status(self):
        msg = RobotiqEPick_robot_input()
        if simData.gOBJ != None:
            msg.gOBJ = np.uint8(simData.gOBJ)
        if simData.gACT != None:
            msg.gACT = np.uint8(simData.gACT)
        if simData.vacuum_on[1] != None:
            msg.gGTO = np.uint8(simData.vacuum_on[1])
        
        msg.gSTA = 3
        self.robotiq_epick_gripper_status_pub.publish(msg)

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

    def fake_client_activation(self):
        """
        Fake client activation:
        If CoppeliaSim can not detect client's activation, it automately
        deativate the client. So we fool the simulation that the client
        is active.
        """
        self.client.simxSetIntSignal('a', 0, client.simxDefaultPublisher())


    def step_simulation(self):
        while not self.client.do_next_step:
            self.client.simxSpinOnce()
            # Check if simulation on. Makes for a clean exit
            if hasattr(self, "sequence_number"):
                if self.sequence_number == -1:
                    rospy.signal_shutdown("Stoping ROS: EPick_Gripper")
                    return
            rospy.sleep(0.002)

        # Select simple or complex model of simulation
        if self.simData.use_complex_model == True:
            epl.epick_sim_step(self.client, self.model, self.simData, self.contact)
        else:
            epl.epick_simple_sim_step(self.client, self.model, self.simData, self.contact)
        
        self.update_gripper_status()
        
        self.client.do_next_step = False
        # Signal to master that we finished all calculations
        self.coppeliasim_synchronous_done()
        


if __name__ == '__main__':
    # init ros
    rospy.init_node('coppeliasim_robotiq_epick_gripper')

    # init sim
    client_ID = 'b0RemoteApi_epick_gripper_client_{}'.format(id_generator())
    client = b0RemoteApi.RemoteApiClient(client_ID, 'b0RemoteApi')

    client.do_next_step=False
    client.runInSynchronousMode=True

    # --------------------------------------------------------------------------------
    # ----- HERE CHANGE THE PARAMETERS HOW THE SUCTION GRIPPER WILL BE SIMULATED -----
    contact_force_simulation = True
    seal_infinite_strength = False
    use_complex_model = False
    # --------------------------------------------------------------------------------

    # Initialise all objects
    model = eps.ModelData()
    if use_complex_model == True:
        print("USING COMPLEX SUCTION MODEL")
        contact = eps.SuctionContact()
    else: 
        print("USING SIMPLE SUCTION MODEL")
        contact = eps.SimpleSuctionContact()
    simData = eps.SimulationData(
        client, contact, 
        contact_force_simulation, 
        seal_infinite_strength, 
        use_complex_model)

    sim = CoppeliaSimEpickGripperDriver(client, client_ID, contact, model, simData)

    print("EPick ClientStart")  

    while not rospy.is_shutdown():
        sim.step_simulation()
