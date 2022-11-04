#! /usr/bin/python
# -*- coding: utf-8 -*-
import time
import string
import random

import rospy

from coppeliasim_remote_api.bluezero import b0RemoteApi
from coppeliasim_master.msg import CoppeliaSimSynchronous

def id_generator():
    """
    Generates a random ID in the form of a string
    --------------
    Returns
    --------------
    id : string
        A random string of ascii characters and digits
    """
    id = ''.join(
        random.choice(string.ascii_uppercase + string.digits)
        for _ in range(8))
    return id


class CoppeliaSimMaster(object):
    def __init__(self, client, synchronous_client_timeout = 2.):
        """
        CoppeliaSimMaster class. it has a unique client_ID of: "coppeliasim_master".
        Only one master should be operating at the same time as it is sending triggers to the coppeliasim.
        Master should be run first.
        """
        self.client = client
        self.client_ID = "coppeliasim_master"

        # set simulation mode as synchrnous mode
        self.do_next_step = True
        self.client.simxSynchronous(True)
        self.client.simxGetSimulationStepStarted(
            self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(
            self.client.simxDefaultSubscriber(self.simulation_step_done_cb))
        self.client.simxGetSimulationState(
            self.client.simxDefaultSubscriber(self.simulation_state_cb))

        # set ros publishers
        self.coppeliasim_synchrnous_trigger_pub = rospy.Publisher("/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)

        # set ros subscribers
        self.coppeliasim_synchrnous_trigger_sub = rospy.Subscriber("/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)

        self.sequence_number = 0
        self.coppeliasim_synchronous_clients = {}

        # SET THE TIMEOUT. DETERMINES HOW LONG WE WAIT BEFORE WE DELETE THE CLIENT
        self.timeout = rospy.Duration(synchronous_client_timeout) # in seconds
        
    def coppeliasim_synchronous_cb(self, msg):
        """
        Callback function that gets called everytime a new message is received on "/coppeliasim_synchronous" topic.
        If message is made by this client it is ignored.
        --------------
        msg : CoppeliaSimSynchronous 
            ros message containing:
            - string client_ID
            - time stamp
            - int64 sequence_number

        Returns
        --------------
        None : None
        """
        # We can skip our own trigger message
        if msg.client_ID == self.client_ID:
            return None
        # We remove the client if time = 0. Our delete command
        if msg.stamp.secs  == 0:
            del self.coppeliasim_synchronous_clients[msg.client_ID]
            return None
        
        # If above not true we add or update the client:
        self.coppeliasim_synchronous_clients[msg.client_ID] = (msg.sequence_number, msg.stamp)
        return None

    def simulation_step_start_cb(self, msg):
        """
        Callback that coppeliasim has started its simulation step.
        --------------
        Returns
        --------------
        """
        pass

    def simulation_step_done_cb(self, msg):
        """
        Callback that coppeliasim has finished its simulation step.
        Master will send trigers via "/coppeliasim_synchronous" topic to other clients.
        --------------
        Returns
        --------------
        """
        # First mark an array of all the clients we will be waiting to get a response from
        clients_temp = self.coppeliasim_synchronous_clients.keys()

        # Increase sequence number
        self.sequence_number += 1

        # Publish the trigger to all other synchrnous clients
        trigger_msg = CoppeliaSimSynchronous()
        trigger_msg.stamp = rospy.Time.now()
        trigger_msg.sequence_number = self.sequence_number
        trigger_msg.client_ID = self.client_ID
        self.coppeliasim_synchrnous_trigger_pub.publish(trigger_msg)

        # Keep checking for a response of all logged clients. If a client does not respond for some time we will time it out.
        while self.do_next_step == False:
            rospy.sleep(0.001)
            for client in clients_temp:
                if self.coppeliasim_synchronous_clients[client][0] == self.sequence_number:
                    continue
                elif rospy.Time.now() > self.coppeliasim_synchronous_clients[client][1]+self.timeout:
                    self.coppeliasim_synchronous_clients[client]
                    clients_temp.remove(client)
                else:
                    break
            else:
                self.do_next_step = True

    def _check_sim_state(self):
        """
        Checks wheather the simulation is still running. 
        --------------
        Returns
        --------------
        True / False
            - True is simulation running or paused.
            - False if simulation stopped or can not get an answer for some time.
        """

        if (self.sim_state[0] == 0):
            # Simulation is stopped return false
            return False
        elif (self.sim_state[1] + rospy.Duration(4)) < rospy.Time.now():
            # The sim might have timed out or just paused?. Send a service call to see what is going on.
            msg = self.client.simxGetSimulationState(self.client.simxServiceCall())
            self.simulation_state_cb(msg)

            # Simulation is stopped -> stop ROS
            if self.sim_state[0] == 0:
                return False
            # Unsuccessfull communication with CoppeliaSim -> stop ROS
            elif (self.sim_state[1] + rospy.Duration(4)) < rospy.Time.now():
                return False
            # Everything is OK
            else:
                return True
        else:
            return True

    def step_simulation(self):
        """
        Perform one step of the simulation.
        --------------
        Returns
        --------------
        """
        while not self.do_next_step:
            self.client.simxSpinOnce()
            if not self._check_sim_state():
                 # Publish trigger message with value -1. It means that master client is shutting down.
                trigger_msg = CoppeliaSimSynchronous()
                trigger_msg.stamp = rospy.Time.now()
                trigger_msg.sequence_number = -1
                trigger_msg.client_ID = self.client_ID
                self.coppeliasim_synchrnous_trigger_pub.publish(trigger_msg)
                rospy.sleep(0.1)
                rospy.signal_shutdown("Signaling ROS shutdown")
                return
            rospy.sleep(0.001)
        self.do_next_step = False
        self.client.simxSynchronousTrigger()

    def simulation_state_cb(self, msg):
        """
        Gets the information about the current state of the coppeliasim simulation 
        --------------
        Returns
        --------------
        """
        if msg[0] == True:
            self.sim_state = (msg[1], rospy.Time.now())

    def start_simulation(self):
        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        rospy.loginfo('Start CoppeliaSim. API: bluezero, MODE: synchronous.')

    def stop_simulation(self):
        self.client.simxStopSimulation(self.client.simxDefaultPublisher())
        rospy.loginfo('Stop CoppeliaSim.')


if __name__ == '__main__':
    # init ros
    rospy.init_node('coppeliasim_master')

    # init sim
    client = b0RemoteApi.RemoteApiClient(
        'b0RemoteApi_master_client_{}'.format(id_generator()),
        'b0RemoteApi')
    sim = CoppeliaSimMaster(client)

    # start coppeliasim simulation
    sim.start_simulation()
    rospy.sleep(1)
    # if the simulation was successfully started if we should get a valid message
    sim.simulation_state_cb(client.simxGetSimulationState(client.simxServiceCall()))

    while not rospy.is_shutdown():
        sim.step_simulation()
    sim.stop_simulation()
