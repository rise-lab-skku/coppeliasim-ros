#! /usr/bin/python
import string
import random

import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from coppeliasim_remote_api.bluezero import b0RemoteApi
from coppeliasim_master.msg import CoppeliaSimSynchronous


def id_generator():
    id = ''.join(
        random.choice(string.ascii_uppercase + string.digits)
        for _ in range(8))
    return id


class CoppeliaSimVisionDriver(object):
    def __init__(self, client, client_ID, object_name, mode):
        self.mode = mode

        # init simulation
        self.client = client
        self.client_ID = client_ID
        self.cv_bridge = CvBridge()

        # get some simulation parameters once
        self.object_handle = self.client.simxGetObjectHandle(
            object_name, self.client.simxServiceCall())[1]
        self.perspective_angle = self.client.simxGetObjectFloatParameter(
            self.object_handle,
            'sim.visionfloatparam_perspective_angle',
            self.client.simxServiceCall(),
        )[1]
        self.width = self.client.simxGetObjectIntParameter(
            self.object_handle,
            'sim.visionintparam_resolution_x',
            self.client.simxServiceCall(),
        )[1]
        self.height = self.client.simxGetObjectIntParameter(
            self.object_handle,
            'sim.visionintparam_resolution_y',
            self.client.simxServiceCall(),
        )[1]

        # set simulation mode as synchrnous mode
        self.do_next_step = True
        self.client.simxGetSimulationStepStarted(
            self.client.simxDefaultSubscriber(self.simulation_step_start_cb))


        # create CoppeliaSim subscribers
        if self.mode == "color":
            # init img_buff with zero image
            buff_size = self.width * self.height * 3
            self.image_buff = np.getbuffer(np.zeros(buff_size, dtype=np.uint8))

            # create subsriber
            self.client.simxGetVisionSensorImage(
                self.object_handle, False,
                self.client.simxCreateSubscriber(self.sim_get_vision_sensor_cb,
                                                 dropMessages=True))
        elif self.mode == "depth":
            # init img_buff with zero image
            buff_size = self.width * self.height
            self.image_buff = np.getbuffer(
                np.zeros(buff_size, dtype=np.float32))

            # create subsriber
            self.client.simxGetVisionSensorDepthBuffer(
                self.object_handle, True, True,
                self.client.simxCreateSubscriber(self.sim_get_vision_sensor_cb,
                                                 dropMessages=True))
        else:
            raise AttributeError(
                "mode should be `color` or `depth`, but get {}".format(
                    self.mode))

        # create ros publishers
        self.ros_image_pub = rospy.Publisher(
            '~image_raw',
            Image,
            queue_size=1,
        )
        self.ros_camera_info_pub = rospy.Publisher(
            '~camera_info',
            CameraInfo,
            queue_size=1,
        )
        self.coppeliasim_synchronous_trigger_pub = rospy.Publisher(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)
        
        # set ros subscribers
        self.coppeliasim_synchronous_trigger_sub = rospy.Subscriber(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)

        rospy.sleep(0.2)

        # screen out camera information
        rospy.loginfo("{} mode camera has been loaded.".format(self.mode))
        rospy.loginfo("Perspective angle: {} deg".format(
            np.rad2deg(self.perspective_angle)))
        rospy.loginfo("Height, Width: {}, {} px".format(
            self.height, self.width))

    @property
    def image_msg(self):
        if self.mode == "color":
            image = np.frombuffer(self.image_buff, dtype=np.uint8)
            image = np.reshape(image, (self.height, self.width, 3))
            image = np.flip(image, axis=0)
            msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        if self.mode == "depth":
            image = np.frombuffer(self.image_buff, dtype=np.float32)
            image = np.reshape(image, (self.height, self.width))
            image = np.flip(image, axis=0)
            msg = self.cv_bridge.cv2_to_imgmsg(image, "32FC1")
        return msg

    @property
    def camera_info_msg(self):
        msg = CameraInfo()

        # calculate intrinsic matrix
        max_h_w = max(self.width, self.height)
        f = float(max_h_w) / (2 * np.math.tan(self.perspective_angle / 2))
        intr = [f, 0, self.width / 2., 0, f, self.height / 2., 0, 0, 1]

        msg.height = self.height
        msg.width = self.width
        msg.K = intr
        return msg

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
            self.do_next_step=True
            return True
        else:
            return False

    def sim_get_vision_sensor_cb(self, msg):
        self.image_buff = msg[2]

    def fake_client_activation(self):
        """Fake client activation

        If CoppeliaSim can not detect client's activation, it automately
        deativate the client. So we fool the simulation that the client
        is active.
        """
        self.client.simxSetIntSignal('a', 0, client.simxDefaultPublisher())

    def simulation_step_start_cb(self, msg):
        self.fake_client_activation()

    def simulation_step_done_cb(self):
        # publish ros topics
        self.ros_image_pub.publish(self.image_msg)
        self.ros_camera_info_pub.publish(self.camera_info_msg)


    def step_simulation(self):
        while not self.do_next_step:
            self.client.simxSpinOnce()
            # Check if simulation on. Makes for a clean exit
            if hasattr(self, "sequence_number"):
                if self.sequence_number == -1:
                    rospy.signal_shutdown("Stoping ROS: Vision Driver")
                    return
            rospy.sleep(0.002)

        # DO THE NECESSARY CALCULATIONS
        self.simulation_step_done_cb()
        
        self.do_next_step = False
        # Signal to master that we finished all calculations
        self.coppeliasim_synchronous_done()



if __name__ == '__main__':
    # init ros
    rospy.init_node('coppeliasim_vision_sensor_driver')
    object_name = rospy.get_param('~vision_sensor_object_name')
    mode = rospy.get_param('~vision_sensor_mode')

    # init sim
    client_ID = 'b0RemoteApi_vision_client_{}'.format(id_generator())
    client = b0RemoteApi.RemoteApiClient(client_ID, 'b0RemoteApi')
    sim = CoppeliaSimVisionDriver(client, client_ID, object_name, mode)

    while not rospy.is_shutdown():
        sim.step_simulation()
