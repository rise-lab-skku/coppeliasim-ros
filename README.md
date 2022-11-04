# CoppeliaSim ROS Interface
![Python](https://img.shields.io/badge/Python-2.7-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-18.04-green)
![ROS](https://img.shields.io/badge/ROS-melodic-yellow)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-4.2.0-red)

# 1. Overview
Here we offer ROS-CoppeliaSim interface modules for the manipulators, the grippers, the vision sensors, etc. Whole packages are based on [B0-based remote API](https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiOverview.htm) and run with [synchronous](https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiModusOperandi.htm) mode.
<!-- [Instructions on how to enable the B0-based remote API on the client side are given here](https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm). -->

# 2. Requirements
Install [MessagePack](https://msgpack.org/index.html) for Python2.
```bash
pip2 install msgpack
```

# 3. Activate B0 Remote API Server
## 3.1 Load
Drag and drop `B0 Remote API Server.ttm` on the scene. The module can be found on `[Model browser]-[tools]`. If `B0 Remote API Server.ttm` is successfuly loaded, `b0RemoteApiServer` object will show on the `Secene hierarachy`.
## 3.2 Channel name
Channel name should be set as `b0RemoteApi`.

# 4. CoppeliaSim master
Only [coppeliasim master](./coppleiasim_master) node may send a [SynchronousTrigger](https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApi-python.htm#simxSynchronousTrigger) signal and [base modules](./coppleiasim_base) must not send the trigger signal by themselves. You **must** have a `coppeliasim master` node running in order for `base modules` to run other modules synchronously.
## 4.1 Usage
The `coppeliasim master` can be launched using the roslaunch.

```bash
roslaunch coppeliasim_master coppeliasim_master.launch
```
> Note: `coppeliasim master` must be launched **only once**.

## 4.2 Asynchronous operation
Clients do not need to signal anything to master node. Master node should still be run to trigger coppelia simulatior.
## 4.3 Synchronous operation
For a client to run in synchronous mode it most subscribe to rostopic: *"/coppeliasim_synchronous"*. On that topic, triggers from master node will be posted. When the client node finishes its calculations for one step, it has to signal that to [coppeliasim master] over "/coppeliasim_synchronous" topic. \\

Below is an example class code of a client running in synchronous mode.


```python
import rospy 

from coppeliasim_remote_api.bluezero import b0RemoteApi
from coppeliasim_master.msg import CoppeliaSimSynchronous

class SynchronousClient(object):
    def __init__(self, client, client_ID):
        """
        This is a coppeliasim client running in synchronous mode.
        Client_ID must not be equal to "coppeliasim_master"
        """
        self.client = client
        self.client_ID = client_ID

        self.do_next_step=False
        self.client.runInSynchronousMode=True


        # set ros publishers
        self.coppeliasim_synchronous_trigger_pub = rospy.Publisher(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, queue_size=100)

        # set ros subscribers
        self.coppeliasim_synchronous_trigger_sub = rospy.Subscriber(
            "/coppeliasim_synchronous", CoppeliaSimSynchronous, callback = self.coppeliasim_synchronous_cb)

        rospy.sleep(0.2)
        rospy.loginfo("Robotiq Synchronous Client ROS interface turned on!")

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
        # Publish the trigger to all other synchronous clients
        trigger_msg = CoppeliaSimSynchronous()
        trigger_msg.stamp = rospy.Time.now()
        trigger_msg.sequence_number = self.sequence_number
        trigger_msg.client_ID = self.client_ID
        self.coppeliasim_synchronous_trigger_pub.publish(trigger_msg)
        return

    self.sim_step_done_cb(self):
        # -------------------------------------------------------------------------------------
        # ---------------- PUT HERE CODE TO RUN EACH SIM STEP ---------------------------------
        # -------------------------------------------------------------------------------------



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
                    rospy.signal_shutdown("Synchronous client STOP: Master signaled shutdown")
                    return
            rospy.sleep(0.002)

        # -------------------------------------------------------------------------------------
        # ---- Function that gets called each simulation step ---------------------------------
        self.sim_step_done_cb() 
        # -------------------------------------------------------------------------------------
    
        self.do_next_step = False
        self.coppeliasim_synchronous_done()

if __name__ == '__main__':
    # init ros
    rospy.init_node('synchronous_coppeliasim_client')

    # init sim
    client_ID = 'b0RemoteApi_epick_gripper_client_{}'.format(id_generator())
    client = b0RemoteApi.RemoteApiClient(client_ID, 'b0RemoteApi')

    client.do_next_step=False
    client.runInSynchronousMode=True

    sim = synchronousClient(client, client_ID)

    print("synchronousClient Start")  

    while not rospy.is_shutdown():
        sim.step_simulation()
```

