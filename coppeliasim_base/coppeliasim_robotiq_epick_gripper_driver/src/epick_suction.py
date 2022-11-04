#!/usr/bin/python2

from __future__ import print_function

import epick_functions as epf
from coppeliasim_remote_api.bluezero import b0RemoteApi

import time

import numpy as np
from numpy import gradient
import matplotlib.pyplot as plt

import scipy
from scipy.fftpack import fft, ifft, fftfreq
from scipy.interpolate import splprep, splev
from scipy import interpolate
from scipy import optimize
from scipy.integrate import trapz

import trimesh
from trimesh.geometry import vertex_face_indices
from trimesh.points import major_axis

class ModelData():
    """
    --------------
    A simple class that acts like a structure. Contains all the parameters of the Suction Cup
    --------------
    """
    def __init__(self, r = 18, d_0 = 34.409, *args, **kwargs):
        self.r = r
        self.d_0 = d_0
        self.max_deformation = 12
        self.spring_zeroDeformation = 28
        self.k_bm =12*10**-3 # Coefficient for curvature
        self.k_def = 2.9 # Coefficient for deformation pressure
        self.coef_friction = 0.65
        self.cup_perimiter = 2*np.pi*self.r
        self.allowedPerimiterError = 2
        self.vacuum_area_constant = 340
        # For the spring stiffness of SC joints in simulator the following equation is used: r^2*k_def/2. 
        # Higher value should probably be used. 
        # Moment is highly nonlinear with deformation so its impossible to describe it with one constant.

class SimulationData():
    """
    --------------
    This class contains the object matrices, positions, ... It also creates all the subscribers and publishers we use for exchanging data
    --------------
    """
    def __init__(self, client, contact,
                contact_force_simulation = True,
                seal_infinite_strength = False,
                use_complex_model = True):

        # Get the object handles
        detectionSphere = client.simxGetObjectHandle("DetectionSphere", client.simxServiceCall())[1]
        suctionCup_visual = client.simxGetObjectHandle("epick_suction_cup_visual", client.simxServiceCall())[1]
        j_y = client.simxGetObjectHandle("EPpick_joint_y", client.simxServiceCall())[1]
        j_x = client.simxGetObjectHandle("EPpick_joint_x", client.simxServiceCall())[1]
        proximitySensor = client.simxGetObjectHandle("EPick_proximity", client.simxServiceCall())[1]
        forceSensor = client.simxGetObjectHandle("EPick_ForceSensor", client.simxServiceCall())[1]

        # Set the vacuum_on signal to 0
        #client.simxSetIntSignal("vacuum_on", 0, client.simxServiceCall())

        # Seth the required parameters as attributes
        self.detectionSphere = detectionSphere # Object handle for the detection sphere
        self.suctionCup_visual = suctionCup_visual # Object handle for the suctionCup_visual
        self.j_y = j_y # Object handle for the joint y
        self.j_x = j_x # Object handle for the joint x
        self.forceSensor = forceSensor # Object handle for the Force sensor
        self.sphere_zeroPos = [0,0,0.028] # How much the contact-sphere is moved from the SC frame in the zero configuration
        self.client = client
        self.contact_object = -1
        self.vacuum_on = [None, None]

        self.gOBJ = None
        self.gACT= None

        self.proximitySensor = proximitySensor


        # Modes of operation
        self.contact_force_simulation = contact_force_simulation # Determines whether we use the force calculation for "pushing" the objects in VREP
        self.seal_infinite_strength = seal_infinite_strength # If True the program will only break the seal to object when the vacuum is turned off (calculations will still be performed)
        self.use_complex_model = use_complex_model # Determines how the calculations for making the seal and max force are performed

        # Create publishers and subscribers for b0 API
        self.create_publishers()
        self.create_subscribers(contact)

    def create_subscribers(self, contact):
        """
        Creates subscriber for all the calls to VREP. We use different subscriber for everything so that the code runs faster 
        (service calls are slow and defult subscriber can get cluttered)
        --------------
        contact : SuctionContact
            
        Returns
        --------------
        None : None
            Subscribers are stored as object attributes
        """
        # GETTING INTERSECTION---------------------------------------------------------
        args = []
        self.client.simxCallScriptFunction('getIntersection@DetectionSphere', 'sim.scripttype_childscript', args,
                                           self.client.simxCreateSubscriber(self.update_intersection, publishInterval=1))

        # GETTING OBJECT MATRICES ----------------------------------------------------
        # Get transformation matrix from world frame to suction cup frame  -- cup_WORLD_f
        self.client.simxGetObjectMatrix(self.suctionCup_visual, -1, self.client.simxDefaultSubscriber(self.update_cup_WORLD_f, publishInterval = 1))

        # From the previously stored points in object frame. Calculate their position in SC frame. -- sc_OBJECT_f
        self.sub_sc_OBJECT_f = self.client.simxCreateSubscriber(self.update_sc_OBJECT_f, publishInterval=1, dropMessages=True)
        self.client.simxGetObjectMatrix(self.suctionCup_visual, contact.contact_object, self.sub_sc_OBJECT_f)

        # Object matrix from suctionCup_frame to forceSensor frame  -- fs_SC_f
        self.client.simxGetObjectMatrix(self.forceSensor, self.suctionCup_visual, self.client.simxDefaultSubscriber(self.update_fs_SC_f, publishInterval=1))
        
    
        # GETTING OBJECT POSITIONS-------------------------------------------------------
        # Position of contact sphere in suction cup coordinates -- sphere_SC_f
        self.client.simxGetObjectPosition(self.detectionSphere, self.suctionCup_visual,
                                          self.client.simxDefaultSubscriber(self.update_sphere_SC_f, publishInterval=1))

        # Position of contact sphere in Object coordinates -- sphere_OBJ_f
        self.sub_sphere_OBJ_f = self.client.simxCreateSubscriber(self.update_sphere_OBJ_f, publishInterval=1, dropMessages=True)
        self.client.simxGetObjectPosition(self.detectionSphere, contact.contact_object,self.sub_sphere_OBJ_f)

        # VACUUM STATE--------------------------------------------------------------------
        #self.client.simxGetIntSignal("vacuum_on", self.client.simxDefaultSubscriber(self.update_vacuumState, publishInterval = 1))

        # FORCE SENSOR -------------------------------------------------------------------
        self.client.simxReadForceSensor(self.forceSensor, self.client.simxDefaultSubscriber(self.update_forceSensor, publishInterval = 1))

        # PROXIMITY SENSOR ---------------------------------------------------------------
        self.client.simxReadProximitySensor(self.proximitySensor, self.client.simxDefaultSubscriber(self.update_proximitySensor, publishInterval=1))

        # SIMULATION STATE----------------------------------------------------------------
        self.client.simxGetSimulationState(self.client.simxDefaultSubscriber(self.update_simState, publishInterval = 1))

    def reset_subscribers(self, contact):
        self.reset_sc_OBJECT_f(contact)
        self.reset_sphere_OBJ_f(contact)

    def create_publishers(self):
        """
        Creates publishers for all the calls to VREP. We use different publishers for everything so that the code runs faster 
        (service calls are slow and defult publishers can get cluttered)
        --------------
            
        Returns
        --------------
        None : None
            Publishers are stored as object attributes
        """
        self.pub_resetObjectForce = self.client.simxCreatePublisher()
        self.pub_addObjectForce = self.client.simxCreatePublisher()
        self.pub_updateSpherePos = self.client.simxCreatePublisher()
        self.pub_create_brake_link = self.client.simxCreatePublisher()


    # Bellow are all the callback functions and reset subscriber functions -------------------------------------------------
    # Callbacks get called when a new message is received and the message value is stored in the appropriate class attribute
    # Subscribers resets can be used to change the values of the certain subscriber. For example if we are contacting a new object 
    # we want to get the transformation matrix with respect to that object.

    # TF matrices and positions are indexed with the following notation child_BASE_f

    def update_forceSensor(self, msg):
        if msg[0] == True:
            self.external_forceMom = msg

    def update_proximitySensor(self, msg):
        if msg[0] == True:
            self.proximity_sensor = msg

    def update_fs_SC_f(self, msg):
        if msg[0] == True:
            cup_tf = np.array(msg[1])
            cup_tf = np.reshape(cup_tf, (3,4))
            self.fs_SC_f = (msg[0],cup_tf)

    def update_cup_WORLD_f(self, msg):
        if msg[0] == True:
            cup_tf = np.array(msg[1])
            cup_tf = np.reshape(cup_tf, (3,4))
            self.cup_WORLD_f = (msg[0],cup_tf)

    def update_sc_OBJECT_f(self, msg):
        if msg[0] == True:
            cup_tf = np.array(msg[1])
            cup_tf = np.reshape(cup_tf, (3,4))
            self.sc_OBJECT_f = (msg[0],cup_tf)
    def reset_sc_OBJECT_f(self, contact):
        self.client.simxRemoveSubscriber(self.sub_sc_OBJECT_f)
        self.sub_sc_OBJECT_f = self.client.simxCreateSubscriber(self.update_sc_OBJECT_f, publishInterval=1)
        self.client.simxGetObjectMatrix(self.suctionCup_visual, self.contact_object, self.sub_sc_OBJECT_f)

    def update_sphere_OBJ_f(self, msg):
        if msg[0] == True:
            self.sphere_OBJ_f = msg[1]
    def reset_sphere_OBJ_f(self, contact):
        self.client.simxRemoveSubscriber(self.sub_sphere_OBJ_f)
        self.sub_sphere_OBJ_f = self.client.simxCreateSubscriber(self.update_sphere_OBJ_f, publishInterval=1)
        self.client.simxGetObjectPosition(self.detectionSphere, self.contact_object, self.sub_sphere_OBJ_f)

    def update_sphere_SC_f(self, msg):
        if msg[0] == True:
            self.sphere_SC_f = msg[1]

    def update_intersection(self, msg):
        self.intersection = msg
        
    def update_simState(self, msg):
        self.simState = msg

    #def update_vacuumState(self, msg):
    #    self.vacuum_on = msg

class SuctionContact():
    """
    --------------
    Main class for storing the data of the suction cup and performing  different calculations
    --------------
    """
    def __init__(self):
        self.in_contact = False
        self.contact_position = np.array([0,0,0])
        self.object_position = np.array([0,0,0])
        self.seal_formed = 0
        self.contact_object = -1
        self.vacuum_on = 0
        self.brake_time = time.time()

    def formatPoints(self, section_nm, section_points, tf_in):
        """
        Takes in intersection points. The function formats the points se they are in nx3 array written in mm and transformed to SC frame.
        --------------
        section_nm : int
            number of sections of intersection
        section_points : list (1,n)
            specific points of intersection in meters; 
        tf_in : np.array(3,4)
            transformation matrix from suction cup frame to world frame in meters; 

        Returns
        --------------
        points_ordered : np.array(m,3)
            points ordered, transformed to SC frame and written in mm; 
        """
        
        # We make a copy of transformation array. And inverse it --> world frame to SC frame
        tf = np.copy(tf_in)
        tf[0:3,0:3] = np.transpose(tf[0:3,0:3])
        tf[:,3] = -np.dot(tf[0:3,0:3],tf[:,3])

        # We store the points as an numpy array
        section_points = np.array(section_points)
        # Extract x, y and z coordinrates
        x_s = section_points[::3].copy()
        y_s = section_points[1::3].copy()
        z_s = section_points[2::3].copy()
        # Format them properly to one array
        section_points = np.array([x_s, y_s, z_s]).T

        # We format the points so they are in order
        section_points = np.around(section_points, decimals = 9)
        point_cur = section_points[0]
        tested = np.ones(np.shape(section_points)[0], dtype=bool)
        tested[0] = False
        point = np.array(point_cur)
        point = np.append(point, 1)
        points_ordered = [point]
        check = False
        while True:
            mask = np.copy(tested[tested])
            dist = np.linalg.norm(section_points[tested]-point_cur, axis = 1)

            dist_min_i = np.argmin(dist)
            dist_min = dist[dist_min_i]


            next_point = section_points[tested][dist_min_i]
            mask[dist_min_i] = False 
            tested[tested] = mask
            point_cur = next_point
            # We ignore points that are too close together. If points are too far apart it means we have to deal with that separately.
            if not(True in tested):
                break
            if dist_min < 0.001:
                continue
            if dist_min > 0.01:
                check = True
                # We still have to deal with this somehow
                print("SUM-TING-WONG")
                print("Wi to LO")
                print("HO LEE FUK")
                print("BANG DING OW")

            point = np.array(next_point)
            point = np.append(point, 1)
            points_ordered.append(point)

        # We transform the new ordered points to numpy array. We also transfrom them to SC frame
        points_ordered = np.asarray(points_ordered)       
        points_ordered_T = np.transpose(points_ordered)
        points_ordered_T = np.dot(tf, points_ordered_T)
        points_ordered = np.transpose(points_ordered_T)
        if check == True:
            epf.plot_3D(points_ordered)

        # We try interpolating a function over the points to smooth them out. If there is less than 5 points we can not do that.
        if np.shape(points_ordered)[0]>5:
            points_ordered[-1,:] = points_ordered[0,:]
            per_func = self.interpolate_perimiter(points_ordered, 0)
            u = np.linspace(0,1,np.shape(points_ordered)[0])
            points_ordered = per_func(u)

        # Return the ordered points in SC frame and transform them to mm
        
        return points_ordered[:,0:3]*1000

    def points_move(self, points, tf_in):
        """
        Transforms points to the inverse of the transformation matrix
        --------------
        points : np.array(m,3)
            a set of points in mm
        tf_in : np.array(3,4)
            transformation matrix in m (we will transform to the inverse of it)

        Returns
        --------------
        points_ordered_T : returns the transformed points written in mm
        """
        # Create a copy of the TF matrix
        tf = np.copy(tf_in)
        # Inverse it
        tf[0:3,0:3] = np.transpose(tf[0:3,0:3])
        tf[:,3] = -np.dot(tf[0:3,0:3],tf[:,3])

        # Create a new that has one added to the end of every points
        points_ordered_T = np.ones((np.shape(points)[0], 4))
        points_ordered_T[:,0:3] = np.copy(points)/1000
        points_ordered_T = np.transpose(points_ordered_T)
        # Transpose the points
        points_ordered_T = np.dot(tf, points_ordered_T)
        # Shape them back to np.array(m,3)
        points_ordered_T = np.transpose(points_ordered_T)

        return points_ordered_T*1000

    def interpolate_perimiter(self, perimiter_pnts, smoothing):
        """
        Interpolates a function (spline) over a set of 3D points.
        --------------
        perimiter_pnts : ordered set of points; np.array(m,3)
        smoothing : smoothing factor, to see how it impacts the interpolation check scipy function splprep; float

        Returns
        --------------
        tck_per_func : returns the interpolatied function that takes in parameter u->[0,1]
        """

        x = perimiter_pnts[:, 0]
        y = perimiter_pnts[:, 1]
        z = perimiter_pnts[:, 2]

        tck_per, u = splprep([x, y, z], s=smoothing, per=True)

        def tck_per_func(u):
            new_p = splev(u, tck_per)
            return np.transpose(new_p)

        return tck_per_func

    def perimiterLength(self, pnts):
        """
        Takes in a set of points in order one to next and calculates the distance from beginning to end. 
        --------------
        pnts : np.array(m,3)
            ordered set of points

        Returns
        --------------
        pnts_length_sum : float
            the calculated length over points
        """

        pnts_diff = np.roll(pnts, -1, axis = 0) - pnts
        pnts_length = np.linalg.norm(pnts_diff, axis = 1)
        pnts_length_sum = np.sum(pnts_length)
        return pnts_length_sum

    def setContact(self, perimiter_pnts, simData, model=ModelData()):
        """
        Assigns the points that were passed in as Perimiter points. It calculates du, and du_cumulative. 
        Calculates all the tangents and normals. And  calculates the deformation pressures. 
        --------------
        perimiter_pnts : ordered set of points written SC frame and mm; np.array(m,3)
        model : the data for the suction cup model

        Returns
        --------------
        """

        self.perimiter = perimiter_pnts
        # Calculate du and du_cumulative 
        self.calculate_along_path()
        # Calculate the normal and tangents of the perimiter
        self.normal, self.radial, self.tangent = self.calculateTangents(perimiter_pnts, simData.sphere_SC_f)

        # Evaluate the contact
        #apex = np.array([0,0,model.max_deformation])
        apex = np.array([0,0,0])
        self.p_bm, self.p_d = self.evaluateDeformationPressures(apex, model)


    def calculateTangents(self, perimiter_pnts, center_pos):
        """
        Calculates the tangent, normal, and radial along the perimiter. 
        Since we don't get the normal from the simulation the calculated one is not exact but just an approximation.
        --------------
        perimiter_pnts : ordered set of points written SC frame and mm; np.array(m,3)
        center_pos : the center position of the detection sphere

        Returns
        --------------
        normal : the normal to the surface of object written in SC frame; np.array(n, 3)
        radial : the radial to the surface of object written in SC frame; np.array(n, 3)
        tangent : the tangent to the surface of object written in SC frame; np.array(n, 3)
        """
        # First calculate the tangent
        locations = self.perimiter
        tangent = gradient(locations, self.du_cumulative, axis = 0)
        # Scale by norm
        norm = np.linalg.norm(tangent, axis=1)
        tangent[:, 0] = np.divide(
            tangent[:, 0], norm)
        tangent[:, 1] = np.divide(
            tangent[:, 1], norm)
        tangent[:, 2] = np.divide(
            tangent[:, 2], norm)
        # Modify last point
        tangent[-1] = tangent[0]

        # Calculate the normal
        # Get vectors from perimiter points to center
        det_sphere_pos = np.array(center_pos)*1000
        inside_vector = det_sphere_pos - locations
        # Calculate the cross product of tangent and vectors
        normal = np.cross(tangent, inside_vector)
        # Scale by norm
        norm = np.linalg.norm(normal, axis=1)
        normal[:, 0] = np.divide(
            normal[:, 0], norm)
        normal[:, 1] = np.divide(
            normal[:, 1], norm)
        normal[:, 2] = np.divide(
            normal[:, 2], norm)
        # Modify last point
        normal[-1] = normal[0]

        # Calculate the radial
        radial = np.cross(normal, tangent)

        return normal,radial,tangent

    def calculate_along_path(self):
        """
        Catches the u and du functions around the perimeter of contact
        --------------
        locations : (n, 3) float; Ordered locations of contact points

        Returns
        --------------
        None : None
            du and du_cumulative get stored as object attributes
        """
        locations = self.perimiter
        distance = np.roll(locations, -1, axis=0) - locations
        distance = np.linalg.norm(distance, axis=1)
        distance_sum = np.sum(distance)
        self.du = distance/distance_sum
        self.du_cumulative = np.cumsum(np.append(0, self.du[:-1]))
        
    def calculate_average_normal(self):
        """
        Calculates the average surface normal. Sets it as the contact attribute and it does not return it.
        --------------
        Returns
        --------------
        """
        average_normal_x = trapz(
            self.normal[:, 0], self.du_cumulative)
        average_normal_y = trapz(
            self.normal[:, 1], self.du_cumulative)
        average_normal_z = trapz(
            self.normal[:, 2], self.du_cumulative)
        average_normal = np.array([average_normal_x, average_normal_y, average_normal_z])
        average_normal = average_normal/np.linalg.norm(average_normal)

        self.average_normal = average_normal

    def _calculate_distance(self, perimiter, model):
        # Deformation Option 1: Consider 3D deformation
        # distance = -(np.linalg.norm(self.perimiter, axis=1) - model.d_0)
        # Deformation Option 2: Consider only z change
        distance = (perimiter[:,2] - model.spring_zeroDeformation)
        distance = distance - self.zero_deformation
        return distance

    def evaluateDeformationPressures(self, apex, model):
        """
        Calculates the pressures along the perimiter because of curvature and deformation of the cup. 
        For the calculations the points stored in self.perimiter are used.
        --------------
        apex : apex of the suction cup; np.array(3)
        model : model data class

        Returns
        --------------
        p_bm : Pressure because of curvature. Not a vector but it should be acting in the direction of surface normal; np.array(n)
        p_d : Pressure because of SC deformation. A 3D vector; np.array(n, 3)
        """


        distance = self._calculate_distance(self.perimiter, model)
        # Check whether any points are in compression
        if (distance >= 2).all():
            self.in_contact = False
        # We can not act with pushing force. So if deformation is negative set it to 0 
        distance = np.where(distance>0, 0, distance)

        # Filter the input tangent and normal
        tangent_fit = epf.unit_array_of_vectors(
            epf.fourier_fit_3d(self.du_cumulative, self.tangent, 4))
        normal_fit = epf.unit_array_of_vectors(
            epf.fourier_fit_3d(self.du_cumulative, self.normal, 4))

        # Calculate the curvature 
        ddg =  np.gradient(tangent_fit, self.du_cumulative, axis = 0, edge_order = 2)
        k_n = (ddg * normal_fit).sum(1)  # Curvature
        # Fit it using a spline to get a nicer periodic function
        tck = scipy.interpolate.splrep(self.du_cumulative, k_n, s = 1, per = True, k = 5)
        k_n = splev(self.du_cumulative, tck)
        # Pressure because of normal curvature. For now it has no direction. This should act in the direction of surface normal.
        p_bm = np.gradient(np.gradient(model.k_bm * k_n, self.du_cumulative), self.du_cumulative, edge_order = 2)



        # Presure because of deformation. We must calculate the direction the force is acting.
        # Option 1: Force is acting towards the apex
        #def_vectors = epf.unit_array_of_vectors(-self.perimiter+np.array([0,0,0]))
        # Option 2: force is acting in the z axis of SC frame
        def_vectors = np.zeros(np.shape(self.perimiter))
        def_vectors[:,2] = 1
        # Calculate actual deformation force. 3D
        p_d = (def_vectors.T * distance * model.k_def).T

        p_d_n = (p_d * self.normal).sum(1) # Amount of pressure in direction of normal to surface
        # Positive pressure means that point is being pressed while negative pressure means the point is being pulled away from SC.
        # For the contact to be good all normal pressure should be positive
        return p_bm, p_d

    def forceOnCup(self, simData):
        """
        Calculates the forces that have to be acting on the suction cup based on the deformation pressure.
        We don't take in to account the bending pressures.
        --------------
        Returns
        --------------
        force_moment : a numpy array containing the forces and moments that must be acting on the suction cup,
                        for it to be deformed the way it is; np.array([F_x, F_y, F_z, [m_x, m_y, m_z]])
        """

        pressure_sum = self.p_d 

        # Projection the pressure to the respective normals
        pressure_app_Z = pressure_sum[:,2] #nx1 Z
        pressure_app_Y = pressure_sum[:,1]
        pressure_app_X = pressure_sum[:,0]
        # Scale the pressure by the length of its section
        force_app_Z = np.dot(pressure_app_Z, self.du) # 1x1
        force_app_Y = np.dot(pressure_app_Y, self.du)
        force_app_X = np.dot(pressure_app_X, self.du)

        # Calculating the applied moment. We must transform everything to mm
        inside_vector = self.perimiter - np.array(simData.sphere_SC_f)*1000
        moment = np.cross(inside_vector, pressure_sum) * self.du[:,np.newaxis]
        # We get a 3D moment in Nmm, We transform it to N*m
        moment_sum = np.sum(moment, axis = 0)/1000

        # We create a Numpy array that we will return. The array represents the forces that should act on the suction cup to be deformed the way it is.
        # Opposite of these forces act on the object.
        # Forces and moments are written in the SC frame.
        force_moment = np.array([force_app_X, force_app_Y, force_app_Z, moment_sum])# / model.max_deformation
        return force_moment

    def formSeal(self, client, simData, model):
        """
        Evaluates whether a seal can be formed in the current configuration.
        --------------

        Returns:
        --------------
            True/False : Seal can be formed/can't be formed
        
        """
        if self.in_contact == False:
            return False
        p_bm = self.p_bm
        p_d = self.p_d

        # Project p_d onto the normal
        def_p_n = (p_d*self.normal).sum(1)

        # Sum all the pressures together
        p_all = def_p_n - p_bm

        # Check wheather all pressures all smaller than 0
        p_min_i = np.argmin(p_all)

        if p_all[p_min_i] < 0:
            if p_all[np.argmax(p_all)] < 0:
                self.normal = -self.normal
                def_p_n = (p_d*self.normal).sum(1)
                return True
            # print(self.normal)
            # print("SEAL NOT FORMED")
            # plt.plot(p_all)
            # plt.show()
            # raise ValueError("EASRASD")
            # Seal was not formed
            return False
        else:
            # Seal was formed
            return True


    def calculate_average_normal(self):
        """
        Calculates the average normal. This is the average normal of the surface we are making contact of. It determines the direction of the vacuum force.
        --------------

        Returns:
        --------------
        None : None
            average_normal gets stored as a contact attribute
        """
        average_normal_x = trapz(
            self.normal[:, 0], self.du_cumulative)
        average_normal_y = trapz(
            self.normal[:, 1], self.du_cumulative)
        average_normal_z = trapz(
            self.normal[:, 2], self.du_cumulative)
        average_normal = np.array([average_normal_x, average_normal_y, average_normal_z])
        average_normal = average_normal/np.linalg.norm(average_normal)

        self.average_normal = average_normal

    
    def resetContact(self, client, simData):
        """
        The function does all the cleanup everytime the contact needs to be reset
        --------------
        Returns:
        --------------
        None : None
        """
        args = []
        ret = client.simxCallScriptFunction('brakeDummyLink@DetectionSphere','sim.scripttype_childscript',args, simData.pub_create_brake_link)
        self.in_contact = False
        self.seal_formed = False
        self.contact_object = -1
        args = []
        ret = client.simxCallScriptFunction('resetObjectForce@DetectionSphere','sim.scripttype_childscript',args, simData.pub_resetObjectForce)
        self.brake_time = time.time()
        self.vacuum_on = 0
        #client.simxSetIntSignal("vacuum_on", 0, client.simxDefaultPublisher())

    def get_external_force(self, simData):
        """
        Gets the external force and moment from the force sensor, formats it to the SC frame and returns the force and torque vector in SC frame.
        --------------
        simData : SimulationData
            Simulation data containing the latest force, moment and transformation matrices

        --------------
        f_dir : np.array(3,)
            A force vector in SC frame originating from the contact point
        m_vec : np.array(3,)
            A moment vector in SC frame
        """
    
        # External forces acting on an object. We must transform them to SC frame
        if (simData.external_forceMom[0] == False) or (simData.external_forceMom[1] == 0) or (simData.external_forceMom[1] == 2):
            simData.external_forceMom = [True, 1, np.array([0,0,0]), np.array([0,0,0])]
        
        # Brake them down into appropriate arrays
        external_force = np.array(simData.external_forceMom[2])
        external_moment = np.array(simData.external_forceMom[3])
        # Transform everything to SC frame
        tf_force = simData.fs_SC_f[1]
        f_dir = np.dot(tf_force[:, 0:3], external_force)
        m_vec = np.dot(tf_force[:, 0:3], external_moment)

        return f_dir, m_vec

    def evaluateForces(self, vacuum, client, simData, model):
        """
        Evaluates whether the suction cup can withstand the outside forces in the current configuration.
        --------------
        vacuum : float
            pressure difference between inside and outside in N/mm^2 (MPa)

        --------------
        success : Bool
            Returns True if it can withstand the force and False otherwise
        """
        self.calculate_average_normal()
        
        # Get external force and moment
        f_dir, m_vec = self.get_external_force(simData)

        # Base Coordinate System ----------------------------
        # Set up CS - This are axis of the contact CS written in global coordinates
        dir_t = np.array([1,0,0])
        z_ax = self.average_normal
        y_ax = np.cross(z_ax, dir_t)
        y_ax = y_ax/np.linalg.norm(y_ax)
        x_ax = np.cross(y_ax, z_ax)

        # Copy of normals, tangents and radial vectors-------
        normal_cp = np.copy(self.normal)
        tangent_cp = np.copy(self.tangent)
        radial_cp = np.copy(self.radial)
        perimiter_cp = np.copy(self.perimiter)

        # Vacuum force ---------------------------------
        proj_points = trimesh.points.project_to_plane(self.perimiter,
                                                      plane_normal=self.average_normal, plane_origin=[
                                                          0, 0, 0],
                                                      transform=None, return_transform=False, return_planar=True)
        # Area
        area = epf.poly_area(proj_points[:, 0], proj_points[:, 1])

        # Vacuum force contribution #340
        vac_n = np.tile(model.vacuum_area_constant*vacuum, np.shape(self.normal)[0])


        # Projecting the deformation pressure
        def_n = -(self.p_d*self.normal).sum(1)
        def_t = -(self.p_d*self.tangent).sum(1)
        def_r = -(self.p_d*self.radial).sum(1)

        moment_sum = self.moment_calc(
            def_n, def_t, def_r, normal_cp, tangent_cp, radial_cp, self.perimiter)



        # In the end we add the moment around z axis to the pressure distribution
        m_z = (m_vec*z_ax).sum()
        # Distances of points to "z" axis that goes trough origin.
        z_ax_stacked = np.tile(z_ax, (np.shape(perimiter_cp)[0], 1))
        leavers_vec = self.perimiter - (z_ax_stacked.T * (self.perimiter*z_ax).sum(1)).T
        leavers = np.linalg.norm(leavers_vec, axis=1)
        # We determine the moment each point has to provide
        m_z_spread = np.linalg.norm(m_z)  # *self.du
        # We calculate the force/pressure at each point
        m_z_p = -m_z_spread/leavers
        # We determine the directions in which the pressure acts
        m_z_p_dir = np.cross(z_ax, leavers_vec)
        m_z_p_dir = epf.unit_array_of_vectors(m_z_p_dir)
        # FInal moment around z axis as a vector distribution of pressure
        m_z_p_vec = (m_z_p_dir.T*m_z_p).T
        m_z_p_n = -(m_z_p_vec*normal_cp).sum(1)
        m_z_p_t = -(m_z_p_vec*tangent_cp).sum(1)
        m_z_p_r = -(m_z_p_vec*radial_cp).sum(1)

        # Next we analyze the plane forces ----------------------------------------
        p_nor = def_n + m_z_p_n + vac_n
        p_tan =  m_z_p_t + def_t 
        p_rad =  m_z_p_r + def_r

        # Actual force in the direction of average_normal
        def_n, def_t, def_r = 0, 0, 0,
        self.premik = 0

        perimiter_transpozed = np.ones((4, np.shape(self.perimiter)[0]))
        perimiter_transpozed[0:3, :] = perimiter_cp.T


        for i in range(10):
            # Calculate force
            force = self.force_calc(p_nor, p_tan, p_rad, normal_cp, tangent_cp, radial_cp, z_ax)
            force_sum = area*vacuum-force
            # What kind of force is desired in the direction of the average_normal
            force_desired = np.dot(-f_dir, z_ax)
            # We have reached equilibrium force, break the loop
            if np.allclose([force_sum], [force_desired], atol=1) == True:
                break
            else:
                # Transform the points up or down to get closer to the desired force
                sign = -np.sign(force_sum - force_desired)
                scale = np.abs(force_sum - force_desired)
                self.premik += sign*z_ax*scale/5
                T_mat = epf.translation(sign*z_ax*scale/5)
                perimiter_transpozed[0:4, :] = np.matmul(T_mat, perimiter_transpozed)
                
                distance = self._calculate_distance(np.transpose(perimiter_transpozed[0:3, :]), model)

                def_vectors = np.zeros(np.shape(self.perimiter))
                def_vectors[:,2] = 1
                #def_vectors = self._calculate_deformation_vectors(a_v)
                # Presure because of deformation
                p_d = (def_vectors.T * distance * model.k_def).T
                # Projecting the deformation pressure
                def_n = -(p_d*normal_cp).sum(1)
                def_t = -(p_d*tangent_cp).sum(1)
                def_r = -(p_d*radial_cp).sum(1)
                p_nor = def_n + m_z_p_n + vac_n
                p_tan =  m_z_p_t + def_t 
                p_rad =  m_z_p_r + def_r




        # Lastly add the pressure form the x and y axis forces
        f_x_already = self.force_calc(p_nor, p_tan, p_rad, normal_cp, tangent_cp, radial_cp, x_ax)

        f_y_already = self.force_calc(p_nor, p_tan, p_rad, normal_cp, tangent_cp, radial_cp, y_ax)
        f_x = (np.dot(f_dir, x_ax)+f_x_already)*x_ax
        f_y = (np.dot(f_dir, y_ax)+f_y_already)*y_ax
        f_x_n = (normal_cp*f_x).sum(1)
        f_x_t = (tangent_cp*f_x).sum(1)
        f_x_r = (radial_cp*f_x).sum(1)
        f_y_n = (normal_cp*f_y).sum(1)
        f_y_t = (tangent_cp*f_y).sum(1)
        f_y_r = (radial_cp*f_y).sum(1)

        p_nor += f_x_n + f_y_n
        p_tan += f_x_t + f_y_t
        p_rad += f_x_r + f_y_r

        # We also add the curvature pressure
        premik = np.linalg.norm(self.premik)
        reduction_p_bm = np.abs(premik-model.max_deformation)
        p_nor += self.p_bm

        # We look at friction for as an integral over whole perimiter.
        t1 = trapz(p_nor*model.coef_friction, self.du_cumulative)
        t2 = trapz(np.sqrt(p_tan**2+p_rad**2), self.du_cumulative)

        if p_nor[np.argmin(p_nor)] < 0:
            #print("Failure because of normal force.")
            return False
        if t2 > t1:
            #print("Failure because of friction force.")
            return False
        else:
            return True

    def force_calc(self, p_nor, p_tan, p_rad, n, t, r, direction):
        """
        Based on the given perimiter distributed force, force direction and perimiter points centered around 1
         the function calculates the moments generated by the distributed force.
        --------------
        p_nor: (n, ) np.array
            Normal component of the distributed force
        p_tan: (n, ) np.array
            Tangent component of the distributed force
        p_rad: (n, ) np.array
            Radial component of the distributed force
        n: (n, 3) np.array
            Matrix containing the normals to the surface along the perimiter
        t: (n, 3) np.array
            Matrix containing the tangents to the surface along the perimiter
        r: (n, 3) np.array
            Matrix containing the radials to the surface along the perimiter
        perimiter: (n, 3) np.array
            Perimiter points centered around (0,0,0)
        --------------
        moment : (3, ) np.array
            Moment vector calculated given the inputs.
        """

        p_nor_v = (n.T * p_nor).T
        p_nor_p = (direction*p_nor_v).sum(1)
        force_n = trapz(p_nor_p, self.du_cumulative)

        p_tan_v = (t.T * p_tan).T
        p_tan_p = (direction*p_tan_v).sum(1)
        force_t = trapz(p_tan_p, self.du_cumulative)

        p_rad_v = (r.T * p_rad).T
        p_rad_p = (direction*p_rad_v).sum(1)
        force_r = trapz(p_rad_p, self.du_cumulative)

        return force_n+force_t+force_r

    def moment_calc(self, p_nor, p_tan, p_rad, n, t, r, perimeter):
        """
        Based on the given perimiter distributed force, force direction and perimiter points centered around 1
         the function calculates the moments generated by the distributed force.
        --------------
        p_nor: (n, ) np.array
            Normal component of the distributed force
        p_tan: (n, ) np.array
            Tangent component of the distributed force
        p_rad: (n, ) np.array
            Radial component of the distributed force
        n: (n, 3) np.array
            Matrix containing the normals to the surface along the perimiter
        t: (n, 3) np.array
            Matrix containing the tangents to the surface along the perimiter
        r: (n, 3) np.array
            Matrix containing the radials to the surface along the perimiter
        perimiter: (n, 3) np.array
            Perimiter points centered around (0,0,0)
        --------------
        moment : (3, ) np.array
            Moment vector calculated given the inputs.
        """
        p_nor_v = (n.T * p_nor).T
        p_tan_v = (t.T * p_tan).T
        p_rad_v = (r.T * p_rad).T
        pressure_sum = p_nor_v+p_tan_v+p_rad_v
        # Calculating the applied moment. We must transform everything to mm
        inside_vector = perimeter  # - np.array(self.p_0)

        moment = np.cross(inside_vector, pressure_sum) * self.du[:, np.newaxis]

        # We get a 3D moment in Nmm, We transform it to N*m
        moment_sum = np.sum(moment, axis=0)

        moment_x = moment_sum[0]
        moment_y = moment_sum[1]
        moment_z = moment_sum[2]

        return np.array([moment_x, moment_y, moment_z])

class SimpleSuctionContact():
    """
    --------------
    Very simple suction gripper. This class wont do any seal analysis. 
    An object will be grasped as long as the gripper is in contact with it.
    --------------
    """
    def __init__(self):
        self.seal_formed = False
        self.contact_object = -1
        self.vacuum_on = 0
        self.brake_time = time.time()

    def formSeal(self, client, simData, model, detection_distance = 40):
        """
        Formes a seal if an object is close enough to the proximity sensor of the suction cup .
        --------------

        *args
        detection_distance : float
            At what distance from the suction do we consider that object and suction cup are in contact.
        
        Returns:
        --------------
        success : Bool
            Returns True if a seal can be formed and False otherwise.
        """
        # Check to see if we ever got any response from the proximity sensor
        if not hasattr(simData, "proximity_sensor") or (simData.proximity_sensor[1] == 0):
            return False
        
        # If the distance to the object is "small enough" we can make a seal
        if simData.proximity_sensor[2] < detection_distance:
            self.contact_object = simData.proximity_sensor[4]
            return True
        else:
            return False

    def evaluateForces(self, vacuum, client, simData, model):
        """
        Evaluates whether the suction cup can withstand the outside forces.
        As force limits the euqations from Dex-Net 3.0 are used 
        --------------
        vacuum : float
            pressure difference between inside and outside in N/mm^2 (MPa)
        Returns:
        --------------
        success : Bool
            Returns True if it can withstand the force and False otherwise
        """
        # Vacuum force
        suction_cup_area = np.pi*model.r**2
        vacuum_force = suction_cup_area*vacuum

        # External forces
        f_ext, mom_ext = self.get_external_force(simData)

        max_wrench = {"f_x" : model.coef_friction*(vacuum_force-f_ext[2]),
                      "f_y" : model.coef_friction*(vacuum_force-f_ext[2]),
                      "f_z" : vacuum_force,
                      "t_x" : np.pi*model.r*0.005,
                      "t_y" : np.pi*model.r*0.005,
                      "t_z" : model.r*model.coef_friction*(vacuum_force-f_ext[2])/1000}


        # Check if all conditions apply:
        # Friction
        if not (np.sqrt(3)*abs(f_ext[0]) <= max_wrench["f_x"]):
            return False
        elif not (np.sqrt(3)*abs(f_ext[1]) <= max_wrench["f_y"]):
            return False
        elif not abs(f_ext[2]) <= max_wrench["f_z"]:
            return False
        elif not (np.sqrt(2)*abs(mom_ext[0]) <= max_wrench["t_x"]):
            return False
        elif not (np.sqrt(2)*abs(mom_ext[1]) <= max_wrench["t_y"]):
            return False
        elif not (np.sqrt(3)*abs(mom_ext[2]) <= max_wrench["t_z"]):
            return False

        # Passed all conditions -> return True
        return True

    def resetContact(self, client, simData):
        """
        The function does all the cleanup everytime the contact needs to be reset
        --------------
        Returns:
        --------------
        None : None
        """
        self.seal_formed = False
        self.contact_object = -1
        self.vacuum_on = 0
        self.proximity_sensor = None
        args = []
        ret = client.simxCallScriptFunction('brakeDummyLink@DetectionSphere','sim.scripttype_childscript',args, simData.pub_create_brake_link)


    def get_external_force(self, simData):
        """
        Gets the external force and moment from the force sensor, formats it to the SC frame and returns the force and torque vector in SC frame.
        --------------
        simData : SimulationData
            Simulation data containing the latest force, moment and transformation matrices

        --------------
        f_dir : np.array(3,)
            A force vector in SC frame originating from the contact point
        m_vec : np.array(3,)
            A moment vector in SC frame
        """
    
        # External forces acting on an object. We must transform them to SC frame
        if (simData.external_forceMom[0] == False) or (simData.external_forceMom[1] == 0) or (simData.external_forceMom[1] == 2):
            simData.external_forceMom = [True, 1, np.array([0,0,0]), np.array([0,0,0])]
        
        # Brake them down into appropriate arrays
        external_force = np.array(simData.external_forceMom[2])
        external_moment = np.array(simData.external_forceMom[3])
        # Transform everything to SC frame
        tf_force = simData.fs_SC_f[1]
        f_dir = np.dot(tf_force[:, 0:3], external_force)
        m_vec = np.dot(tf_force[:, 0:3], external_moment)

        return f_dir, m_vec
