#!/usr/bin/python2


from __future__ import print_function

import epick_suction as eps
import epick_functions as epf
from coppeliasim_remote_api.bluezero import b0RemoteApi
import time
import numpy as np


def suctionCup_step(client, model, simData, contact):
    """
    Containts the logic for updating the suction cup
    --------------
    Returns
    --------------
    """

    # We check whether we are in contact or not
    if contact.in_contact == False:
    
        contact.in_contact = False
        contact.contact_object = -1
        args = []
        ret = client.simxCallScriptFunction('resetObjectForce@DetectionSphere','sim.scripttype_childscript',args, simData.pub_resetObjectForce) ## PP
        client.simxSetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, simData.sphere_zeroPos, simData.pub_updateSpherePos) ## PP


        # Check for collisions/get intersections
        try:
            ret = simData.intersection
            # Reset subscribers if contact object changed
            if simData.intersection[3] != simData.contact_object:
                simData.contact_object = simData.intersection[3]
                simData.reset_subscribers(contact)
        except:
            # The VREP probably didn't initialize yet so the function has not yet been called and no data is available.
            return
        if ret[0] == False:
            print("Error in comunication, calling function getIntersection unsuccessfully")
            return
        # If the function did not return any intersections we can't do much processing
        if ret[1] == 0:
            return

        else:
            # Get transformation matrix from world frame to suction cup frame
            if simData.cup_WORLD_f[0] == False:
                return       
            cup_tf = simData.cup_WORLD_f[1]

            # Format the points to be properly arranged 
            try:           
                interPnts_formated = contact.formatPoints(ret[1], ret[2], cup_tf)
            except:
                print("Error formating the points. Skipping the pass.",ret)
                return
            # Calculate the length of the intersection
            interLength = contact.perimiterLength(interPnts_formated)

            # In case the intersection length matches the perimiter we con say that the suction sphere has made contact
            if (interLength > (model.cup_perimiter-model.allowedPerimiterError)) and (interLength < (model.cup_perimiter+model.allowedPerimiterError)):
                set_newContact(ret[3], interPnts_formated,0, client, model, simData, contact)
            else:
                # We wait until robot moves in position where it is making contact with an object
                return


    # We are already in contact with an object
    else:
        # Check if the object has moved -> must reset the contact
        # To do that check the position compared to previous step
        temp = np.array(simData.sphere_SC_f[0:2])
        delta = np.sqrt(np.sum(np.power(np.array(temp),2)))
        # We are looking just only if it moved out from axis. Z movement will be checked otherwise
        if (delta>0.005):
            # We must try and reset the contact
            succ = contactReset(client, model, simData, contact)
            if succ == False:
                contact.in_contact = False
                contact.contact_object = -1
                args = []
                ret = client.simxCallScriptFunction('resetObjectForce@DetectionSphere','sim.scripttype_childscript',args,simData.pub_resetObjectForce)
                return
        else:
            # We set the position of the contact sphere. We must also update its position in SC frame
            client.simxSetObjectPosition(simData.detectionSphere, contact.contact_object, contact.contact_point, simData.pub_updateSpherePos)

            # From the previously stored points in object frame. Calculate their position in SC frame.
            cup_tf = simData.sc_OBJECT_f[1]
            points_formated = contact.points_move(contact.points_unformated, cup_tf)
            # Calculate new tangents, deformation pressures, ... for the updated points
            contact.setContact(points_formated, simData)

        # Add force to object. We can run the model without this feature
        if simData.contact_force_simulation == True:
            object_addSpringForce(client, model, simData, contact)

def contactReset(client, model, simData, contact, search_new = True):
    """
    Tries to reset the contat
    --------------
    search_new : whether we even try to find a new contact or just break it. 

    Returns
    --------------
    True : If successfully found a new contact, False otherwise
    """

    # First we reset the position of the contact sphere to the starting point (un-deformed)
    reset_pos = simData.sphere_zeroPos
    client.simxSetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, simData.sphere_zeroPos, client.simxServiceCall())
    # Complete loss of contact with object reset the state to defult
    if search_new == False:
        contact.in_contact = False
        return True
    else: 
        # Get transformation matrix from world frame to suction cup frame
        cup_tf = simData.cup_WORLD_f[1]

        theoretical_deformation = 0
        while theoretical_deformation < model.max_deformation+15:
            # Call to get an intersection between the object and contact sphere
            args = [simData.detectionSphere, contact.contact_object]
            ret = client.simxCallScriptFunction('getIntersection_specific@DetectionSphere','sim.scripttype_childscript',args,client.simxServiceCall())
            if ret[0]:
                if ret[1] == 0:
                    # If we got no intersection just continue
                    theoretical_deformation += 1
                    continue
                else:
                    # We found an intersection, check wheather it is the right length    
                    interPnts_formated = contact.formatPoints(ret[1], ret[2], cup_tf)
                    interLength = contact.perimiterLength(interPnts_formated)
                    if (interLength > (model.cup_perimiter-model.allowedPerimiterError)) and (interLength < (model.cup_perimiter+model.allowedPerimiterError)):
                        # It is the right length, set up a new contact
                        set_newContact(contact.contact_object, interPnts_formated,theoretical_deformation, client, model, simData, contact)
                        print("new contact set")
                        return True
                    else:
                        # Not the right length -> Maybe the sphere is already compressed so move it inwards
                        theoretical_deformation += 1
                        new_pos = [0,0, simData.sphere_zeroPos[2]-0.001*theoretical_deformation]
                        client.simxSetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, new_pos, client.simxServiceCall())

        #We could not find a new contact. Completely reset theposition of the sphere
        client.simxSetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, reset_pos, simData.pub_updateSpherePos)
        print(False)
        return False

def object_addSpringForce(client, model, simData, contact):
    """
    Adds a force and moment to the object we are contacting with the suction cup. Force and moment are based on the deformation pressures
    --------------

    Returns
    --------------
    True/False : wheather we successfully added the force to the object
    """
    # Get rotation matrix from SC frame to OBJ frame
    rotation_matrix = simData.sc_OBJECT_f[1]
    
    force_moment = contact.forceOnCup(simData)

    # Calculate the acting force in object frame
    force_X = -force_moment[0]
    force_Y = -force_moment[1]
    force_Z = -force_moment[2]
    force = np.array([force_X, force_Y, force_Z])
    force = np.dot(rotation_matrix[:,0:3],force)

    # Rotate moment to absolute frame
    rotation_matrix_mom = simData.cup_WORLD_f[1] 
    moment = -force_moment[3]
    moment = np.dot(rotation_matrix_mom[:,0:3],moment)
    
    args = [contact.contact_object, list(force), list(simData.sphere_OBJ_f), list(moment)]
    ret = client.simxCallScriptFunction('addObjectForce@DetectionSphere','sim.scripttype_childscript',args, simData.pub_addObjectForce)

    # Return wheather we successfully applied the forces to the object
    return ret

def set_newContact(contact_obj, interPnts_formated, th_def, client, model, simData, contact):
    """
    It saves the points object frame and SC frame and calculates all the normals, tangents, ... in SC frame
    --------------
    contact_obj : object we are contacting; int
    interPnts_formated : perimiter points, ordered, written in mm and SC frame; np.array(m, 3)
    client : client object
    model : suction_cup object model that contains suction cup parameters
    simData : an object that contains necessary simulation data
    contact : contact class
    --------------
    success : Returns True if new contact was successfully made
    """
    
    # Change the status of the object as in_contact = True and set which object we are contacting
    contact.in_contact = True
    contact.contact_object = contact_obj
    
    # Save the intersection in object coordinates
    # Get transformation that transforms points in contact_object frame to SC frame
    cup_tf = client.simxGetObjectMatrix(contact.contact_object, simData.suctionCup_visual, client.simxServiceCall())
    # Format the transformation matrix to be proper shape
    if cup_tf[0]:
        cup_tf = np.array(cup_tf[1])
        cup_tf = np.reshape(cup_tf, (3,4))
    points_formated = contact.points_move(interPnts_formated, cup_tf)
    contact.points_unformated = np.copy(points_formated)


    # Set 0 deformation:
    distance_min_i = np.argmin(interPnts_formated[:,2])
    contact.zero_deformation = interPnts_formated[distance_min_i,2] - simData.sphere_zeroPos[2]*1000 +th_def
    
    # Find the position of contact on the object we are contacting
    # Position of contact sphere in Object coordinates

    # Position of object in suction cup coordinates coordinates
    contact.object_position_sc = epf.b0_ObjectMatrix_inv(simData.sc_OBJECT_f[1])[0:3, 3]

    # Contact point
    simData.sphere_OBJ_f = client.simxGetObjectPosition(simData.detectionSphere, contact.contact_object, client.simxServiceCall())[1]
    contact.contact_point = simData.sphere_OBJ_f
    #contact.contact_point = client.simxGetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, client.simxServiceCall())[1]
    simData.sphere_SC_f = client.simxGetObjectPosition(simData.detectionSphere, simData.suctionCup_visual, client.simxServiceCall())[1]

    # We set the contact (It means we calculate all the pressures and normals, ...)
    contact.setContact(interPnts_formated, simData)
    
    return True

def epick_sim_step(client, model, simData, contact):
    """
    Performs one step of EPick gripper simulation with all the logic
    --------------
    contact_obj : object we are contacting; int
    interPnts_formated : perimiter points, ordered, written in mm and SC frame; np.array(m, 3)
    client : client object
    model : suction_cup object model that contains suction cup parameters
    simData : an object that contains necessary simulation data
    contact : contact class
    --------------
    None : None
    """
    # Get vacuum state.
    contact.vacuum_on = simData.vacuum_on[1]

    # Do not try making a new contact right after vacuum was turned off
    if contact.brake_time + 1 >= time.time():
        return None
    
    if contact.seal_formed == False:
        suctionCup_step(client, model, simData, contact)
        if contact.vacuum_on == 1:
            seal_formed = contact.formSeal(client, simData, model)
            contact.seal_formed = seal_formed
            if seal_formed == True:
                #Attach two objects together
                args = contact.contact_object
                ret = client.simxCallScriptFunction('createDummyLink@DetectionSphere','sim.scripttype_childscript',args, simData.pub_create_brake_link)
                simData.gOBJ = 1
            else:
                simData.vacuum_on[1] = 0
                simData.gACT = 0
                simData.gOBJ = 0

    if contact.seal_formed == True:
        # Check if the vacuum state has changed if 0 brake the contact
        # Check wheather forces broke the seat
        if contact.vacuum_on == 0:
            simData.gOBJ = 0
            print("Turning vacuum off.")
            contact.resetContact(client, simData)
        points_formated = contact.points_move(contact.points_unformated, simData.sc_OBJECT_f[1])
        # Calculate new tangetns, deformation pressures, ... for the updated points
        contact.setContact(points_formated, simData)
        # Analyze the acting forces
        vacuum = 0.07
        if simData.seal_infinite_strength != True:
            contact_state = contact.evaluateForces(vacuum, client, simData, model)
            if contact_state == False:
                print("Seal was broken. Turning vacuum off.")
                simData.gOBJ = 0
                simData.gACT = 0
                contact.resetContact(client, simData)
                return None

        simData.gOBJ = 1


    return None


def epick_simple_sim_step(client, model, simData, contact):
    """
    Performs one step of EPick gripper simulation with very simple logic
    --------------
    contact_obj : object we are contacting; int
    interPnts_formated : perimiter points, ordered, written in mm and SC frame; np.array(m, 3)
    client : client object
    model : suction_cup object model that contains suction cup parameters
    simData : an object that contains necessary simulation data
    contact : contact class
    --------------
    None : None
    """
    # Get vacuum state.
    contact.vacuum_on = simData.vacuum_on[1]

    if contact.seal_formed == False:
        if contact.vacuum_on == 1:
            seal_formed = contact.formSeal(client, simData, model)
            contact.seal_formed = seal_formed
            if seal_formed == True:
                #Attach two objects together
                args = contact.contact_object
                ret = client.simxCallScriptFunction(
                    'createDummyLink@DetectionSphere', 'sim.scripttype_childscript', args, simData.pub_create_brake_link)
                simData.gOBJ = 1
            else:
                simData.vacuum_on[1] = 0
                simData.gACT = 0
                simData.gOBJ = 0

    if contact.seal_formed == True:
        # Check if the vacuum state has changed if 0 brake the contact
        # Check wheather forces broke the seat
        if contact.vacuum_on == 0:
            print("Turning vacuum off.")
            simData.gOBJ = 0
            contact.resetContact(client, simData)
            return None

        # Analyze the acting forces
        vacuum = 0.07
        if simData.seal_infinite_strength != True:
            contact_state = contact.evaluateForces(vacuum, client, simData, model)
            if contact_state == False:
                print("Seal was broken. Turning vacuum off.")
                simData.gOBJ = 0
                simData.gACT = 0
                contact.resetContact(client, simData)
                return None

        simData.gOBJ = 1

    return None
