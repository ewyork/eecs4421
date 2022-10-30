###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
"""
4421 a3: This file contains example code from Kinova exmaples

https://github.com/Kinovarobotics/kortex/tree/master/api_python/examples

"""
import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 100

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def set_gripper(base, position):
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.value = position
    print(f"Going to position {position}")
    base.SendGripperCommand(gripper_command)


def get_gripper(base):
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
    if len (gripper_measure.finger):
        print(f"Current position is : {gripper_measure.finger[0].value}")
        return gripper_measure.finger[0].value
    return None

 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_angular_action_movement(base, angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    print(actuator_count.count)
    if actuator_count.count != len(angles):
        print(f"bad lengths {actuator_count.count} {len(angles)}")
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic, pos):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    """
    cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
    """
    cartesian_pose.x = pos[0]          # (meters)
    cartesian_pose.y = pos[1]    # (meters)
    cartesian_pose.z = pos[2]    # (meters)
    
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def populateAngularPose(jointPose,durationFactor):
    waypoint = Base_pb2.AngularWaypoint()
    waypoint.angles.extend(jointPose)
    waypoint.duration = durationFactor*5.0    
    
    return waypoint
 
def example_trajectory(base, base_cyclic):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    jointPoses = tuple(tuple())
    product = base.GetProductConfiguration()

    if(   product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L53 
    or product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
        if(product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
            jointPoses = (  (0.0,  344.0, 75.0,  360.0, 300.0, 0.0),
                            (0.0,  21.0,  145.0, 272.0, 32.0,  273.0),
                            (42.0, 334.0, 79.0,  241.0, 305.0, 56.0))
        else:
            # Binded to degrees of movement and each degrees correspond to one degree of liberty
            degreesOfFreedom = base.GetActuatorCount();

            if(degreesOfFreedom.count == 6):
                jointPoses = (  ( 360.0, 35.6, 281.8, 0.8,  23.8, 88.9 ),
                                ( 359.6, 49.1, 272.1, 0.3,  47.0, 89.1 ),
                                ( 320.5, 76.5, 335.5, 293.4, 46.1, 165.6 ),
                                ( 335.6, 38.8, 266.1, 323.9, 49.7, 117.3 ),
                                ( 320.4, 76.5, 335.5, 293.4, 46.1, 165.6 ),
                                ( 28.8,  36.7, 273.2, 40.8,  39.5, 59.8 ),
                                ( 360.0, 45.6, 251.9, 352.2, 54.3, 101.0 ))
            else:
                jointPoses = (  ( 360.0, 35.6, 180.7, 281.8, 0.8,   23.8, 88.9  ),
                                ( 359.6, 49.1, 181.0, 272.1, 0.3,   47.0, 89.1  ),
                                ( 320.5, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6 ),
                                ( 335.6, 38.8, 177.0, 266.1, 323.9, 49.7, 117.3 ),
                                ( 320.4, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6 ),
                                ( 28.8,  36.7, 174.7, 273.2, 40.8,  39.5, 59.8  ),
                                ( 360.0, 45.6, 171.0, 251.9, 352.2, 54.3, 101.0 ))
            
    else:
        print("Product is not compatible to run this example please contact support with KIN number bellow")
        print("Product KIN is : " + product.kin())


    waypoints = Base_pb2.WaypointList()    
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False
    
    index = 0
    for jointPose in jointPoses:
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(index)
        durationFactor = 1
        # Joints/motors 5 and 7 are slower and need more time
        if(index == 4 or index == 6):
            durationFactor = 6 # Min 30 seconds
        
        waypoint.angular_waypoint.CopyFrom(populateAngularPose(jointPose, durationFactor))
        index = index + 1 
    
    
    # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching angular pose trajectory...")
        
        
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    else:
        print("Error found in trajectory") 
        print(result.trajectory_error_report)
        return finished
 

def example_forward_kinematics(base):
    # Current arm's joint angles (in home position)
    try:
        print("Getting Angles for every joint...")
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Joint ID : Joint Angle")
    for joint_angle in input_joint_angles.joint_angles:
        print(joint_angle.joint_identifier, " : ", joint_angle.value)
    print()
    
    # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
    try:
        print("Computing Foward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Pose calculated : ")
    print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
    print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
    print()
    return True

def example_inverse_kinematics(base):
    # get robot's pose (by using forward kinematics)
    try:
        input_joint_angles = base.GetMeasuredJointAngles()
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to get current robot pose")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    # Object containing cartesian coordinates and Angle Guess
    input_IkData = Base_pb2.IKData()
    
    # Fill the IKData Object with the cartesian coordinates that need to be converted
    input_IkData.cartesian_pose.x = pose.x
    input_IkData.cartesian_pose.y = pose.y
    input_IkData.cartesian_pose.z = pose.z
    input_IkData.cartesian_pose.theta_x = pose.theta_x
    input_IkData.cartesian_pose.theta_y = pose.theta_y
    input_IkData.cartesian_pose.theta_z = pose.theta_z

    # Fill the IKData Object with the guessed joint angles
    for joint_angle in input_joint_angles.joint_angles :
        jAngle = input_IkData.guess.joint_angles.add()
        # '- 1' to generate an actual "guess" for current joint angles
        jAngle.value = joint_angle.value - 1
    
    try:
        print("Computing Inverse Kinematics using joint angles and pose...")
        computed_joint_angles = base.ComputeInverseKinematics(input_IkData)
    except KServerException as ex:
        print("Unable to compute inverse kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Joint ID : Joint Angle")
    joint_identifier = 0
    for joint_angle in computed_joint_angles.joint_angles :
        print(joint_identifier, " : ", joint_angle.value)
        joint_identifier += 1

    return True
