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

import sys
import os
import time
import threading
import numpy as np
import a3kinematics
import kinova_gen3

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

"""
Assignment 3

#
# https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/110-Waypoints/01-send_angular_wapoint_trajectory.py
#
"""

def straightline_trajectory(base, jointPoses):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)


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


def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities


    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_forward_kinematics(base)     # print angles
        success &= example_angular_action_movement(base, [90,0,0,0,0,0])
        success &= example_forward_kinematics(base)     # print angles
        
        # record angles for test points
        # results should match ik()
        test_pts = [[0.400, 0, 0], [0.057,-0.01, 0.10033], [0.760, -0.01, 0.1863],[0.01, 0.480 , 0.4663]]
        for p in test_pts:
            success &= example_cartesian_action_movement(base, base_cyclic, p)
            joint_angles = base.GetMeasuredJointAngles()
            print("Joint ID : Joint Angle")
            for joint_angle in input_joint_angles.joint_angles:
                print(joint_angle.joint_identifier, " : ", joint_angle.value)
            print()
        
        # run example trajectory code
        success &= example_trajectory(base, base_cyclic)
        
        """
        success &= example_cartesian_action_movement(base, base_cyclic)
        # For now, move first 3 axis so we know the connection is working 
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [154,0,0,0,0,0])
        """
        
        # 
        # return to home position
        success &= example_move_to_home_position(base)

        # Run A3 algorithms
        line=input('Enter start location, format x y z: ')
        start = np.array([float(x) for x in line.split()])
        start = start[:3]
        print('Start location: ', start)
    
        # Q4: 
        # Send the end effector to start of trajectory
        # points to test: 1.  0.057 -0.01 0.10033, 0.760 -0.01 0.1863, 0.01 0.480 0.4663
        joint, success = a3kinematics.MoveToPoint(start)
        if not success:
            print(start, ' Not Reachable')
            return 0
        
        success &= example_angular_action_movement(base, [joint[0],joint[1], joint[2],0,0,0])
        success &= example_forward_kinematics(base)     # print angles & pos
             
        #feedback = base_cyclic.RefreshFeedback()  
        #print(f'Reached position: ({feedback.base.tool_pose_x}, {feedback.base.tool_pose_y}, {feedback.base.tool_pose_z})')
        
        # Q5: Move End effector from start to end in a straight line trajectory
        
        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space
        
        line = input("Enter end location: ")
        stop = np.array([float(x) for x in line.split()])
        stop = stop[:3]
        print('End location: ', stop)
    
        waypoints, success= a3kinematics.ComputeJointTrajectory(start, stop)
        if not success:
            print(f'Straight line trajectory {start} to {end} not possible')
            return 0
        
        print("Sending precomputed trajectory to gen3...")
        success &= straightline_trajectory(base, waypoints)
        success &= example_forward_kinematics(base)     # print angles & pos
           
        return 0 if success else 1      

if __name__ == "__main__":
    exit(main())


