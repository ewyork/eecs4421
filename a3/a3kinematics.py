# -*- coding: utf-8 -*-
"""
EECS4421 Assignment 3
Nov. 4, 2022

"""

import numpy as np
import math

# inverse kinematics 3 DoF
# calculate joint angles for given
def ik(x,y,z):   
    d1 = 0.2433
    l2=0.28
    l3=0.48

    # adjust for offset
    y = y+0.01
    x = x-0.057

    r1 = math.sqrt(x**2 + y**2)
    r2 = z - d1
    lsq = r2**2 + r1**2
    #print(f'r1={r1}, r2={r2}, l={lsq}')

    # 2 solutions +/-arctan2(y, x)
    theta1 = np.arctan2(y, x)
    # check limit
    q1 = math.degrees(theta1)
    if abs(q1) > 154.1:
        print('q1 out of range=', q1)
        return None, False

    # theta3
    ct3 = (lsq - l2**2 - l3**2) / (2*l2*l3)
    if abs(ct3) > 1:
        print(f'cosq3= {ct3}')
        ct3 = 1.0

    # +/-
    st3 = -math.sqrt(1-ct3**2)
    #print(f'cosq3= {ct3}, {st3}')
    theta3 = np.arctan2(st3, ct3)

    q3 =  -math.degrees(theta3)   # 2 values
    if abs(q3) > 150.0:
        print('q3 out of range=', q3)
        return None, False

    # theta2
    alpha = np.arctan2(r2, r1)
    beta = np.arctan2(l3*st3, l2+l3*ct3)
    theta2 = alpha - beta

    q2 = math.degrees(theta2)-90
    if abs(q2) > 150.0:
        print('q2 out of range=', q2)
        return None, False

    return [q1, q2, q3], True

def iklist(points):
    numpts =points.shape[0]
    qs = np.zeros_like(points)
    for i in range(numpts):
        p = points[i]
        qs[i], success = ik(p[0], p[1], p[2])
        if success == False:
            print(f'No inverse k for {p}')
            return None, False

    return qs, True


# Accepte les valeurs theta, d, a, alpha. Les angles doivent être en radians.
def dh(theta,d,a,alpha):
    theta = math.radians(theta)
    alpha = math.radians(alpha)

    return np.array([
        [np.cos(theta), -1*np.sin(theta)*np.cos(alpha),    np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),    np.cos(theta)*np.cos(alpha), -1*np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [            0,                  np.sin(alpha),                  np.cos(alpha),               d],
        [            0,                              0,                              0,               1]
    ])

"""
# Fonction dk(x, y, z, w, p, r)
# Arguments: Theta 1, Theta 2 , Theta 3  en degrées.
# Sortie: Vecteur [x, y, z, w, p, r] en mm et en degrées.
def dk(t1,t2,t3):   
    H10 = dh(t1,0.2433,0,90)
    H21 = dh(t2+90,0.010,.280,180)
    H32 = dh(t3+90,0,.057,90)
    Htool3 = dh(0,.480,0,0)
    
    H = np.matmul(H10, H21)
    H = np.matmul(H, H32)
    H = np.matmul(H, Htool3)
    
    if abs(H[2,0]) == 1:
        p = -H[2,0]*np.pi/2
        w = 0 # valeur arbitraire, on choisit w = 0
        r = np.degrees(np.arctan2(-H[2,0]*H[1,2], H[1,1])) #*180/np.pi
    else:
        p = np.arctan2(-H[2,0], math.sqrt(math.pow(H[0,0],2)+math.pow(H[1,0],2)))
        cp = np.cos(p)
        r = np.arctan2(H[1,0]/cp, H[0,0]/cp)
        w = np.arctan2(H[2,1]/cp, H[2,2]/cp)
        
        p = np.degrees(p)
        r = np.degrees(r)
        w = np.degrees(w)
        
    #return np.array([H[0,3], H[1,3], H[2,3], w, p, r])
    return [H[0,3], H[1,3], H[2,3]]
"""

def fk(q1, q2, q3):
    q1 =  math.radians(q1)
    q23 = math.radians(q2-q3)
    q2 = math.radians(q2)
    
    a = 0.057*np.cos(q23)-0.48*np.sin(q23)-0.28*np.sin(q2)
    x = np.cos(q1)*a + 0.01*np.sin(q1)
    y = np.sin(q1)*a - 0.01*np.cos(q1)
    z = 0.057*np.sin(q23)+0.48*np.cos(q23)+0.28*np.cos(q2)+0.2433

    return [x, y, z]

# Q3 Move to (x, y, z)
def MoveToPoint(p):
    #p =[0.760, -0.01, 0.1863]

    # check if inside effective workspace

    # compute inverse kinematics
    joint, success = ik(p[0], p[1], p[2])
    print('joint=', joint)

    if success:
        # calculate fwd with return returned angles to see if they match
        pos = fk(joint[0], joint[1], joint[2])
        print('pos=', pos)

        return joint, True
    else:
        print('Unable to move to ', p)
        return None, False


def ComputeJointTrajectory(start, stop):
    delta = np.array(stop - start).reshape(1,-1)
    dt=0.1
    dist = np.sqrt(np.sum(delta**2))
    t = np.arange(0, 1+dt/dist, step=dt/dist, dtype=float).reshape(-1,1)
    waypoints = start + delta*t
    waypoints[-1] = stop
    print('waypoints=', waypoints)
      
    # inverse k 
    qs_deg,success = iklist(points)
    if not success:
        print('Inverse K failed')
        return None, False
        """
    v = np.diff(qs_deg)/dt;
    v[0,:] = 0;
    v[-1,:] = 0;
    acc = np.diff(v)/dt;
    acc[0,:] = 0;
    acc[-1,:] = 0;
    #print(v, acc)
    timestamp = np.arange(0, t[-1], step=0.001);
    qs_deg = np.interp(t,qs_deg,timestamp);
    vel = np.interp(t,vel,timestamp);
    acc = np.interp(t,acc,timestamp);
    """
    return waypoints, True

# test
def a3test():
    line=input('Enter start location, format x y z: ')
    start = np.array([float(x) for x in line.split()])
    start = start[:3]
    print('Start location: ', start)
    
    join, success = MoveToPoint(start)
    #joint = [0, 0, 0]
    if success:
        #success &= example_angular_action_movement(base, [joint[0],joint[1], joint[2],0,0,0])
        print("Reached destination: ", start, 'Joints=', joint)
        
        line = input("Enter end location: ")
        stop = np.array([float(x) for x in line.split()])
        stop = stop[:3]
        print('End location: ', stop)
    
        waypoints, success= ComputeJointTrajectory(start, stop)
        if success:
            # send to traj
            print("Sending precomputed trajectory to gen3...")
    
    # cleanup
