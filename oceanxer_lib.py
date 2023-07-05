# -*- coding: UTF-8 -*-
# 探索海洋python程序软件库。其中程序用于机器人的控制与机械臂Jacobian矩阵计算
import numpy as np
from math import cos,sin
def Rot_dh(alpha,theta):   #pass test
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)

    R = np.array([[ct,-st*ca,st*sa],
                  [st ,ct*ca, -ct*sa],
                  [ 0 ,sa ,ca]])
    return R

def Homogeneous_dh(DH_i):  #pass test with  order a alpha d theta
# input: DH_i     dim 1x4     row i of the Denavit-Hartenberg table
# output: T      dim 4x4      Homogeneous transformation matrix  T_i^{i-1}
# Homogeneous transformation matrix between consecutive frames according to DH convention
    a = DH_i[0]
    alpha=DH_i[1]
    d=DH_i[2]
    theta=DH_i[3]
    R = Rot_dh(alpha,theta)
    #print("rotational matrix",R)
    p = np.array([a*cos(theta),a*sin(theta),d]).reshape(3,1)
    T = np.vstack( (np.hstack((R   ,  p)),
                    np.array([0,0,0,1])))
    #print("T",T)
    return T

def S(a):     # pass test

    a1=a[0]
    a2=a[1]
    a3=a[2]
    sa = np.array([ [0,-a3,a2],
                    [a3,0,-a1],
                    [-a2,a1,0] ]  )
    return sa

def J_man(DH):   #pass test  
    #np.dot()   is not the same as *, pay attention!
    # calcute base Jacobian matrix from frame 2 to frame 0, frame0 is the same with frame1 but 0 is static
    # if want to calcute ee to frame0, then DH need to be 3*4, the third line is static, is the ee frame to joint2 frame
    # one leg only have 2 joints.(neglect third joint)
    n = len(DH)   #(for array with 2 dimention, len() will return row of the array)
                     # for array with only 1 dimention, len() will return num of elements of the array
    J = np.zeros((6,n))
    p = np.zeros((3,n))  #i coloum of p matrix is position vector from base frame to frame i
    z = np.zeros((3,n))
    # compute homog. transf. from base frame
    T0 = np.zeros((4,4,n))
    T0[:,:,0] = Homogeneous_dh(DH[0,:])
    # p: 3xn matrix, the generic column is the position of frame i expressed in inertial frame
    p[:,0] = T0[0:3,3,0]
    # z: 3xn matrix, the generic column is the z-versor of frame i expressed in inertial frame
    z[:,0] = T0[0:3,2,0]
    
    for i in range(1,n):
        T_i = Homogeneous_dh(DH[i,:])
        T0[:,:,i] = np.dot(T0[:,:,i-1],T_i)
        p[:,i] = T0[0:3,3,i]
        z[:,i] = T0[0:3,2,i]
    z0 = np.array([0,0,1]).T
    p0 = np.array([0,0,0]).T
    J[:,0] = np.hstack( (np.cross(z0, p[:,n-1]-p0 )   ,
                                         z0    )).T
    for i in range(1,n):
        J[:,i] = np.hstack( [np.cross(z[:,i-1],p[:,n-1]-p[:,i-1]),
                                             z[:,i-1]     ]) .T                                    
    return J

def Rpy2Rot(rpy):  #pass test   if we want get R_i^b, just transpose this func is ok
    #R_I_B = Rpy2Rot(eta[3:6]).transpose()
    #R_B_I = Rpy2Rot(eta[3:6])
    phi   = rpy[0]
    theta = rpy[1]
    psi   = rpy[2]
    cp = cos(psi) ;sp = sin(psi);ct = cos(theta);st = sin(theta);cf = cos(phi);sf = sin(phi)
    R_B_I = np.array([  [ cp*ct, -sp*cf+cp*st*sf , sp*sf+cp*cf*st        ],
                        [ sp*ct , cp*cf+sf*st*sp ,-cp*sf+st*sp*cf         ],
                        [   -st  ,   ct*sf        ,      ct*cf        ]    ])
    return R_B_I  #R_b^i   

def Directkinematics(eta,DH,T_0_B):    # pass test
    #direct kinematics  
    # input:
    #       eta    dim 6x1     vehicle position/orientation
    #       DH     dim nx4     Denavit-Hartenberg table
    #       T_0_B  dim 4x4     Homogeneous transformation               T_0^B
    #                          matrix from vehicle to zero frame
    #output:T      dim 4x4    Homogeneous transformation  matrix from inertial to end-effector   
    n = len(DH)     # joint number
    eta1 = eta[0:3]
    eta2 = eta[3:6]
    RBI = Rpy2Rot(eta2)
    T = np.vstack( (np.hstack((RBI,eta1.reshape(3,1))),
                np.array([0,0,0,1])  ))   #T_B^I
    # from vehicle-fixed to zero
    T = np.dot(T,T_0_B)
    # manipulator cycle
    for i in range (n):
        TT = Homogeneous_dh(DH[i,:])
        T = np.dot(T,TT)
    return T  
def Directkinematics2(DH,T_0_B):    # pass test
    #usage  T = Directkinematics2(DH,T_0_B)
    #direct kinematics  
    # input:
    #       eta    dim 6x1     vehicle position/orientation
    #       DH     dim nx4     Denavit-Hartenberg table
    #       T_0_B  dim 4x4     Homogeneous transformation               T_0^B
    #                          matrix from vehicle to zero frame
    #output:T      dim 4x4    Homogeneous transformation  matrix from bodyframe to end-effector   
    #  T = T_0_B * T_1_0 * T_ee_1   = T_ee^b
    n = len(DH)     # joint number
    T = T_0_B                  
    # manipulator cycle
    for i in range (n):
        TT = Homogeneous_dh(DH[i,:])
        T = np.dot(T,TT)
    return T  

def Jacobian(eta,DH,T_0_B):   #pass test   vee jacobian and v3 jacobian are the same
    # Computes the Jacobian from zita to end-effector linear and angular velocities expressed in the inertial frame
    #vee = J * [v1 v2 q_dot]'
    n = len(DH)
    eta1 = eta[0:3]
    eta2 = eta[3:6]
    r_B0_B = T_0_B[0:3,3]
    R_0_B = T_0_B[0:3,0:3]
    J = np.zeros((6,6+n))

    Jman = J_man(DH);   R_B_I = Rpy2Rot(eta2);    R_0_I = np.dot(R_B_I , R_0_B)
    T = Directkinematics(eta,DH,T_0_B); eta_ee1 = T[0:3,3];  r_B0_I = np.dot(R_B_I,r_B0_B);   eta_0ee_I = eta_ee1 - eta1 - r_B0_I

    # Position Jacobian
    J[0:3,0:3]   = R_B_I
    J[0:3,3:6]   = np.dot(-(S(r_B0_I) + S(eta_0ee_I))   , R_B_I)
    J[0:3,6:6+n] = np.dot(R_0_I,  Jman[0:3,:])

    # Orientation Jacobian
    J[3:6,0:3]   = np.zeros((3,3))
    J[3:6,3:6]   = R_B_I
    J[3:6,6:6+n] = np.dot(R_0_I,Jman[3:6,:])
    return J

def J_ko(rpy):    #pass test
    phi   = rpy[0]
    theta = rpy[1]
    psi   = rpy[2]
    cp = cos(psi);		sp = sin(psi)
    ct = cos(theta);	st = sin(theta)
    cf = cos(phi);		sf = sin(phi)
    J_ko = np.array([[ 1    ,0      ,-st ],
                    [ 0     ,cf     ,ct*sf    ],
                    [ 0     ,-sf    ,ct*cf    ]])
    #J_ko   $ v_2 = J_ko * \eta_2_dot $
    return J_ko
def J_ko_inv(rpy):    #pass test
    phi   = rpy[0]
    theta = rpy[1]
    psi   = rpy[2]
    cp = cos(psi);		sp = sin(psi)
    ct = cos(theta);	st = sin(theta)
    cf = cos(phi);		sf = sin(phi)
    J_ko_inv = 1/ct  * np.array([[ ct    ,sf*st      ,cf *st ],
                                 [ 0     ,cf * ct     ,-sf*ct    ],
                                  [ 0     ,  sf    ,  cf   ]])
    #J_ko   $ v_2 = J_ko * \eta_2_dot $
    #J_ko_inv    eta2_dot = J_ko_inv * v2
    return J_ko_inv
def J_e(rpy):   #v = J_e *  eta_dot   pass test 
    # jacobian from velociety in inertia frame to body frame
    phi   = rpy[0]
    theta = rpy[1]
    psi   = rpy[2]
    cp = cos(psi);		sp = sin(psi)
    ct = cos(theta);	st = sin(theta)
    cf = cos(phi);		sf = sin(phi)
    Jko = J_ko(rpy)
    R_B_I = Rpy2Rot(rpy)
    zeros = np.zeros((3,3))
    J_e = np.vstack(( np.hstack((R_B_I.T,zeros))      ,
                        np.hstack((zeros, Jko))      ))#R_B_I.T is inverse of R_B_I
    #v = J_e *  eta_dot
    return J_e

def S800_ThrustAllocation(torque, config_file):
    """
       推力分配计算
       :param torque: torque vector
       :param TCM: thruster allocation matrix
       
       :return: 分配的8个推进器的推力,4x1矩阵
    """
    # torque2thrusterforce
    thruster_coeffs = np.array(config_file["thruster_coeffs"]).reshape(6,8)
    forces = np.linalg.pinv(thruster_coeffs)*torque
    #force to PWM
    pwm_values = []
    NEUTRAL_PWM = 1500
    pwm_file = config_file["thruster"]
    thruster_types = pwm_file["THRUST_TYPE"]
    for i in range(thruster_coeffs.shape[0]):
            pwm = NEUTRAL_PWM

            if (abs(forces[i]) < pwm_file["MIN_THRUST"]):
                pwm = NEUTRAL_PWM

            elif (forces[i] > 0 and forces[i] <= pwm_file["STARTUP_THRUST"]):
                if  thruster_types[i] == 0:
                    pwm = (int)( pwm_file["SU_THRUST"]["POS_SLOPE"] * forces[i] +  pwm_file["SU_THRUST"]["POS_YINT"])
                else:
                    pwm = (int)(- pwm_file["SU_THRUST"]["POS_SLOPE"] * forces[i] +  pwm_file["SU_THRUST"]["NEG_YINT"])

            elif (forces[i] > 0 and forces[i] >  pwm_file["STARTUP_THRUST"]):
                if  thruster_types[i] == 0:
                    pwm = (int)( pwm_file["THRUST"]["POS_SLOPE"] * forces[i] +  pwm_file["THRUST"]["POS_YINT"])
                else:
                    pwm = (int)(- pwm_file["THRUST"]["POS_SLOPE"] * forces[i] +  pwm_file["THRUST"]["NEG_YINT"])

            elif (forces[i] < 0 and forces[i] >= - pwm_file["STARTUP_THRUST"]):
                if  thruster_types[i] == 0:
                    pwm = (int)( pwm_file["SU_THRUST"]["NEG_SLOPE"] * forces[i] +  pwm_file["SU_THRUST"]["NEG_YINT"])
                else:
                    pwm = (int)(- pwm_file["SU_THRUST"]["NEG_SLOPE"] * forces[i] +  pwm_file["SU_THRUST"]["POS_YINT"])

            elif (forces[i] < 0 and forces[i] < - pwm_file["STARTUP_THRUST"]):
                if  thruster_types[i] == 0:
                    pwm = (int)( pwm_file["THRUST"]["NEG_SLOPE"] * forces[i] +  pwm_file["THRUST"]["NEG_YINT"])
                else:
                    pwm = (int)(- pwm_file["THRUST"]["NEG_SLOPE"] * forces[i] +  pwm_file["THRUST"]["POS_YINT"])

            else:
                pwm = NEUTRAL_PWM

            pwm_values.append(pwm)
    return pwm_values