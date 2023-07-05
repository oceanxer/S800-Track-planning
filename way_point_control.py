# -*- coding: UTF-8 -*-
import time
import numpy as np
from pymavlink import mavutil
from controller_lib import *
from oceanxer_lib import *
import yaml

#S800ROV vehicle params
config_path = "param.yaml"
with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)

#connect the S800 ROV, and receieve the heartbeat signal
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

#S800 vehicle class
#此类中定义了机器人常用的功能，如解锁，关锁，读取深度传感器信息与姿态信息，还定义了推进器命令的程序
class S800_ROV():
    def __init__(self):
        self.arm()
        self.x_controller = PID(10,0.0,0.01)
        self.y_controller = PID(10,0.0,0.01)
        self.depth_controller = PID(65, 0.9, 0.01)
        self.roll_controller = PID(10, 0.9, 0.01)
        self.pitch_controller = PID(10, 0.9, 0.01)
        self.yaw_controller = PID(10, 0.9, 0.01)
        controller_period = 0.1
        self.x_controller.set_time(controller_period)
        self.y_controller.set_time(controller_period)
        self.depth_controller.set_time(controller_period)
        self.pitch_controller.set_time(controller_period)
        self.roll_controller.set_time(controller_period)
        self.yaw_controller.set_time(controller_period)
        while True:
            try:
                # 设定深度目标值
                pos_d = np.array([10,10,10])
                pos   = np.array([0,0,0])
                rpy = self.angle_data()
                Ts = 0.2
                pose_d= self.way_point_task(self,pos,pos_d,rpy,Ts)
                tau = self.basic_controller(pose_d)
                pwm = S800_ThrustAllocation(tau,config_file)
                self.S800_go(pwm)
            except KeyboardInterrupt:
                self.disarm()
                break

    # Arm 解锁S800_ROV
    def arm():
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
    
    # Disarm 锁定S800_ROV
    def disarm():
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
    
    def depth_data():
        """
        读取深度信息
        :return: depth
        """
        dep = 0
        while True:
            msg = master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'VFR_HUD':
                dep = -msg.alt
                break
        return dep
    

    # 读取角度信息
    # return: pitch_angle, roll_angle
    def angle_data():
        
        theta, phi = 0, 0
        while True:
            msg = master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'ATTITUDE':
                theta = msg.pitch
                phi = msg.roll
                break
        return  phi,theta
    
    # 设置每个推进器的pwm
    def set_motor_pwm(channel, pwm):
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                    0, channel, 1, pwm, 1000, 1, 0, 0)
    
    #control the vehicle depth, and remain the roll and pitch at the same time --OCEANXER
    def basic_controller(self,x_target,y_target,depth_target,roll_target=0,pitch_target=0):
        x_target = 0
        y_target = 0
        depth = self.depth_data()
        roll, pitch = self.angle_data()
        self.depth_controller.set_target(depth_target)
        self.pitch_controller.set_target(pitch_target)
        self.roll_controller.set_target(roll_target)

        self.depth_controller.calcu(depth)
        self.pitch_controller.calcu(pitch)
        self.roll_controller.calcu(roll)
        #control force = [X Y Z K M N]   <-- SNAME notation
        tau = np.array([0,0,self.depth_controller.output,self.roll_controller.output,self.pitch_controller.output,0])
        return tau
    def USBLbringup(self):
        pos = np.array([0,0,0])
        return pos
    def S800_go(self,pwm):
        for i in range(8):
            self.set_motor_pwm(i,pwm[i])
        return
    def way_point_task(self,pos,pos_d,rpy,Ts):
        #根据当前位置与期望位置，以及此刻机器人的姿态，进行运动规划
        #规划出想要的速度
        #首先计算任务向量，标量，让当前姿态与期望姿态的norm为0
        sigma = (pos_d-pos).transpose().dot(pos_d-pos)
        R_B_I = Rpy2Rot(rpy)
        zero = np.zeros(1,3)
        J = np.hstack((-(pos_d-pos).reshape(1,3)/sigma * R_B_I, zero))  #shape (1, 6)
        vd = np.linalg.pinv(J)*sigma # shape (6,1)
        pose_d = pos + vd*Ts
        return pose_d
        
         

    

 

 
if __name__ == '__main__':
    rov = S800_ROV()
