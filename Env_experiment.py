#!/user/bin/env python
# coding: UTF-8

import sys
import numpy as np
import datetime
import time
import termios
from timeout_decorator import timeout, TimeoutError
import pandas as pd
import matplotlib.pyplot as plt

import rospy
import tf2_ros
import tf_conversions
import tf
from crazyswarm.msg  import GenericLogData

# 関連モジュールのインポート
from tools.Decorator import run_once
from frames_setup import Frames_setup
from tools.Mathfunction import LowPath_Filter, Mathfunction
from tools.Log import Log_data

# 定値制御
class Env_Experiment(Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self, Texp, Tsam, num):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

        self.Tend = Texp
        self.Tsam = Tsam
        self.t = 0


        self.mathfunc = Mathfunction()
        self.LowpassP = LowPath_Filter()
        self.LowpassP.Init_LowPass2D(fc=5)
        self.LowpassV = LowPath_Filter()
        self.LowpassV.Init_LowPass2D(fc=5)
        self.LowpassE = LowPath_Filter()
        self.LowpassE.Init_LowPass2D(fc=5)
        
        self.set_frame()
        self.set_key_input()
        self.set_log_function()
        self.init_state()
        
        self.log = Log_data(num)
        
        time.sleep(0.5)

    def set_frame(self):
        self.world_frame = Frames_setup().world_frame
        self.child_frame = Frames_setup().children_frame[0]
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        time.sleep(0.5)

    def set_key_input(self):
    # 以下ではターミナルからの入力をEnterなしで実行するための設定変更
        self.fd = sys.stdin.fileno()

        self.old = termios.tcgetattr(self.fd)
        self.new = termios.tcgetattr(self.fd)

        self.new[3] &= ~termios.ICANON
        self.new[3] &= ~termios.ECHO

    def set_log_function(self):

        self.cmd_sub = rospy.Subscriber("/cf20/log1", GenericLogData, self.log_callback)

    def init_state(self):
        self.P = np.zeros(3)
        self.Ppre = np.zeros(3)
        self.Vrow = np.zeros(3)
        self.Vfiltered = np.zeros(3)
        self.R = np.zeros((3, 3))
        self.Euler = np.zeros(3)

        try:
            f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

        # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            exit()

        self.P[0] = f.transform.translation.x; self.P[1] = f.transform.translation.y; self.P[2] = f.transform.translation.z
        self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
        self.Euler = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
        self.Eulerpre = self.Euler
        # self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)[:3, :3]
        self.R = self.mathfunc.Euler2Rot(self.Euler)
# ----------------------　ここまで　初期化関数-------------------------

    def update_state(self):
        try:
            f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

        # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            exit()

        # 位置情報の更新
        self.P[0] = f.transform.translation.x; self.P[1] = f.transform.translation.y; self.P[2] = f.transform.translation.z
        self.P = self.LowpassP.LowPass2D(self.P, self.dt)
        # self.P[0] = self.mathfunc.Remove_outlier(self.P[0], self.Ppre[0], 0.2)
        # self.P[1] = self.mathfunc.Remove_outlier(self.P[1], self.Ppre[1], 0.2)
        # self.P[2] = self.mathfunc.Remove_outlier(self.P[2], self.Ppre[2], 0.1)
        # 速度情報の更新
        self.Vrow = self.mathfunc.deriv(self.P, self.Ppre, self.dt)
        self.Vfiltered = self.LowpassV.LowPass2D(self.Vrow, self.dt)
        # 姿勢角の更新
        self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
        self.Euler = self.LowpassE.LowPass2D(tf_conversions.transformations.euler_from_quaternion(self.Quaternion), self.dt)
        self.Euler[0] = self.mathfunc.Remove_outlier(self.Euler[0], self.Eulerpre[0], 0.1)
        self.Euler[1] = self.mathfunc.Remove_outlier(self.Euler[1], self.Eulerpre[1], 0.1)
        self.Euler[2] = self.mathfunc.Remove_outlier(self.Euler[2], self.Eulerpre[2], 0.1)
        # self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)[:3, :3]
        self.R = self.mathfunc.Euler2Rot(self.Euler)
        self.Ppre[0] = self.P[0]; self.Ppre[1] = self.P[1]; self.Ppre[2] = self.P[2]
        self.Eulerpre = self.Euler
    
    def set_clock(self, t):
        self.t = t

    def set_dt(self, dt):
        self.dt = dt

    def log_callback(self, log):
        self.M = log.values


    def set_reference(self, controller,  
                            P=np.array([0.0, 0.0, 0.0]),   
                            V=np.array([0.0, 0.0, 0.0]), 
                            R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                            Euler=np.array([0.0, 0.0, 0.0]), 
                            Wb=np.array([0.0, 0.0, 0.0]), 
                            Euler_rate=np.array([0.0, 0.0, 0.0]),
                            traj="circle",
                            controller_type="pid",
                            command = "hovering",
                            init_controller=True):
        if init_controller:
            controller.select_controller()
        if controller_type == "pid":
            if command =="hovering":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type) 
            else:
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)
        elif controller_type == "mellinger":
            controller.set_reference(traj, self.t)


    def take_log(self, ctrl):
        self.log.write_state(self.t, self.P, self.Vfiltered, self.R, self.Euler, np.zeros(3), np.zeros(3), self.M)
        ctrl.log(self.log, self.t)

    def save_log(self):
        self.log.csvWriter_state(self.t, self.P, self.Vfiltered, self.R, self.Euler, self.M)

    @timeout(0.01)
    def input_with_timeout(self, msg=None):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.new)
        return sys.stdin.read(1)

    def get_input(self, input_thrust):
        flag = False
        try:
            input = self.input_with_timeout("key:")
            if input == "w":
                input_thrust += 0.1
            elif input == 'x':
                input_thrust -= 0.1
            elif input == "c":
                flag = True
                input_thrust = 0.0
            else:
                input = "Noinput"
        except TimeoutError:
            input = "Noinput"

        return input_thrust, flag

    def time_check(self, Tint, Tend):
        if Tint < self.Tsam:
            time.sleep(self.Tsam - Tint)
        if self.t > Tend:
            return True
        return False

    @run_once
    def land(self, controller):
        controller.switch_controller("pid")
        self.set_reference(controller=controller, command="land", init_controller=True, P=self.land_P)
        
    @run_once
    def hovering(self, controller, P, Yaw=0.0):
        self.set_reference(controller=controller, command="hovering", P=P, Euler=np.array([0.0, 0.0, Yaw]))
        self.land_P = np.array([0.0, 0.0, 0.1])

    def takeoff(self, controller):
        self.set_reference(controller=controller, traj="takeoff", controller_type="mellinger")
        self.land_P = np.array([0.0, 0.0, 0.1])

    def land_track(self, controller):
        self.set_reference(controller=controller, traj="land", controller_type="mellinger", init_controller=False)

    def track_straight(self, controller, flag):
        self.set_reference(controller=controller, traj="straight", controller_type="mellinger", init_controller=flag)

    def track_circle(self, controller, flag):
        self.set_reference(controller=controller, traj="circle", controller_type="mellinger", init_controller=flag)

    def stop_track(self, controller):
        self.set_reference(controller=controller, traj="stop", controller_type="mellinger", init_controller=False)
        self.land_P[0:2] = self.P[0:2]