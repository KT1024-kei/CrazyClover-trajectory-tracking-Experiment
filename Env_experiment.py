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
from frames_setup import Frames_setup
from tools.Mathfunction import Mathfunction
from tools.Log import Log_data

# 定値制御
class Env_Experiment(Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self, T, Tsam, num):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

        self.Tend = T
        self.Tsam = Tsam

        self.set_cf_function()
        self.set_frame()
        self.set_key_input()
        self.set_log_function()
        self.init_state()

        self.mathfunc = Mathfunction()
        self.mathfunc.Init_LowPass2D(5, self.Tsam)
        
        self.log = Log_data(num)
        
        time.sleep(0.5)

    def set_frame(self):
        self.world_frame = Frames_setup().world_frame
        self.child_frame = Frames_setup().children_frame[0]
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

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
        self.Wb = np.zeros(3)

    def update_state(self, dt):
        try:
            f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

        # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            exit()

        self.P[0] = f.transform.translation.x; self.P[1] = f.transform.translation.y; self.P[2] = f.transform.translation.z
        self.Vrow = self.mathfunc.deriv(self.P, self.Ppre, dt)
        self.Vfiltered = self.mathfunc.LowPass2D(self.Vrow)
        self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
        self.Euler = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
        self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)

    def log_callback(self, log):
        self.M = log.values

    def save_log(self, t):
        self.log.csvWriter_state(t, self.P, self.Vfiltered, self.R, self.Euler, self.M)

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

    def time_check(self, t, Tint, Tend):
        if Tint < self.Tsam:
            time.sleep(self.Tsam - Tint)
        if t > Tend:
            return True
        return False

