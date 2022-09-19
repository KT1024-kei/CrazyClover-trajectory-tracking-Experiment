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

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
from crazyswarm.msg  import GenericLogData

# 関連モジュールのインポート
from frames_setup import Frames_setup

# 定値制御
class attitude_control( Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self, T):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

        self.Tend = T
        self.Tsam = 0.01

        self.cmd_sub = rospy.Subscriber("/cf20/log1", GenericLogData, self.log_callback)
        datalen = int(self.Tend/self.Tsam)
        self.cmd_thrust = [0]*datalen
        self.cmd_roll = [0]*datalen
        self.cmd_pitch = [0]*datalen
        self.cmd_yaw = [0]*datalen

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cf = self.allcfs.crazyflies[0]
        
        self.world_frame = Frames_setup().world_frame
        self.child_frame = Frames_setup().children_frame[0]
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    # 以下ではターミナルからの入力をEnterなしで実行するための設定変更
        self.fd = sys.stdin.fileno()

        self.old = termios.tcgetattr(self.fd)
        self.new = termios.tcgetattr(self.fd)

        self.new[3] &= ~termios.ICANON
        self.new[3] &= ~termios.ECHO

        
        time.sleep(0.5)

        self.zero3d = np.array([0, 0, 0])
    def log_callback(self, log):
        self.Log = log.values

    def get_state(self):
        try:
            f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

        # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            exit()
        return f

    def get_log(self, cnt):
        print(self.Log)
        self.cmd_thrust[cnt] = self.Log[0]
        self.cmd_roll[cnt] = self.Log[1]
        self.cmd_pitch[cnt] = self.Log[2]
        self.cmd_yaw[cnt] = self.Log[3]
        
    def save_log(self):
        data = {"input thrust": self.cmd_thrust}
        data["input roll"] = self.cmd_roll
        data["input pitch"] = self.cmd_pitch
        data["input yaw"] = self.cmd_yaw
        df = pd.DataFrame(data)
        df.to_csv("thrust_data_log{}".format(datetime.date.today()))

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

    def land(self, hight, duration):
        Ts = time.time()
        while True:
            Tint = time.time()
            t = time.time() - Ts
            f = self.get_state()
            X = f.transform.translation.x; Y = f.transform.translation.y; Z = f.transform.translation.z
            self.cf.cmdVelocityWorld(np.array([-X, -Y, 1*(hight-Z)]), yawRate=0)
            if self.time_check(t, time.time() - Tint, duration):
                break

    def main(self):

        # 実験開始
        print("Experiment Start!!")
        time.sleep(1)
        cnt = 0
        Ts = time.time()
        input_thrust = 0.00
        while True:
            Tint = time.time()
            t = time.time() - Ts
            f = self.get_state()
            X = f.transform.translation.x; Y = f.transform.translation.y; Z = f.transform.translation.z
            # input_thrust, flag = self.get_input(input_thrust)
            # print(input_thrust)
            # self.cf.cmdFullState(pos=self.zero3d, vel=self.zero3d, acc=np.array([0,0,input_thrust]), yaw=0.0, omega=self.zero3d)

            self.cf.cmdVelocityWorld(np.array([-X, -Y, 1*(0.5-Z)]), yawRate=0)
            # self.cf.cmdPosition(np.array([0.0,0.0,0.2]), 0.0)
            self.get_log(cnt)

            if self.time_check(t, time.time() - Tint, self.Tend):
                break
            cnt += 1
            
        # save log data
        self.save_log()
        self.land(0.02, 4)
        self.cf.notifySetpointsStop(100)
        rospy.sleep(1)
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old)

if __name__ == '__main__':
    T = float(input("exp time"))
    attitude_control(T).main()
    
    exit()