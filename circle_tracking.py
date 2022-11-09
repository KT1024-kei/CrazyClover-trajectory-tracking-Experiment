#!/user/bin/env python
# coding: UTF-8

import sys
import time
import numpy as np

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

from Env_experiment import Env_Experiment
from Exp_Controller.Controllers import Controllers

def Experiment(Texp, Tsam, num_drone):

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies
    Tsam_lock = Tsam

    Env = Env_Experiment(Texp, Tsam, 0)
    Drone_env = [0]*num_drone
    Drone_ctrl = [0]*num_drone

    circle_flag = True
    stop_flag = True

    land_time = 5

    zero = np.zeros(3)
    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "pid", "position")

        
    for i in range(num_drone):          # ホバリング
        P = np.array([0.0, 0.0, 0.4])
        Drone_env[i].hovering(Drone_ctrl[i], P)
        timeHelper.sleep(2)             # 5秒静止

    Ts = timeHelper.time()
    Te = -Tsam
    t = 0

    while True:
        for i in range(num_drone):
            # print(Drone_ctrl[i])
            Drone_env[i].take_log(t, Drone_ctrl[i])    #　状態と入力を記録

        # -------------- コントローラの変更 ----------------
        # 5秒から15秒でコントローラを軌道追従用に変更
        if 5 < t < 25:
            if circle_flag:
                Drone_ctrl[i].switch_controller("mellinger")
                Drone_env[i].track_circle(Drone_ctrl[i], False)
                circle_flag = False

        # 軌道追従を終了
        if Texp < t:
            if stop_flag:
                Drone_env[i].stop_track(Drone_ctrl[i])
                stop_flag = False

        if t > Texp+land_time:                      # 着陸
            for i in range(num_drone):
                Drone_env[i].land(Drone_ctrl[i])

        # -------------- ------------------ ----------------

        # コントローラに状態を渡して入力加速度と角速度を計算
        for i in range(num_drone):      
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].Vfiltered, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)

        for i in range(num_drone):      # ドローンに入力を送信
            input_acc = Drone_ctrl[i].input_acc
            input_W = np.array([Drone_ctrl[i].input_Wb[0]*1, Drone_ctrl[i].input_Wb[1]*1, Drone_ctrl[i].input_Wb[2]])

            cf[i].cmdFullState(zero, 
                            zero, 
                            np.array([0.0, 0.0, input_acc/100.0]), 
                            0.0, 
                            input_W)

        t = timeHelper.time() - Ts      # ループ周期を一定に維持
        Tsam = t - Te
        Te = t
        if Env.time_check(t, t - Te, Texp+land_time+5): break

        # 状態を更新
        for i in range(num_drone):
            Drone_env[i].set_dt(dt=Tsam_lock)
            Drone_env[i].update_state() # 状態を更新

    for i in range(num_drone):
        cf[i].cmdFullState(zero, zero, zero, 0.0, zero)

if __name__ == "__main__":
    Experiment(30, 0.005, 1)


# モータマップ4つめ修正
# 制御周期を姿勢周期にする
