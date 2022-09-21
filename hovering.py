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

    Env = Env_Experiment(Texp, Tsam, 0)
    Drone_env = [0]*num_drone
    Drone_ctrl = [0]*num_drone


    zero = np.zeros(3)
    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "pid")

        
    for i in range(num_drone):          # ホバリング
        P = np.array([0.0, 0.0, 0.2])
        Drone_env[i].hovering(Drone_ctrl[i], P)
        timeHelper.sleep(5)             # 5秒静止

    Ts = timeHelper.time()
    Te = 0
    t = 0

    while True:

        t = timeHelper.time() - Ts      # ループ周期を一定に維持
        Tsam = Te - t
        # print(1)
        if Env.time_check(t, t - Te, Texp + 5): break
        else: Te = timeHelper.time() - Ts
        for i in range(num_drone):
            # print(Drone_ctrl[i])
            Drone_env[i].take_log(t, Drone_ctrl[i])    #　状態と入力を記録

        for i in range(num_drone):
            Drone_env[i].set_dt(dt=Tsam)
            Drone_env[i].update_state() # 状態を更新

        for i in range(num_drone):      # コントローラに状態を渡して入力加速度と角速度を計算
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].Vfiltered, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)
        
        for i in range(num_drone):      # ドローンに入力を送信
            cf[i].cmdFullState(zero, 
                            zero, 
                            np.array([0.0, 0.0, Drone_ctrl[i].input_acc/100]), 
                            0.0, 
                            Drone_ctrl[i].input_Wb)
        
        if t > Texp:                      # 着陸
            for i in range(num_drone):
                Drone_env[i].land(Drone_ctrl[i])

    for i in range(num_drone):
        cf[i].cmdFullState(zero, zero, zero, 0.0, zero)

if __name__ == "__main__":
    Experiment(1, 0.01, 1)




