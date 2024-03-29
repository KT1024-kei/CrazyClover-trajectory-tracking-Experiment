import sys
sys.path.append('../')

import numpy as np
from Exp_Controller.Trajectory import Trajectory
from tools.Mathfunction import Mathfunction

class Mellinger(Mathfunction):
  def __init__(self, dt):
    self.dt = dt
  def mellinger_init(self):
    print("Init Mellinger Controller")

    # init trajectory
    self.kp = np.array([4.3, 4.3, 3.0])
    self.kv = np.array([3.8, 3.8, 2.5])
    self.ka = np.array([0.0, 0.0, 0.0])
    self.kR = np.array([10.0, 10.0, 0.5])

    self.Euler_nom = np.array([0.0, 0.0, 0.0])
    self.Euler_rate_nom = np.array([0.0, 0.0, 0.0])
    self.traj_W = np.zeros(3)

    self.input_acc = 0.0
    self.input_Wb = np.zeros(3)

    self.trajectory = Trajectory()

  def set_reference(self, traj_plan, t, tmp_P=np.zeros(3)):
    self.trajectory.set_clock(t)
    self.trajectory.set_traj_plan(traj_plan)

    self.tmp_pos = np.zeros(3)
    # * set takeoff position for polynominal land trajectory 
    if traj_plan == "takeoff" or traj_plan == "takeoff_50cm":
      self.tmp_pos = tmp_P
    # * set landing position for polynominal land trajectory 
    elif traj_plan == "land" or traj_plan == "land_50cm":
      self.tmp_pos = tmp_P
    # * set stop position when stop tracking trajectory
    elif traj_plan == "stop":
      self.tmp_pos = tmp_P

  def set_state(self, P, V, R, Euler):

    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler

  def Position_controller(self):

    # set trajectory of each state
    traj_pos = self.trajectory.traj_pos + self.tmp_pos
    traj_vel = self.trajectory.traj_vel
    traj_acc = self.trajectory.traj_acc
    
    # calcurate nominal acceleration
    self.ref_acc = self.kp * (traj_pos - self.P) + self.kv*(traj_vel - self.V) + traj_acc

    # nominal acceleraion 
    self.input_acc = np.dot(self.ref_acc, np.matmul(self.R, np.array([0.0, 0.0, 1.0])))

  def Attitude_controller(self):
    
    # set trajectory of each state
    traj_acc = self.trajectory.traj_acc
    traj_jer = self.trajectory.traj_jer
    traj_yaw = self.trajectory.traj_yaw
    traj_yaw_rate = self.trajectory.traj_yaw_rate

    # calculate nominal Rotation matrics
    traj_R = np.zeros((3, 3))
    traj_Rxc = np.array([np.cos(traj_yaw), np.sin(traj_yaw), 0.0])
    traj_Ryc = np.array([-np.sin(traj_yaw), np.cos(traj_yaw), 0.0])
    traj_Rz = self.ref_acc/np.linalg.norm(self.ref_acc)
    traj_Rx = np.cross(traj_Ryc, traj_Rz)/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    traj_Ry = np.cross(traj_Rz, traj_Rx)/np.linalg.norm(np.cross(traj_Rz, traj_Rx))
    # traj_Ry = np.cross(traj_Rz, traj_Rxc)
    # traj_Rx = np.cross(traj_Ry, traj_Rz)

    traj_R[:, 0] = traj_Rx
    traj_R[:, 1] = traj_Ry
    traj_R[:, 2] = traj_Rz


    # calculate nominal Angular velocity
    traj_wy =  np.dot(traj_Rx, traj_jer) / np.dot(traj_Rz, self.ref_acc)
    traj_wx = -np.dot(traj_Ry, traj_jer) / np.dot(traj_Rz, self.ref_acc)
    traj_wz = (traj_yaw_rate * np.dot(traj_Rxc, traj_Rx) + traj_wy * np.dot(traj_Ryc, traj_Rz))/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    self.traj_W[0] = traj_wx
    self.traj_W[1] = traj_wy
    self.traj_W[2] = traj_wz
    
    # calculate input Body angular velocity
    self.input_Wb = self.traj_W + self.kR*self.Wedge(-(np.matmul(traj_R.T, self.R) - np.matmul(self.R.T, traj_R))/2.0)
    
    # calculate nominal Euler angle and Euler angle rate
    self.Euler_nom[1] =  np.arctan( ( self.ref_acc[0]*np.cos(traj_yaw) + self.ref_acc[1]*np.sin(traj_yaw) ) / (self.ref_acc[2]))                                                        
    self.Euler_nom[0] = np.arctan( ( self.ref_acc[0]*np.sin(traj_yaw) - self.ref_acc[1]*np.cos(traj_yaw) ) / np.sqrt( (self.ref_acc[2])**2 + ( self.ref_acc[0]*np.cos(traj_yaw) + self.ref_acc[2]*np.sin(traj_yaw) )**2));  
    self.Euler_nom[2] = traj_yaw

    self.input_Euler_rate = self.BAV2EAR(self.Euler_nom, self.input_Wb)
    self.Euler_rate_nom = self.BAV2EAR(self.Euler_nom, self.traj_W)

  def mellinger_ctrl(self, t):
    self.trajectory.set_clock(t)
    self.trajectory.set_traj()
    self.Position_controller()
    self.Attitude_controller()

  def stop_tracking(self):
    self.set_reference("stop")

  def log_nom(self, log, t):
    log.write_nom(t=t, input_acc=self.input_acc, input_Wb=self.input_Wb, P=self.trajectory.traj_pos+self.tmp_pos, V=self.trajectory.traj_vel, Euler=self.Euler_nom, Wb=self.traj_W, Euler_rate=self.Euler_rate_nom)

    











    




    