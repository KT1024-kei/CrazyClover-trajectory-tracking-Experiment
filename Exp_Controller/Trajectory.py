import sys
import numpy as np
import pandas as pd
sys.path.append('../')
from tools.Mathfunction import Mathfunction as MF
from tools.Decorator import run_once

class Trajectory():

  def __init__(self):
    print("Init trajectory planning")
    self.traj_pos = np.zeros(3)
    self.traj_vel = np.zeros(3)
    self.traj_acc = np.zeros(3)
    self.traj_jer = np.zeros(3)
    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0


  def set_traj_plan(self, trajectory_plan):
    self.trajectory_plan = trajectory_plan
    print(trajectory_plan)
    
  
  def set_clock(self, t):
    self.t = t

  @run_once
  def poly_traj_init(self, trajectory_plan):
    # polynominal trajectory planning
    if trajectory_plan == "straight":
      self.traj = pd.read_csv('~/crazyswarm/ros_ws/src/crazyswarm/CrazyClover-Experiment/Exp_Controller/Trajectory segment parametors/traj_straight_4s.csv')
    
    self.len_seg = self.traj["N_segment"][0]
    self.segs_T = self.traj["Tseg"][0:self.len_seg]
    self.Xcoeffs = self.traj["Xcoeff"]
    self.Ycoeffs = self.traj["Ycoeff"]
    self.Zcoeffs = self.traj["Zcoeff"]
    self.Order = self.traj["Order"][0]

    self.seg_now = 0
    self.T = 0
    self.Toffset = self.t

  def poly_traj(self):
    t = self.t
    # print(sum(self.segs_T) + self.Toffset, t)
    if sum(self.segs_T) + self.Toffset < t:
      return 0
    if sum(self.segs_T[:self.seg_now+1])+self.Toffset < t:
      self.T += self.segs_T[self.seg_now]
      self.seg_now += 1
    t -= (self.T + self.Toffset)
    # print(t)
    
    Xcoeff = self.Xcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Ycoeff = self.Ycoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Zcoeff = self.Zcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    
    poly_T0 = MF().time_polyder(t, 0, self.Order)
    poly_T1 = MF().time_polyder(t, 1, self.Order)
    poly_T2 = MF().time_polyder(t, 2, self.Order)
    poly_T3 = MF().time_polyder(t, 3, self.Order)

    self.traj_pos = np.array([np.dot(Xcoeff, poly_T0), np.dot(Ycoeff, poly_T0), np.dot(Zcoeff, poly_T0)])
    self.traj_vel = np.array([np.dot(Xcoeff, poly_T1), np.dot(Ycoeff, poly_T1), np.dot(Zcoeff, poly_T1)])
    self.traj_acc = np.array([np.dot(Xcoeff, poly_T2), np.dot(Ycoeff, poly_T2), np.dot(Zcoeff, poly_T2)]) + np.array([0.0 ,0.0, 9.8])
    self.traj_jer = np.array([np.dot(Xcoeff, poly_T3), np.dot(Ycoeff, poly_T3), np.dot(Zcoeff, poly_T3)])

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  def traj_circle(self):
    T = 7.0
    A = 1.0
    w = 2*np.pi/T
    self.traj_pos[0] =  A*np.cos(w*self.t);      self.traj_pos[1] =  A*np.sin(w*self.t);      self.traj_pos[2] = 0.4
    self.traj_vel[0] = -A*w*np.sin(w*self.t);    self.traj_vel[1] =  A*w*np.cos(w*self.t);    self.traj_vel[2] = 0.0
    self.traj_acc[0] = -A*w**2*np.cos(w*self.t); self.traj_acc[1] = -A*w**2*np.sin(w*self.t); self.traj_acc[2] = 9.8
    self.traj_jer[0] =  A*w**3*np.sin(w*self.t); self.traj_jer[1] = -A*w**3*np.cos(w*self.t); self.traj_jer[2] = 0.0
    # print(self.traj_acc)

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0
  
  def stop_track(self):

    # self.traj_pos[0] = 0.0; self.traj_pos[1] = 0.0;  self.traj_pos[2] = 1.0
    self.traj_vel[0] = 0.0; self.traj_vel[1] = 0.0;  self.traj_vel[2] = 0.0
    self.traj_acc[0] = 0.0; self.traj_acc[1] = 0.0;  self.traj_acc[2] = 8.5
    self.traj_jer[0] = 0.0; self.traj_jer[1] = 0.0;  self.traj_jer[2] = 0.0
    # print(self.traj_acc)

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0
  
  def set_traj(self):
    
    if self.trajectory_plan == "circle":
      self.traj_circle()
    
    elif self.trajectory_plan == "stop":
      self.stop_track()

    elif self.trajectory_plan == "straight":
      
      self.poly_traj_init("straight")
      self.poly_traj()
    


  
  # def Traj_pos(self, t):
  #   self.pos_vel[0] =  self.A*np.cos(self.w*t)      
  #   self.traj_pos[1] =  self.A*np.sin(self.w*t) 
  #   self.traj_pos[2] = 0.0
  
  # def Traj_vel(self, t):
  #   self.vel_vec[0] = -self.A*self.w*np.sin(self.w*t)    
  #   self.vel_vec[1] =  self.A*self.w*np.cos(self.w*t)  
  #   self.vel_vec[2] = 0.0
  
  # def Traj_acc(self, t):
  #   self.acc_vec[0] = -self.A*self.w**2*np.cos(self.w*t) 
  #   self.acc_vec[1] = -self.A*self.w**2*np.sin(self.w*t) 
  #   self.acc_vec[2] = 9.8

  # def Traj_jer(self, t):
  #   self.jer_vec[0] =  self.A*self.w**3*np.sin(self.w*t) 
  #   self.jer_vec[1] = -self.A*self.w**3*np.cos(self.w*t)
  #   self.jer_vec[2] = 0.0




    

