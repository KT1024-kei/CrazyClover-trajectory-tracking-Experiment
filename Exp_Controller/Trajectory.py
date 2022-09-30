import numpy as np

class Trajectory():

  def __init__(self, trajctory_plan):
    print("Init trajectory planning")
    self.traj_pos = np.zeros(3)
    self.traj_vel = np.zeros(3)
    self.traj_acc = np.zeros(3)
    self.traj_jer = np.zeros(3)
    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

    self.trajectory_plan = trajctory_plan
    print(trajctory_plan)
    
  
  def set_clock(self, t):
    self.t = t

  def traj_circle(self):
    T = 10.0
    A = 1.0
    w = 2*np.pi/T
    self.traj_pos[0] =  A*np.cos(w*self.t);      self.traj_pos[1] =  A*np.sin(w*self.t);      self.traj_pos[2] = 0.5
    self.traj_vel[0] = -A*w*np.sin(w*self.t);    self.traj_vel[1] =  A*w*np.cos(w*self.t);    self.traj_vel[2] = 0.0
    self.traj_acc[0] = -A*w**2*np.cos(w*self.t); self.traj_acc[1] = -A*w**2*np.sin(w*self.t); self.traj_acc[2] = 9.8
    self.traj_jer[0] =  A*w**3*np.sin(w*self.t); self.traj_jer[1] = -A*w**3*np.cos(w*self.t); self.traj_jer[2] = 0.0
    # print(self.traj_acc)

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0
  
  def stop_track(self):

    self.traj_pos[0] = 0.0; self.traj_pos[1] = 0.0;  self.traj_pos[2] = 1.0
    self.traj_vel[0] = 0.0; self.traj_vel[1] = 0.0;  self.traj_vel[2] = 0.0
    self.traj_acc[0] = 0.0; self.traj_acc[1] = 0.0;  self.traj_acc[2] = 9.8
    self.traj_jer[0] = 0.0; self.traj_jer[1] = 0.0;  self.traj_jer[2] = 0.0
    # print(self.traj_acc)

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0
  
  def set_traj(self):
    
    if self.trajectory_plan == "circle":
      self.traj_circle()
    
    elif self.trajectory_plan == "stop":
      self.stop_track()
    


  
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




    

