from msilib import PID_APPNAME
import numpy as np
import sys
sys.path.append('../')

from tools.pid import PID
from Drone.Inner_controller import Controller_attituede_rate

class Controller():

  def __init__(self, dt, mode):
    print("initialize Controller")

    self.dt = dt
    self.mode = mode
    self.inner_Controller = Controller_attituede_rate(dt)


    self.input_thrust_gf = 0.0
    self.input_M_gf = np.array([0.0, 0.0, 0.0])
    
    self.input_thrust_pwm = 0.0
    self.input_M_pwm = np.array([0.0, 0.0, 0.0])

    self.gravity_calcel = 9.8
    self.rad2deg = 180/np.pi
    
  def set_reference(self, P, V, R, Euler, Wb, Euler_rate, mode):
      
    # print("set references")
    self.Pref = P
    self.Vref = V
    self.Rref = R
    self.Eulerref = Euler
    self.Wbref = Wb
    self.Euler_rateref = Euler_rate

    if mode == "pid":
      self.pid_init()

  def set_state(self, P, V, R, Euler, Wb, Euler_rate):
      
    # print("set references")
    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler
    self.Wb = Wb
    self.Euler_rate = Euler_rate

  def pid_init(self):
    self.R_pid = PID(15.0, 5.0, 0.0, self.dt)
    self.P_pid = PID(15.0, 5.0, 0.0, self.dt)
    self.Y_pid = PID(10.0, 3.0, 0.0, self.dt)

    self.Vx_pid = PID(1.0, 0.0, 0.0, self.dt)
    self.Vy_pid = PID(1.0, 0.0, 0.0, self.dt)
    self.Vz_pid = PID(10.0, 5.0, 0.0, self.dt)

    self.Px_pid = PID(1.0, 0.0, 0.0, self.dt)
    self.Py_pid = PID(1.0, 0.0, 0.0, self.dt)
    self.Pz_pid = PID(1.0, 0.0, 0.0, self.dt)

  def controller_attitude_pid(self):
    # print("PID Attitude Controller")

    if self.mode == "attitude":
      self.R_pid.desired = self.Eulerref[0]
      self.P_pid.desired = self.Eulerref[1]
      self.Y_pid.desired = self.Eulerref[2]


    self.R_pid.runpid(self.Euler[0])
    self.P_pid.runpid(self.Euler[1])

    # update PID for yaw axis
    Yaw_Error = self.Y_pid.desired - self.Euler[2]
    if Yaw_Error > np.pi:
      Yaw_Error -= 2*np.pi
    elif Yaw_Error < -np.pi:
      Yaw_Error += 2*np.pi
    self.Y_pid.Err = Yaw_Error
    self.Y_pid.runpid(self.Euler[2])

    self.inner_Controller.R_rate_pid.desired = self.R_pid.output*self.rad2deg
    self.inner_Controller.P_rate_pid.desired = self.P_pid.output*self.rad2deg
    self.inner_Controller.Y_rate_pid.desired = self.Y_pid.output*self.rad2deg
    # self.inner_Controller.R_rate_pid.desired = 10.0
    # self.inner_Controller.P_rate_pid.desired = 0.0
    # self.inner_Controller.Y_rate_pid.desired = 0.0

    self.inner_Controller.inner_controller(self.input_thrust_gf + self.gravity_calcel, self.Euler_rate*self.rad2deg)

    self.input_thrust_pwm = self.inner_Controller.FM_pwm[0]
    self.input_M_pwm = self.inner_Controller.FM_pwm[1:]

  def controller_velocity_pid(self):
    # print("PID Velocity controller")
    if self.mode == "velocity":
      self.Vx_pid.desired = self.Vref[0]
      self.Vy_pid.desired = self.Vref[1]
      self.Vz_pid.desired = self.Vref[2]

    cosY = np.cos(self.Euler[2]); sinY = np.sin(self.Euler[2])

    self.Vx_pid.runpid(self.V[0])
    self.Vy_pid.runpid(self.V[1])
    self.Vz_pid.runpid(self.V[2])

    self.R_pid.desired = -(self.Vy_pid.output * cosY) + (self.Vx_pid.output  * sinY)
    self.P_pid.desired = -(-(self.Vx_pid.output  * cosY) - (self.Vy_pid.output * sinY))

    self.input_thrust_gf = self.Vz_pid.output
    
    # print(self.Vz_pid.output)
    # print(self.P_pid.desired)
    self.controller_attitude_pid()
    

  def controller_position_pid(self):
    # print("PID Position Controller")
    if self.mode == "position":
      self.Px_pid.desired = self.Pref[0]
      self.Py_pid.desired = self.Pref[1]
      self.Pz_pid.desired = self.Pref[2]
    cosY = np.cos(self.Euler[2]); sinY = np.sin(self.Euler[2])

    # calcurate desired body velocity
    self.Px_pid.runpid(self.P[0])
    self.Py_pid.runpid(self.P[1])
    self.Pz_pid.runpid(self.P[2])

    self.Vx_pid.desired = self.Px_pid.output * cosY - self.Py_pid.output * sinY
    self.Vy_pid.desired = self.Py_pid.output * cosY + self.Px_pid.output * sinY
    self.Vz_pid.desired = self.Pz_pid.output

    self.controller_velocity_pid()
    


  def controller_trajectory_tracking(self, refs):
    print("Trajectory tracking controller")

  