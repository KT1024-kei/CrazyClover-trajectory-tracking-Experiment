import sys
sys.path.append('../')

import numpy as np


from Exp_Controller.Pid_Controller import Pid_Controller
from Exp_Controller.Mellinger_controller import Mellinger

class Controllers():

  def __init__(self, dt, controller_mode, mode="position", traj="circle"):
    print("initialize Controller")
    

    self.dt = dt
    self.mode = mode
    self.traj = traj
    self.controller_mode = controller_mode
    self.pid_controller = Pid_Controller(self.dt, self.mode)
    self.mellinger_controller = Mellinger(self.dt)


    self.input_thrust_gf = 0.0
    self.input_M_gf = np.array([0.0, 0.0, 0.0])
    
    self.input_thrust_pwm = 0.0
    self.input_M_pwm = np.array([0.0, 0.0, 0.0])

    self.gravity_calcel = 9.8
    self.rad2deg = 180/np.pi
    
  def select_controller(self):

    if self.controller_mode == "pid":
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom

    elif self.controller_mode == "mellinger":
      self.controller = self.mellinger_controller
      self.cal_output = self.mellinger_controller.mellinger_ctrl
      self.set_state = self.mellinger_controller.set_state
      self.init_controller = self.mellinger_controller.mellinger_init
      self.set_reference = self.mellinger_controller.set_reference
      self.stop_tracking = self.mellinger_controller.stop_tracking
      self.log = self.mellinger_controller.log_nom
    self.init_controller()

  def switch_controller(self, controller_type):

    if controller_type == "pid":
      self.controller_mode = "pid"
      self.pid_controller = Pid_Controller(self.dt, self.mode)
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom

    elif controller_type == "mellinger":
      self.controller_mode = "mellinger"
      self.mellinger_controller = Mellinger(self.dt)
      self.controller = self.mellinger_controller
      self.cal_output = self.mellinger_controller.mellinger_ctrl
      self.set_state = self.mellinger_controller.set_state
      self.init_controller = self.mellinger_controller.mellinger_init
      self.set_reference = self.mellinger_controller.set_reference
      self.stop_tracking = self.mellinger_controller.stop_tracking
      self.log = self.mellinger_controller.log_nom

    self.init_controller()
    print("switch controller {}". format(controller_type))
  
  def get_output(self, t):
    self.cal_output(t)
    self.input_acc = self.controller.input_acc
    self.input_Wb = self.controller.input_Wb

  def controller_trajectory_tracking(self, refs):
    print("Trajectory tracking controller")

  