from termios import B0
import numpy as np

class Mathfunction():

  def Vee(self, Vector):
    return np.array([[0, -Vector[2], Vector[1]], [Vector[2], 0, -Vector[0]], [-Vector[1], Vector[0], 0]])

  def Wedge(self, Mat):
    return np.array([Mat[2, 1], -Mat[2, 0], Mat[1, 0]])

  def saturation(self, param, UP_limit, LOW_limit):

    return max(min(param, UP_limit), LOW_limit)
  
  # Body angular velocity to Euler angle rate
  def BAV2EAR(self, Euler, Wb): 
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Euler_angle_rate = np.matmul(np.linalg.inv(np.array([[1.0, 0.0, -sinP],
                                        [0.0, cosR,  cosP * sinR],
                                        [0.0, -sinR, cosP * cosR]])) , Wb)
    # Euler_angle_rate = np.matmul(np.array([[cosR, sinR, 0.0],
    #                                       [-cosP*sinR, cosP*cosR, 0.0],
    #                                       [sinP*cosR, sinP*sinR, cosP]])/cosP, Wb)
  
    return Euler_angle_rate

  # Euler angular velocity to Body angular velocity
  def EAR2BAV(self, Euler, Euler_rate):
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Wb = np.matmul((np.array([[1.0, 0.0, -sinP],
                              [0.0, cosR,  cosP * sinR],
                              [0.0, -sinR, cosP * cosR]])) , Euler_rate)
    
    return Wb
  
  ## https://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/
  ## https://en.wikipedia.org/wiki/Digital_biquad_filter See transposed direct form
  def Init_LowPass2D(self, fc, dt): # cut off frecency, sampling rate
    
    self.dt = dt
    fr = 1.0/(fc * self.dt)     
    omega = np.tan(np.pi/fr)

    c = 1.0 + 2.0/np.sqrt(2)*omega + omega**2
    self.b0 = omega**2/c
    self.b1 = 2.0*self.b0
    self.b2 = self.b0

    self.a1 = 2*(omega**2 - 1)/c
    self.a2 = (1.0 - 2.0/np.sqrt(2)*omega + omega**2)/c

    self.r0 = np.zeros(3)
    self.r1 = np.zeros(3)
    self.r2 = np.zeros(3)

  def LowPass2D(self, V0):
    self.r0 = V0 - self.a1*self.r1 - self.a2*self.r2
    self.Vout = self.b0*self.r0 + self.b1*self.r1 + self.b2*self.r2

    self.r2 = self.r1
    self.r1 = self.r0
    
  def deriv(self, now, pre,  dt):
    return (now-pre)/dt

class Integration():
  
  def __init__(self, dt, param):
    self.dt = dt
    self.integral = param

  def integration(self, param):
    self.integral += param*self.dt    
    return self.integral