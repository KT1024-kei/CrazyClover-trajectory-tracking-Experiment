class PID():

  def __init__(self, kp, ki, kd, dt):

    self.dt = dt

    self.kp = kp
    self.ki = ki
    self.kd = kd

    self.state = 0
    self.prev = .0
    self.desired = .0

    self.Err = .0
    self.Err_pre = .0
    self.Err_sum = 0.0

    self.output = .0

  def runpid2(self, state):

    self.output = 0.0
    self.Err = self.desired - state
    self.Err_sum += self.Err*self.dt
    self.Err_div  = (self.Err-self.Err_pre)/self.dt 
    self.Err_pre = self.Err

    self.outP = self.kp * self.Err
    self.output += self.outP

    self.outD = self.kd * self.Err_div
    self.output += self.outD

    self.outI = self.ki * self.Err_sum
    self.output += self.outI
  
  def runpid(self):
    
    self.output = 0.0
    self.Err = self.desired - self.state
    self.Err_sum += self.Err*self.dt
    self.Err_div  = (self.Err-self.Err_pre)/self.dt 
    self.Err_pre = self.Err

    self.outP = self.kp * self.Err
    self.output += self.outP

    self.outD = self.kd * self.Err_div
    self.output += self.outD

    self.outI = self.ki * self.Err_sum
    self.output += self.outI
  
