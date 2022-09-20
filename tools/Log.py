import os
import csv
class Log_data():

  def __init__(self, i):
    filename = f"data{i}_state.csv"
    os.remove(filename)
    self.f_state = open(filename, 'a', newline='')
    self.csvWriter_state = csv.writer(self.f_state)
    self.csvWriter_state.writerow(['T', 'Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz', 'Roll', 'Pitch', 'Yaw', 'R_rate', 'P_rate', 'Y_rate', "T", "M1", "M2", "M3", "Wb1", "Wb2", "Wb3"])

    filename = f"data{i}_nom.csv"
    os.remove(filename)
    self.f_nom = open(filename, 'a', newline='')
    self.csvWriter_nom = csv.writer(self.f_nom)
    self.csvWriter_nom.writerow(['T', 'Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz', 'Roll', 'Pitch', 'Yaw', 'R_rate', 'P_rate', 'Y_rate', "Wb1", "Wb2", "Wb3"])

  def write_state(self, t, P, V, R, Euler, Wb, Euler_rate, M):
    self.csvWriter_state.writerow([t, P[0], P[1], P[2], V[0], V[1], V[2], Euler[0], Euler[1], Euler[2], Euler_rate[0], Euler_rate[1], Euler_rate[2], M[0], M[1], M[2], M[3], Wb[0], Wb[1], Wb[2]])

  def write_nom(self, t, input_acc, input_Wb, P, V, Euler, Wb, Euler_rate):
    self.csvWriter_nom.writerow([t, input_acc, input_Wb, P[0], P[1], P[2], V[0], V[1], V[2], Euler[0], Euler[1], Euler[2], Euler_rate[0], Euler_rate[1], Euler_rate[2], Wb[0], Wb[1], Wb[2]])

  def close_file(self):
    self.f_state.close()
    self.f_nom.close()
