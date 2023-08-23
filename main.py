from CONFIG import *
from Classes import *
from Ultrasonic import Ultrasonic
from ShaftEncoder import ShaftEncoder
import numpy as np
from Multiprocessing import Process, Value, Array
from gpiozero import Motor, RotaryEncoder
from matplotlib import pyplot as plt

def main():

  ultra_arr = Array('i',[0,0,0]) # 'i' for signed integer, 'd' for double prec float

  TH_General = Process(target = )
  TH_Ultrasonic = Process(target = Ultrasonic, args=(ultra_arr))
  TH_ShaftEncoder = Process(target = ShaftEncoder)

  TH_General.start()
  TH_Ultrasonic.start()
  TH_ShaftEncoder.start()
  robot = DiffDriveRobot(inertia=10, dt=0.1, drag=2, wheel_radius=0.05, wheel_sep=0.15)
  controller = RobotController(Kp=1.0,Ki=0.15,wheel_radius=0.05,wheel_sep=0.15)
  planner = TentaclePlanner(dt=0.1,steps=5,alpha=1,beta=1e-5)


  poses = []
  velocities = []
  duty_cycle_commands = []

  goal_x = 2*np.random.rand()-1 #NEEDS CHANGING
  goal_y = 2*np.random.rand()-1 #NEEDS CHANGING
  goal_th = 2*np.pi*np.random.rand()-np.pi #NEEDS CHANGING

  for i in range(200):

      # Plan using tentacles
      v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th)
        
      duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr) 
        
      # Simulate robot motion - send duty cycle command to robot
      x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
        
      # Log data
      poses.append([x,y,th])
      duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
      velocities.append([robot.wl,robot.wr])

