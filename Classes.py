import gpiozero
from CONFIG import *
from Ultrasonic import Ultrasonic
from ShaftEncoder import ShaftEncoder
import numpy as np
from Multiprocessing import Process, Value, Array
from gpiozero import Motor, RotaryEncoder
from matplotlib import pyplot as plt
from main import *

class Motor:
    def __init__(self, forward_pin, backward_pin, enable_pin):
        """
        Initialise the Motor object
        :param forward_pin: The forward pin number
        :param backward_pin: The backward pin number
        :param enable_pin: The enable pin number (ensure PWM)
        """
        self.forward_pin = gpiozero.OutputDevice(pin=forward_pin)
        self.backward_pin = gpiozero.OutputDevice(pin=backward_pin)
        self.enable_pwm = gpiozero.PWMOutputDevice(pin=enable_pin, active_high=True, initial_value=0, frequency=100)

    def stop(self):
        """
        Stops the motors
        """
        self.forward_pin.value = False
        self.backward_pin.value = False
        self.enable_pwm.value = 0

    def forward(self, speed):
        """
        Makes the motors move forward
        :param speed: A float between 0 and 1 representing the speed of the motor
        """
        self.forward_pin.value = True
        self.enable_pwm.value = speed

    def backward(self, speed):
        """
        Makes the motors move backward
        :param speed: A float between 0 and 1 representing the speed of the motor
        """
        self.forward_pin.value = True
        self.enable_pwm.value = speed

    def reverse(self):
        """
        Reverse the direction of the motor
        """
        self.forward_pin.value = not self.backward_pin.value
        self.backward_pin.value = not self.forward_pin.value

    def is_active(self):
        """
        Checks if the motor is active
        :return: A boolean representing if the motor is active
        """
        return self.enable_pwm.value > 0

class DiffDriveRobot:  
    def __init__(self, dt=0.1, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION):
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational vdelocity right wheel
        
        self.dt = dt
        
        self.r = wheel_radius
        self.l = wheel_sep
    

    
    # Veclocity motion model
    def base_velocity(self,wl,wr):
        
        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wl*self.r - wr*self.r)/self.l
        
        return v, w
    
    # Kinematic motion model
    def pose_update(self,enc_arr):

        v, w = self.base_velocity(enc_arr[0],enc_arr[1])
        self.x = self.x + self.dt*v*np.cos(self.th)
        self.y = self.y + self.dt*v*np.sin(self.th)
        self.th = self.th + w*self.dt
        
        return self.x, self.y, self.th
    
    
        
class TentaclePlanner:
    
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.0,0.5),(0.0,-0.5),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
        
        self.alpha = alpha
        self.beta = beta
        self.avoid_left = False
        self.left_counter = 0
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
        
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        return self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
        
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        front_sensor, left_sensor, right_sensor = Ultrasonic()
        costs =[]
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
        
        best_idx = np.argmin(costs)
        
        return self.tentacles[best_idx]


class RobotController:
    
    def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.02, wheel_sep=0.1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum):
        
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        
        e_sum = e_sum + (w_desired-w_measured)
        
        return duty_cycle, e_sum
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        
        wl_desired = (v_desired + self.l*w_desired/2)/self.r
        wr_desired = (v_desired - self.l*w_desired/2)/self.r
        
        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r

