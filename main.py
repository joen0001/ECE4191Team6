from CONFIG import *
from Ultrasonic import Ultrasonic
from Multiprocessing import Process, Value
from gpiozero import Motor, RotaryEncoder

class DiffDriveRobot:  
    def __init__(self,inertia=5, dt=0.1, drag=0.2, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION):
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational velocity right wheel
        
        self.I = inertia
        self.d = drag
        self.dt = dt
        
        self.r = wheel_radius
        self.l = wheel_sep
    
    # Motor encoder measurement which measures how fast wheel is turning
  # NEED TO COMPLETE
    def RotEnc(A, B):
      encoder = RotaryEncoder(A, B, max_steps=10000)
      #encoder.steps
      return w
    
    # Veclocity motion model
    def base_velocity(self,wl,wr):
        
        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wl*self.r - wr*self.r)/self.l
        
        return v, w
    
    # Kinematic motion model
    def pose_update(self,duty_cycle_l,duty_cycle_r):
        
        self.wl = self.motor_simulator(self.wl,duty_cycle_l)
        self.wr = self.motor_simulator(self.wr,duty_cycle_r)
        
        v, w = self.base_velocity(self.wl,self.wr)
        
        self.x = self.x + self.dt*v*np.cos(self.th)
        self.y = self.y + self.dt*v*np.sin(self.th)
        self.th = self.th + w*self.dt
        
        return self.x, self.y, self.th
        
