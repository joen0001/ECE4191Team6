import numpy as np


class DiffDriveRobot:
    def __init__(self, dt=0.1, wheel_radius=28, wheel_sep=220):

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

        # Linear velocity
        v = (wl*self.r + wr*self.r)/2.0
        # Angular velocity
        w = (wl*self.r - wr*self.r)/self.l

        return v, w

    # Kinematic motion model
    def pose_update(self,enc_arr):

        v, w = self.base_velocity(enc_arr[0],enc_arr[1])
        self.x = self.x + self.dt*v*np.cos(self.th)
        self.y = self.y + self.dt*v*np.sin(self.th)
        self.th = self.th + w*self.dt

        return self.x, self.y, self.th
