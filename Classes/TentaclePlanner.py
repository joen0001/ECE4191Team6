import numpy as np


class TentaclePlanner:

    def __init__(self,us_sensor,dt=0.1,steps=5,alpha=1,beta=0.1):
        self.us_sensor = us_sensor
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        # self.tentacles = [(0.0,0.5),(0.0,-0.5),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
        self.tentacles = [(0.0,-0.51),(0.0,0.51),(0.1,0.0),(0.1,0.5),(0.1,-0.5),(0.1,0.35),(0.1,-0.3),(0.1,0.2),(0.1,-0.2),(0.1,0.4),(0.1,-0.4)]
        # v,w - (turning in intended direction)
        self.alpha = alpha
        self.beta = beta
        self.avoid_left = False
        self.left_counter = 0
        # Turning tentacles:
        self.left_turning = []
        self.right_turning = []
        self.straight = []
        # Categorize tentacles from large tentacle options:
        self.cat = self.categorize_tentacles()

    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):

        for j in range(self.steps):
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)

        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))

        return self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)

    # Categorize tentacles:
    def categorize_tentacles(self):
        for v, w in self.tentacles:
            if w < -0.5:  # Threshold for left turning
                self.left_turning.append((v, w))
            elif w > 0.5:  # Threshold for right turning
                self.right_turning.append((v, w))
            else:
                self.straight.append((v, w))

        return {
            "left_turning": self.left_turning,
            "right_turning": self.right_turning,
            "straight": self.straight
        }

    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        """
        Obstacle avoidance code within planning tentacles

        (v,w) linear velocity (v) and angular velocity (w)

        Use the sensor data to influence which tentacles are valid,
        refer to ultrasonic class for obstacle avoidance code.
        """

        modified_tentacles = self.us_sensor.detect_obstacle(self.cat)
        costs = []
        # Provide only free routes as per threshold
        if modified_tentacles is not None:
            for v,w in modified_tentacles:
                costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
            best_idx = np.argmin(costs)
            # Print tests for tentacles:
            # print(modified_tentacles)
            # print(costs)
            return modified_tentacles[best_idx]
        # Provide all routes
        else:
            for v,w in self.tentacles:
                costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
            best_idx = np.argmin(costs)
            return self.tentacles[best_idx]
