import time
import numpy as np

class WallFollower:
    
    def __init__(self, us_sensor):
        self.us_sensor = us_sensor
        self.forward_speed = 0.5
        self.turn_speed = 0.1

    def drive_forward(self):
        return self.forward_speed,0 
        
    def maintain_left_distance(self):
        left_distance = self.us_sensor.fleft_distance()
        print(left_distance)
        # Drive forward only if the left distance is between 20 and 30 cm.
        if 0.08 < left_distance < 0.11:
            print('f')
            return 0.1,0
        # Return turn left (-)
        elif (left_distance > 0.11):
            print('l')
            return 0.1, -np.pi/24
        # Return turn right (+)
        elif (left_distance < 0.08):
            print('r')
            return 0.1, np.pi/24

    def is_at_corner(self):
        # Assuming 20 as the distance in cm to detect corner/wall.
        if self.us_sensor.front1_distance() < 0.15 or self.us_sensor.front2_distance() < 0.15:
            # 90 degree turn right
            return 0, 0
            # return 0,-np.pi/2
        else:
            return None, None
