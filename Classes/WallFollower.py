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
        left2_distance = self.us_sensor.fleft_distance()
        left_distance = self.us_sensor.front1_distance()
        front_distance = self.us_sensor.front2_distance()
        print('L'+str(left2_distance*100))
        print('R'+str(left_distance*100))
        print('F'+str(front_distance*100))
        threshold = 0.06
        #print(left_distance)
        # Drive forward only if the left distance is between 20 and 30 cm.
        #print('Distance Sensor'+str(min(left_distance,left2_distance)))
        if (left_distance<0.055):
            print('RIGHT')
            return 0.1, (0.055-left_distance)*5
        elif (left2_distance>0.065):
            print('LEFT')
            return 0.1, -(left_distance-0.065)*2.1
        else:
            return 0.1,0
        ''''
        
        if (left_distance>threshold and left2_distance<threshold):
            print('LEFT')
            if (left_distance>threshold):
                return 0.1, -(left_distance-threshold)*2.1
            else:
                return 0.1, -(left2_distance-threshold)*2.1
        elif (left_distance<threshold and left2_distance>threshold):
            print('RIGHT')
            # return 0.1, (threshold-min_dist)*2.5
            if (left_distance<threshold):
                return 0.1, (threshold-left_distance)*2.1
            else:
                return 0.1, (threshold-left_distance)*2.1
        else:
            print('STRAIGHT')
            return 0.1, 0
        #min_dist = min(left_distance,left2_distance)
        #if (min_dist<threshold):
        #    print('RIGHT')
        #    return 0.1, (threshold-min_dist)*2.1
        #else:
        #    print('LEFT')
        #    return 0.1, -(min_dist-threshold)*2.1
        '''
    def is_at_corner(self):
        # Assuming 20 as the distance in cm to detect corner/wall.
        if self.us_sensor.front2_distance() < 0.12:
            # 90 degree turn right
            return 0, 0
            # return 0,-np.pi/2
        else:
            return None, None
