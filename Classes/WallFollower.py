import time
import numpy as np

class WallFollower:
    
    def __init__(self, us_sensor):
        self.us_sensor = us_sensor
        self.forward_speed = 0.5
        self.turn_speed = 0.1

    def drive_forward(self):
        return self.forward_speed,0 

    # def maintain_left_distance(self):
    #     """
    #     DIFFERENCE METHOD:
    #     This method aims to maintain the robot's left-side distance from a wall.
    #     It uses the difference between two left-side ultrasonic sensors to determine 
    #     the robot's angle relative to the wall.
    #     If the robot is angled away from the wall, the front sensor will read a larger 
    #     distance than the back sensor. Conversely, if the robot is angled towards the wall,
    #     the back sensor will read a larger distance.
    #     """

    #     # Sensor closer to the back
    #     left2_distance = self.us_sensor.fleft_distance()
        
    #     # Sensor closer to the front
    #     left_distance = self.us_sensor.front1_distance()
    #     front_distance = self.us_sensor.front2_distance()

    #     # Debugging prints to monitor distances
    #     print('L' + str(left2_distance * 100))
    #     print('R' + str(left_distance * 100))
    #     print('F' + str(front_distance * 100))

    #     difference = left_distance - left2_distance

    #     # Negative difference means robot should turn left
    #     if (difference < -0.01 or left_distance < 0.055):
    #         print('RIGHT')  # This may seem contradictory, but turning the robot "RIGHT" compensates for its leftward angle
    #         return 0.1, (0.055 - left_distance) * 2.1

    #     # Positive difference means robot should turn right
    #     elif (difference > 0.01 or left2_distance > 0.65):
    #         print('LEFT')  # Similarly, turning "LEFT" compensates for its rightward angle
    #         return 0.1, -(left_distance - 0.065) * 2.1

    #     # Robot is approximately parallel to the wall
    #     else:
    #         return 0.1, 0

    
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
        '''
        
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
        # Change last value to change distance from turning from wall
        if self.us_sensor.front2_distance() < 0.12:
            # 90 degree turn right
            return 0, 0
            # return 0,-np.pi/2
        else:
            return None, None
