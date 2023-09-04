# from CONFIG import *
from gpiozero import DistanceSensor

class UltrasonicSensor:
    # def __init__(self, fe=9, ft=10, re=26, rt=16, le=1, lt=7, bre=0, brt=0, ble=0, blt=0, thresholdf=20, thresholdlr=15):
    def __init__(self, f1e=9, f1t=10, f2e=0, f2t=0, re=26, rt=16, le=1, lt=7, bre=0, brt=0, ble=0, blt=0, thresholdf=20, thresholdlr=15):
        self.sensor_front1 = DistanceSensor(echo=f1e,trigger=f2t)
        print("front1 initialised")
        self.sensor_front2 = DistanceSensor(echo=f1e,trigger=f2t)
        print("front2 initialised")
        # self.sensor_front = DistanceSensor(echo=fe,trigger=ft)
        # print("front initialised")
        self.sensor_fright = DistanceSensor(echo=re, trigger=rt)
        print("right initialised")
        self.sensor_fleft = DistanceSensor(echo=le, trigger=lt)
        print("left initialised")
        # self.sensor_right = DistanceSensor(bre, brt)
        # self.sensor_left = DistanceSensor(ble, blt)
        # Distance threshold
        self.thresholdf = thresholdf
        self.thresholdlr = thresholdlr
        # # Counter to stop noise readings
        # self.counter = counter
        # Tentacle planner function
        # self.tentacle_planner = tentacle_planner
        # self.categorized_tentacles = self.tentacle_planner.categorize_tentacles()
        
    # Inside threshold
    def inside(self, distance, threshold): 
        return distance < threshold
    
    # Outside threshold
    def outside(self, distance, threshold):
        return distance > threshold

    # def front_distance(self):
    #     return self.sensor_front.distance * 100
    def front1_distance(self):
        return self.sensor_front1.distance * 100
    
    def front2_distance(self):
        return self.sensor_front2.distance * 100

    def fright_distance(self):
        return self.sensor_fright.distance * 100

    def fleft_distance(self):
        return self.sensor_fleft.distance * 100

    def right_distance(self):
        return self.sensor_right.distance * 100

    def left_distance(self):
        return self.sensor_left.distance * 100
    
    # detect if tentacles are chill, therefore no obstacles, add into array of costs

    def both_front(self, front_dist1, front_dist2, threshold):
        return (self.outside(front_dist1, threshold) or self.outside(front_dist2, threshold))
    
    def detect_obstacle(self, categorized_tentacles):
        """Returns new required trajectory in order to avoid obstacle as per sensor readings"""
        front1_dist = self.front1_distance()
        front2_dist = self.front2_distance()
        # front_dist = self.front_distance()
        fright_dist = self.fright_distance()
        fleft_dist = self.fleft_distance()
        
        """TODO call tentacle planner options, best cost within range of movement"""
        
        # Fresh available options everytime.
        avail = []
        
        # Counter for distances beyond threshold, check continually clear:
        # if self.outside(front_dist) or self.outside(fright_dist) or self.outside(fleft_dist):
        #     self.counter += 1  # Increment the counter
        # else:
        #     self.avail = []
        #     self.counter = 0  # Reset the counter

        # If distances remain consistently beyond the threshold over several checks
        # if self.counter >= 5:
        # if self.outside(front_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr) and self.outside(fright_dist, self.thresholdlr):
        #     # Include all tentacle paths
        #     print('all')
        #     avail.extend(categorized_tentacles["left_turning"])
        #     avail.extend(categorized_tentacles["straight"])
        #     avail.extend(categorized_tentacles["right_turning"])
        # elif self.outside(front_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr):
        #     # Include front and left paths
        #     print('f+l')
        #     avail.extend(categorized_tentacles["left_turning"])
        #     # avail.extend(categorized_tentacles["straight"])
        # elif self.outside(front_dist, self.thresholdf) and self.outside(fright_dist, self.thresholdlr):
        #     # Include front and right paths
        #     print('f+r')
        #     # avail.extend(categorized_tentacles["straight"])
        #     avail.extend(categorized_tentacles["right_turning"])
        # elif self.outside(fleft_dist, self.thresholdlr) and self.outside(fright_dist, self.thresholdlr):
        #     # Include left and right paths
        #     print('r+l')
        #     avail.extend(categorized_tentacles["left_turning"])
        #     # avail.extend(categorized_tentacles["right_turning"])
        # elif self.outside(front_dist, self.thresholdf):
        #     print('f')
        #     # Include front path
        #     avail.extend(categorized_tentacles["straight"])
        # elif self.outside(fleft_dist, self.thresholdlr):
        #     print('l')
        #     # Include left path
        #     avail.extend(categorized_tentacles["left_turning"])
        # elif self.outside(fright_dist, self.thresholdlr):
        #     print('r')
        #     # Include right path
        #     avail.extend(categorized_tentacles["right_turning"])
        # else:
        #     # No clear path, return None
        #     return None

        if self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr) and self.outside(fright_dist, self.thresholdlr):
            # Include all tentacle paths
            print('all')
            avail.extend(categorized_tentacles["left_turning"])
            avail.extend(categorized_tentacles["straight"])
            avail.extend(categorized_tentacles["right_turning"])
        elif self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr):
            # Include front and left paths
            print('f+l')
            avail.extend(categorized_tentacles["left_turning"])
            # avail.extend(categorized_tentacles["straight"])
        elif self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fright_dist, self.thresholdlr):
            # Include front and right paths
            print('f+r')
            # avail.extend(categorized_tentacles["straight"])
            avail.extend(categorized_tentacles["right_turning"])
        elif self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fright_dist, self.thresholdlr):
            # Include left and right paths
            print('r+l')
            avail.extend(categorized_tentacles["left_turning"])
            # avail.extend(categorized_tentacles["right_turning"])
        elif self.both_front(front1_dist, front2_dist, self.thresholdf):
            print('f')
            # Include front path
            avail.extend(categorized_tentacles["straight"])
        elif self.outside(fleft_dist, self.thresholdlr):
            print('l')
            # Include left path
            avail.extend(categorized_tentacles["left_turning"])
        elif self.outside(fright_dist, self.thresholdlr):
            print('r')
            # Include right path
            avail.extend(categorized_tentacles["right_turning"])
        else:
            # No clear path, return None
            return None

        return avail
        
        
        # """Other code"""
        # full_tentacles = tentacle_planner.tentacles.copy()
        # # Counter for obstacle to avoid noise:
        # # If obstacle detected in any of the sensors
        # if self.inside(front_dist) or self.inside(fright_dist) or self.inside(fleft_dist):
        #     self.counter += 1  # Increment the counter
        # else:
        #     self.counter = 0  # Reset the counter

        # # If obstacle directly ahead and the counter threshold is breached
        # if self.counter >= 5:
        #     # Check if very close to wall on both sides
        #     if self.inside(fright_dist) and self.inside(fleft_dist):
        #         # Choose direction based on which side has a longer distance to the obstacle
        #         if fleft_dist > fright_dist:
        #             # TODO: remove right options and front tentacles for a number of steps
        #             return -0.1, 0.5  # backup and turn left
        #         elif fleft_dist < fright_dist:
        #             # TODO: remove left options and front tentacles for a number of steps
        #             return -0.1, -0.5  # backup and turn right
        #         else:
        #             # TODO: do something crazy
        #             return -0.1, 0.0  # just back up, try getting another reading
        #     # If only the right side is blocked, turn left
        #     elif self.inside(fright_dist):
        #         # TODO: rmove right options and tentacles
        #         return 0.1, 0.5  # move forward and turn left
        #     # If only the left side is blocked, turn right
        #     elif self.inside(fleft_dist):
        #         # TODO: rmove left options and tentacles
        #         return 0.1, -0.5  # move forward and turn right

        # # No obstacle detected based on threshold
        # return None, None
    