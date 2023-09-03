from CONFIG import *
from gpiozero import DistanceSensor

class UltrasonicSensor:
    def __init__(self, fe, ft, re, rt, le, lt, bre, brt, ble, blt, threshold=20):
        self.sensor_front = DistanceSensor(echo=fe,trigger=ft)
        print("Front sensor initialised...")
        self.sensor_fright = DistanceSensor(echo=re, trigger=rt)
        print("Right sensor initialised...")
        self.sensor_fleft = DistanceSensor(echo=le, trigger=lt)
        print("Left sensor initialised...")
        # self.sensor_right = DistanceSensor(bre, brt)
        # self.sensor_left = DistanceSensor(ble, blt)
        # Distance threshold
        self.threshold = threshold
        
    # Inside threshold
    def inside(self, distance): 
        # Condition less than 100 accounts for error reading
        return (distance < self.threshold) and (distance < 100)
    
    # Outside threshold
    def outside(self, distance):
        return (distance > self.threshold) and (distance < 100)

    def front_distance(self):
        return self.sensor_front.distance * 100

    def fright_distance(self):
        return self.sensor_fright.distance * 100

    def fleft_distance(self):
        return self.sensor_fleft.distance * 100

    def right_distance(self):
        return self.sensor_right.distance * 100

    def left_distance(self):
        return self.sensor_left.distance * 100
    
    # detect if tentacles are chill, therefore no obstacles, add into array of costs
    
    def detect_obstacle(self, categorized_tentacles):
        """Returns new required trajectory in order to avoid obstacle as per sensor readings"""
        front_dist = self.front_distance()
        fright_dist = self.fright_distance()
        fleft_dist = self.fleft_distance()
        
        # Fresh available options everytime.
        avail = []
        
        # Future add counter.
        # Figure out why can't add arrays, check array correctly extended.

        if self.outside(front_dist) and self.outside(fleft_dist) and self.outside(fright_dist):
            # Include all tentacle paths
            print('All tentacles')
            avail.extend(categorized_tentacles["left_turning"])
            avail.extend(categorized_tentacles["straight"])
            avail.extend(categorized_tentacles["right_turning"])
        elif self.outside(front_dist) and self.outside(fleft_dist):
            # Include front and left paths
            print('Front and left tentacles')
            avail.extend(categorized_tentacles["left_turning"])
            # avail.extend(categorized_tentacles["straight"])
        elif self.outside(front_dist) and self.outside(fright_dist):
            # Include front and right paths
            print('Front and right tentacles')
            # avail.extend(categorized_tentacles["straight"])
            avail.extend(categorized_tentacles["right_turning"])
        elif self.outside(fleft_dist) and self.outside(fright_dist):
            # Include left and right paths
            print('Right and left tentacles')
            avail.extend(categorized_tentacles["left_turning"])
            # avail.extend(categorized_tentacles["right_turning"])
        elif self.outside(front_dist):
            print('Front tentacles')
            # Include front path
            avail.extend(categorized_tentacles["straight"])
        elif self.outside(fleft_dist):
            print('Left tentacles')
            # Include left path
            avail.extend(categorized_tentacles["left_turning"])
        elif self.outside(fright_dist):
            print('Right tentacles')
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
    