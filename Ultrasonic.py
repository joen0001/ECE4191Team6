#from CONFIG import *
from gpiozero import DistanceSensor


<<<<<<< Updated upstream
def Ultrasonic():
    sensor_front = DistanceSensor(ULT_FRONT_ECHO, ULT_FRONT_TRIG)
    sensor_right = DistanceSensor(ULT_RIGHT_ECHO, ULT_RIGHT_TRIG)
    sensor_left = DistanceSensor(ULT_LEFT_ECHO, ULT_LEFT_TRIG)
    return sensor_front.distance*100, sensor_right.distance*100, sensor_left.distance*100
=======
class UltrasonicSensor:
    def __init__(self, threshold):
        self.sensor_front = DistanceSensor(ULT_FRONT_ECHO, ULT_FRONT_TRIG)
        self.sensor_fright = DistanceSensor(ULT_RIGHT_ECHO, ULT_RIGHT_TRIG)
        self.sensor_fleft = DistanceSensor(ULT_LEFT_ECHO, ULT_LEFT_TRIG)
        self.sensor_right = DistanceSensor(ULT_BACK_RIGHT_ECHO, ULT_BACK_RIGHT_TRIG)
        self.sensor_left = DistanceSensor(ULT_BACK_LEFT_ECHO, ULT_BACK_LEFT_TRIG)
        self.threshold = threshold
        
    def _check_threshold(self, distance):  # A private method to check threshold
        if distance < self.threshold:
            return True
        return False

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
    
    def detect_obstacle(self):
        """Returns new required trajectory in order to avoid obstacle as per sensor readings"""
        front_dist = self.front_distance()
        fright_dist = self.fright_distance()
        fleft_dist = self.fleft_distance()
        
        """TODO:"""
        # Need to add obstacle detection for front left and front right rather than just down the middle.

        # If obstacle directly ahead
        if front_dist < self.threshold:
            # Check if very close to wall on both sides
            if fright_dist < self.threshold and fleft_dist < self.threshold:
                # if can see further left, go left
                if fleft_dist > fright_dist:
                    return -0.1, 0.5  # backup and turn left
                # if can see further right, go right
                elif fleft_dist < fright_dist:
                    return -0.1, -0.5  # backup and turn right
                # if can't see further in either direction
                else:
                    return -0.1, 0.0  # just back up, try getting another reading
            # If only the right side is blocked, turn left
            elif fright_dist < self.threshold:
                return 0.1, 0.5  # move forward and turn left
            # If only the left side is blocked, turn right
            elif fleft_dist < self.threshold:
                return 0.1, -0.5  # move forward and turn right

        # No obstacle detected based on threshold
        return None, None
>>>>>>> Stashed changes
