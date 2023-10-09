from CONFIG import *
from gpiozero import DistanceSensor

class UltrasonicSensor:
    def __init__(self, f1e, f1t, f2e, f2t, re, rt, le, lt, thresholdf, thresholdlr):

        # Initializing Front Sensors
        self.sensor_front1 = DistanceSensor(echo=f1e, trigger=f1t)
        print("Front left sensor initialised...")

        self.sensor_front2 = DistanceSensor(echo=f2e, trigger=f2t)
        print("Front right sensor initialised...")

        # Initializing Right and Left Sensors
        self.sensor_fright = DistanceSensor(echo=re, trigger=rt)
        print("Right sensor initialised...")

        self.sensor_fleft = DistanceSensor(echo=le, trigger=lt)
        print("Left sensor initialised...")

        # Initializing thresholds
        self.thresholdf = thresholdf
        self.thresholdlr = thresholdlr

    # Helper method to calculate distance
    def _calculate_distance(self, sensor):
        return sensor.distance
    
    def fleft_distance(self):
        fleft_dist = self._calculate_distance(self.sensor_fleft)
        return fleft_dist
    
    def front1_distance(self):
        front1_dist = self._calculate_distance(self.sensor_front1)
        return front1_dist
    
    def front2_distance(self):
        front2_dist = self._calculate_distance(self.sensor_front2)
        return front2_dist

    # Method to check if the distance is inside the threshold
    def inside(self, distance, threshold):
        return distance < threshold

    # Method to check if the distance is outside the threshold
    def outside(self, distance, threshold):
        return distance > threshold

    # Method to check if both front distances are outside the threshold
    def both_front(self, front_dist1, front_dist2, threshold):
        return self.outside(front_dist1, threshold) and self.outside(front_dist2, threshold)

    def detect_obstacle(self, categorized_tentacles):
        """
        This method detects the obstacles according to the sensor readings
        and returns new required trajectory in order to avoid obstacle.
        """

        # Getting Distances
        front1_dist = self._calculate_distance(self.sensor_front1)
        front2_dist = self._calculate_distance(self.sensor_front2)
        fright_dist = self._calculate_distance(self.sensor_fright)
        fleft_dist = self._calculate_distance(self.sensor_fleft)

        # Initializing available options
        avail = []

        # Checking sensor readings and adding available options
        if self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr) and self.outside(fright_dist, self.thresholdlr):
            print('All tentacles')
            avail.extend(categorized_tentacles["straight"])

        elif self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fleft_dist, self.thresholdlr):
            print('Front and left tentacles')
            avail.extend(categorized_tentacles["left_turning"])

        elif self.both_front(front1_dist, front2_dist, self.thresholdf) and self.outside(fright_dist, self.thresholdlr):
            print('Front and right tentacles')
            avail.extend(categorized_tentacles["right_turning"])

        elif self.both_front(front1_dist, front2_dist, self.thresholdf):
            print('Front tentacles')
            avail.extend(categorized_tentacles["straight"])

        elif self.outside(fleft_dist, self.thresholdlr):
            print('Left tentacles')
            avail.extend(categorized_tentacles["left_turning"])

        elif self.outside(fright_dist, self.thresholdlr):
            print('Right tentacles')
            avail.extend(categorized_tentacles["right_turning"])

        else:
            print('No clear path')
            return None  # No clear path

        return avail

    