import time

class Robot:
    def __init__(self, sensor_front, sensor_left):
        self.sensor_front = sensor_front  # instance of UltrasonicSensor
        self.sensor_left = sensor_left  # instance of UltrasonicSensor

    def drive_forward(self):
        # Placeholder for driving forward code
        print("Driving forward")

    def adjust_left_distance(self):
        # Placeholder for adjusting the distance from the left wall
        print("Adjusting left distance")
        
    def maintain_left_distance(self):
        left_distance = self.sensor_left.fleft_distance()
        # Drive forward only if the left distance is between 20 and 30 cm.
        if 20 < left_distance < 30:
            self.drive_forward()
        else:
            self.adjust_left_distance()

    def turn_right(self):
        # Placeholder for turning right code
        print("Turning right")

    def is_at_corner(self):
        # Assuming 20 as the distance in cm to detect corner/wall.
        return self.sensor_front.front1_distance() < 20 and self.sensor_left.fleft_distance() < 20

    def follow_wall(self):
        while not self.is_at_corner():
            # Maintain 20-30cm from the left wall and drive forward if within range
            self.maintain_left_distance()

            # Small delay to avoid busy-waiting
            time.sleep(0.1)

        # Now at the corner, so turn right
        self.turn_right()


# Initialize your sensor objects here
front_sensor = None  # Initialize with an actual UltrasonicSensor object
left_sensor = None  # Initialize with another UltrasonicSensor object

# Create a Robot object
robot = Robot(sensor_front=front_sensor, sensor_left=left_sensor)

# Make the robot follow the square arena
for _ in range(4):  # Assuming square arena
    robot.follow_wall()
