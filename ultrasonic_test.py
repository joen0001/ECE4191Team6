from gpiozero import DistanceSensor

def main():
    ULT_FRONT_ECHO = 1
    ULT_FRONT_TRIG = 7
    sensor_front = DistanceSensor(ULT_FRONT_ECHO, ULT_FRONT_TRIG)
    print(sensor_front.distance*100)

