# Adapted from https://pimylifeup.com/raspberry-pi-rfid-rc522/
# Ensure SPI enabled in raspi-config
# Needs "sudo pip3 install spidev"
# Needs "sudo pip3 install mfrc522"

# PINS
# RST = 22
# MISO = 21
# MOSI = 19
# SCK = 23
# SDA = 24

# TO BE PLACED WITHIN MAIN LOOP

from CONFIG import *

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522


reader = SimpleMFRC522()
destination = [0,0,0]

try:
        id, goal = reader.read()
        print("Parcel belongs to location " + goal)
        if goal == 'A':
               destination = goal_A
        elif goal == 'B':
                destination = goal_B
        elif goal == 'C':
                destination = goal_C
        else:
            print("Unknown parcel detected")
                
finally:
        GPIO.cleanup()