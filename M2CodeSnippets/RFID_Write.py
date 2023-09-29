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

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

reader = SimpleMFRC522()

try:
        goal = input('Select the location A/B/C to write to tag.\n')
        while (goal != 'A' or 'B' or 'C'):
                goal = input('Ensure only "A", "B" or "C" is entered.\n')
        print('Now place your tag to write')
        reader.write(goal)
        print('Tag written as location ' + goal + '.')
finally:
        GPIO.cleanup()