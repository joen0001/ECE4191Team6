import gpiozero


class Motor:
    def __init__(self, forward_pin, backward_pin, enable_pin):
        """
        Initialise the Motor object
        :param forward_pin: The forward pin number
        :param backward_pin: The backward pin number
        :param enable_pin: The enable pin number (ensure PWM)
        """
        self.forward_pin = gpiozero.OutputDevice(pin=forward_pin)
        self.backward_pin = gpiozero.OutputDevice(pin=backward_pin)
        self.enable_pwm = gpiozero.PWMOutputDevice(pin=enable_pin, active_high=True, initial_value=0, frequency=100)

    def stop(self):
        """
        Stops the motors
        """
        self.forward_pin.value = False
        self.backward_pin.value = False
        self.enable_pwm.value = 0

    def forward(self, speed):
        """
        Makes the motors move forward
        :param speed: A float between 0 and 1 representing the speed of the motor
        """
        self.forward_pin.value = True
        self.enable_pwm.value = speed

    def backward(self, speed):
        """
        Makes the motors move backward
        :param speed: A float between 0 and 1 representing the speed of the motor
        """
        self.forward_pin.value = True
        self.enable_pwm.value = speed

    def reverse(self):
        """
        Reverse the direction of the motor
        """
        self.forward_pin.value = not self.backward_pin.value
        self.backward_pin.value = not self.forward_pin.value

    def is_active(self):
        """
        Checks if the motor is active
        :return: A boolean representing if the motor is active
        """
        return self.enable_pwm.value > 0
