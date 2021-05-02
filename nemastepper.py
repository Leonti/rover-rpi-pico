# https://elfnor.com/micropython-stepper-motor-control-with-a-a4988-carrier-board.html

from machine import Pin

class Stepper:
    """
    Handles  A4988 hardware driver for bipolar stepper motors
    """
    
    def __init__(self, dir_pin, step_pin):
        self.step_pin = Pin(step_pin, Pin.OUT)
        self.dir_pin = Pin(dir_pin, Pin.OUT)    
        self.dir = 0
        self.pulserate = 100
        self.count = 0
        self.speed = 0
 

    def do_step(self):   # called by timer interrupt every 100us
        if self.dir == 0:
            return
        self.count = (self.count+1)%self.pulserate
        if self.count == 0:
            self.step_pin.high()
            pass
            self.step_pin.low()

    def set_speed(self, speed):
        self.speed = speed   

        # set direction
        if self.speed>0:
            self.dir = 1
            self.dir_pin.high()    
        elif self.speed<0:
            self.dir = -1
            self.dir_pin.low()      
        else:
            self.dir = 0
        if abs(self.speed)>0:
            self.pulserate = 10000//abs(self.speed)