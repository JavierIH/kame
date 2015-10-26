import os
import sys
sys.path.append(os.path.abspath(__file__).replace('kame/code/main.py', '') + 'pybotics/')

import time
import smbus
from driver.pca9685.pca9685 import ServoController
from sensor.bno055.bno055 import Inclinometer
import control.octosnake.octosnake as octosnake
from scipy import signal

class Kame(object):
    
    def __init__(self, servo_trims, servo_pins, i2c_bus=0, pca9685_address=0x40, name='kame'):
        
        #Configuration
        self._name = name
        self._i2c_bus = i2c_bus
        self._servo_trims = servo_trims
        self._servo_pins = servo_pins
        self._pca9685_address = pca9685_address
    
        #Setting up hardware
        self._bus = smbus.SMBus(self._i2c_bus)
        if not self._bus:
            raise Exception('I2C bus connection failed!')

        self.controller = ServoController(self._bus, self._pca9685_address)

        #Setting up OctoSnake
        self.osc = []
        self.osc.append(octosnake.Oscillator())
        self.osc.append(octosnake.Oscillator(octosnake.semiSin))
        self.osc.append(octosnake.Oscillator())
        self.osc.append(octosnake.Oscillator(octosnake.semiSin))
        self.osc.append(octosnake.Oscillator())

        self.osc[1].ref_time = self.osc[0].ref_time
        self.osc[2].ref_time = self.osc[0].ref_time
        self.osc[3].ref_time = self.osc[0].ref_time
        self.osc[4].ref_time = self.osc[0].ref_time

        #Setting up servo controller
        for i in range(len(self._servo_pins)):
            self.controller.addServo(self._servo_pins[i], self._servo_trims[i])

        self.controllerler.servos[self._servo_pins[1]].reverse = True
        self.controller.servos[self._servo_pins[3]].reverse = True


    def walk(self, steps):

        T = 900                 #milliseconds 
        period = [T, T, T, T, T]
        amplitude = [left_x_amp, z_amp, right_x_amp, z_amp, swing_amp]
        offset = [-10, -75, -10, -75, 0]
        phase = [90, 180, 270, 0, 335+90]

        for i in range(len(self.osc)):
            self.osc[i].period = period[i]
            self.osc[i].amplitude = amplitude[i]
            self.osc[i].phase = phase[i]
            self.osc[i].offset = offset[i]
 
        final = time.time() + float(T*steps/1000) 
        while time.time() < final:
            try:
                for i in range(len(self.osc)):
                    self.osc[i].refresh()

                self.controller.move(self._servo_pins[0], osc[0].output[0])

            except IOError:
                self._bus = smbus.SMBus(self._i2c_bus)
