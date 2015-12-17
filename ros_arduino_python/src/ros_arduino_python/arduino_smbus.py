#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from smbus import SMBus

class ArduinoSMBus(Arduino):
    def __init__(self, port = 1, device = 0x42):
        self.port = port
        self.device = device
        self.bus = None
        self.base_init()

    def connect(self):
        self.bus = SMBus(port)

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        self.mutex.acquire()
        self.bus.write_i2c_block_data(self.device, 0x50, [ Kp & 0xff, (Kp >> 8) & 0xff ])
        self.bus.write_i2c_block_data(self.device, 0x52, [ Ki & 0xff, (Ki >> 8) & 0xff ])
        self.bus.write_i2c_block_data(self.device, 0x54, [ Kd & 0xff, (Kd >> 8) & 0xff ])
        self.bus.write_i2c_block_data(self.device, 0x56, [ Ko & 0xff, (Ko >> 8) & 0xff ])
        self.bus.write_byte_data(self.device, 0x40, 0x75)
        self.mutex.release()
        return True

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return 0;

    def get_encoder_counts(self):
        self.mutex.acquire()
        r_value_array = self.bus.read_i2c_block_data(self.device, 0x48, 4)
        l_value_array = self.bus.read_i2c_block_data(self.device, 0x44, 4)
        self.mutex.release()

        r_value = r_value_array[3] << 24 | r_value_array[2] << 16 | r_value_array[1] << 8 | r_value_array[0]
        l_value = l_value_array[3] << 24 | l_value_array[2] << 16 | l_value_array[1] << 8 | l_value_array[0]

        return [ l_value, r_value ]

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        self.mutex.acquire()
        self.bus.write_byte_data(self.device, 0x40, 0x72)
        self.mutex.release()
        return True

    def drive(self, left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        self.mutex.acquire()
        self.bus.write_i2c_block_data(self.device, 0x4e, [ left & 0xff, (left >> 8) & 0xff ])
        self.bus.write_i2c_block_data(self.device, 0x4c, [ right & 0xff, (right >> 8) & 0xff ])
        self.bus.write_byte_data(self.device, 0x40, 0x6d)
        self.mutex.release()
        return True

    def analog_read(self, pin):
        return 0

    def analog_write(self, pin, value):
        return True

    def digital_read(self, pin):
        return 0

    def digital_write(self, pin, value):
        return True

    def pin_mode(self, pin, mode):
        return True

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''
        return True

    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''
        return 0

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return 0

    def get_maxez1(self, triggerPin, outputPin):
        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
            sensor connected to the General Purpose I/O lines, triggerPin, and
            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
            power up, otherwise it wont range correctly for object less than 6
            inches away! The sensor reading defaults to use English units
            (inches). The sensor distance resolution is integer based. Also, the
            maxsonar trigger pin is RX, and the echo pin is PW.
        '''
        return 0


""" Basic test for connectivity """
if __name__ == "__main__":
    myArduino = ArduinoSMBus(port = 1)
    myArduino.connect()

    print "Sleeping for 1 second..."
    time.sleep(1)

    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()

    print "Connection test successful.",

    print "Shutting down Arduino."

