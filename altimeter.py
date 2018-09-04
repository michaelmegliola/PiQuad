import numpy as np
import time
from nxp_imu import IMU
import VL53L1X

VL53L1X_RANGE_SHORT = 1
VL53L1X_RANGE_MEDIUM = 2
VL53L1X_RANGE_LONG = 3

class TOF_VL53L1X:  # see https://github.com/pimoroni/vl53l1x-python
    
    def __init__(self):
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L1X_RANGE_SHORT)
        self.distance = None
        self.dt = None
        self.velocity = None
        self.i = 0
        
    def calibrate(self):
        print('Warming up TOF sensor (altimeter)')
        for n in range(100):
            d = self.tof.get_distance()
        if self.tof.get_distance() == 0:
            raise ValueError('Alitude sensor is reporting zero')
     
    def get_altitude(self):
        return self.tof.get_distance() / 1000.0 # meters
