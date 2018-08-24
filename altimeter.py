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
        print('Calibrating TOF sensor (altimeter)')
        i = 0
        while i < 100:
            d = self.tof.get_distance()
            i += 1
            if d > 50.0:
                print('Cannot calibrate TOF sensor (altimeter); distance = ', d)
                i = 0
            # need visual feedback here
       
    # OLD CODE    
    def run(self):
        d0 = self.tof.get_distance()
        t0 = time.time()
        while not self.stopping:
            d1 = self.tof.get_distance()
            t1 = time.time()
            self.distance = d1
            self.dt = t1 - t0
            self.velocity = (d1-d0)/(t1-t0)
            self.i += 1
            d0 = d1
            t0 = t1
 