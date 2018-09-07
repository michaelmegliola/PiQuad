import numpy as np
import time
from nxp_imu import IMU
import VL53L1X

VL53L1X_RANGE_SHORT = 1
VL53L1X_RANGE_MEDIUM = 2
VL53L1X_RANGE_LONG = 3

class TOF_VL53L1X:  # see https://github.com/pimoroni/vl53l1x-python
    
    TIME_HORIZON_SECONDS = 0.20  # 5 hz
    
    def __init__(self):
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L1X_RANGE_SHORT)
        self.i = 0
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.t0 = None
        self.t_expire = None
        self.dt = None
        
    def calibrate(self):
        print('Warming up TOF sensor (altimeter)')
        for n in range(10):
            d = self.tof.get_distance()
        if self.tof.get_distance() == 0:
            raise ValueError('Alitude sensor is reporting zero')
     
    def get_altitude(self):
        
        self.i += 1
        
        if self.t0 == None:
            altitude = self.tof.get_distance() / 1000.0
            alt_time = time.time()
            self.xyz[2] = altitude
            self.xyz_dot[2] = 0.0
            self.t0 = alt_time
            self.t_expire = alt_time + TOF_VL53L1X.TIME_HORIZON_SECONDS
            self.dt = 0.0
        elif time.time() > self.t_expire:
            altitude = self.tof.get_distance() / 1000.0
            alt_time = time.time()           
            da = altitude - self.xyz[2]
            self.xyz[2] = altitude
            self.dt = alt_time - self.t0
            self.xyz_dot[2] = da / self.dt
            self.t0 = alt_time
            self.t_expire = alt_time + TOF_VL53L1X.TIME_HORIZON_SECONDS
            
        return self.xyz, self.xyz_dot, self.dt
        
