import numpy as np
import time
from Adafruit_BNO055 import BNO055
from nxp_imu import IMU

class AHRS_BNO055:
    def __init__(self):
        self.imu = BNO055.BNO055(serial_port='/dev/serial0', rst=18) # connected to UART @ 115200 baud, not I2C bus
        self.xyz = (None, None, None)
        self.t0 = None
        self.dt = None
        self.i = 0
        
    def get(self):
        self.i += 1
        y,r,p = self.imu.read_euler()  # TODO: docs say to use quaternions instead of euler angles
        return [r,p,y], time.time()
        
    def start(self):
        if not self.imu.begin():
            raise Exception("IMU failed to initialize")
        self.xyz, self.t0 = self.get()
        
    def run(self):
        while not self.stopping:
            self.xyz, t1 = self.get()
            self.dt = t1 - self.t0
            self.t0 = t1
        print("Stopping AHRS_BNO055")
        
    def __str__(self):
        return 'AHRS_BNO055 i=' + str(self.i) + ' (roll,pitch,yaw), dt: ' + str(self.xyz) + ', ' + str(self.dt)


class GYRO_FXAS21002:
    '''
    NOTE: sensor is mounted upside-down
    '''
    def __init__(self):
        self.gyro = IMU(dps=250, gyro_bw=800, verbose=False).gyro
        self.calibrated = False
        self.drift = [0.0,0.0,0.0]
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.t0 = None
        self.dt = None
        self.i = 0

    def get(self):
        self.i += 1
        x_dot,y_dot,z_dot = self.gyro.get()
        return np.subtract((x_dot,y_dot,z_dot), self.drift), time.time()
        
    def start(self):
        self.calibrate()
        self.xyz, self.t0 = self.get()
        
    def run(self):
        while not self.stopping:
            self.xyz, t1 = self.get()
            self.dt = t1 - self.t0
            self.t0 = t1
        print("Stopping GYRO_FXAS21002")
        
    def calibrate(self):
        print('Calibrating GYRO_FXAS21002...')

        for i in range(10):
            self.gyro.get()

        for i in range(400):
            x, y, z = self.gyro.get()
            v = (x,y,z)
            self.drift = np.add(v, self.drift)

        self.drift = np.divide(self.drift, 400)
        print('Gyro calibration:', self.drift)
        
    def __str__(self):
        return 'GYRO_FXAS21002 i=' + str(self.i) + ' (x_dot,y_dot,z_dot), dt: ' + str(self.xyz) + ', ' + str(self.dt)

class AHRS():
    def __init__(self):
        self.gyro = GYRO_FXAS21002()
        self.ahrs = AHRS_BNO055()
        self.i = 0
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.t0 = None
        self.dt = None
        
    def start(self):
        self.gyro.start()
        self.ahrs.start()
        
    def get_angular_position(self):
        self.xyz, self.t0 = self.ahrs.get()
        loop_time = self.t0
        
        if time.time() > loop_time + 0.10: # reset to absolute readings @ 10Hz
            xyz, t1 = self.ahrs.get()
            self.dt = t1 - self.t0
            self.xyz_dot[0] = (xyz[0] - self.xyz[0]) / self.dt
            self.xyz_dot[1] = (xyz[1] - self.xyz[1]) / self.dt
            self.xyz_dot[2] = (xyz[2] - self.xyz[2]) / self.dt
            self.xyz = xyz
            loop_time = t1
        else:
            self.xyz_dot, t1 = self.gyro.get()
            self.dt = t1 - self.t0
            self.xyz[0] += self.xyz_dot[0] * self.dt  # between absolute readings, integrate
            self.xyz[1] += self.xyz_dot[1] * self.dt  # gyro reading over time to advance
            self.xyz[2] += self.xyz_dot[2] * self.dt  # estimate of absolute euler angles.
            
        self.t0 = t1
        self.i += 1
        self.stop_time = time.time()

        return self.xyz, self.xyz_dot, self.dt
    
    def __str__(self):   
        return 'AHRS i=' + str(self.i) + ',' + str(self.ahrs.i) + ',' + str(self.gyro.i) + ' ' + str(self.xyz) + str(self.xyz_dot) + str(self.dt)
