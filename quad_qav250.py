import numpy as np
import time
from Adafruit_BNO055 import BNO055
from nxp_imu import IMU
import VL53L1X
from ahrs import *
from pid import *
from esc import *

class QuadQav250:
    
    PID_ANGULAR = [[0.06,0.0,0.0],[0.06,0,0],[0,0,0]]
    
    def __init__(self):
        try:
            self.thrust = ESC()
            self.thrust.start()
            self.ahrs = AHRS()
            self.angular_pid_ahrs = PidController(QuadQav250.PID_ANGULAR, self.ahrs.get_angular_position, [0,0,0], PidController.t_angular)
            self.ahrs.start()
        except Exception:
            self.shutdown()
            print('Failed to initialize')
        
    def shutdown(self):
        try:
            self.thrust.disarm()
        finally:
            print('===DISARMED===')
    
    def test(self):
        for n in range(100):
            pass
            #print(self.ahrs.get_angular_position())
    
    def kill(self):
        self.shutdown()
    
    def fly(self):
        try:
            print('============================')
            print('STARTING TEST FLIGHT')
            
            self.thrust.spin_test()
            
            base_throttle = [.30,.30,.30,.30]
            
            t0 = time.time()
            i = 0
            while time.time() < t0 + 12.0:
                pid_update = self.angular_pid_ahrs.update()
                v_throttle = np.add(base_throttle, pid_update)
                self.thrust.set_throttle(v_throttle)
                #print(self.angular_pid_ahrs)
                #print(self.thrust)
                #self.thrust.set_throttle(base_throttle)
            
            self.thrust.set_throttle([0,0,0,0])
            
            
        finally:
            self.shutdown()
        
q = QuadQav250()
#q.test()
q.fly()





