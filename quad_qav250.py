import numpy as np
import time
from ahrs import AHRS
from pid import PidController
from esc import ESC
from altimeter import TOF_VL53L1X

class QuadQav250:
    
    PID_ANGULAR = [[0.006,0.0,0.0],[0.006,0,0],[0,0,0]]
    
    def __init__(self):
        try:
            self.altimeter = TOF_VL53L1X()
            self.thrust = ESC()
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
            
            base_throttle = [.20,.20,.20,.20]
            
            t0 = time.time()
            flying = True
            while flying:
                if time.time() > t0 + 0.20:
                        base_throttle = np.add(base_throttle, 0.001)
                pid_update = self.angular_pid_ahrs.update()
                v_throttle = np.add(base_throttle, pid_update)
                self.thrust.set_throttle(v_throttle)
                xyz, xyz_dot, dt = self.altimeter.get_altitude()
                if xyz[2] > 0.200:
                        flying = False
                print(v_throttle,self.thrust.v_pwm,self.ahrs.xyz)
            self.thrust.set_throttle([.2,.2,.2,.2])
            time.sleep(0.5)
            self.thrust.set_throttle([0,0,0,0])
            
            
        finally:
            self.shutdown()
        
q = QuadQav250()
#q.test()
q.fly()






