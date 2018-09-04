
import numpy as np
import sys
import time
import random
import pigpio

class ESC:
    FRONT_RIGHT = 0
    REAR_RIGHT = 1
    REAR_LEFT = 2
    FRONT_LEFT = 3
    
    FORE = 0
    STARBOARD = 1
    AFT = 2
    PORT = 3
    
    PIN_FR = 22
    PIN_RR = 23
    PIN_RL = 24
    PIN_FL = 25
    
    ESC_PWM_MIN = 1000  # esc off = pulse width of 1,000us = 0.001s
    ESC_PWM_MAX = 2000  # esc max = pulse width of 2,000us = 0.002s
    
    def __init__(self, ESC_limit=1.0):
        self.pi = pigpio.pi()
        if ESC_limit > 1.0 or ESC_limit < 0.0:
            print("WARNING: ESC limit is out of bounds; disabling ESC", ESC_limit)
            self.ESC_limit = 0.0
        else:
            self.ESC_limit = max(min(ESC_limit, 1.0), 0.0)
        self.esc_pins = [ESC.PIN_FR,ESC.PIN_RR,ESC.PIN_RL,ESC.PIN_FL]
        self.v_pwm = ([ESC.ESC_PWM_MIN,ESC.ESC_PWM_MIN,ESC.ESC_PWM_MIN,ESC.ESC_PWM_MIN])
        self.esc_range = ESC.ESC_PWM_MAX - ESC.ESC_PWM_MIN
        
    # setting must be [FORE,STARBOARD,AFT,PORT], each value 0.0 to 100.0
    # will be distributed in X-configuration (front-left, rear-left...)
    def set_throttle(self, setting):
        self.throttle = np.maximum(setting, 0.0)
        self.throttle = np.minimum(self.throttle, self.ESC_limit)
        
        self.v_pwm[ESC.FRONT_RIGHT] = (self.throttle[ESC.FORE] + self.throttle[ESC.STARBOARD])/2
        self.v_pwm[ESC.REAR_RIGHT] =  (self.throttle[ESC.AFT]  + self.throttle[ESC.STARBOARD])/2
        self.v_pwm[ESC.REAR_LEFT] =   (self.throttle[ESC.AFT]  + self.throttle[ESC.PORT])/2
        self.v_pwm[ESC.FRONT_LEFT] =  (self.throttle[ESC.FORE] + self.throttle[ESC.PORT])/2
        
        self.v_pwm = np.multiply(self.v_pwm, self.esc_range)
        self.v_pwm = np.rint(self.v_pwm)
        self.v_pwm = np.add(self.v_pwm, ESC.ESC_PWM_MIN)
        
        self.pi.set_servo_pulsewidth(ESC.PIN_FR, int(self.v_pwm[ESC.FRONT_RIGHT]))
        self.pi.set_servo_pulsewidth(ESC.PIN_RR, int(self.v_pwm[ESC.REAR_RIGHT]))
        self.pi.set_servo_pulsewidth(ESC.PIN_RL, int(self.v_pwm[ESC.REAR_RIGHT]))
        self.pi.set_servo_pulsewidth(ESC.PIN_FL, int(self.v_pwm[ESC.FRONT_LEFT]))

    
    def throttle_off(self):
        for pin in self.esc_pins:
            self.pi.set_servo_pulsewidth(ESC.PIN_FR, ESC.ESC_PWM_MIN)
        
    def arm(self):
        print('Arming ESCs...')
        '''
        for pin in self.esc_pins:
            self.pi.set_servo_pulsewidth(pin, 1400)
        time.sleep(0.5)
        for pin in self.esc_pins:
            self.pi.set_servo_pulsewidth(pin, ESC.ESC_PWM_MIN) # Minimum throttle.
        time.sleep(0.5)
        ''' 
        print('...ESCs are armed.')
        return True
        
    def disarm(self):
        print('Disarming ESCs...')
        self.throttle_off()
        for pin in self.esc_pins:
            self.pi.set_servo_pulsewidth(ESC.PIN_FR, 0)
        self.pi.stop()
        print('ESCs are disarmed...')
        
    def spin_test(self):
        print('starting spin test')
        for pin in self.esc_pins:
            print('powering up pin =', pin)
            self.pi.set_servo_pulsewidth(pin, 1300)
            time.sleep(0.5)
        for pin in self.esc_pins:
            print('powering down pin =', pin)
            self.pi.set_servo_pulsewidth(pin, 1000)
            time.sleep(0.5)
        
    def __str__(self):
        esc_pct = np.subtract(self.v_pwm, ESC.ESC_PWM_MIN)
        esc_pct = np.divide(esc_pct, self.esc_range)
        return 'FR,RR,RL,FL' + str(esc_pct) + str(self.v_pwm)

