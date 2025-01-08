from machine import Pin, PWM

PWM_FREQ = 1000  # Constante pour la fréquence PWM

class Motor:
    def __init__(self, pwm_pin, dir_pin_A, dir_pin_B, direction=1):
        self.pwm = PWM(Pin(pwm_pin))
        self.dir_A = Pin(dir_pin_A, Pin.OUT)
        self.dir_B = Pin(dir_pin_B, Pin.OUT)
        self.direction = direction
        self.pwm.freq(PWM_FREQ)

    def set_speed(self, speed):
        speed *= self.direction
        if speed >= 0:
            self.dir_A.value(1)
            self.dir_B.value(0)
            self.pwm.duty_u16(int(speed * 65535))
        else:
            self.dir_A.value(0)
            self.dir_B.value(1)
            self.pwm.duty_u16(int(-speed * 65535))
