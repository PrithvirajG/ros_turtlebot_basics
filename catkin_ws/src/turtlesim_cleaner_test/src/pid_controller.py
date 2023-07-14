

class PID:
    def __init__(self, P=2.0, I=0.0, D=1.0, derivative=0, integral=0, integral_max=500, integral_min=-500):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.derivative = derivative
        self.integral = integral
        self.integral_max = integral_max
        self.integral_min = integral_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, error):
        self.error = error

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.derivative)
        self.derivative = self.error

        self.integral = self.integral + self.error

        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < self.integral_min:
            self.integral = self.integral_min

        self.I_value = self.integral * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

