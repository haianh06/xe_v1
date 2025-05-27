import time
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.
        self.prev_error = 0.
        self.last_time = time.time()
    def compute_output(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time
        output = self.Kp * error + self.Ki * integral + self.Kd * derivative
        self.prev_error = error
        return output