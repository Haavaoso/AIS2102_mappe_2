import sys
import sys


class PID:
    def __init__(self):
        self.kp = 0.05
        self.ki = 0.01
        self.kd = 0.0005

        self.windup = 0
        self.lastIntegral = 0
        self.lastError = 0
        self.useWindup = False
        self.debug = True

    def regulate(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        error = setpoint - measured_value

        P = error * self.kp
        self.lastIntegral += error * dt
        I = self.lastIntegral * self.ki
        D = ((error - self.lastError) / dt) * self.kd
        self.lastError = error
        output = P + I + D

        output = max(min(output, 24), -24)

        if self.debug:
            print(f'PID Output: {output}, P: {P}, I: {I}, D: {D}')


        return output

    def copy(self, pid):
        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup