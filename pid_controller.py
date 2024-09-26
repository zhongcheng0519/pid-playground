import numpy as np

def simulate_system(pid, setpoint, initial_value, duration, dt):
    time = np.arange(0, duration, dt)
    process_variable = np.zeros_like(time)
    process_variable[0] = initial_value

    for i in range(1, len(time)):
        control_output = pid.compute(setpoint, process_variable[i-1])
        process_variable[i] = process_variable[i-1] + control_output * dt

    return time, process_variable

class PIDController:
    def __init__(self, kp, ki, kd):
        """
        Initialize the PID controller.

        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.prev_prev_error = 0

    def compute(self, setpoint, process_variable):
        """
        Compute the control output based on the setpoint and process variable.

        This implementation uses the formula:
        kP * (e_k - e_pk) + kI * e_k + kD * (e_k - 2 * e_pk + e_ppk)

        Where:
        - e_k is the current error
        - e_pk is the previous error
        - e_ppk is the error before the previous error

        :param setpoint: The desired value
        :param process_variable: The current measured value
        :return: The computed control output
        """
        error = setpoint - process_variable

        output = (
            self.kp * (error - self.prev_error) + \
            self.ki * error + \
            self.kd * (error - 2 * self.prev_error + self.prev_prev_error)
        )

        # Update error history
        self.prev_prev_error = self.prev_error
        self.prev_error = error

        return output