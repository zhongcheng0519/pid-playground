import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, process_variable, dt):
        error = setpoint - process_variable
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def simulate_system(pid, setpoint, initial_value, duration, dt):
    time = np.arange(0, duration, dt)
    process_variable = np.zeros_like(time)
    process_variable[0] = initial_value

    for i in range(1, len(time)):
        control_output = pid.compute(setpoint, process_variable[i-1], dt)
        process_variable[i] = process_variable[i-1] + control_output * dt

    return time, process_variable

st.title("PID Controller Playground")

st.sidebar.header("PID Parameters")
kp = st.sidebar.slider("Proportional (P)", 0.0, 10.0, 1.0, 0.1)
ki = st.sidebar.slider("Integral (I)", 0.0, 10.0, 0.0, 0.1)
kd = st.sidebar.slider("Derivative (D)", 0.0, 10.0, 0.0, 0.1)

st.sidebar.header("Simulation Parameters")
setpoint = st.sidebar.number_input("Setpoint", 0.0, 100.0, 50.0, 1.0)
initial_value = st.sidebar.number_input("Initial Value", 0.0, 100.0, 0.0, 1.0)
duration = st.sidebar.slider("Simulation Duration (seconds)", 1, 60, 30)
dt = 0.1

pid = PIDController(kp, ki, kd)
time, process_variable = simulate_system(pid, setpoint, initial_value, duration, dt)

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(time, process_variable, label="Process Variable")
ax.axhline(y=setpoint, color='r', linestyle='--', label="Setpoint")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_title("PID Controller Response")
ax.legend()
ax.grid(True)

st.pyplot(fig)

st.write(f"""
## PID Controller Settings
- Proportional (P): {kp}
- Integral (I): {ki}
- Derivative (D): {kd}

## Simulation Settings
- Setpoint: {setpoint}
- Initial Value: {initial_value}
- Duration: {duration} seconds
""")

st.write("""
## How to use this playground
1. Adjust the PID parameters (P, I, D) using the sliders in the sidebar.
2. Set the desired setpoint and initial value for the system.
3. Choose the simulation duration.
4. Observe how the system responds to different PID settings.

## Understanding PID Control
- **P (Proportional)**: Responds to the current error. Higher values result in a faster response but may cause overshooting.
- **I (Integral)**: Responds to the accumulated error over time. Helps eliminate steady-state error but may cause oscillations.
- **D (Derivative)**: Responds to the rate of change of error. Helps reduce overshooting and settling time.

Experiment with different combinations to see how they affect the system's response!
""")