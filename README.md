# PID Controller Playground

This is an interactive Streamlit application that demonstrates the behavior of a PID (Proportional-Integral-Derivative) controller. Users can adjust PID parameters and observe how they affect the system's response to a setpoint.

## Prerequisites

- Python 3.7 or higher
- pip (Python package installer)

## Installation

1. Clone this repository or download the project files.

2. Navigate to the project directory:
   ```
   cd pid_playground
   ```

3. Create a virtual environment (optional but recommended):
   ```
   python -m venv venv
   ```

4. Activate the virtual environment:
   - On Windows:
     ```
     venv\Scripts\activate
     ```
   - On macOS and Linux:
     ```
     source venv/bin/activate
     ```

5. Install the required packages:
   ```
   pip install -r requirements.txt
   ```

## Running the Application

1. Make sure you're in the project directory and your virtual environment is activated (if you created one).

2. Run the Streamlit application:
   ```
   streamlit run main.py
   ```

3. Your default web browser should open automatically with the application. If it doesn't, you can manually open the URL displayed in the terminal (usually http://localhost:8501).

## Using the PID Controller Playground

1. Use the sliders in the sidebar to adjust the PID parameters (P, I, and D).
2. Set the desired setpoint and initial value for the system using the number inputs.
3. Adjust the simulation duration using the slider.
4. Observe how the system responds to different PID settings in the graph.
5. Read the explanation below the graph to understand the effects of each PID component.

Experiment with different combinations of P, I, and D values to see how they affect the system's response to the setpoint!

## Understanding PID Control

- **P (Proportional)**: Responds to the current error. Higher values result in a faster response but may cause overshooting.
- **I (Integral)**: Responds to the accumulated error over time. Helps eliminate steady-state error but may cause oscillations.
- **D (Derivative)**: Responds to the rate of change of error. Helps reduce overshooting and settling time.

Have fun exploring the world of PID control!