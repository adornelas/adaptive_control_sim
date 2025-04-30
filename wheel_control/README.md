# Wheel Control Module

This module simulates the control of each wheel in a differential drive robot using a PID controller and a first-order motor dynamics model.

## Files

- `wheel_pid_controller.py`: PID control loop for individual wheel angular velocity.
- `wheel_dynamics.py`: First-order system modeling the wheel's response to PWM input.
- `wheel_simulation.py`: Example simulation using both classes with a reference signal.

## How It Works

1. `ω_ref` (reference angular velocity) is given for each wheel.
2. The `WheelPIDController` computes the PWM signal based on the error.
3. The `WheelDynamics` simulates how the wheel responds to the applied PWM.
4. The results (angular velocity, PWM, and error) are plotted.

## Usage
Run `wheel_simulation.py` to visualize the closed-loop control response of both wheels.

## Example Plot
- Desired vs measured wheel angular velocities
- Applied PWM signals
- Error over time

## Next Steps
You can modify the reference (`ω_ref`) to test:
- Step inputs
- Sinusoids
- Square or ramp waves

You can also test the controller with:
- Asymmetric parameters (different τ per wheel)
- Disturbances or saturation
- Different gain tuning for PID
