# Adaptive Control Simulation for Mobile Robots

This repository contains simulation scripts to test linear velocity control using:
- Classic PID controllers
- Adaptive PID strategies

## Features

- Simulated first-order model of a differential-drive robot
- Sensor noise modeling (for encoder/IMU)
- Real-time parameter adaptation
- Visualization of system response and control signals

## Using the simulations with virtual environment

```bash
git clone https://github.com/adornelas/adaptive_control_sim.git
cd adaptive_control_sim
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
python main.py