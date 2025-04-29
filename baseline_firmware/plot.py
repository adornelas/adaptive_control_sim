import matplotlib.pyplot as plt
import numpy as np
from controllers.pid import PIDController
from baseline_firmware.firmware_controller import FirmwareController
from models.robot_model import RobotModel

class SimulationPlotter:
    def __init__(self, T=10, dt=0.01, v_setpoint=1.0, w_setpoint=0.0, type_v='step', type_w='step'):
        self.T = T
        self.dt = dt
        self.v_setpoint = v_setpoint
        self.w_setpoint = w_setpoint
        self.type_v = type_v
        self.type_w = type_w

    def run_simulation(self):
        steps = int(self.T / self.dt)
        time = np.arange(0, self.T, self.dt)

        v_ref = self.generate_reference(time, type=self.type_v, amplitude=self.v_setpoint)
        w_ref = self.generate_reference(time, type=self.type_w, amplitude=self.w_setpoint)

        v_measured = np.zeros(steps)
        w_measured = np.zeros(steps)

        v_error = np.zeros(steps)
        w_error = np.zeros(steps)

        pwm_r = np.zeros(steps)
        pwm_l = np.zeros(steps)

        robot = RobotModel()
        v_pid = PIDController(kp=1.0, ki=1.1, kd=0.05, imax=2.0)
        w_pid = PIDController(kp=1.0, ki=1.1, kd=0.05, imax=2.0)

        for t in range(1, steps):
            v_error[t] = v_ref[t] - v_measured[t-1]
            w_error[t] = w_ref[t] - w_measured[t-1]

            v_cmd = v_pid.update(v_error[t], self.dt)
            w_cmd = w_pid.update(w_error[t], self.dt)

            pwm_r[t] = np.clip(v_cmd + w_cmd, -1.0, 1.0)
            pwm_l[t] = np.clip(v_cmd - w_cmd, -1.0, 1.0)

            robot.update(pwm_r[t], pwm_l[t], self.dt)
            v_measured[t] = robot.v
            w_measured[t] = robot.w

        return time, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l
    
    def run_simulation_firmware(self):
        steps = int(self.T / self.dt)
        time = np.arange(0, self.T, self.dt)

        v_ref = self.generate_reference(time, type=self.type_v, amplitude=self.v_setpoint)
        w_ref = self.generate_reference(time, type=self.type_w, amplitude=self.w_setpoint)

        v_measured = np.zeros(steps)
        w_measured = np.zeros(steps)

        v_error = np.zeros(steps)
        w_error = np.zeros(steps)

        pwm_r = np.zeros(steps)
        pwm_l = np.zeros(steps)

        robot = RobotModel()
        # Constants from the original firmware
        w_pid = FirmwareController(kp=2.70, ki=0.3, kd=-0.03, imax=64.0)

        for t in range(1, steps):
            v_error[t] = v_ref[t] - v_measured[t-1]
            w_error[t] = w_ref[t] - w_measured[t-1]

            v_cmd, w_cmd = w_pid.update(v_ref=v_ref[t], w_error=w_error[t], dt= self.dt)

            pwm_r[t] = np.clip(v_cmd, -1.0, 1.0)
            pwm_l[t] = np.clip(w_cmd, -1.0, 1.0)

            robot.update(pwm_r[t], pwm_l[t], self.dt)
            v_measured[t] = robot.v
            w_measured[t] = robot.w
            
        return time, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l

    def plot(self, v=True, w=True, pwm=True, error=True, firmware_like=True):
        if firmware_like:
            time, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l = self.run_simulation_firmware()
        else:
            time, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l = self.run_simulation()

        config = {
            'v_plot': v,
            'w_plot': w,
            'error_plot': error,
            'pwm_plot': pwm
        }
        self.plot_all(time, config, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l)

    def plot_response(self, time, data, labels, title, ylabel):
        for y, label in zip(data, labels):
            linestyle = '--' if 'ref' in label else '-'  # dashed line for reference signals
            plt.plot(time, y, label=label, linestyle=linestyle)
        plt.title(title)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid()

    def plot_all(self, time, config, v_ref, v_measured, w_ref, w_measured, v_error, w_error, pwm_r, pwm_l):
        plots = sum(config.values())
        plt.figure(figsize=(10, 3 * plots))
        plot_idx = 1

        if config['v_plot']:
            plt.subplot(plots, 1, plot_idx)
            self.plot_response(time, [v_ref, v_measured], ['v_ref', 'v_measured'], 'Linear Velocity', 'v (m/s)')
            plot_idx += 1

        if config['w_plot']:
            plt.subplot(plots, 1, plot_idx)
            self.plot_response(time, [w_ref, w_measured], ['w_ref', 'w_measured'], 'Angular Velocity', 'w (rad/s)')
            plot_idx += 1

        if config['error_plot']:
            plt.subplot(plots, 1, plot_idx)
            self.plot_response(time, [v_error, w_error], ['v_error', 'w_error'], 'Control Errors', 'Error')
            plot_idx += 1

        if config['pwm_plot']:
            plt.subplot(plots, 1, plot_idx)
            self.plot_response(time, [pwm_r, pwm_l], ['PWM Right', 'PWM Left'], 'Control Signals', 'PWM')
            plot_idx += 1

        plt.xlabel('Time (s)')
        plt.tight_layout()
        plt.show()

    def generate_reference(self, time, type='step', amplitude=1.0, frequency=1.0):
        if type == 'step':
            return amplitude * np.ones_like(time)
        elif type == 'square':
            return amplitude * np.sign(np.sin(2 * np.pi * frequency * time))
        elif type == 'sawtooth':
            return amplitude * (2 * (time * frequency - np.floor(time * frequency + 0.5)))
        elif type == 'sinusoid':
            return amplitude * np.sin(2 * np.pi * frequency * time)
        else:
            raise ValueError("Unknown reference type")
