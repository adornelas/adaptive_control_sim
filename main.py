from plots.plot import SimulationPlotter, FirmwareController

sim = SimulationPlotter(T=10, dt=0.01, v_setpoint=1.0, w_setpoint=0.0, type_v='step', type_w='step')
sim.plot(firmware_like=True ,v=True, w=True, pwm=True, error=True)