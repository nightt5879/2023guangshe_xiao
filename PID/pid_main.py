import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.error_prev = 0.0

    def reset(self):
        self.error_sum = 0.0
        self.error_prev = 0.0

    def update(self, error, dt):
        self.error_sum += error * dt
        error_rate = (error - self.error_prev) / dt
        self.error_prev = error
        return self.kp * error + self.ki * self.error_sum + self.kd * error_rate

class System:
    def __init__(self, x0=0.0):
        self.x = x0

    def reset(self, x0=0.0):
        self.x = x0

    def update(self, u, dt):
        self.x += u * dt
        return self.x

class Simulation:
    def __init__(self, controller, system, target):
        self.controller = controller
        self.system = system
        self.target = target
        self.reset()

    def reset(self):
        self.time = [0.0]
        self.target_history = [self.target]
        self.state_history = [self.system.x]
        self.controller.reset()
        self.system.reset()

    def run(self, duration, dt=0.01):
        time = self.time[-1]
        while time < duration:
            time += dt
            self.time.append(time)
            error = self.target - self.system.x
            u = self.controller.update(error, dt)
            self.system.update(u, dt)
            self.target_history.append(self.target)
            self.state_history.append(self.system.x)

    def plot(self, ax=None):
        if ax is None:
            ax = plt.gca()
        ax.plot(self.time, self.target_history, label='target')
        ax.plot(self.time, self.state_history, label='state')
        ax.legend()

class GUI:
    def __init__(self, simulation):
        self.simulation = simulation
        self.root = tk.Tk()

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=3)

        self.slider_kp = tk.Scale(self.root, from_=0, to=2, resolution=0.01, length=600, orient=tk.HORIZONTAL, label='Kp')
        self.slider_kp.grid(row=2, column=0)

        self.slider_ki = tk.Scale(self.root, from_=0, to=2, resolution=0.01, length=600, orient=tk.HORIZONTAL, label='Ki')
        self.slider_ki.grid(row=2, column=1)

        self.slider_kd = tk.Scale(self.root, from_=0, to=2, resolution=0.01, length=600, orient=tk.HORIZONTAL, label='Kd')
        self.button = tk.Button(self.root, text='Run', command=self.run)
        self.button.grid(row=0, column=2)

    def update_plot(self):
        self.ax.clear()
        self.simulation.plot(self.ax)
        self.canvas.draw()

    def run(self):
        kp = self.slider_kp.get()
        ki = self.slider_ki.get()
        kd = self.slider_kd.get()
        self.simulation.controller.kp = kp
        self.simulation.controller.ki = ki
        self.simulation.controller.kd = kd
        self.simulation.run(duration=10)
        self.update_plot()

if __name__ == '__main__':
    controller = PIDController()
    system = System()
    simulation = Simulation(controller, system, target=1.0)
    gui = GUI(simulation)
    gui.root.mainloop()