import tkinter as tk
from tkinter import ttk
import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class MotorPIDController:
    def __init__(self, master):
        self.master = master
        master.title("Motor PID Control")
        
        # Serial Connection
        self.ser = None
        self.connect_serial()
        
        # Data Storage
        self.time_data = []
        self.rpm_data = []
        self.setpoint = 0
        
        # Create GUI Components
        self.create_widgets()
        
        # Start Serial Reading Thread
        self.stop_thread = False
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Create Real-time Plot
        self.setup_plot()
    
    def connect_serial(self):
        try:
            self.ser = serial.Serial('COM3', 9600, timeout=1)
        except Exception as e:
            print(f"Serial connection error: {e}")
    
    def create_widgets(self):
        # PID Tuning Frame
        pid_frame = ttk.LabelFrame(self.master, text="PID Tuning")
        pid_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        # Kp Tuning
        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0)
        self.kp_scale = ttk.Scale(pid_frame, from_=0, to=1, orient=tk.HORIZONTAL, 
                                   command=lambda val: self.update_pid_param('Kp', val))
        self.kp_scale.set(0.18)
        self.kp_scale.grid(row=0, column=1)
        self.kp_entry = ttk.Entry(pid_frame, width=10)
        self.kp_entry.insert(0, "0.18")
        self.kp_entry.grid(row=0, column=2)
        self.kp_entry.bind("<Return>", lambda e: self.update_from_entry('Kp', self.kp_entry, self.kp_scale))
        
        # Ki Tuning
        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=0)
        self.ki_scale = ttk.Scale(pid_frame, from_=0, to=1, orient=tk.HORIZONTAL, 
                                   command=lambda val: self.update_pid_param('Ki', val))
        self.ki_scale.set(0.1)
        self.ki_scale.grid(row=1, column=1)
        self.ki_entry = ttk.Entry(pid_frame, width=10)
        self.ki_entry.insert(0, "0.10")
        self.ki_entry.grid(row=1, column=2)
        self.ki_entry.bind("<Return>", lambda e: self.update_from_entry('Ki', self.ki_entry, self.ki_scale))
        
        # Kd Tuning
        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0)
        self.kd_scale = ttk.Scale(pid_frame, from_=0, to=0.1, orient=tk.HORIZONTAL, 
                                   command=lambda val: self.update_pid_param('Kd', val))
        self.kd_scale.set(0.01)
        self.kd_scale.grid(row=2, column=1)
        self.kd_entry = ttk.Entry(pid_frame, width=10)
        self.kd_entry.insert(0, "0.01")
        self.kd_entry.grid(row=2, column=2)
        self.kd_entry.bind("<Return>", lambda e: self.update_from_entry('Kd', self.kd_entry, self.kd_scale))
        
        # Motor Control Frame
        motor_frame = ttk.LabelFrame(self.master, text="Motor Control")
        motor_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        
        # RPM Setpoint
        ttk.Label(motor_frame, text="RPM Setpoint:").grid(row=0, column=0)
        self.rpm_entry = ttk.Entry(motor_frame)
        self.rpm_entry.grid(row=0, column=1)
        
        # Set RPM Button
        ttk.Button(motor_frame, text="Set RPM", 
                   command=self.set_rpm).grid(row=1, column=0, columnspan=2)
        
        # Direction Buttons
        ttk.Button(motor_frame, text="Clockwise", 
                   command=lambda: self.set_direction("CW")).grid(row=2, column=0)
        ttk.Button(motor_frame, text="Counter-Clockwise", 
                   command=lambda: self.set_direction("CCW")).grid(row=2, column=1)
        
        # Status Display
        self.status_var = tk.StringVar()
        ttk.Label(self.master, textvariable=self.status_var).grid(row=2, column=0)
        
        # Transient Response Frame
        response_frame = ttk.LabelFrame(self.master, text="Transient Response")
        response_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")
        
        self.response_text = tk.Text(response_frame, height=6, width=40)
        self.response_text.pack()

    def setup_plot(self):
        # Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(6,4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=1, rowspan=4)
        
        self.line, = self.ax.plot([], [], label='RPM')
        self.setpoint_line = self.ax.axhline(y=0, color='r', linestyle='--', label='Setpoint')
        self.ax.set_title('Motor RPM')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('RPM')
        self.ax.legend()
    
    def update_pid_param(self, param, value):
        if self.ser:
            command = f"{param}:{float(value)}\n"
            self.ser.write(command.encode())
        self.sync_entry_with_scale(param, value)

    def update_from_entry(self, param, entry, scale):
        try:
            value = float(entry.get())
            scale.set(value)
            self.update_pid_param(param, value)
        except ValueError:
            self.status_var.set(f"Invalid {param} value")
    
    def sync_entry_with_scale(self, param, value):
        if param == 'Kp':
            self.kp_entry.delete(0, tk.END)
            self.kp_entry.insert(0, f"{float(value):.2f}")
        elif param == 'Ki':
            self.ki_entry.delete(0, tk.END)
            self.ki_entry.insert(0, f"{float(value):.2f}")
        elif param == 'Kd':
            self.kd_entry.delete(0, tk.END)
            self.kd_entry.insert(0, f"{float(value):.2f}")

    def set_rpm(self):
        try:
            rpm = int(self.rpm_entry.get())
            if self.ser:
                command = f"RPM:{rpm}\n"
                self.ser.write(command.encode())
                self.setpoint = rpm
                self.setpoint_line.set_ydata([rpm, rpm])
                self.canvas.draw()
        except ValueError:
            self.status_var.set("Invalid RPM value")
    
    def set_direction(self, direction):
        if self.ser:
            command = f"DIR:{direction}\n"
            self.ser.write(command.encode())
    
    def read_serial_data(self):
        while not self.stop_thread:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    
                    if line.startswith("RPM:"):
                        rpm = float(line.split(":")[1])
                        self.time_data.append(len(self.time_data) * 0.1)  # Assuming 0.1s intervals
                        self.rpm_data.append(rpm)
                        
                        # Update plot
                        self.update_plot()
                        
                        # Update transient response
                        self.update_transient_response()
                        
                        # Update status
                        self.status_var.set(f"Current RPM: {rpm}")
                    
                except Exception as e:
                    print(f"Serial reading error: {e}")
    
    def update_transient_response(self):
        if not self.rpm_data or self.setpoint == 0:
            return
        
        data = np.array(self.rpm_data)
        time = np.array(self.time_data)
        setpoint = self.setpoint
        
        # Overshoot
        overshoot = (np.max(data) - setpoint) / setpoint * 100
        
        # Peak time
        peak_time = time[np.argmax(data)]
        
        # Rise time (10% to 90% of setpoint)
        rise_start = np.where(data >= 0.1 * setpoint)[0]
        rise_end = np.where(data >= 0.9 * setpoint)[0]
        rise_time = (time[rise_end[0]] - time[rise_start[0]]) if len(rise_start) > 0 and len(rise_end) > 0 else None
        
        # Settling time (within 5% of setpoint)
        settling_idx = np.where(np.abs(data - setpoint) <= 0.05 * setpoint)[0]
        settling_time = time[settling_idx[-1]] if len(settling_idx) > 0 else None
        
        # Steady-state error
        steady_error = setpoint - data[-1]
        
        # Update transient response display
        self.response_text.delete("1.0", tk.END)
        self.response_text.insert(tk.END, f"Overshoot: {overshoot:.2f}%\n")
        self.response_text.insert(tk.END, f"Peak Time: {peak_time:.2f}s\n")
        self.response_text.insert(tk.END, f"Rise Time: {rise_time:.2f}s\n" if rise_time else "Rise Time: N/A\n")
        self.response_text.insert(tk.END, f"Settling Time: {settling_time:.2f}s\n" if settling_time else "Settling Time: N/A\n")
        self.response_text.insert(tk.END, f"Steady-state Error: {steady_error:.2f}\n")
    
    def update_plot(self):
        self.line.set_data(self.time_data, self.rpm_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
    
    def __del__(self):
        self.stop_thread = True
        if self.ser:
            self.ser.close()

def main():
    root = tk.Tk()
    app = MotorPIDController(root)
    root.mainloop()

if __name__ == '__main__':
    main()
