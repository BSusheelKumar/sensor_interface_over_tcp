import tkinter as tk
from tkinter import messagebox
import subprocess









def call_service(service_name):
    try:
        
        
        result = subprocess.run(
            ["ros2", "service", "call", service_name, "std_srvs/srv/Trigger"],
            capture_output=True,
            text=True
        )
       
        if result.returncode == 0:
            response_lines = result.stdout.splitlines()
            for line in response_lines:
                if "success=True" in line:
                    messagebox.showinfo("Service Call", f"{service_name} executed successfully.")
                    return
            messagebox.showwarning("Service Call", f"{service_name} failed. Response: {result.stdout}")
        else:
            messagebox.showerror("Service Call", f"Error calling {service_name}.\n{result.stderr}")
    except Exception as e:
        messagebox.showerror("Service Call", f"Unexpected error: {str(e)}")



def set_parameter():
    try:

        param_value = param_value_entry.get()
      
        result = subprocess.run(
            ["ros2", "param", "set", "/tcp_client_node", "interval", param_value],
            capture_output=True,
            text=True
        )



        if result.returncode == 0:
            if "Set parameter successful" in result.stdout:
                messagebox.showinfo("Parameter Set", "Parameter set successfully.")
            else:
           
                messagebox.showwarning("Parameter Set", f"Failed to set parameter. Response: {result.stdout}")
        else:

            messagebox.showerror("Parameter Set", f"Error setting parameter.\n{result.stderr}")
    except Exception as e:
        messagebox.showerror("Parameter Set", f"Unexpected error: {str(e)}")


## initializing main application window
root = tk.Tk()
root.title("ROS2 Service and Parameter Control")
#





## start button
start_button = tk.Button(root, text="Start", command=lambda: call_service("/start_command"))
start_button.pack(pady=10)
#


##stop button
stop_button = tk.Button(root, text="Stop", command=lambda: call_service("/stop_command"))
stop_button.pack(pady=10)
#



##param 
param_frame = tk.Frame(root)
param_frame.pack(pady=20)
#


## param input frame
tk.Label(param_frame, text="Interval Value:").grid(row=0, column=0, padx=5, pady=5)
param_value_entry = tk.Entry(param_frame)
param_value_entry.grid(row=0, column=1, padx=5, pady=5)
#

##param set button 
param_set_button = tk.Button(param_frame, text="Set Parameter", command=set_parameter)
param_set_button.grid(row=1, column=0, columnspan=2, pady=10)
#




root.mainloop()
