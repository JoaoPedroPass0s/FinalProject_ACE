import socket
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PICO_IP = "192.168.1.2"  # Replace with your Pico W's IP address
PORT = 80

def update_display(data, labels):
    """
    Updates the labels on the GUI with the received data.
    """
    lines = data.split("\n")
    for i, line in enumerate(lines):
        if i < len(labels):
            labels[i].config(text=line)

def send_pid_values(client_socket, Kp, Ki, Kd, vel):
    """
    Sends the PID values to the Pico W over the socket connection.
    """
    try:
        pid_data = f"{Kp},{Ki},{Kd},{vel}\n"  # Format the PID values as a string
        client_socket.send(pid_data.encode())  # Send the data to the Pico W
        print(f"Sent PID values: Kp={Kp}, Ki={Ki}, Kd={Kd}, Velocity={vel}")
    except Exception as e:
        print(f"Error sending PID values: {e}")

def main():
    # Create the main window
    root = tk.Tk()
    root.title("Pico W Data Display and PID Graph")

    # Use a PanedWindow to split the GUI into two sections: left and right
    main_pane = tk.PanedWindow(root, orient="horizontal")
    main_pane.pack(fill="both", expand=True)

    # Left frame for info and PID control
    left_frame = tk.Frame(main_pane, padx=10, pady=10)
    main_pane.add(left_frame)

    # Right frame for the graph
    right_frame = tk.Frame(main_pane, padx=10, pady=10)
    main_pane.add(right_frame)

    # Create labels for each line of data in the left frame
    labels = [tk.Label(left_frame, text="", font=("Arial", 14), anchor="w") for _ in range(6)]
    for label in labels:
        label.pack(fill="x", pady=2)

    # Create sliders for Kp, Ki, Kd, and Velocity
    Kp_var = tk.DoubleVar(value=75.0)
    Ki_var = tk.DoubleVar(value=0.68)
    Kd_var = tk.DoubleVar(value=0.9)
    Vel_var = tk.DoubleVar(value=2.0)

    Kp_slider = tk.Scale(left_frame, from_=0, to_=150, resolution=0.01, orient="horizontal", label="Kp", variable=Kp_var)
    Kp_slider.pack(fill="x", pady=5)

    Ki_slider = tk.Scale(left_frame, from_=0, to_=3, resolution=0.01, orient="horizontal", label="Ki", variable=Ki_var)
    Ki_slider.pack(fill="x", pady=5)

    Kd_slider = tk.Scale(left_frame, from_=0, to_=10, resolution=0.01, orient="horizontal", label="Kd", variable=Kd_var)
    Kd_slider.pack(fill="x", pady=5)

    Vel_slider = tk.Scale(left_frame, from_=0, to_=5, resolution=0.01, orient="horizontal", label="Velocity", variable=Vel_var)
    Vel_slider.pack(fill="x", pady=5)

    # Create a matplotlib figure for the graph in the right frame
    fig = Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111)
    ax.set_title("PID Error Over Time")
    ax.set_xlabel("Time (samples)")
    ax.set_ylabel("PID Error")
    line, = ax.plot([], [], "r-", label="PID Error")
    ax.legend()

    canvas = FigureCanvasTkAgg(fig, master=right_frame)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill="both", expand=True)

    # Variables for graph data
    pid_error_values = []
    max_points = 100

    def update_graph(new_error):
        pid_error_values.append(new_error)
        if len(pid_error_values) > max_points:
            pid_error_values.pop(0)
        line.set_data(range(len(pid_error_values)), pid_error_values)
        ax.relim()
        ax.autoscale_view()
        canvas.draw()

    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            # Connect to the Pico W's server
            print(f"Connecting to {PICO_IP}:{PORT}...")
            client_socket.connect((PICO_IP, PORT))
            print(f"Connected to {PICO_IP}:{PORT}")

            # Function to send updated PID values
            def pid_updated(*args):
                send_pid_values(client_socket, Kp_var.get(), Ki_var.get(), Kd_var.get(), Vel_var.get())

            Kp_var.trace_add("write", pid_updated)
            Ki_var.trace_add("write", pid_updated)
            Kd_var.trace_add("write", pid_updated)
            Vel_var.trace_add("write", pid_updated)

            # Function to receive data from the server
            def receive_data():
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        print("Disconnected from server.")
                        root.quit()
                        return
                    decoded_data = data.decode("utf-8").strip()
                    print(decoded_data)
                    update_display(decoded_data, labels)

                    # Extract PID error from the received data
                    for line in decoded_data.split("\n"):
                        if "Error:" in line:
                            try:
                                error_value = float(line.split(":")[1].strip())
                                update_graph(error_value)
                            except ValueError:
                                print(f"Error parsing PID error: {line}")
                    root.after(100, receive_data)
                except Exception as e:
                    print(f"Error: {e}")
                    root.quit()

            receive_data()
            send_pid_values(client_socket, Kp_var.get(), Ki_var.get(), Kd_var.get(), Vel_var.get())
            root.mainloop()

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
