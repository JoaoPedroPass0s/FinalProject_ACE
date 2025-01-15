import socket
import tkinter as tk

# Replace with the Pico W's IP address from Serial Monitor
PICO_IP = "192.168.1.7"  # Replace with your Pico W's IP address
PORT = 80

def update_display(data, labels):
    """
    Updates the labels on the GUI with the received data.
    """
    lines = data.split("\n")
    for i, line in enumerate(lines):
        if i < len(labels):
            labels[i].config(text=line)

def send_pid_values(client_socket, Kp, Ki, Kd,vel):
    """
    Sends the PID values to the Pico W over the socket connection.
    """
    try:
        pid_data = f"{Kp},{Ki},{Kd},{vel}\n"  # Format the PID values as a string
        client_socket.send(pid_data.encode())  # Send the data to the Pico W
        print(f"Sent PID values: Kp={Kp}, Ki={Ki}, Kd={Kd}")
        print(f"Send velocity: {vel}")
    except Exception as e:
        print(f"Error sending PID values: {e}")

def main():
    # Create the GUI
    root = tk.Tk()
    root.title("Pico W Data Display")

    # Create labels for each line of data
    labels = [tk.Label(root, text="", font=("Arial", 14), anchor="w") for _ in range(6)]
    for label in labels:
        label.pack(fill="x", padx=10, pady=5)

    # Create sliders for Kp, Ki, and Kd
    Kp_var = tk.DoubleVar(value=75.0)
    Ki_var = tk.DoubleVar(value=0.68)
    Kd_var = tk.DoubleVar(value=0.9)
    Vel_var = tk.DoubleVar(value=2.0)

    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            # Connect to the Pico W's server
            print(f"Connecting to {PICO_IP}:{PORT}...")
            client_socket.connect((PICO_IP, PORT))
            print(f"Connected to {PICO_IP}:{PORT}")

            # Function to send updated PID values
            def pid_updated(*args):
                send_pid_values(client_socket, Kp_var.get(), Ki_var.get(), Kd_var.get(),Vel_var.get())

            # Trace changes to the variables
            Kp_var.trace_add("write", pid_updated)
            Ki_var.trace_add("write", pid_updated)
            Kd_var.trace_add("write", pid_updated)
            Vel_var.trace_add("write",pid_updated)

            Kp_slider = tk.Scale(root, from_=0, to_=200, resolution=0.01, orient="horizontal", label="Kp", variable=Kp_var)
            Kp_slider.pack(fill="x", padx=10, pady=5)

            Ki_slider = tk.Scale(root, from_=0, to_=20, resolution=0.01, orient="horizontal", label="Ki", variable=Ki_var)
            Ki_slider.pack(fill="x", padx=10, pady=5)

            Kd_slider = tk.Scale(root, from_=0, to_=20, resolution=0.01, orient="horizontal", label="Kd", variable=Kd_var)
            Kd_slider.pack(fill="x", padx=10, pady=5)

            Vel_slider = tk.Scale(root, from_=0, to_=5, resolution=0.01, orient="horizontal", label="Velocity", variable=Vel_var)
            Vel_slider.pack(fill="x", padx=10, pady=5)

            # Receive data from the server
            def receive_data():
                try:
                    data = client_socket.recv(1024)  # Buffer size
                    if not data:
                        print("Disconnected from server.")
                        root.quit()
                        return
                    # Decode and update the display
                    decoded_data = data.decode('utf-8').strip()
                    print(decoded_data)  # For debugging
                    update_display(decoded_data, labels)
                    root.after(100, receive_data)  # Schedule the next read
                except Exception as e:
                    print(f"Error: {e}")
                    root.quit()

            receive_data()
            # Initial send of PID values to Pico W
            send_pid_values(client_socket, Kp_var.get(), Ki_var.get(), Kd_var.get(),Vel_var.get())
            root.mainloop()

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
