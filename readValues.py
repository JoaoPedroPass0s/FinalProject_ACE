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

def main():
    # Create the GUI
    root = tk.Tk()
    root.title("Pico W Data Display")

    # Create labels for each line of data
    labels = [tk.Label(root, text="", font=("Arial", 14), anchor="w") for _ in range(6)]
    for label in labels:
        label.pack(fill="x", padx=10, pady=5)

    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            # Connect to the Pico W's server
            print(f"Connecting to {PICO_IP}:{PORT}...")
            client_socket.connect((PICO_IP, PORT))
            print(f"Connected to {PICO_IP}:{PORT}")

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
            root.mainloop()

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
