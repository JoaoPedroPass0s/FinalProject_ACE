import socket

# Replace with the Pico W's IP address from Serial Monitor
PICO_IP = "192.168.1.7"  # Replace with your Pico W's IP address
PORT = 80

def main():
    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            # Connect to the Pico W's server
            print(f"Connecting to {PICO_IP}:{PORT}...")
            client_socket.connect((PICO_IP, PORT))
            print(f"Connected to {PICO_IP}:{PORT}")

            # Receive data from the server
            while True:
                data = client_socket.recv(1024)  # Buffer size
                if not data:
                    print("Disconnected from server.")
                    break
                # Decode and print the received data
                print("", data.decode('utf-8').strip())

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
