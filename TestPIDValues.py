import optuna
import socket
import time

PICO_IP = "192.168.1.5"  # Replace with your Pico W's IP address
PORT = 80

# Global variables
total_error = 0
total_velocity = 0
error_count = 0
velocity_count = 0
last_time = time.time()  # Keep track of last time

# Define the Optuna optimization function
def objective(trial):
    global total_error, total_velocity, error_count, velocity_count, last_time

    # Suggest PID values
    P = trial.suggest_uniform('P', 20.0, 70.0)
    I = trial.suggest_uniform('I', 0.0, 10.0)
    D = trial.suggest_uniform('D', 0.0, 20.0)

    # Send PID values to the robot
    send_pid_values(client_socket, P, I, D, velocity=2.5)
    print(f"Testing PID: Kp={P:.2f}, Ki={I:.2f}, Kd={D:.2f}")

    # Reset accumulated values for each test
    total_error = 0
    total_velocity = 0
    error_count = 0
    velocity_count = 0

    # Start the test and collect data for 2 seconds
    start_time = time.time()
    while time.time() - start_time < 10:  # Run the test for 2 seconds
        receive_data()

    # Calculate mean error and velocity after 2 seconds
    mean_error = total_error / error_count if error_count > 0 else 0
    mean_velocity = total_velocity / velocity_count if velocity_count > 0 else 0

    # Print and calculate the objective
    print(f"Mean error: {mean_error:.4f}, Mean velocity: {mean_velocity:.4f}")
    objective_value = mean_error - mean_velocity * 50.0  # Adjust as necessary
    print(f"Objective value: {objective_value:.4f}")

    # Return the objective value for optimization (can be adjusted to suit your needs)
    return objective_value

# Function to send PID values to the robot
def send_pid_values(client_socket, Kp, Ki, Kd, velocity):
    try:
        pid_data = f"{Kp},{Ki},{Kd},{velocity}\n"
        client_socket.send(pid_data.encode())
        print(f"Sent PID values: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}, Velocity={velocity:.2f}")
    except Exception as e:
        print(f"Error sending PID values: {e}")

# Function to receive data from the robot
def receive_data():
    global total_error, total_velocity, error_count, velocity_count
    try:
        # Receive data from the robot
        data = client_socket.recv(1024)
        if not data:
            print("Disconnected from server.")
            exit()

        # Decode and process data
        decoded_data = data.decode("utf-8").strip()
        #print(f"Raw data received: {decoded_data}")

        # Extract PID error from the received data
        for line in decoded_data.split("\n"):
            if "Error:" in line:
                try:
                    error_value = float(line.split(":")[1].strip())
                    total_error += abs(error_value)
                    error_count += 1  # Increment error count
                except ValueError:
                    print(f"Error parsing PID error: {line}")
            elif "ve:" in line:
                try:
                    velocity_value = float(line.split(":")[1].strip())
                    total_velocity += velocity_value
                    velocity_count += 1  # Increment velocity count
                except ValueError:
                    print(f"Error parsing velocity: {line}")
    except Exception as e:
        print(f"Error in receive_data: {e}")
        exit()

# Main function
if __name__ == "__main__":
    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            # Connect to the Pico W
            print(f"Connecting to {PICO_IP}:{PORT}...")
            client_socket.connect((PICO_IP, PORT))
            print(f"Connected to {PICO_IP}:{PORT}")

            # Create an Optuna study
            study = optuna.create_study(direction="minimize")
            print("Starting PID optimization...")

            # Start the optimization loop
            study.optimize(objective, n_trials=15)

            # Print the best PID values
            print("Optimization completed!")
            print("Best PID values:")
            print(study.best_params)

        except Exception as e:
            print(f"Error: {e}")
