import socket
import time
import csv
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION
# ==========================================
ESP_IP = "172.20.10.10"   # <--- REPLACE WITH YOUR IP
PORT = 8080
DURATION = 5.0             # Seconds to record
FILENAME = "imu_data.csv"  # Output file name
SAMPLE_RATE = 416          # Hz (Used for creating the time axis on the plot)

# ==========================================
# MAIN SCRIPT
# ==========================================
def main():
    print(f"Connecting to {ESP_IP}:{PORT}...")
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0) # Timeout if connection fails
        s.connect((ESP_IP, PORT))
        print("Connected! Starting recording...")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Storage for data
    # Structure: [ [ax, ay, az, gx, gy, gz], ... ]
    recorded_data = []
    
    start_time = time.time()
    buffer = ""
    
    try:
        # Loop until the duration has passed
        while (time.time() - start_time) < DURATION:
            
            try:
                # Receive data
                chunk = s.recv(4096).decode('utf-8', errors='ignore')
                
                if not chunk:
                    break
                
                buffer += chunk
                
                # Process buffer for newlines
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    
                    if line:
                        try:
                            parts = line.split(',')
                            if len(parts) == 6:
                                # Parse floats
                                row = [float(x) for x in parts]
                                recorded_data.append(row)
                        except ValueError:
                            # Skip malformed lines
                            continue
                            
            except socket.timeout:
                # If no data comes for a while, just continue checking time
                continue
                
    except KeyboardInterrupt:
        print("Recording stopped by user.")
    finally:
        s.close()
        print(f"Recording finished. captured {len(recorded_data)} samples.")

    # ==========================================
    # 1. SAVE TO CSV
    # ==========================================
    if recorded_data:
        print(f"Saving to {FILENAME}...")
        with open(FILENAME, 'w', newline='') as f:
            writer = csv.writer(f)
            # Write Header
            writer.writerow(["Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z"])
            # Write Data
            writer.writerows(recorded_data)
        print("Save complete.")

        # ==========================================
        # 2. PLOT DATA
        # ==========================================
        print("Generating Plot...")
        
        # Unpack columns for plotting
        # recorded_data is list of rows, we need list of columns
        # zip(*recorded_data) transposes the matrix
        columns = list(zip(*recorded_data))
        
        ax_data = columns[0]
        ay_data = columns[1]
        az_data = columns[2]
        gx_data = columns[3]
        gy_data = columns[4]
        gz_data = columns[5]
        
        # Create Time Axis based on Sample Rate
        # t = sample_index / frequency
        t_axis = [i / SAMPLE_RATE for i in range(len(recorded_data))]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Plot Accelerometer
        ax1.plot(t_axis, ax_data, label='X', color='r', linewidth=1)
        ax1.plot(t_axis, ay_data, label='Y', color='g', linewidth=1)
        ax1.plot(t_axis, az_data, label='Z', color='b', linewidth=1)
        ax1.set_title(f"Accelerometer Data ({len(recorded_data)} samples)")
        ax1.set_ylabel("m/s^2")
        ax1.grid(True)
        ax1.legend(loc="upper right")
        
        # Plot Gyroscope
        ax2.plot(t_axis, gx_data, label='X', color='r', linewidth=1)
        ax2.plot(t_axis, gy_data, label='Y', color='g', linewidth=1)
        ax2.plot(t_axis, gz_data, label='Z', color='b', linewidth=1)
        ax2.set_title("Gyroscope Data")
        ax2.set_ylabel("rad/s")
        ax2.set_xlabel("Time (seconds)")
        ax2.grid(True)
        ax2.legend(loc="upper right")

        plt.tight_layout()
        plt.show()
        
    else:
        print("No data was recorded.")

if __name__ == "__main__":
    main()