import socket
import time
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION
# ==========================================
ESP_IP = "172.20.10.10"   # <--- REPLACE WITH YOUR ESP32 IP ADDRESS
PORT = 8080
DURATION = 3.0            # Seconds to record
SAMPLING_RATE = 416       # Hz (samples per second)

# ==========================================
# MAIN EXECUTION
# ==========================================
def main():
    data_buffer = [] 
    dt = 1.0 / SAMPLING_RATE  # Time step between samples (~0.0024s)

    print(f"Connecting to {ESP_IP}:{PORT}...")

    try:
        # 1. CONNECT
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((ESP_IP, PORT))
        print(f"Connected! Recording {DURATION} seconds...")

        # 2. RECORD LOOP
        start_record_time = time.time()
        socket_buffer = ""
        
        while (time.time() - start_record_time) < DURATION:
            try:
                chunk = s.recv(4096).decode('utf-8', errors='ignore')
                if not chunk: break
                
                socket_buffer += chunk
                
                # Parse complete lines
                while "\n" in socket_buffer:
                    line, socket_buffer = socket_buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        try:
                            # Parse CSV: ax, ay, az, gx, gy, gz
                            parts = line.split(',')
                            if len(parts) == 6:
                                row = [float(x) for x in parts]
                                data_buffer.append(row)
                        except ValueError:
                            pass
            except socket.timeout:
                print("Socket timed out.")
                break

        s.close()
        
        count = len(data_buffer)
        print(f"Finished! Captured {count} samples.")
        print(f"Time resolution: {dt*1000:.2f}ms per point.")

        # 3. GENERATE TIME DOMAIN DATA
        if count == 0:
            print("No data received.")
            return

        # Generate X-axis (Time in seconds)
        # [0.0, 0.0024, 0.0048, ... 3.0]
        time_axis = [i * dt for i in range(count)]

        # Unpack Y-axis data
        ax_data = [row[0] for row in data_buffer]
        ay_data = [row[1] for row in data_buffer]
        az_data = [row[2] for row in data_buffer]
        gx_data = [row[3] for row in data_buffer]
        gy_data = [row[4] for row in data_buffer]
        gz_data = [row[5] for row in data_buffer]

        # 4. PLOT
        fig, (plot1, plot2) = plt.subplots(2, 1, figsize=(12, 8))
        plt.subplots_adjust(hspace=0.4)

        # Accelerometer Plot
        plot1.plot(time_axis, ax_data, 'r', label='Acc X', linewidth=1)
        plot1.plot(time_axis, ay_data, 'g', label='Acc Y', linewidth=1)
        plot1.plot(time_axis, az_data, 'b', label='Acc Z', linewidth=1)
        plot1.set_title(f"Accelerometer ({count} samples @ {SAMPLING_RATE}Hz)")
        plot1.set_ylabel("Amplitude (m/s^2)")
        plot1.set_xlabel("Time (s)")
        plot1.set_xlim(0, max(time_axis))  # Lock x-axis to actual data time
        plot1.legend(loc="upper right")
        plot1.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Gyroscope Plot
        plot2.plot(time_axis, gx_data, 'r', label='Gyro X', linewidth=1)
        plot2.plot(time_axis, gy_data, 'g', label='Gyro Y', linewidth=1)
        plot2.plot(time_axis, gz_data, 'b', label='Gyro Z', linewidth=1)
        plot2.set_title("Gyroscope")
        plot2.set_ylabel("Amplitude (rad/s)")
        plot2.set_xlabel("Time (s)")
        plot2.set_xlim(0, max(time_axis))
        plot2.legend(loc="upper right")
        plot2.grid(True, which='both', linestyle='--', linewidth=0.5)

        print("Displaying time-domain plot...")
        plt.show()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()