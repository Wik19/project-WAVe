import socket
import struct
import time
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
ESP_IP = "172.20.10.10"
PORT = 8080
DURATION = 10             # Seconds to record
PACKET_FORMAT = "<6h"     # Little-endian, 6 x 16-bit integers
PACKET_SIZE = struct.calcsize(PACKET_FORMAT) # 12 bytes

# --- SENSITIVITY CONSTANTS (For Reference) ---
# 2g Scale   = 0.061 mg/LSB
# 250dps Scale = 8.75 mdps/LSB
SCALE_ACC = 0.061 / 1000.0  # Convert LSB to g
SCALE_GYR = 8.75 / 1000.0   # Convert LSB to dps

def record_data(duration):
    data_points = []
    print(f"Connecting to {ESP_IP}:{PORT}...")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((ESP_IP, PORT))
        s.settimeout(None)
        
        # Disable Nagle to receive packets immediately
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        print(f"Connected! Recording for {duration} seconds...")
        start_time = time.time()
        byte_buffer = b""

        while (time.time() - start_time) < duration:
            try:
                chunk = s.recv(4096)
                if not chunk: break
                byte_buffer += chunk

                while len(byte_buffer) >= PACKET_SIZE:
                    packet = byte_buffer[:PACKET_SIZE]
                    byte_buffer = byte_buffer[PACKET_SIZE:]
                    
                    # Unpack: Accel X,Y,Z then Gyro X,Y,Z
                    ax, ay, az, gx, gy, gz = struct.unpack(PACKET_FORMAT, packet)
                    data_points.append((ax, ay, az, gx, gy, gz))

            except BlockingIOError:
                continue
                
    except Exception as e:
        print(f"Connection Error: {e}")
    finally:
        s.close()
        print(f"Done. Captured {len(data_points)} samples.")
        print(f"Average Sample Rate: {len(data_points)/duration:.1f} Hz")
    
    return data_points

def plot_data(data):
    if not data: return

    # Convert to NumPy array
    raw = np.array(data)
    
    # Create time axis (assuming ~416Hz)
    t = np.linspace(0, DURATION, len(raw))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Plot Accelerometer
    ax1.plot(t, raw[:, 0], label='X', color='#ff5555', linewidth=0.6)
    ax1.plot(t, raw[:, 1], label='Y', color='#55aa55', linewidth=0.6)
    ax1.plot(t, raw[:, 2], label='Z', color='#5555ff', linewidth=0.6)
    ax1.set_title(f"Accelerometer (Raw LSB) - {len(raw)} Samples")
    ax1.set_ylabel("Amplitude (LSB)")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Plot Gyroscope
    ax2.plot(t, raw[:, 3], label='X', color='#ff5555', linewidth=0.6)
    ax2.plot(t, raw[:, 4], label='Y', color='#55aa55', linewidth=0.6)
    ax2.plot(t, raw[:, 5], label='Z', color='#5555ff', linewidth=0.6)
    ax2.set_title("Gyroscope (Raw LSB)")
    ax2.set_ylabel("Amplitude (LSB)")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    data = record_data(DURATION)
    plot_data(data)