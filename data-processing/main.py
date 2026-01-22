import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
import wave
import csv
import argparse
import sys
from scipy.fft import fft, fftfreq # Standard FFT library
import datetime
# ==========================================
# DEFAULT CONFIGURATION
# ==========================================
DEFAULT_PORT = 8080

# Settings
MIC_SAMPLE_RATE = 16000   
AUDIO_PAYLOAD   = 1024    

IMU_SAMPLE_RATE = 416
IMU_PAYLOAD     = 240     
IMU_STRUCT_FMT  = '<hhhhhh' 
ACC_SCALE  = 0.488 / 1000.0 * 9.80665 
GYRO_SCALE = 70.0  / 1000.0 * 0.017453 
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# Insert the timestamp into the filenames
OUTPUT_WAV = f"mixed_recording_{timestamp}.wav"
OUTPUT_CSV = f"imu_data_{timestamp}.csv"
SEQ_SIZE = 4 


# Add this before your plotting logic
plt.rcParams.update({
    'font.size': 14,          # Default text size
    'axes.titlesize': 18,     # Title size (e.g., "Accelerometer (Raw)")
    'axes.labelsize': 16,     # Axis label size (e.g., "m/s^2")
    'xtick.labelsize': 14,    # X-axis numbers size
    'ytick.labelsize': 14,    # Y-axis numbers size
    'legend.fontsize': 14     # Legend text size
})

# ==========================================
# HELPER
# ==========================================
def recv_exact(sock, n_bytes):
    data = bytearray()
    while len(data) < n_bytes:
        packet = sock.recv(n_bytes - len(data))
        if not packet: return None
        data.extend(packet)
    return data

def main():
    # --- 1. PARSE ARGUMENTS ---
    parser = argparse.ArgumentParser(description='ESP32 TCP Client')
    parser.add_argument('ip', type=str, help='IP Address of ESP32')
    parser.add_argument('duration', type=float, help='Duration in seconds')
    
    args = parser.parse_args()
    ESP_IP = args.ip
    DURATION = args.duration

    print("-" * 40)
    print(f"Target IP: {ESP_IP}")
    print(f"Duration:  {DURATION} seconds")
    print("-" * 40)

    # Data Storage
    audio_raw_bytes = bytearray()
    csv_rows = [] 
    imu_plot_data = [] 

    # Counters
    mic_recv_count = 0
    imu_recv_count = 0
    
    first_mic_packet = True
    first_imu_packet = True
    
    # Internal sequence tracking
    expected_mic_seq = 0
    expected_imu_seq = 0

    print(f"Connecting to {ESP_IP}:{DEFAULT_PORT}...")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0) 
        s.connect((ESP_IP, DEFAULT_PORT))
        print(f"Connected! Recording to {OUTPUT_CSV}...")
        
        start_time = time.time()
        last_print_time = start_time
        
        # --- RECORDING LOOP ---
        while True:
            elapsed = time.time() - start_time
            if elapsed >= DURATION:
                print("\nTime limit reached.")
                break
            
            if time.time() - last_print_time > 1.0:
                print(f"Recording... [ {elapsed:.1f}s / {DURATION:.1f}s ]\r", end="")
                last_print_time = time.time()

            try:
                header = s.recv(1)
                if not header: break
                
                rx_time = time.time() - start_time 

                if header == b'\xAA': 
                    # === AUDIO ===
                    seq_bytes = recv_exact(s, SEQ_SIZE)
                    if not seq_bytes: break
                    current_seq = struct.unpack('<I', seq_bytes)[0]

                    chunk = recv_exact(s, AUDIO_PAYLOAD)
                    if not chunk: break
                    audio_raw_bytes.extend(chunk)
                    
                    mic_recv_count += 1
                    
                    if first_mic_packet:
                        expected_mic_seq = current_seq + 1
                        first_mic_packet = False
                    else:
                        expected_mic_seq = current_seq + 1

                elif header == b'\xBB':
                    # === IMU ===
                    seq_bytes = recv_exact(s, SEQ_SIZE)
                    if not seq_bytes: break
                    current_seq = struct.unpack('<I', seq_bytes)[0]

                    chunk = recv_exact(s, IMU_PAYLOAD)
                    if not chunk: break
                    
                    imu_recv_count += 1

                    if first_imu_packet:
                        expected_imu_seq = current_seq + 1
                        first_imu_packet = False
                    else:
                        expected_imu_seq = current_seq + 1

                    offset = 0
                    while offset < len(chunk):
                        sample_bytes = chunk[offset : offset + 12]
                        offset += 12
                        raw = struct.unpack(IMU_STRUCT_FMT, sample_bytes)
                        
                        gx = raw[0] * GYRO_SCALE
                        gy = raw[1] * GYRO_SCALE
                        gz = raw[2] * GYRO_SCALE
                        ax = raw[3] * ACC_SCALE
                        ay = raw[4] * ACC_SCALE
                        az = raw[5] * ACC_SCALE
                        
                        imu_plot_data.append([ax, ay, az, gx, gy, gz])
                        csv_rows.append([current_seq, round(rx_time, 4), ax, ay, az, gx, gy, gz])

            except socket.timeout:
                print("\nSocket timed out.")
                break
            except KeyboardInterrupt:
                print("\nStopped by user.")
                break

        s.close()

        # ==========================================
        # CALCULATE STATISTICS
        # ==========================================
        mic_packets_per_sec = MIC_SAMPLE_RATE / (AUDIO_PAYLOAD / 4)
        mic_expected = int(DURATION * mic_packets_per_sec)
        imu_packets_per_sec = IMU_SAMPLE_RATE / (IMU_PAYLOAD / 12)
        imu_expected = int(DURATION * imu_packets_per_sec)

        print("\n" + "="*45)
        print("              FINAL STATISTICS               ")
        print("="*45)
        print(f"AUDIO Packets Received: {mic_recv_count} / {mic_expected} Expected")
        print(f"IMU   Packets Received: {imu_recv_count} / {imu_expected} Expected")
        print("="*45)

        # ==========================================
        # 1. SAVE CSV
        # ==========================================
        print(f"\nSaving data to {OUTPUT_CSV}...")
        with open(OUTPUT_CSV, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Packet_ID", "Rx_Time_Sec", "Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z"])
            writer.writerows(csv_rows)
        print("-> Done.")

        # ==========================================
        # 2. SAVE WAV & PREPARE AUDIO DATA
        # ==========================================
        audio_float = np.array([])
        
        if len(audio_raw_bytes) > 0:
            audio_np = np.frombuffer(audio_raw_bytes, dtype='<i4')
            audio_float = audio_np.astype(np.float32)
            
            # Remove DC Offset (Center the wave at 0)
            audio_float -= np.mean(audio_float)
            
            # Normalize to -1.0 to 1.0
            m = np.max(np.abs(audio_float))
            if m > 0: audio_float /= m
            
            pcm_16 = (audio_float * 32767).astype(np.int16)

            with wave.open(OUTPUT_WAV, "w") as f:
                f.setnchannels(1)
                f.setsampwidth(2)
                f.setframerate(MIC_SAMPLE_RATE)
                f.writeframes(pcm_16.tobytes())
            print(f"Saved Audio: {OUTPUT_WAV}")

        # ==========================================
        # 3. PLOT IMU (Figure 1)
        # ==========================================
        if len(imu_plot_data) > 0:
            count = len(imu_plot_data)
            dt = 1.0 / IMU_SAMPLE_RATE
            t = [i * dt for i in range(count)]
            cols = list(zip(*imu_plot_data))
            
            plt.figure(1, figsize=(10, 8))
            
            plt.subplot(2, 1, 1)
            plt.plot(t, cols[0], label='Ax'); plt.plot(t, cols[1], label='Ay'); plt.plot(t, cols[2], label='Az')
            plt.title("Accelerometer (Raw)")
            plt.ylabel("m/s^2")
            plt.legend(); plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.plot(t, cols[3], label='Gx'); plt.plot(t, cols[4], label='Gy'); plt.plot(t, cols[5], label='Gz')
            plt.title("Gyroscope (Raw)")
            plt.xlabel("Time (s)"); plt.ylabel("rad/s")
            plt.legend(); plt.grid(True)
            
            plt.tight_layout()

        # ==========================================
        # 4. PLOT AUDIO & FFT (Figure 2)
        # ==========================================
        if len(audio_float) > 0:
            N = len(audio_float)
            T = 1.0 / MIC_SAMPLE_RATE
            time_axis = np.linspace(0.0, N*T, N, endpoint=False)

            # --- FFT CALCULATION ---
            yf = fft(audio_float)
            xf = fftfreq(N, T)[:N//2]
            magnitude = 2.0/N * np.abs(yf[0:N//2])

            plt.figure(2, figsize=(10, 8))

            # A. Waveform Plot
            plt.subplot(2, 1, 1)
            plt.plot(time_axis, audio_float, color='tab:blue')
            plt.title("Audio Waveform (Time Domain)")
            plt.ylabel("Amplitude (Normalized)")
            plt.xlabel("Time (s)")
            plt.grid(True)

            

            # B. FFT Plot
            magnitude_db = 20 * np.log10(magnitude + 1e-9) 

            plt.subplot(2, 1, 2)
            plt.plot(xf, magnitude_db, color='tab:orange')
            plt.title("Audio Spectrum (Logarithmic / dB)")
            plt.ylabel("Magnitude (dB)")
            # Set floor to -100dB (silence) and ceiling to 0dB (max volume)
            plt.ylim(-150, 0) 
            plt.xlabel("Frequency (Hz)")
            plt.grid(True)

            
            
            plt.tight_layout()

        plt.show()

    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()