import csv

# CONFIGURATION
CSV_FILE = "/home/olaf/Desktop/thesis_18/project-WAVe/data-processing/imu_data.csv"
EXPECTED_INTERVAL = 0.048  # 20 samples * (1/416Hz) = ~48ms per packet
TIME_TOLERANCE    = 0.020  # Allow 20ms of jitter before complaining

def analyze_csv():
    print(f"--- ANALYZING {CSV_FILE} ---")
    
    try:
        with open(CSV_FILE, 'r') as f:
            reader = csv.DictReader(f)
            
            prev_id = -1
            prev_time = -1.0
            
            total_gaps = 0
            total_time_jumps = 0
            packets_processed = 0
            
            # New Stats
            total_lag_seconds = 0.0
            max_lag_seconds = 0.0
            
            seen_ids = set()

            for row in reader:
                curr_id = int(row["Packet_ID"])
                curr_time = float(row["Rx_Time_Sec"])
                
                if curr_id in seen_ids:
                    continue
                seen_ids.add(curr_id)
                packets_processed += 1

                if prev_id == -1:
                    prev_id = curr_id
                    prev_time = curr_time
                    continue

                # 1. CHECK GAPS
                if curr_id != prev_id + 1:
                    gap = curr_id - prev_id - 1
                    print(f"[!] GAP DETECTED at Time {curr_time:.3f}s")
                    print(f"    ... Previous Packet: {prev_id}")
                    print(f"    ... Current Packet:  {curr_id}")
                    print(f"    ... LOST: {gap} packets")
                    total_gaps += gap

                # 2. CHECK FREEZES / LAG
                time_diff = curr_time - prev_time
                if time_diff > (EXPECTED_INTERVAL + TIME_TOLERANCE):
                    # Calculate how much we lagged behind expected time
                    lag = time_diff - EXPECTED_INTERVAL
                    
                    print(f"[?] TIME FREEZE at Packet {curr_id}")
                    print(f"    ... Expected ~{EXPECTED_INTERVAL*1000:.1f}ms")
                    print(f"    ... Actual:   {time_diff*1000:.1f}ms (Lag of {lag*1000:.1f}ms)")
                    
                    total_time_jumps += 1
                    total_lag_seconds += lag
                    if lag > max_lag_seconds:
                        max_lag_seconds = lag

                prev_id = curr_id
                prev_time = curr_time

        print("\n" + "="*40)
        print("             FINAL REPORT               ")
        print("="*40)
        print(f"Unique Packets:   {packets_processed}")
        print(f"Packet Gaps:      {total_gaps} lost")
        print(f"Time Freezes:     {total_time_jumps} events")
        print("-" * 40)
        print(f"Total Accumulated Lag: {total_lag_seconds*1000:.1f} ms")
        print(f"Worst Single Freeze:   {max_lag_seconds*1000:.1f} ms")
        print("="*40)
        
        if total_gaps == 0 and total_time_jumps == 0:
            print("\n✅ PERFECT STREAM. No data loss detected.")
        else:
            print("\n⚠️ ISSUES DETECTED (See summary above).")

    except FileNotFoundError:
        print(f"Error: Could not find file at {CSV_FILE}")

if __name__ == "__main__":
    analyze_csv()