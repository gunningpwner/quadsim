import serial
import struct
import csv
import time
import argparse
import sys
import os
import math
# Packet configurations mapping ID to name, struct format, and CSV fields
# Based on C++ struct definitions[cite: 6]
PACKET_CONFIGS = {
    0x01: ('FastIMUData', '<Q6f', ['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']),
    0x02: ('FastRPMData', '<Q4I', ['timestamp', 'rpm0', 'rpm1', 'rpm2', 'rpm3']),
    0x03: ('SensorData', '<QB3x6f', ['timestamp', 'sensor_type', 'd0', 'd1', 'd2', 'd3', 'd4', 'd5']),
    0x04: ('StateEstimate', '<Q16f', [
        'timestamp', 
        'pos_n', 'pos_e', 'pos_d', 
        'vel_n', 'vel_e', 'vel_d', 
        'q0', 'q1', 'q2', 'q3', 
        'ref_lat', 'ref_lon', 'ref_alt', 
        'p', 'q', 'r'
    ])
}

def start_new_files(base_name, file_counter, folder=None):
    """Closes old files (if exist) and opens new CSV and BIN files."""
    name_parts = os.path.splitext(base_name)
    csv_name = f"{name_parts[0]}_part{file_counter}.csv"
    bin_name = f"{name_parts[0]}_part{file_counter}.bin"
    if folder:
        csv_name = os.path.join(folder, csv_name)
        bin_name = os.path.join(folder, bin_name)
    print(f"--> Starting Log #{file_counter}: {csv_name} & {bin_name}")
    
    f_csv = open(csv_name, 'w', newline='')
    f_bin = open(bin_name, 'wb')
    return f_csv, f_bin

def main():
    parser = argparse.ArgumentParser(description='Dump and Parse Flash Data from STM32')
    parser.add_argument('port', type=str, help='COM port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--output', type=str, default='flight_log.csv', help='Base Output CSV file')
    
    if len(sys.argv) == 1:
        my_args = ['COM6', '--baud', '57600'] 
        print(f"No cmdline args found. Using defaults: {my_args}")
        args = parser.parse_args(my_args)
    else:
        args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open port: {e}")
        sys.exit(1)

    time.sleep(2)
    print("Sending DUMP command...")
    ser.write(b'DUMP')
    
    print("Reading stream... (Ctrl+C to stop)")
    raw_buffer = bytearray()
    capture_active = False
    
    try:
        while True:
            chunk = ser.read(1024) 
            if not chunk:
                if capture_active:
                    print("Read timeout - assuming end.")
                    break
                continue

            raw_buffer.extend(chunk)

            if not capture_active:
                start_idx = raw_buffer.find(b'Dumping Flash Data')
                if start_idx != -1:
                    print("Stream start detected.")
                    newline_idx = raw_buffer.find(b'\n', start_idx)
                    if newline_idx != -1:
                        raw_buffer = raw_buffer[newline_idx+1:]
                        capture_active = True

            if capture_active:
                end_idx = raw_buffer.find(b'Dump Complete')
                if end_idx != -1:
                    print("End of stream detected.")
                    raw_buffer = raw_buffer[:end_idx]
                    break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.close()

    print(f"Total Capture: {len(raw_buffer)} bytes.")

    # --- STREAM PARSING & RESYNCHRONIZING LOGIC ---
    print(f"Processing packet stream...")
    folder = 'data'
    os.makedirs(folder, exist_ok=True)
    
    file_counter = 1
    current_csv_file, current_bin_file = start_new_files(args.output, file_counter, folder=folder)
    
    # We use multiple csv writers dynamically based on packet type or a unified writer
    # Writing to separate files or a unified format can be done; here we use standard DictWriters mapped per type or combined.
    writers = {}
    
    def get_writer(packet_type_name, fields):
        if packet_type_name not in writers:
            # Create a CSV file specific to each packet type or handle multiplexing
            pass
        return writers

    pos = 0
    records_written = 0
    last_timestamp = 0

    # Save raw dump to bin
    current_bin_file.write(raw_buffer)
    current_bin_file.close()

    # Open unified or type-specific CSVs
    csv_file_path = os.path.join(folder, f"parsed_log_part{file_counter}.csv")
    f_csv = open(csv_file_path, 'w', newline='')
    
    # Collect all unique fieldnames for a generalized CSV output
    all_fields = ['timestamp', 'packet_type', 'sensor_type', 
                  'ax', 'ay', 'az', 'gx', 'gy', 'gz', 
                  'rpm0', 'rpm1', 'rpm2', 'rpm3', 
                  'd0', 'd1', 'd2', 'd3', 'd4', 'd5',
                  'pos_n', 'pos_e', 'pos_d', 'vel_n', 'vel_e', 'vel_d', 
                  'q0', 'q1', 'q2', 'q3', 'ref_lat', 'ref_lon', 'ref_alt', 'p', 'q', 'r']
    
    writer = csv.DictWriter(f_csv, fieldnames=all_fields, extrasaction='ignore')
    writer.writeheader()

    while pos < len(raw_buffer) - 1:
        packet_id = raw_buffer[pos]
        payload_size = raw_buffer[pos+1]
        # print(packet_id,payload_size)
        # Check if this matches a known packet header configuration
        if packet_id in PACKET_CONFIGS:
            name, fmt, fields = PACKET_CONFIGS[packet_id]
            expected_size = struct.calcsize(fmt)
            print(name,expected_size,payload_size)
            # Verify payload size matches struct size and doesn't exceed buffer bounds
            # if payload_size == expected_size and pos + 2 + expected_size <= len(raw_buffer):
            if payload_size >= expected_size and pos + 2 + expected_size <= len(raw_buffer):
                payload = raw_buffer[pos+2 : pos+2+expected_size]
                try:
                    unpacked = struct.unpack(fmt, payload)
                    row = dict(zip(fields, unpacked))
                    row['packet_type'] = name
                    
                    timestamp = row.get('timestamp', 0)
                    
                    # Time jump / reboot detection
                    if last_timestamp != 0 and (timestamp < last_timestamp or timestamp - last_timestamp > 10e6):
                        print(f"Time jump detected! (Old: {last_timestamp}, New: {timestamp})")
                        f_csv.close()
                        file_counter += 1
                        csv_file_path = os.path.join(folder, f"parsed_log_part{file_counter}.csv")
                        f_csv = open(csv_file_path, 'w', newline='')
                        writer = csv.DictWriter(f_csv, fieldnames=all_fields, extrasaction='ignore')
                        writer.writeheader()

                    writer.writerow(row)
                    last_timestamp = timestamp
                    records_written += 1
                    
                    pos += 2 + payload_size
                    continue
                except struct.error:
                    print("OH GOD")
            elif(packet_id==0xFF):
                print("overflow")
            else:
                print("shit")
        else:
            print(packet_id, " aww")
        # Resynchronize by sliding 1 byte forward if header is invalid, padding (0x00), or erased flash (0xFF)
        pos += 1

    f_csv.close()
    print(f"Done. Processed {records_written} valid telemetry packets into {folder}/.")

if __name__ == '__main__':
    main()