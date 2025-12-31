import serial
import struct
import csv
import time
import argparse
import sys
import os

# Configuration matches C++ struct alignment for STM32
STRUCT_FMT = '<QB3x6f4x' 
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)
PAGE_SIZE = 256
ITEMS_PER_PAGE = 6

# Sensor Types Enum
SENSOR_TYPE_MAP = {
    0: 'IMU',
    1: 'GPS',
    2: 'MAG'
}

def parse_sensor_data(raw_bytes):
    try:
        unpacked = struct.unpack(STRUCT_FMT, raw_bytes)
        timestamp = unpacked[0]
        sensor_type_id = unpacked[1]
        data_floats = unpacked[2:] 

        sensor_type = SENSOR_TYPE_MAP.get(sensor_type_id, f"UNKNOWN({sensor_type_id})")

        return {
            'timestamp': timestamp,
            'type': sensor_type,
            'd0': data_floats[0],
            'd1': data_floats[1],
            'd2': data_floats[2],
            'd3': data_floats[3],
            'd4': data_floats[4],
            'd5': data_floats[5]
        }
    except struct.error as e:
        print(f"Error unpacking struct: {e}")
        return None

def start_new_files(base_name, file_counter, fieldnames):
    """Closes old files (if exist) and opens new CSV and BIN files."""
    name_parts = os.path.splitext(base_name)
    
    # 1. Create filenames
    csv_name = f"{name_parts[0]}_part{file_counter}.csv"
    bin_name = f"{name_parts[0]}_part{file_counter}.bin"
    
    print(f"--> Starting Log #{file_counter}: {csv_name} & {bin_name}")
    
    # 2. Open CSV
    f_csv = open(csv_name, 'w', newline='')
    writer = csv.DictWriter(f_csv, fieldnames=fieldnames)
    writer.writeheader()

    # 3. Open BIN
    f_bin = open(bin_name, 'wb')
    
    return f_csv, writer, f_bin

def main():
    parser = argparse.ArgumentParser(description='Dump Flash Data from STM32')
    parser.add_argument('port', type=str, help='COM port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--output', type=str, default='flight_log.csv', help='Base Output CSV file')
    
    if len(sys.argv) == 1:
        my_args = ['COM10', '--baud', '57600'] 
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

    # --- PARSING AND SPLITTING LOGIC ---
    print(f"Processing pages...")
    
    keys = ['timestamp', 'type', 'd0', 'd1', 'd2', 'd3', 'd4', 'd5']
    
    # State tracking
    last_timestamp = 0
    file_counter = 1
    
    current_csv_file = None
    csv_writer = None
    current_bin_file = None
    
    # Initialize first file set
    current_csv_file, csv_writer, current_bin_file = start_new_files(args.output, file_counter, keys)

    total_pages = len(raw_buffer) // PAGE_SIZE
    records_written = 0
    
    for i in range(total_pages):
        page_start = i * PAGE_SIZE
        page_end = page_start + PAGE_SIZE
        
        # Grab the raw page bytes
        page_data = raw_buffer[page_start : page_end]

        # Scan the page first to check for reboots
        # We look at the first valid item to decide if this page belongs to a new file
        first_valid_ts = None
        for j in range(ITEMS_PER_PAGE):
            item_bytes = page_data[j * STRUCT_SIZE : (j + 1) * STRUCT_SIZE]
            parsed = parse_sensor_data(item_bytes)
            if parsed and parsed['timestamp'] != 0 and parsed['timestamp'] != 0xFFFFFFFFFFFFFFFF:
                first_valid_ts = parsed['timestamp']
                break
        
        # TIME JUMP DETECTION (Page Level)
        # If the first valid item in this page is "older" than our last seen time, we split.
        if first_valid_ts is not None and first_valid_ts < last_timestamp:
            print(f"Time jump detected! (Old: {last_timestamp}, New: {first_valid_ts})")
            
            # Close old files
            current_csv_file.close()
            current_bin_file.close()
            
            # Start new files
            file_counter += 1
            current_csv_file, csv_writer, current_bin_file = start_new_files(args.output, file_counter, keys)
            
            # Reset tracking
            last_timestamp = 0

        # WRITE BINARY: Dump the entire raw page into the current .bin file
        # This preserves the exact Flash structure (including padding)
        current_bin_file.write(page_data)

        # PROCESS CSV: Parse items and write rows
        for j in range(ITEMS_PER_PAGE):
            item_bytes = page_data[j * STRUCT_SIZE : (j + 1) * STRUCT_SIZE]
            parsed = parse_sensor_data(item_bytes)
            
            if not parsed or parsed['timestamp'] == 0xFFFFFFFFFFFFFFFF or parsed['timestamp'] == 0:
                continue
                
            csv_writer.writerow(parsed)
            last_timestamp = parsed['timestamp']
            records_written += 1

    # Final Cleanup
    if current_csv_file:
        current_csv_file.close()
    if current_bin_file:
        current_bin_file.close()

    print(f"Done. Processed {records_written} records across {file_counter} file sets.")

if __name__ == '__main__':
    main()