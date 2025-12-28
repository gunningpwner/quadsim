# -*- coding: utf-8 -*-
"""
Created on Sun Dec 28 15:28:59 2025

@author: gunni
"""

import serial
import struct
import csv
import time
import argparse
import sys

# Configuration matches C++ struct alignment for STM32
# Format: < (Little Endian) Q (uint64) B (uint8) 3x (pad) 6f (6 floats) 4x (tail pad)
# Total size: 8 + 1 + 3 + 24 + 4 = 40 bytes
STRUCT_FMT = '<QB3x6f4x' 
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)
PAGE_SIZE = 256

# Sensor Types Enum
SENSOR_TYPE_MAP = {
    0: 'IMU',
    1: 'GPS',
    2: 'MAG'
}

def parse_sensor_data(raw_bytes):
    """
    Parses a single 40-byte chunk into a dictionary.
    """
    try:
        unpacked = struct.unpack(STRUCT_FMT, raw_bytes)
        timestamp = unpacked[0]
        sensor_type_id = unpacked[1]
        data_floats = unpacked[2:] # Tuple of 6 floats

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

def main():
    parser = argparse.ArgumentParser(description='Dump Flash Data from STM32')
    parser.add_argument('port', type=str, help='COM port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--output', type=str, default='flight_log.csv', help='Output CSV file')
    if len(sys.argv) == 1:
        # EDIT THESE VALUES FOR YOUR SETUP
        my_args = ['COM10', '--baud', '57600']
        print(f"No cmdline args found. Using manual defaults: {my_args}")
        args = parser.parse_args(my_args)
    else:
        # Otherwise, parse arguments from the terminal as usual
        args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open port: {e}")
        sys.exit(1)

    # Give the connection a moment to settle
    time.sleep(2)

    # 1. Send the DUMP command
    print("Sending DUMP command...")
    ser.write(b'DUMP')
    
    # 2. Read the stream
    print("Reading data stream... (Press Ctrl+C to stop early)")
    
    raw_buffer = bytearray()
    capture_active = False
    
    try:
        while True:
            # Read chunks
            chunk = ser.read(1024) 
            if not chunk:
                # If timeout occurs and we were capturing, we might be done
                if capture_active:
                    print("Read timeout - assuming end of stream.")
                    break
                continue

            raw_buffer.extend(chunk)

            # Check for Start condition if not yet active
            if not capture_active:
                # Look for the print statement from C++ code
                start_idx = raw_buffer.find(b'Dumping Flash Data')
                if start_idx != -1:
                    print("Stream started detected.")
                    # Trim everything before the start message + newline
                    # Assuming C++ prints "Dumping Flash Data...\n"
                    newline_idx = raw_buffer.find(b'\n', start_idx)
                    if newline_idx != -1:
                        raw_buffer = raw_buffer[newline_idx+1:]
                        capture_active = True

            # Check for End condition
            if capture_active:
                end_idx = raw_buffer.find(b'Dump Complete')
                if end_idx != -1:
                    print("End of stream detected.")
                    # Trim the end message
                    raw_buffer = raw_buffer[:end_idx]
                    break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    ser.close()

    # 3. Parse the buffer
    print(f"Captured {len(raw_buffer)} bytes.")
    
    records = []
    
    # Process buffer in PAGE_SIZE (256) byte chunks
    # We only process full pages.
    total_pages = len(raw_buffer) // PAGE_SIZE
    
    print(f"Processing {total_pages} pages...")

    for i in range(total_pages):
        page_start = i * PAGE_SIZE
        page_end = page_start + PAGE_SIZE
        page_data = raw_buffer[page_start:page_end]

        # In every 256-byte page, there are N items followed by padding
        # ITEMS_PER_PAGE = 256 // 40 = 6
        items_per_page = 6
        
        for j in range(items_per_page):
            item_start = j * STRUCT_SIZE
            item_end = item_start + STRUCT_SIZE
            
            item_bytes = page_data[item_start:item_end]
            
            parsed = parse_sensor_data(item_bytes)
            
            # Filter out empty records (timestamp 0 usually means unwritten flash)
            # You can remove this check if 0 is valid for you
            if parsed and parsed['timestamp'] != 0xFFFFFFFFFFFFFFFF and parsed['timestamp'] != 0:
                records.append(parsed)

    # 4. Write to CSV
    if records:
        print(f"Writing {len(records)} records to {args.output}...")
        keys = ['timestamp', 'type', 'd0', 'd1', 'd2', 'd3', 'd4', 'd5']
        
        with open(args.output, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(records)
        print("Done.")
    else:
        print("No valid records found.")

if __name__ == '__main__':
    main()