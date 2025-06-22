#!/usr/bin/env python3
"""
Upload trained models to ESP32 SPIFFS partition
"""

import serial
import time
import argparse
from pathlib import Path

def upload_file_to_spiffs(ser, local_path, remote_path):
    """Upload a file to ESP32 SPIFFS via serial"""
    with open(local_path, 'rb') as f:
        file_data = f.read()
    
    print(f"Uploading {local_path} to {remote_path} ({len(file_data)} bytes)")
    
    # Send upload command
    ser.write(f"UPLOAD_FILE:{remote_path}:{len(file_data)}\n".encode())
    time.sleep(0.1)
    
    # Send file data in chunks
    chunk_size = 512
    for i in range(0, len(file_data), chunk_size):
        chunk = file_data[i:i+chunk_size]
        ser.write(chunk)
        time.sleep(0.01)  # Small delay between chunks
        
        # Wait for acknowledgment
        response = ser.readline().decode('utf-8').strip()
        if response == "CHUNK_OK":
            print(f"Uploaded chunk {i//chunk_size + 1}/{(len(file_data)-1)//chunk_size + 1}")
        else:
            print(f"Error uploading chunk: {response}")
            return False
    
    print(f"Upload complete: {remote_path}")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Upload models to ESP32')
    parser.add_argument('--port', required=True, help='Serial port')
    parser.add_argument('--models', required=True, help='Directory containing models')
    
    args = parser.parse_args()
    
    models_dir = Path(args.models)
    ser = serial.Serial(args.port, 115200, timeout=1)
    
    # Upload model files
    for model_file in models_dir.glob("*.tflite"):
        upload_file_to_spiffs(ser, model_file, f"/spiffs/{model_file.name}")
    
    # Upload preprocessing parameters
    params_file = models_dir / "preprocessing_params.json"
    if params_file.exists():
        upload_file_to_spiffs(ser, params_file, "/spiffs/preprocessing_params.json")
    
    ser.close()
    print("All models uploaded successfully!")