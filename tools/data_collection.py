#!/usr/bin/env python3
"""
Data collection script for sign language glove
Connects to ESP32 via serial and collects training data
"""

import serial
import json
import time
import numpy as np
import argparse
from pathlib import Path

class DataCollector:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.data = []
        
    def collect_gesture_data(self, gesture_name, num_samples=50, sample_duration=2.0):
        """Collect training data for a specific gesture"""
        print(f"Collecting data for gesture: {gesture_name}")
        print(f"Will collect {num_samples} samples, {sample_duration}s each")
        
        for i in range(num_samples):
            input(f"Sample {i+1}/{num_samples} - Press Enter when ready to record '{gesture_name}'...")
            
            print(f"Recording... ({sample_duration}s)")
            start_time = time.time()
            sample_data = []
            
            while time.time() - start_time < sample_duration:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('SENSOR_DATA:'):
                    try:
                        data_str = line.replace('SENSOR_DATA:', '')
                        sensor_data = json.loads(data_str)
                        sensor_data['timestamp'] = time.time()
                        sample_data.append(sensor_data)
                    except json.JSONDecodeError:
                        continue
            
            if sample_data:
                self.data.append({
                    'gesture': gesture_name,
                    'sample_id': i,
                    'data': sample_data,
                    'duration': sample_duration,
                    'num_frames': len(sample_data)
                })
                print(f"Recorded {len(sample_data)} data points")
            else:
                print("No data recorded, retrying...")
                i -= 1  # Retry this sample
    
    def save_data(self, filename):
        """Save collected data to JSON file"""
        with open(filename, 'w') as f:
            json.dump(self.data, f, indent=2)
        print(f"Data saved to {filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Collect gesture training data')
    parser.add_argument('--port', required=True, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--output', required=True, help='Output JSON file')
    parser.add_argument('--gestures', nargs='+', required=True, help='Gesture names to collect')
    parser.add_argument('--samples', type=int, default=30, help='Samples per gesture')
    parser.add_argument('--duration', type=float, default=2.0, help='Duration per sample (seconds)')
    
    args = parser.parse_args()
    
    collector = DataCollector(args.port)
    
    for gesture in args.gestures:
        collector.collect_gesture_data(gesture, args.samples, args.duration)
    
    collector.save_data(args.output)
    print("Data collection complete!")