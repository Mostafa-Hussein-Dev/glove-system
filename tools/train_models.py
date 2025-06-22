#!/usr/bin/env python3
"""
Train TensorFlow Lite models for gesture recognition
"""

import json
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
import argparse
from pathlib import Path

class GestureModelTrainer:
    def __init__(self, data_file):
        self.data_file = data_file
        self.label_encoder = LabelEncoder()
        self.scaler = StandardScaler()
        
    def load_and_preprocess_data(self):
        """Load data and extract features"""
        with open(self.data_file, 'r') as f:
            raw_data = json.load(f)
        
        static_features = []
        dynamic_features = []
        labels = []
        
        for sample in raw_data:
            gesture = sample['gesture']
            data_points = sample['data']
            
            if len(data_points) < 10:  # Skip samples with too little data
                continue
            
            # Extract static features (from middle frame)
            mid_frame = data_points[len(data_points)//2]
            static_feat = self.extract_static_features(mid_frame)
            
            # Extract dynamic features (temporal changes)
            dynamic_feat = self.extract_dynamic_features(data_points)
            
            if static_feat is not None and dynamic_feat is not None:
                static_features.append(static_feat)
                dynamic_features.append(dynamic_feat)
                labels.append(gesture)
        
        return np.array(static_features), np.array(dynamic_features), np.array(labels)
    
    def extract_static_features(self, data_point):
        """Extract features for static gesture recognition"""
        try:
            features = []
            
            # Flex sensor angles
            if 'flex_data' in data_point:
                flex_angles = data_point['flex_data'].get('angles', [])
                features.extend(flex_angles[:10])  # Max 10 flex sensors
                
                # Pad if we have fewer sensors
                while len(features) < 10:
                    features.append(0.0)
            
            # IMU data
            if 'imu_data' in data_point:
                imu_data = data_point['imu_data']
                features.extend(imu_data.get('accel', [0, 0, 0]))
                features.extend(imu_data.get('gyro', [0, 0, 0]))
                features.extend(imu_data.get('orientation', [0, 0, 0]))
            
            # Derived features
            if len(features) >= 10:  # We have flex data
                # Adjacent finger differences
                for i in range(4):
                    diff = abs(features[i*2] - features[(i+1)*2])
                    features.append(diff)
                
                # MCP-PIP differences
                for i in range(5):
                    if i*2+1 < len(features):
                        diff = abs(features[i*2] - features[i*2+1])
                        features.append(diff)
            
            return np.array(features[:32])  # Limit to 32 features
            
        except Exception as e:
            print(f"Error extracting static features: {e}")
            return None
    
    def extract_dynamic_features(self, data_points):
        """Extract features for dynamic gesture recognition"""
        try:
            if len(data_points) < 5:
                return None
            
            features = []
            
            # Sample evenly across the gesture
            indices = np.linspace(0, len(data_points)-1, 10, dtype=int)
            sampled_points = [data_points[i] for i in indices]
            
            # Extract features from each time step
            for point in sampled_points:
                step_features = []
                
                # Flex sensor data
                if 'flex_data' in point:
                    flex_angles = point['flex_data'].get('angles', [])
                    step_features.extend(flex_angles[:5])  # Use first 5 sensors
                    while len(step_features) < 5:
                        step_features.append(0.0)
                
                # IMU data (just acceleration for simplicity)
                if 'imu_data' in point:
                    accel = point['imu_data'].get('accel', [0, 0, 0])
                    step_features.extend(accel)
                
                features.extend(step_features[:8])  # 8 features per timestep
            
            return np.array(features).reshape(10, 8)  # 10 timesteps, 8 features each
            
        except Exception as e:
            print(f"Error extracting dynamic features: {e}")
            return None
    
    def create_static_model(self, input_shape, num_classes):
        """Create CNN model for static gesture recognition"""
        model = tf.keras.Sequential([
            tf.keras.layers.Input(shape=input_shape),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dropout(0.3),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dropout(0.3),
            tf.keras.layers.Dense(16, activation='relu'),
            tf.keras.layers.Dense(num_classes, activation='softmax')
        ])
        
        model.compile(
            optimizer='adam',
            loss='sparse_categorical_crossentropy',
            metrics=['accuracy']
        )
        
        return model
    
    def create_dynamic_model(self, input_shape, num_classes):
        """Create LSTM model for dynamic gesture recognition"""
        model = tf.keras.Sequential([
            tf.keras.layers.Input(shape=input_shape),
            tf.keras.layers.LSTM(32, return_sequences=True),
            tf.keras.layers.Dropout(0.3),
            tf.keras.layers.LSTM(16),
            tf.keras.layers.Dropout(0.3),
            tf.keras.layers.Dense(16, activation='relu'),
            tf.keras.layers.Dense(num_classes, activation='softmax')
        ])
        
        model.compile(
            optimizer='adam',
            loss='sparse_categorical_crossentropy',
            metrics=['accuracy']
        )
        
        return model
    
    def convert_to_tflite(self, model, output_path):
        """Convert Keras model to TensorFlow Lite"""
        converter = tf.lite.TFLiteConverter.from_keras_model(model)
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.target_spec.supported_types = [tf.float32]
        
        tflite_model = converter.convert()
        
        with open(output_path, 'wb') as f:
            f.write(tflite_model)
        
        print(f"TensorFlow Lite model saved to {output_path}")
        print(f"Model size: {len(tflite_model)} bytes")
    
    def train_models(self, output_dir):
        """Train both static and dynamic models"""
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        # Load and preprocess data
        print("Loading and preprocessing data...")
        static_X, dynamic_X, y = self.load_and_preprocess_data()
        
        if len(static_X) == 0:
            print("No valid data found!")
            return
        
        print(f"Loaded {len(static_X)} samples")
        print(f"Static features shape: {static_X.shape}")
        print(f"Dynamic features shape: {dynamic_X.shape}")
        
        # Encode labels
        y_encoded = self.label_encoder.fit_transform(y)
        num_classes = len(self.label_encoder.classes_)
        
        print(f"Gesture classes: {list(self.label_encoder.classes_)}")
        
        # Normalize features
        static_X_norm = self.scaler.fit_transform(static_X)
        dynamic_X_norm = dynamic_X.reshape(len(dynamic_X), -1)
        dynamic_X_norm = self.scaler.fit_transform(dynamic_X_norm)
        dynamic_X_norm = dynamic_X_norm.reshape(dynamic_X.shape)
        
        # Split data
        (static_X_train, static_X_test, 
         dynamic_X_train, dynamic_X_test, 
         y_train, y_test) = train_test_split(
            static_X_norm, dynamic_X_norm, y_encoded, 
            test_size=0.2, random_state=42, stratify=y_encoded
        )
        
        # Train static model
        print("Training static model...")
        static_model = self.create_static_model(static_X_train.shape[1:], num_classes)
        static_model.fit(
            static_X_train, y_train,
            validation_data=(static_X_test, y_test),
            epochs=50,
            batch_size=16,
            verbose=1
        )
        
        # Convert and save static model
        self.convert_to_tflite(static_model, output_dir / "static_cnn.tflite")
        
        # Train dynamic model
        print("Training dynamic model...")
        dynamic_model = self.create_dynamic_model(dynamic_X_train.shape[1:], num_classes)
        dynamic_model.fit(
            dynamic_X_train, y_train,
            validation_data=(dynamic_X_test, y_test),
            epochs=50,
            batch_size=16,
            verbose=1
        )
        
        # Convert and save dynamic model
        self.convert_to_tflite(dynamic_model, output_dir / "dynamic_lstm.tflite")
        
        # Save preprocessing parameters
        preprocessing_params = {
            'classes': list(self.label_encoder.classes_),
            'scaler_mean': self.scaler.mean_.tolist(),
            'scaler_scale': self.scaler.scale_.tolist()
        }
        
        with open(output_dir / "preprocessing_params.json", 'w') as f:
            json.dump(preprocessing_params, f, indent=2)
        
        print("Training complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train gesture recognition models')
    parser.add_argument('--data', required=True, help='Input JSON data file')
    parser.add_argument('--output', required=True, help='Output directory for models')
    
    args = parser.parse_args()
    
    trainer = GestureModelTrainer(args.data)
    trainer.train_models(args.output)