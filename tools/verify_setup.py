#!/usr/bin/env python3
"""
Verify that all required Python packages are installed
"""

def check_imports():
    """Check if all required packages can be imported"""
    required_packages = {
        'serial': 'pyserial',
        'tensorflow': 'tensorflow', 
        'sklearn': 'scikit-learn',
        'numpy': 'numpy',
        'pandas': 'pandas',
        'json': 'built-in',
        'matplotlib': 'matplotlib'
    }
    
    print("🔍 Checking Python dependencies...")
    print("-" * 50)
    
    all_good = True
    
    for package, pip_name in required_packages.items():
        try:
            if package == 'sklearn':
                import sklearn
                version = sklearn.__version__
            elif package == 'serial':
                import serial
                version = serial.__version__
            elif package == 'tensorflow':
                import tensorflow as tf
                version = tf.__version__
            else:
                module = __import__(package)
                version = getattr(module, '__version__', 'unknown')
            
            print(f"✅ {package:12} v{version:10} ({pip_name})")
            
        except ImportError as e:
            print(f"❌ {package:12} {'MISSING':10} - Install with: pip install {pip_name}")
            all_good = False
        except Exception as e:
            print(f"⚠️  {package:12} {'ERROR':10} - {str(e)}")
    
    print("-" * 50)
    
    if all_good:
        print("🎉 All dependencies are installed correctly!")
        return True
    else:
        print("❌ Some dependencies are missing. Please install them.")
        return False

def test_tensorflow():
    """Test TensorFlow installation with a simple model"""
    try:
        import tensorflow as tf
        print(f"\n🧠 TensorFlow Test:")
        print(f"   Version: {tf.__version__}")
        print(f"   GPU Available: {tf.config.list_physical_devices('GPU')}")
        
        # Create a simple model to test
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(10, activation='relu', input_shape=(5,)),
            tf.keras.layers.Dense(3, activation='softmax')
        ])
        
        # Test model creation
        model.compile(optimizer='adam', loss='sparse_categorical_crossentropy')
        print(f"   Model Creation: ✅ Success")
        
        # Test TensorFlow Lite conversion
        converter = tf.lite.TFLiteConverter.from_keras_model(model)
        tflite_model = converter.convert()
        print(f"   TFLite Conversion: ✅ Success ({len(tflite_model)} bytes)")
        
        return True
        
    except Exception as e:
        print(f"   TensorFlow Test: ❌ Failed - {e}")
        return False

def test_serial():
    """Test serial communication"""
    try:
        import serial
        import serial.tools.list_ports
        
        print(f"\n📡 Serial Communication Test:")
        print(f"   PySerial Version: {serial.__version__}")
        
        # List available ports
        ports = list(serial.tools.list_ports.comports())
        print(f"   Available Ports: {len(ports)}")
        
        for port in ports[:3]:  # Show first 3 ports
            print(f"     - {port.device}: {port.description}")
        
        print(f"   Serial Test: ✅ Success")
        return True
        
    except Exception as e:
        print(f"   Serial Test: ❌ Failed - {e}")
        return False

if __name__ == "__main__":
    print("🔧 Python Environment Verification")
    print("=" * 50)
    
    # Check basic imports
    basic_ok = check_imports()
    
    if basic_ok:
        # Test TensorFlow functionality
        tf_ok = test_tensorflow()
        
        # Test Serial functionality  
        serial_ok = test_serial()
        
        if tf_ok and serial_ok:
            print("\n🎉 ALL TESTS PASSED! Your environment is ready.")
        else:
            print("\n⚠️  Some advanced tests failed. Check the errors above.")
    else:
        print("\n❌ Please install missing dependencies first.")
        print("\n💡 Quick fix:")
        print("   pip install tensorflow scikit-learn pyserial numpy pandas")