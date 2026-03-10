#!/usr/bin/env python3
"""
Test BlueLily IMU connection via serial
Reads and displays IMU data from BlueLily
"""

import serial
import time
import sys

def test_bluelily_connection(port='/dev/ttyACM0', baudrate=115200, timeout=5):
    """Test BlueLily serial connection and display IMU data"""
    
    print("=" * 60)
    print("  BlueLily IMU Connection Test")
    print("=" * 60)
    print(f"\nPort: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Timeout: {timeout}s")
    print("\nAttempting to connect...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        
        print("✅ Connected successfully!")
        print("\nReading IMU data (Ctrl+C to stop)...\n")
        print("-" * 60)
        
        line_count = 0
        start_time = time.time()
        
        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        line_count += 1
                        
                        # Parse IMU data
                        if 'IMU' in line or 'accel' in line.lower() or 'gyro' in line.lower():
                            print(f"[{line_count:04d}] {line}")
                        elif 'SENSOR' in line:
                            print(f"[{line_count:04d}] {line}")
                        else:
                            # Print other data with lower priority
                            if line_count % 10 == 0:  # Print every 10th line
                                print(f"[{line_count:04d}] {line}")
                        
                        # Show statistics every 50 lines
                        if line_count % 50 == 0:
                            elapsed = time.time() - start_time
                            rate = line_count / elapsed
                            print(f"\n📊 Stats: {line_count} lines, {rate:.1f} Hz\n")
                            
                except UnicodeDecodeError:
                    print("⚠️  Decode error, skipping line")
                    continue
            
            time.sleep(0.001)  # Small delay to prevent CPU spinning
            
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if BlueLily is connected: ls -la /dev/ttyACM*")
        print("  2. Add user to dialout group: sudo usermod -a -G dialout $USER")
        print("  3. Check permissions: sudo chmod 666 /dev/ttyACM0")
        print("  4. Verify BlueLily is powered on")
        return False
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("  Test Complete")
        print("=" * 60)
        elapsed = time.time() - start_time
        rate = line_count / elapsed if elapsed > 0 else 0
        print(f"\nReceived {line_count} lines in {elapsed:.1f}s ({rate:.1f} Hz)")
        print("✅ BlueLily connection verified!")
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
        
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\n🔌 Serial port closed")
    
    return True

if __name__ == "__main__":
    # Check if user is in dialout group
    import os
    import grp
    
    try:
        dialout_gid = grp.getgrnam('dialout').gr_gid
        user_groups = os.getgroups()
        
        if dialout_gid not in user_groups:
            print("⚠️  Warning: User not in 'dialout' group")
            print("   Run: sudo usermod -a -G dialout $USER")
            print("   Then log out and back in\n")
    except:
        pass
    
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    success = test_bluelily_connection(port)
    sys.exit(0 if success else 1)
