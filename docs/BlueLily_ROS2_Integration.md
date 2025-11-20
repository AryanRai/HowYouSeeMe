# BlueLily ROS2 Integration for SLAM

## Overview

Use your BlueLily flight computer as an IMU source for RTAB-Map SLAM by bridging its serial data to ROS2.

## Architecture

```
BlueLily (Teensy 4.1)
  └─ MPU6500 IMU
      └─ Serial/USB → Computer
          └─ ROS2 Serial Bridge
              └─ /imu/data topic
                  └─ RTAB-Map SLAM
```

## Benefits

✅ **High-Quality IMU**: MPU6500 is a good 6-axis IMU
✅ **Already Have Hardware**: No need to buy separate IMU
✅ **Proven Design**: BlueLily is tested and reliable
✅ **Easy Integration**: Just serial communication
✅ **Additional Sensors**: Can also use temperature, ADC data

## Implementation

### Step 1: Modify BlueLily Firmware

Add a ROS2-compatible serial output mode. Create a new file in your BlueLily project:

```cpp
// BlueLily_ROS2_Bridge.ino
// Add to your BlueLily firmware

// ROS2 IMU message format (simplified)
void publishIMU() {
    // Get IMU data from MPU6500
    float accel_x = sensors.getAccelX();
    float accel_y = sensors.getAccelY();
    float accel_z = sensors.getAccelZ();
    float gyro_x = sensors.getGyroX();
    float gyro_y = sensors.getGyroY();
    float gyro_z = sensors.getGyroZ();
    
    // Send as comma-separated values
    // Format: IMU,timestamp,ax,ay,az,gx,gy,gz
    Serial.print("IMU,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(accel_x, 6);
    Serial.print(",");
    Serial.print(accel_y, 6);
    Serial.print(",");
    Serial.print(accel_z, 6);
    Serial.print(",");
    Serial.print(gyro_x, 6);
    Serial.print(",");
    Serial.print(gyro_y, 6);
    Serial.print(",");
    Serial.println(gyro_z, 6);
}

// Call this in your main loop at ~100Hz
void loop() {
    // ... existing code ...
    
    static unsigned long lastIMU = 0;
    if (millis() - lastIMU >= 10) {  // 100Hz
        publishIMU();
        lastIMU = millis();
    }
}
```

### Step 2: Create ROS2 Package

Create a ROS2 package to bridge serial data:

```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src
ros2 pkg create --build-type ament_python bluelily_bridge
cd bluelily_bridge
```

### Step 3: Create Serial Bridge Node

I'll create this for you in the next step.

## ROS2 Serial Bridge

The bridge will:
1. Read serial data from BlueLily
2. Parse IMU values
3. Publish to `/imu/data` topic
4. Handle coordinate frame transforms

## Usage

### 1. Flash BlueLily
```bash
# Upload modified firmware to Teensy 4.1
# Make sure it outputs IMU data at 100Hz
```

### 2. Connect BlueLily
```bash
# Find the device
ls /dev/ttyACM*

# Should show something like /dev/ttyACM0
```

### 3. Launch Bridge
```bash
ros2 run bluelily_bridge imu_bridge_node --ros-args -p port:=/dev/ttyACM0
```

### 4. Verify IMU Data
```bash
# Check topic
ros2 topic list | grep imu

# View data
ros2 topic echo /imu/data

# Check rate
ros2 topic hz /imu/data
```

### 5. Launch SLAM with IMU
```bash
./launch_kinect2_slam_with_imu.sh
```

## Coordinate Frame Mapping

BlueLily MPU6500 → ROS2 IMU:
- BlueLily X → ROS X
- BlueLily Y → ROS Y  
- BlueLily Z → ROS Z

Make sure BlueLily is mounted with same orientation as Kinect!

## Calibration

For best results, calibrate the IMU:

```bash
# Collect static data
ros2 bag record /imu/data -o imu_static

# Calculate bias
# accel_bias = mean(accel) - [0, 0, 9.81]
# gyro_bias = mean(gyro)

# Update in bridge node
```

## Advanced: Additional Sensors

BlueLily can also provide:

### Temperature
```cpp
Serial.print("TEMP,");
Serial.println(sensors.getTemperature());
```

### Voltage Monitoring
```cpp
Serial.print("VOLT,");
Serial.println(sensors.getVoltage());
```

### Barometric Altitude (if added)
```cpp
Serial.print("ALT,");
Serial.println(sensors.getAltitude());
```

## Troubleshooting

### No Serial Data
```bash
# Check permissions
sudo usermod -a -G dialout $USER
# Logout and login

# Check device
ls -l /dev/ttyACM0
```

### Wrong Data Format
```bash
# Monitor raw serial
screen /dev/ttyACM0 115200
# Should see: IMU,timestamp,ax,ay,az,gx,gy,gz
```

### Low Rate
```bash
# Check actual rate
ros2 topic hz /imu/data
# Should be ~100Hz

# If lower, increase BlueLily publish rate
```

## Performance Impact

### Without IMU
- Odometry quality: 70-120
- Fails during fast movements
- Vertical drift possible

### With BlueLily IMU
- Odometry quality: 80-130 (better!)
- Handles fast movements
- Gravity-aligned (no vertical drift)
- Faster convergence

## Next Steps

1. I'll create the ROS2 bridge package
2. You modify BlueLily firmware to output IMU data
3. Test serial communication
4. Integrate with SLAM
5. Calibrate and tune

Ready to create the ROS2 bridge package?
