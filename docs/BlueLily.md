
---

# BlueLily
A High-Performance Sensing, Control, Communication and Realtime Logic Array, customizable for Robotics, Flight Computer, High-Powered Rocketry, Payloads, and CubeSats

### **Project Goal**
BlueLily is designed as a **versatile flight computer** tailored for **high-powered rocketry**, **payloads**, and **CubeSats**. Its primary objectives are to:
- **Collect and process real-time sensor data** (temperature, IMU, ADC readings) for flight monitoring.
- **Facilitate robust, multi-method communication** (RS485, CANBUS, Bluetooth, LoRa) for telemetry and remote control.
- **Log comprehensive flight data** to SD and flash storage for post-flight analysis.
- **Control actuators** (relays, PWM devices) autonomously or remotely from a ground station.
- **Provide an interactive user interface** via buttons, potentiometer, LEDs, and an OLED display.
- **Ensure reliability and flexibility** through a modular design, state machine-based flight logic, and runtime configurability.

Built around the **Teensy 4.1 microcontroller**, BlueLily integrates advanced sensors, communication protocols, dual storage, actuation mechanisms, and a human interface device (HID) system, all orchestrated by a sophisticated flight controller.

---

### **Modules and Features**

#### **Sensors**
- **MAX31855** (Adafruit Thermocouple Module):
  - Measures temperature via a thermocouple.
  - Used for thermal monitoring during flight.
- **MPU6500** (Inertial Measurement Unit):
  - Tracks 6-axis motion (accel X/Y/Z, gyro X/Y/Z).
  - Key for detecting liftoff, apogee, and orientation.
- **ADS1115** (16-bit ADC):
  - Provides precise analog measurements across 4 channels.
  - Enables additional sensor inputs (e.g., voltage monitoring).

#### **Communication**
- **RS485 (MAX485 Breakout Board):**
  - Robust, long-distance serial communication.
  - Supports high-speed telemetry and remote commands.
- **CANBUS (MCP2515):**
  - Industrial-grade networking for multi-device integration.
  - Reliable data exchange between onboard systems.
- **Bluetooth (HC-05):**
  - Wireless short-range communication.
  - Ideal for pre-flight configuration and diagnostics.
- **LoRa (SX1278):**
  - Long-range, low-power wireless transmission.
  - Primary method for in-flight telemetry over extended distances.
- **Features:**
  - All sensor data relayed at configurable intervals (e.g., 100ms).
  - State change notifications broadcast via all enabled methods.
  - Remote actuator control supported from any method.

#### **Storage (Logger)**
- **Teensy Built-in SD Card:**
  - Primary storage for high-rate flight data logging.
  - Pre-allocated file for efficient writes.
- **W25Q128 Flash (16MB):**
  - Secondary high-speed buffer for critical data.
  - Syncs to SD during flight or post-landing.
- **Features:**
  - Logs timestamped sensor data, flight states, and events.
  - Data preview available post-flight via HID.

#### **Actuation**
- **Solid-State Relay (Pin 21):**
  - Controls high-power loads (e.g., parachute deployment).
  - Triggered autonomously at apogee or remotely.
- **PWM Actuator (Pin 29):**
  - Supports variable control (e.g., servos, motors).
  - Configurable via scheduling or remote commands.
- **Features:**
  - State machine-driven or remote overrides.
  - Scheduled events configurable via SD file (pending implementation).

#### **Human Interface Device (HID)**
- **Input:**
  - **4 Buttons:** Select (26), Back (27), and 2 spares for future use.
  - **Potentiometer (Pin 17):** Menu navigation and settings adjustment.
- **Output:**
  - **4 LEDs:** Red (20), Yellow (22), Green (37), Blue (36) for boot animation and status.
  - **SSD1306 OLED (128x64):** Displays menus, settings, and real-time previews.
- **Features:**
  - Multi-level menu system for module configuration.
  - Real-time sensor data previews with horizontal scrolling.
  - Vertical menu scrolling for >4 items.
  - Boot animation on LEDs during startup.

#### **Flight Controller**
- **Core Logic:**
  - Manages flight phases via a state machine:
    - **IDLE:** Pre-launch standby.
    - **ARMED:** Ready, awaiting liftoff detection.
    - **ASCENT:** High-rate data collection during climb.
    - **APOGEE:** Triggers deployment (e.g., relay on).
    - **DESCENT:** Monitors descent parameters.
    - **LANDED:** Finalizes logging and shutdown.
  - Integrates sensor data, actuation, logging, communication, and HID updates.
- **Features:**
  - State transitions based on accel Z (liftoff), velocity/altitude (apogee/landing).
  - Broadcasts state changes via all communication methods.
  - Relays all sensor data (temp, accel, gyro, voltages, altitude) at 100ms intervals.
  - Supports remote actuator control and state override from ground station.

---

### **Implementation Details**
- **Teensy 4.1:** High-performance microcontroller with ample I/O and processing power.
- **Modular Design:** Each module (`Sensors`, `Communication`, etc.) is enableable/disableable via `Config.h`.
- **State Machine:** Driven by sensor thresholds (e.g., 20 m/s² for liftoff) and time, with runtime override capability.
- **Data Flow:**
  - Sensors → FlightController → Logger/Communication/HID/Actuation.
  - Ground station commands → Communication → Configurator → FlightController/Actuation.
- **Pending Features:**
  - `loadScheduleFromSD()`: Load actuator schedules from SD (`schedule.txt`).
  - Barometric altitude integration for precise height measurement.

---

### **Usage**
- **Pre-Flight:**
  - Configure via HID menu (enable/disable modules, view previews).
  - Arm system (5s delay in IDLE → ARMED).
- **In-Flight:**
  - Autonomous state transitions and actuator control.
  - Continuous telemetry broadcast and data logging.
  - Remote control via ground station commands (e.g., "ACT0=1").
- **Post-Flight:**
  - Review logged data via HID preview or SD/flash extraction.

---

### **Hardware Pinout**
- **Sensors:** MAX31855 (10), MPU6500 (I2C), ADS1115 (I2C).
- **Communication:** RS485 (6-8), CANBUS (9), Bluetooth (0-1), LoRa (15-17).
- **Actuation:** Relay (21), PWM (29).
- **HID:** Buttons (26, 27), Pot (17), LEDs (20, 22, 37, 36), OLED (I2C).

---

### **Future Enhancements**
- **Barometer:** Add BMP280 for accurate altitude.
- **Advanced Telemetry:** Include real packet counts and error rates.
- **SD Scheduling:** Implement `loadScheduleFromSD()` for pre-programmed actuation.
- **Redundancy:** Dual-sensor checks for critical state transitions.

BlueLily combines flexibility, reliability, and performance, making it an ideal flight computer for advanced rocketry applications.

--- 
