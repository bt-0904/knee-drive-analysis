# Knee Drive Analysis System

> [中文版 README](README_TW.md)

## Overview

This project uses an **ESP32-C3** microcontroller with a **Flex Sensor** to analyze knee drive motion during running via **Wi-Fi + MQTT** wireless transmission. The system records thigh angle and knee position coordinates to quantify how different knee lift heights affect running efficiency.

### Key Features

- ✅ **Flex Sensor Angle Detection**: Simple and reliable bend angle measurement
- ✅ **FreeRTOS Dual-Task Architecture**: Separated sampling and transmission for data integrity
- ✅ **50Hz Sampling + 2Hz Batch Transmission**: Reduced network overhead, improved efficiency
- ✅ **Ring Buffer Mechanism**: Tolerates up to 4 seconds of network interruption without data loss
- ✅ **Fixed Offset Calibration**: One-time calibration for long-term use
- ✅ **Real-time Coordinate Calculation**: Knee Y/Z axis position tracking
- ✅ **CSV Data Storage**: Complete recording for subsequent analysis

## Hardware Requirements

### Bill of Materials

| Component              | Specification                  | Qty |
| ---------------------- | ------------------------------ | --- |
| ESP32-C3 Super Mini    | RISC-V architecture, Wi-Fi/BLE | 1   |
| Flex Sensor            | 2.2" or 4.5" bend sensor       | 1   |
| Resistor               | 10KΩ (1/4W)                    | 1   |
| Lithium Battery        | 3.7V (18650/103450)            | 1   |
| Battery Charging Board | TP4056 or similar              | 1   |

### Wiring Configuration

#### Flex Sensor Voltage Divider Circuit

```
3.3V ──────┬────── Flex Sensor ──────┬────── GPIO4 (ADC)
           │                         │
           └─────── 10KΩ ────────────┴────── GND
```

| Connection Point                     | ESP32-C3 Super Mini |
| ------------------------------------ | ------------------- |
| Flex Sensor (one end)                | 3.3V                |
| Flex Sensor (other end) + 10KΩ (end) | GPIO4 (ADC Input)   |
| 10KΩ (other end)                     | GND                 |

#### Power System

| Charging Module | ESP32-C3 Super Mini  |
| --------------- | -------------------- |
| OUT+            | 5V (via Type-C)      |
| OUT-            | GND                  |
| BAT+            | Battery Positive (+) |
| BAT-            | Battery Negative (-) |

### System Circuit Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      Power System                            │
│  ┌──────────┐      ┌──────────┐      ┌──────────────────┐   │
│  │ 3.7V     │      │ TP4056   │      │ ESP32-C3         │   │
│  │ Li-ion   │─BAT→─│ Charging │─OUT→─│ Super Mini       │   │
│  │ Battery  │      │ Module   │      │                  │   │
│  └──────────┘      └──────────┘      │   ┌────────────┐ │   │
│                                      │   │ GPIO4 (ADC)│←┼───┼─── Flex Sensor
│                                      │   │ GPIO8 (LED)│ │   │    + 10KΩ Divider
│                                      │   │ 3.3V / GND │ │   │
│                                      │   └────────────┘ │   │
│                                      └──────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**Important Notes**:

- **GPIO4** is used for ADC input (ADC1_CH4), avoiding strapping pins
- **GPIO8** is the onboard LED (LOW = ON, HIGH = OFF)
- The 10KΩ resistor and Flex Sensor form a voltage divider, converting bend angle to voltage change

## Software Requirements

### ESP32 Development Environment

- **Platform**: PlatformIO
- **Framework**: Arduino
- **MCU**: ESP32-C3
- **Language**: C++
- **Protocol**: Wi-Fi + MQTT

### Data Receiver Environment

- **Language**: Python 3.x
- **Required Packages**: paho-mqtt
- **Data Format**: JSON (batch) → CSV

## System Architecture

### Data Flow

```
Flex Sensor ADC (50Hz) → FreeRTOS Ring Buffer (200 samples) → MQTT Batch (2Hz) → Python → CSV
```

### FreeRTOS Dual-Task Architecture

```
┌─────────────────┐    Ring Buffer    ┌─────────────────┐
│  samplingTask   │ ──────────────→  │  transmitTask   │
│  (50Hz, Core 0) │   Mutex Protected │  (2Hz, Core 0)  │
│  Priority 3     │   200 samples     │  Priority 1     │
│  Stack: 4KB     │                   │  Stack: 8KB     │
└─────────────────┘                   └─────────────────┘
        ↓                                     ↓
   Read ADC Value                        Batch JSON Packaging
   Angle Calculation                     MQTT Transmission (25/batch)
   Coordinate Transform                  Statistics Output
   Write to Buffer
```

### Overall System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                      Sensor Layer (ESP32-C3)                        │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐            │
│  │ Flex Sensor  │──→│ samplingTask │──→│ Ring Buffer  │            │
│  │ ADC (50Hz)   │   │ Angle/Coord  │   │ (200 samples)│            │
│  └──────────────┘   └──────────────┘   └──────┬───────┘            │
│                                               │                     │
│                     ┌──────────────┐          │                     │
│                     │ transmitTask │←─────────┘                     │
│                     │ Batch JSON   │                                │
│                     └──────┬───────┘                                │
└────────────────────────────┼───────────────────────────────────────┘
                             │ Wi-Fi + MQTT
                             ↓
┌────────────────────────────────────────────────────────────────────┐
│                      Transport Layer                                │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐            │
│  │ Mobile       │──→│ 4G/5G        │──→│ MQTT Broker  │            │
│  │ Hotspot      │   │ Network      │   │ Cloud Server │            │
│  └──────────────┘   └──────────────┘   └──────┬───────┘            │
└────────────────────────────────────────────────┼───────────────────┘
                                                 │ Internet
                                                 ↓
┌────────────────────────────────────────────────────────────────────┐
│                      Receiver Layer (Computer)                      │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐            │
│  │ Python       │──→│ Parse JSON   │──→│ CSV Storage  │            │
│  │ mqtt_receiver│   │ Batch Data   │   │ Timestamped  │            │
│  └──────────────┘   └──────────────┘   └──────────────┘            │
└────────────────────────────────────────────────────────────────────┘
```

### Core Algorithms

#### 1. ADC to Angle Conversion (Linear Mapping)

```cpp
// Flex Sensor: Lower ADC value = Greater bend angle (inverse resistance)
float angle = (float)(OFFSET_FLAT_ADC - rawValue) / (OFFSET_FLAT_ADC - OFFSET_BENT_ADC) * 90.0;
```

**Parameters**:

- `OFFSET_FLAT_ADC`: ADC reading when Flex Sensor is flat (0°), approximately 2358
- `OFFSET_BENT_ADC`: ADC reading when Flex Sensor is bent (90°), approximately 1737
- Output range: 0° (standing) to 90° (knee horizontal)

#### 2. Coordinate Transformation

```cpp
// Polar to Cartesian coordinates (hip joint as origin)
float angleRad = angle * PI / 180.0;
kneeY = THIGH_LENGTH * sin(angleRad);   // Forward/backward displacement (0~45cm)
kneeZ = -THIGH_LENGTH * cos(angleRad);  // Up/down displacement (-45~0cm)
```

**Coordinate System Definition**:

- Origin (0, 0): Hip joint position
- Y-axis: Forward/backward direction (positive = forward)
- Z-axis: Up/down direction (negative = downward)
- Standing position: kneeZ ≈ -45cm (knee directly below hip)

## 🔧 Flex Sensor Calibration Guide

### Why Calibration?

Each Flex Sensor has slightly different resistance characteristics. Calibration ensures accurate angle calculation. **Calibration only needs to be done once**, and the values are stored in the code.

### Calibration Steps

#### Step 1: Prepare Hardware

1. Complete Flex Sensor wiring (refer to wiring configuration above)
2. Connect ESP32 to computer via USB
3. Attach Flex Sensor to thigh (or simulate by hand)

#### Step 2: Observe ADC Readings

1. Open **Serial Monitor** in PlatformIO (115200 baud)
2. After uploading the program, observe the `ADC` field in serial output

#### Step 3: Record ADC Values for Two Positions

| Position       | Description                  | Recorded ADC Value |
| -------------- | ---------------------------- | ------------------ |
| **Flat (0°)**  | Standing, leg fully extended | e.g., 2358         |
| **Bent (90°)** | Knee raised to horizontal    | e.g., 1737         |

> 💡 **Tip**: Hold each position for 3-5 seconds to get a stable average ADC reading

#### Step 4: Update Code Constants

Edit `src/main.cpp` (around lines 48-49):

```cpp
// ===== Fixed Calibration Offset (Pre-measured baseline values) =====
#define OFFSET_FLAT_ADC 2358  // ← Replace with your flat ADC value
#define OFFSET_BENT_ADC 1737  // ← Replace with your bent ADC value
```

#### Step 5: Re-upload Program

1. Use PlatformIO **Upload** to flash the new program
2. Check Serial Monitor to verify correct angle display:
   - Standing should show approximately 0°
   - Knee horizontal should show approximately 90°

### Calibration Verification

| Check Item             | Expected Result                  |
| ---------------------- | -------------------------------- |
| Standing angle         | 0° ± 5°                          |
| Knee horizontal angle  | 90° ± 5°                         |
| Angle change direction | Angle increases when lifting leg |

> ⚠️ If angle direction is reversed, swap `OFFSET_FLAT_ADC` and `OFFSET_BENT_ADC` values

## Project Structure

```
knee-drive-analysis/
├── platformio.ini           # PlatformIO configuration file
├── src/
│   └── main.cpp             # ESP32 main program (FreeRTOS + MQTT)
├── include/                 # Header files
├── lib/                     # Project libraries
├── test/                    # Test programs
├── mqtt_receiver.py         # Python MQTT data receiver script
├── analyze_knee_motion.py   # Python data analysis and visualization
├── USAGE.md                 # Detailed usage guide
├── README.md                # Project description (this file)
├── README_TW.md             # Chinese version README
└── .github/
    └── copilot-instructions.md  # AI development guidelines
```

## Quick Start

### Step 1: Hardware Setup

1. Connect ESP32-C3 and Flex Sensor according to wiring configuration
2. Connect 3.7V lithium battery to charging module (mind polarity)
3. Connect charging module output to ESP32-C3 Super Mini via Type-C
4. Prepare mobile phone with Wi-Fi hotspot enabled (SSID: `Bt`, Password: `bt_980904`)

### Step 2: Upload ESP32 Program

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension
3. Connect ESP32 to computer via USB
4. (Optional) Modify Wi-Fi settings in `src/main.cpp`
5. Click **Upload** button in PlatformIO sidebar

**First Upload Note**:

- Hold **BOOT** button → Press **RESET** → Release → Start upload

### Step 3: Calibrate Flex Sensor

For first-time use, refer to the "🔧 Flex Sensor Calibration Guide" section above.

### Step 4: Install Python Environment

```bash
# Install required packages
pip install paho-mqtt matplotlib pandas numpy
```

### Step 5: Start Data Collection

1. Enable mobile hotspot (provides ESP32 network connection)
2. Power on ESP32 (will automatically connect to Wi-Fi and start transmission)
3. Run receiver script on computer:
   ```bash
   python mqtt_receiver.py
   ```
4. Start running, data will be saved to CSV in real-time

### Step 6: Data Analysis

```bash
python analyze_knee_motion.py
```

### Complete Usage Guide

Please refer to [USAGE.md](USAGE.md) for detailed setup and usage instructions.

## Features

### Completed ✅

- [x] Flex Sensor ADC reading (50Hz sampling)
- [x] FreeRTOS dual-task architecture (sampling + transmission separated)
- [x] Ring Buffer mechanism (200 sample capacity)
- [x] Moving average filter (5-point smoothing)
- [x] Wi-Fi connectivity
- [x] MQTT batch data transmission (2Hz, 25 samples/batch)
- [x] Fixed Offset calibration system
- [x] Knee Y/Z coordinate calculation (relative to hip joint)
- [x] Stability detection
- [x] Real-time data display (Serial + MQTT)
- [x] Python data receiver script (batch format support)
- [x] CSV format data storage
- [x] Data analysis and visualization script

### In Development 🚧

- [ ] Running efficiency evaluation algorithm
- [ ] Web-based real-time visualization interface
- [ ] Historical data analysis tools

## Research Goals

1. **Data Collection**: Record knee motion data at different running speeds
2. **Feature Extraction**: Analyze key metrics like lift height and frequency
3. **Efficiency Evaluation**: Establish correlation model between knee lift height and running efficiency
4. **Optimization Suggestions**: Provide personalized running posture recommendations based on analysis

## Technical Specifications

### Hardware Specifications

| Item              | Specification                              |
| ----------------- | ------------------------------------------ |
| Sensor            | Flex Sensor (bend sensor)                  |
| ADC Pin           | GPIO4 (ADC1_CH4)                           |
| LED Pin           | GPIO8 (onboard LED)                        |
| Sampling Rate     | 50 Hz                                      |
| Transmission Rate | 2 Hz (one batch every 500ms)               |
| Batch Size        | 25 samples/batch                           |
| Buffer Capacity   | 200 samples (tolerates 4s network outage)  |
| Operating Voltage | 3.0V - 4.2V (lithium battery powered)      |
| Operating Current | Approx. 80-150mA (during Wi-Fi connection) |

### Software Specifications

| Item             | Specification                            |
| ---------------- | ---------------------------------------- |
| Serial Baud Rate | 115200 baud                              |
| MQTT QoS         | 0 (at most once delivery)                |
| JSON Format      | Batch format `{"type":"batch",...}`      |
| Thigh Length     | 45 cm (adjustable)                       |
| Filter           | Moving average (5-point)                 |
| FreeRTOS Tasks   | samplingTask (50Hz) + transmitTask (2Hz) |

### Data Format

- **Transmission Format**: JSON (batch)
- **Storage Format**: CSV (UTF-8 with BOM)
- **Timestamp**: Millisecond precision
- **Coordinate System**: Y/Z 2D coordinates (hip joint as origin)

## Data Output Format

### MQTT JSON Format (Batch)

```json
{
  "type": "batch",
  "n": 25,
  "d": [
    {
      "t": 1.23,
      "a": 45.0,
      "s": 1,
      "dy": 28.3,
      "dz": -12.5
    },
    {
      "t": 1.25,
      "a": 46.2,
      "s": 1,
      "dy": 29.1,
      "dz": -11.8
    }
  ]
}
```

**Field Descriptions**:

| Field | Description                  | Unit    |
| ----- | ---------------------------- | ------- |
| `t`   | Runtime (timestamp)          | seconds |
| `a`   | Thigh lift angle             | degrees |
| `s`   | Stability flag               | 0/1     |
| `dy`  | Knee forward/backward offset | cm      |
| `dz`  | Knee up/down offset          | cm      |

### CSV Columns

| Column     | Description       | Unit    |
| ---------- | ----------------- | ------- |
| Timestamp  | Data receive time | -       |
| Runtime(s) | ESP32 runtime     | seconds |
| Angle(deg) | Thigh lift angle  | degrees |
| ΔY(cm)     | Knee forward/back | cm      |
| ΔZ(cm)     | Knee up/down      | cm      |
| Stable     | Value stability   | Yes/No  |

## Troubleshooting

### Flex Sensor Issues

- **ADC reading not changing**
  - Check Flex Sensor wiring connections
  - Verify 10KΩ resistor is properly connected
  - Check GPIO4 pin contact

- **Inaccurate angle calculation**
  - Re-run calibration procedure
  - Verify `OFFSET_FLAT_ADC` and `OFFSET_BENT_ADC` values are correct
  - Check if Flex Sensor is securely attached

### Wi-Fi Connection Failed

- Check if mobile hotspot is enabled
- Verify SSID and password are correct (case-sensitive)
- Keep ESP32 within 5-10 meters of phone

### MQTT Transmission Failed

- Check serial monitor status output
- Verify network connection is stable
- Check if MQTT broker is accessible

### Unstable Data

- Ensure Flex Sensor is securely attached
- Check battery power stability (voltage > 3.3V)
- Moving average filter will smooth short-term fluctuations

## References

- [ESP32-C3 Technical Documentation](https://www.espressif.com/en/products/socs/esp32-c3)
- [Flex Sensor Usage Guide](https://www.sparkfun.com/products/10264)
- [MQTT Protocol Documentation](https://mqtt.org/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [FreeRTOS Official Documentation](https://www.freertos.org/)

## License

This project is licensed under the MIT License.

## Authors

Knee Drive Analysis Project Team

---

**Last Updated**: 2026-01-24
