# ZTEERSTICK v0.1-RC3

Professional Zwift steering device for M5StickCPlus2 with advanced IMU filtering, intelligent power management, and smooth 10Hz display updates.

## 🙏 Based on ESP32Sterzo

This is an **IMU-driven fork** of the excellent **[@stefaandesmet2003/ESP32Sterzo](https://github.com/stefaandesmet2003/ESP32Sterzo.git)** project, optimized for M5StickCPlus2 but **adaptable to any ESP32 with an IMU**. 

Major enhancements include Mahony AHRS filtering, position-based sleep system, and professional power management.

## 🎯 Features

- **Zwift BLE Integration** - Full compatibility with handshake authentication
- **Mahony AHRS + 10Hz Hardware Filtering** - Ultra-stable yaw calculation
- **Position-Based Sleep System** - 6-second wake intervals, 1° motion threshold
- **Smooth 10Hz Display** - Flicker-free updates with selective rendering
- **Zero-Yaw Functionality** - Instant steering center with button press
- **Activity Detection** - Works regardless of BLE connection status
- **Professional Power Management** - Active (5%) → Idle (1%) → Sleep countdown

## 🛒 Hardware

**M5StickC PLUS2** - [Buy on AliExpress](https://s.click.aliexpress.com/e/_okImkbs) *(Affiliate Link)*

- ESP32-PICO-V3-02 with WiFi/BLE
- MPU6886 6-axis IMU with hardware filtering
- 135x240 Color LCD with adaptive brightness
- 200mAh battery with USB-C charging
- Compact 25.4mm x 54mm design

## 📦 Installation

```bash
# Clone and build
git clone https://github.com/Felixrising/ZteerStick.git
cd ZteerStick
pio run --target upload
```

**Dependencies:** M5Unified, ESP32 BLE Arduino, Preferences (auto-managed by PlatformIO)

## 🎮 Usage

### Setup
1. **Power On** - Auto gyro calibration on first boot
2. **Zwift Pairing** - Search for "STERZO" in Zwift connections
3. **Zero Yaw** - Press Button A or B to set center position

### Controls
- **Button A/B Short** - Zero yaw to current position
- **Button A/B Long** - Full gyro calibration (2-3s hold)
- **Button C** - Power/wake functionality

### Operation
- **Steering Range** - ±40° physical rotation = full Zwift steering
- **Activity States** - Green dot (active), yellow dot (idle), countdown (sleep)
- **Auto-Sleep** - 30-second timeout, wakes every 6 seconds to check position

## 🔧 Technical Details

### IMU & Filtering
- **MPU6886** with 10Hz hardware low-pass filter
- **Mahony AHRS** for quaternion-based orientation
- **Gyro Calibration** - 3000-sample bias compensation
- **Motion Detection** - Gravity vector comparison (1° threshold)

### Power Management
- **Position-Based Sleep** - Deep sleep with RTC memory persistence
- **GPIO4 Power Hold** - Proper M5StickCPlus2 power management
- **Adaptive Brightness** - Automatic adjustment based on activity state

### BLE Protocol
- **Service UUID** - `347b0001-7635-408b-8918-8ff3949ce592`
- **Steering Data** - Float32 angle (-40° to +40°)
- **Update Rate** - On-change notification (1° resolution)

## 🐛 Troubleshooting

**Not connecting to Zwift:** Enable Bluetooth, restart Zwift, search for "STERZO"

**Goes to sleep despite movement:** Ensure actual orientation changes (>1°), not just vibration

**Steering unstable:** Calibrate (long press A/B), secure mounting, zero yaw (short press A/B)

**Display issues:** System handles selective updates automatically

**Battery drain:** Check sleep system working (green → yellow → countdown → sleep)

## 📊 v0.1-RC3 Status

✅ **Stable Core** - Eliminated resets, fixed activity detection  
✅ **Smooth Display** - No flashing, 10Hz updates, state transitions  
✅ **Power Optimized** - Position-based sleep, timing synchronization  
✅ **Hardware Filtered** - 10Hz LPF, configurable architecture  

### Key Improvements
- Fixed display redraw after status messages
- Eliminated timing race conditions  
- Added major state transition detection
- Improved activity detection reliability

## 🙏 Additional Thanks

- **M5Stack** - M5StickCPlus2 hardware and libraries
- **Zwift** - Steering protocol documentation and support

---

**Built with ❤️ for the Zwift cycling community** 