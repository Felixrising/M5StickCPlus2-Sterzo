# ZTEERSTICK v0.1-RC2

A high-performance Zwift steering device using the M5StickCPlus2 with advanced IMU/AHRS implementation, intelligent power management, and seamless BLE connectivity.

## 🎯 About

This project transforms your M5StickCPlus2 into a professional-grade steering controller for Zwift cycling simulation called the **ZTEERSTICK**. Based on the excellent work from [@stefaandesmet2003/ESP32Sterzo](https://github.com/stefaandesmet2003/ESP32Sterzo.git), this implementation adds significant enhancements including:

- **Mahony AHRS algorithm** for stable yaw calculation with configurable hardware filtering
- **Intelligent activity detection** with position-based sleep system
- **Smooth 10Hz display updates** with selective region rendering (no flashing)
- **Advanced drift compensation** with distance-proportional centering
- **Comprehensive calibration system** with visual feedback
- **1-degree steering precision** with configurable update frequencies
- **Professional button handling** with instant yaw zeroing
- **Automatic brightness control** based on activity state

## 🛒 Hardware - M5StickC PLUS2

![M5StickC PLUS2 Overview](images/M5StickC-PLUS2-overview.jpeg)

![M5StickC PLUS2 Device](images/M5StickC-PLUS2-device.jpeg)

The M5StickC PLUS2 is a compact ESP32-based development board perfect for the ZTEERSTICK project. It features:

- **ESP32-PICO-V3-02** - Dual-core processor with built-in WiFi/BLE
- **MPU6886** - 6-axis IMU (gyroscope + accelerometer) with hardware low-pass filtering
- **135x240 Color LCD** - 1.14" ST7789V2 display with adaptive brightness
- **200mAh Battery** - Built-in LiPo with charging circuit
- **Compact Design** - 25.4mm x 54mm form factor
- **Multiple Buttons** - A, B, and C (power) buttons
- **USB-C** - For programming and charging

### 🛍️ Where to Buy

**[M5StickC PLUS2 on AliExpress](https://s.click.aliexpress.com/e/_okImkbs)** *(Affiliate Link)*

*Note: This is an affiliate link. Purchasing through this link supports the project development at no additional cost to you.*

## 🚀 Features

### Core Functionality
- ✅ **Zwift BLE Integration** - Full compatibility with Zwift steering protocol
- ✅ **Mahony AHRS Filter** - Superior stability with configurable 10Hz hardware LPF
- ✅ **1° Steering Precision** - Fine-grained control with configurable update rates
- ✅ **Intelligent Activity Detection** - Position-based motion sensing prevents false sleep
- ✅ **Instant Yaw Zeroing** - Button press immediately centers steering
- ✅ **Smart Drift Compensation** - Distance-proportional centering prevents oscillation

### Display & Interface
- 📱 **Smooth 10Hz Updates** - Flicker-free selective region rendering
- 📱 **Adaptive Brightness** - Auto-dims from 5% → 1% based on activity
- 📱 **Real-time Indicators** - Yaw angle, steering bin, connection status, battery
- 📱 **Visual Limit Alerts** - Flashing arrows at ±40° steering limits
- 📱 **Activity Status** - Clear indicators: Active (A), Idle (I), Sleep countdown (Zzzz)
- 📱 **Status Messages** - Full-screen redraw for calibration and system messages

### Power Management
- ⚡ **Position-Based Sleep** - 30-second activity timeout with 1-second position checks
- ⚡ **Deep Sleep Cycles** - 6-second timer wake-ups for motion detection
- ⚡ **GPIO4 Power Hold** - Proper M5StickCPlus2 power management
- ⚡ **Configurable Frequencies** - 10Hz IMU/display/motion processing
- ⚡ **RTC Memory** - Preserves wake counts and gravity baseline across sleep

### Button Controls
- 🔘 **Button A Short** - Instant yaw zero (no delay)
- 🔘 **Button A Long (2s)** - Full gyro calibration sequence
- 🔘 **Button B Short** - Instant yaw zero (no delay)
- 🔘 **Button B Long (3s)** - Full gyro calibration sequence
- 🔘 **Button C Short** - Wake display
- 🔘 **Button C Long (3s)** - Force deep sleep

## 🛠 Hardware Requirements

- **M5StickCPlus2** - See [Hardware section](#-hardware---m5stickc-plus2) above for details and purchase links
- **USB-C Cable** - For programming and charging
- **Mounting Solution** - Handlebar mount or secure attachment method (optional)

## 📦 Installation

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- [Git](https://git-scm.com/) for cloning

### Setup
```bash
# Clone the repository
git clone https://github.com/yourusername/ZTEERSTICK.git
cd ZTEERSTICK

# Build and upload
pio run --target upload

# Monitor serial output (optional)
pio device monitor
```

### Dependencies
All dependencies are automatically managed by PlatformIO:
- `M5Unified` - M5StickCPlus2 hardware abstraction
- `ESP32 BLE Arduino` - Bluetooth Low Energy support
- `Preferences` - Non-volatile storage

## 🎮 Usage

### First Time Setup
1. **Power On** - ZTEERSTICK performs automatic gyro calibration on first boot
2. **Zwift Pairing** - Open Zwift → Settings → Connections → Search for "STERZO"
3. **Calibration** - Hold Button A or B for 2-3 seconds for manual calibration if needed

### Normal Operation
1. **Mount Device** - Attach to handlebars or hold in hand
2. **Connect to Zwift** - Device advertises as "STERZO" 
3. **Steering Range** - ±40° physical rotation = full Zwift steering
4. **Auto-Centering** - Device gradually returns to center with smart drift compensation
5. **Instant Centering** - Press Button A or B for immediate yaw zero

### Activity States & Power Management
- **Active (A)** - Green indicator, 5% brightness, full performance
- **Idle (I)** - Yellow indicator, 1% brightness, reduced power
- **Sleep Countdown (Zzzz)** - Gray indicator with countdown, 1% brightness
- **Position Check** - Automatic 1-second motion detection every 30 seconds
- **Deep Sleep** - 6-second timer wake-ups, minimal power consumption

## 🔧 Technical Details

### IMU Configuration
- **Sensor** - MPU6886 6-axis IMU (gyroscope + accelerometer)
- **Algorithm** - Mahony AHRS for quaternion-based orientation
- **Hardware Filtering** - Configurable 10Hz low-pass filter
- **Update Rate** - Configurable 10Hz motion processing
- **Calibration** - Gyro bias compensation with visual feedback

### Display System
- **Update Rate** - Smooth 10Hz refresh with selective region rendering
- **Brightness Control** - Automatic 5% active / 1% idle brightness
- **Flicker-Free** - Only redraws changed regions, no screen clearing
- **State Transitions** - Full redraw on major activity state changes
- **Status Messages** - Full-screen overlay with automatic restoration

### BLE Protocol
- **Service UUID** - `347b0001-7635-408b-8918-8ff3949ce592`
- **Steering Data** - Float32 angle in degrees (-40° to +40°)
- **Update Rate** - On-change notification (1° resolution)
- **Zwift Handshake** - Full protocol compatibility with challenge/response

### Power Specifications
- **Active Current** - ~80-120mA (10Hz updates, BLE active, 5% brightness)
- **Idle Current** - ~30-50mA (10Hz updates, BLE active, 1% brightness)
- **Sleep Current** - ~5-10mA (6-second timer wake-ups)
- **Battery Life** - 3-5 hours active use, weeks standby

### Configuration Constants
```cpp
// Configurable system frequencies
const float IMU_LPF_FREQUENCY_HZ = 10.0f;        // Hardware filtering
const float MOTION_UPDATE_FREQUENCY_HZ = 10.0f;   // Software processing
const float DISPLAY_UPDATE_FREQUENCY_HZ = 10.0f;  // Display refresh

// Activity detection
const unsigned long ACTIVITY_TIMEOUT = 30000;     // 30 seconds
const unsigned long ACTIVITY_CHECK_DURATION = 1000; // 1 second
const float POSITION_CHANGE_THRESHOLD = 1.0f;     // 1 degree
```

## 🔄 Calibration System

### Automatic Calibration (First Boot)
1. **Gyro Bias** - 3000 samples over 3 seconds (device stationary)
2. **Visual Feedback** - Progress display with completion confirmation
3. **Instant Zero** - Current orientation set as perfect center

### Manual Calibration
- **Instant Zero** - Short press Button A or B (immediate response)
- **Full Calibration** - Long press Button A (2s) or Button B (3s)
- **Progress Feedback** - Real-time visual progress with status messages

## 🐛 Troubleshooting

### Common Issues

**Device not connecting to Zwift:**
- Ensure Bluetooth is enabled on your device
- Restart Zwift and search for "STERZO" in connections
- Try power cycling the ZTEERSTICK

**Steering feels unstable:**
- Perform full calibration (long press Button A or B)
- Ensure device is mounted securely
- Check battery level (low battery affects performance)

**Device goes to sleep despite movement:**
- Ensure you're making actual position changes (1° threshold)
- Activity detection works regardless of BLE connection
- Check serial monitor for "Position changed: YES" messages

**Display flashing or flickering:**
- RC2 version eliminates flashing with selective region updates
- If issues persist, check power supply and battery level

**Yaw drift over time:**
- Built-in drift compensation with distance-proportional centering
- Press Button A or B for instant zero (no delay)
- Long press for full recalibration if needed

**Screen too dim or bright:**
- Automatic brightness: 5% active, 1% idle
- Move device to trigger active state for brighter display
- Brightness changes automatically with activity state

### Debug Information
Enable serial monitoring to see detailed debug output:
```bash
pio device monitor
```

Debug output includes:
- IMU readings and filtering status
- Activity detection with position change details
- Display update timing and selective rendering
- BLE connection events and steering data
- Power management state transitions
- Button press detection and calibration progress

## 🔋 Battery Optimization Tips

1. **Automatic power management** - Device optimizes power based on activity
2. **Position-based sleep** - Only sleeps when truly inactive
3. **Efficient display updates** - 10Hz selective rendering saves power
4. **Deep sleep cycles** - Minimal power consumption during inactivity
5. **Regular charging** - Don't let battery fully discharge

## 🆕 Version 0.1-RC2 Improvements

### Major Fixes
- ✅ **Activity Detection Bug** - Fixed device sleeping despite active steering
- ✅ **Display Flashing** - Eliminated with selective region rendering
- ✅ **Timing Race Conditions** - Synchronized activity detection timing
- ✅ **Screen Redraw Issues** - Fixed status message screen restoration

### New Features
- 🆕 **Configurable System Architecture** - All frequencies configurable
- 🆕 **Hardware IMU Filtering** - 10Hz low-pass filter implementation
- 🆕 **Smart Activity States** - Active/Idle/Sleep with automatic transitions
- 🆕 **Adaptive Brightness** - Automatic 5%/1% brightness control
- 🆕 **Position-Based Detection** - 1-degree threshold motion sensing

### Performance Improvements
- ⚡ **10Hz Smooth Updates** - Consistent timing across all systems
- ⚡ **Selective Rendering** - Only redraws changed display regions
- ⚡ **Optimized Sleep Logic** - Proper deep sleep with GPIO4 hold
- ⚡ **Enhanced Drift Compensation** - Distance-proportional centering

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## 🏷️ About the Name

**ZTEERSTICK** combines "Z" (for Zwift), "STEER" (steering functionality), and "STICK" (the compact M5StickC form factor), creating a memorable name that perfectly describes this Zwift steering controller.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **[@stefaandesmet2003](https://github.com/stefaandesmet2003/ESP32Sterzo.git)** - Original ESP32Sterzo implementation
- **M5Stack** - M5StickCPlus2 hardware and libraries
- **Zwift** - Steering protocol documentation and support
- **Mahony** - AHRS algorithm implementation

## 📊 Project Status

- ✅ **Core Functionality** - Complete and stable (RC2)
- ✅ **Power Management** - Optimized with intelligent activity detection
- ✅ **Zwift Integration** - Fully compatible and tested
- ✅ **Display System** - Smooth 10Hz updates without flashing
- ✅ **Activity Detection** - Position-based motion sensing
- ✅ **Bug Fixes** - Major timing and display issues resolved
- 🔄 **Documentation** - Updated for RC2 features
- 🔄 **Testing** - Continuous validation with real-world usage

---

**Built with ❤️ for the Zwift cycling community**

*Version 0.1-RC2 - Release Candidate 2 with major stability and performance improvements* 