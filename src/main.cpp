#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_pm.h>

// —————— Function Declarations ——————
void performGyroCalibration();
void recenterYaw();
void sendSteeringBin(float bin);
void filterGyroReadings(float gx, float gy, float gz);
bool calibrationExists();
void performAutoCalibration();
void performAutoRecenter();
void initializeMahonyFilter();
void quickRecenterYaw();
bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az);
void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az);
void updateLEDBreathing();
void configurePowerManagement();
void setupIMUWakeup();
void handleWakeupFromSleep();
void enterScreenOffMode();
void exitScreenOffMode();
void enterULPSleep();
void wakePulse();
bool waitForInterval(unsigned long &lastTime, unsigned long intervalMicros);
void enterBLEWaitingMode();
void enterLowPowerMode();
void enterBLEActiveMode();
void stopBLE();
void startBLE();

// —————— NEW DISPLAY MANAGEMENT SYSTEM ——————
void initializeDisplay();
void updateMainDisplay();
void showStatusMessage(const char* title, const char* message, uint16_t color = WHITE, int duration = 2000);
void showProgressBar(const char* title, int progress, int total);
void showButtonHelp();
void clearDisplaySafe();
void setDisplayOrientation(bool landscape = false);
bool shouldUseLandscape(const char* text);
void drawCenteredText(const char* text, int y, int textSize = 2, uint16_t color = WHITE);
void drawStatusBar();
void showCalibrationScreen(const char* stage, int progress = -1);
void showWakeupScreen(const char* reason, const char* details = nullptr);
void showConnectionScreen(bool connected);

// —————— Pin & BLE UUIDs ——————
static const int LED_PIN = 19;    // onboard LED on GPIO19

#define STERZO_SERVICE_UUID "347b0001-7635-408b-8918-8ff3949ce592"
#define CHAR14_UUID         "347b0014-7635-408b-8918-8ff3949ce592"
#define CHAR30_UUID         "347b0030-7635-408b-8918-8ff3949ce592"
#define CHAR31_UUID         "347b0031-7635-408b-8918-8ff3949ce592"
#define CHAR32_UUID         "347b0032-7635-408b-8918-8ff3949ce592"

// —————— Mahony AHRS Implementation ——————
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

float twoKp = twoKpDef;    // 2 * proportional gain (Kp)
float twoKi = twoKiDef;    // 2 * integral gain (Ki)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // integral error terms scaled by Ki
float invSampleFreq;
char anglesComputed;

// —————— Globals ——————
Preferences    prefs;

// BLE objects
BLEServer*        pServer;
BLEService*       pSvc;
BLECharacteristic* pChar14;
BLECharacteristic* pChar30;
BLECharacteristic* pChar31;
BLECharacteristic* pChar32;
BLE2902*          p2902_14;
BLE2902*          p2902_30;
BLE2902*          p2902_32;

bool deviceConnected = false;
bool ind32On        = false;
bool challengeOK    = false;

// IMU calibration & steering state
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;
float accelBiasX=0, accelBiasY=0, accelBiasZ=0;
float yawOffset   = 0;
float lastSentBin = NAN;
bool isCalibrating = false;

// Yaw drift compensation
float yawDriftRate = 0.5f;  // degrees per second drift back to center

// Gyro filtering
float gyroFilterAlpha = 0.1f; // Low-pass filter coefficient (0.0 = no filtering, 1.0 = no change)
float filteredGx = 0, filteredGy = 0, filteredGz = 0;

// Timing for dynamic sample frequency
float dt;
float loopfreq;

// —————— NEW POWER MANAGEMENT SYSTEM ——————
// Power mode states
enum PowerMode {
  POWER_BLE_ACTIVE,    // BLE active, 80MHz CPU, screen on/off based on timer
  POWER_BLE_WAITING,   // Waiting for BLE connection, 80MHz CPU, reduced activity
  POWER_LOW_POWER,     // No BLE, 10MHz CPU, IMU/button monitoring only
  POWER_ULP_SLEEP      // ULP coprocessor monitoring, main CPU off
};
PowerMode currentPowerMode = POWER_BLE_WAITING;

// Guard flag to prevent multiple enterULPSleep() calls
bool enteringSleep = false;

// —————— DISPLAY MANAGEMENT SYSTEM ——————
// Display mode states
enum DisplayMode {
  DISPLAY_MAIN,           // Main steering display
  DISPLAY_STATUS,         // Status/message display
  DISPLAY_CALIBRATION,    // Calibration progress
  DISPLAY_WAKEUP,         // Wake-up information
  DISPLAY_CONNECTION      // BLE connection status
};

// Display constants
const int DISPLAY_WIDTH_PORTRAIT = 135;
const int DISPLAY_HEIGHT_PORTRAIT = 240;
const int DISPLAY_WIDTH_LANDSCAPE = 240;
const int DISPLAY_HEIGHT_LANDSCAPE = 135;
const int STATUS_BAR_HEIGHT = 20;
const int BUTTON_HELP_HEIGHT = 25;

// Display state variables
DisplayMode currentDisplayMode = DISPLAY_MAIN;
bool displayLandscape = false;
unsigned long lastDisplayUpdate = 0;
unsigned long statusMessageEndTime = 0;

// Timing constants
const unsigned long SCREEN_ON_TIMEOUT = 60000;    // 60 seconds screen on after button
const unsigned long BLE_WAIT_TIMEOUT = 300000;     // 5 minutes waiting for BLE connection
const unsigned long LOW_POWER_TIMEOUT = 120000;    // 2 minutes in low power before ULP sleep
const unsigned long ULP_WAKE_INTERVAL = 6000;     // 6 seconds ULP wake interval

// Activity tracking
unsigned long lastButtonTime = 0;
unsigned long lastMotionTime = 0;
unsigned long lastBLEActivityTime = 0;
bool screenOn = false;

// IMU frequency management
const float NORMAL_IMU_FREQUENCY = 10.0f;     // 10Hz for normal operation
const float CALIBRATION_IMU_FREQUENCY = 50.0f; // 50Hz during calibration
float currentIMUFrequency = NORMAL_IMU_FREQUENCY;

// CPU frequency management
const int BLE_CPU_FREQ = 80;         // 80MHz minimum for BLE operation
const int LOW_POWER_CPU_FREQ = 10;   // 10MHz for I2C/IMU/button checks (no BLE)
const int DEEP_SLEEP_CPU_FREQ = 10;  // 10MHz for ULP wake checks

// LED breathing pattern
unsigned long ledBreathingStartTime = 0;
bool ledBreathingEnabled = true;

// Motion detection with noise filtering
const float MOTION_THRESHOLD = 1.0f; // degrees/second for gyro motion detection (reduced for better sensitivity)
const float ACCEL_MOTION_THRESHOLD = 0.1f; // g-force for accelerometer motion detection (reduced for better sensitivity)
float lastAccelMagnitude = 1.0f; // Initialize to 1g (gravity)
float lastGyroMagnitude = 0.0f;
float motionAccumulator = 0.0f;
unsigned long lastMotionAccumulatorReset = 0;

// Store the last measured raw yaw and timestamp from recentering
float lastMeasuredRawYaw = 0.0f;
unsigned long lastYawMeasurementTime = 0;
bool hasValidYawMeasurement = false;

// ULP coprocessor variables
RTC_DATA_ATTR int ulp_wake_count = 0;
RTC_DATA_ATTR int ulp_motion_detected = 0;
unsigned long bleStartTime = 0;

// —————— Utility: 32-bit rotate + hash ——————
static uint32_t rotate_left32(uint32_t value, uint32_t count) {
  const uint32_t mask = (CHAR_BIT * sizeof(value)) - 1;
  count &= mask;
  return (value << count) | (value >> (-count & mask));
}

static uint32_t hashed(uint64_t seed) {
  uint32_t ret = (uint32_t)(seed + 0x16fa5717);
  uint64_t rax = seed * 0xba2e8ba3ULL;
  uint64_t eax = (rax >> 35) * 0xb;
  uint64_t ecx = seed - eax;
  uint32_t edx = rotate_left32((uint32_t)seed, (uint32_t)(ecx & 0x0F));
  return ret ^ edx;
}

// —————— Mahony AHRS Functions ——————
void initMahonyData() {
  twoKp = twoKpDef;  // 2 * proportional gain (Kp)
  twoKi = twoKiDef; // 2 * integral gain (Ki)
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  anglesComputed = 0;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float samplefrequency) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply proportional feedback
    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / samplefrequency));    // pre-multiply common factors
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
}

float getYaw() {
  // Calculate yaw directly from quaternion components
  // This is the correct formula for yaw (heading) from quaternion
  float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  
  // Convert to degrees
  yaw *= 57.29578f;
  
  // Normalize to [-180, 180] range
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;
  
  // Invert yaw direction: clockwise rotation = positive, counter-clockwise = negative
  return -yaw;
}

float getRoll() {
  // Calculate roll directly from quaternion components
  float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  return roll * 57.29578f;
}

float getPitch() {
  // Calculate pitch directly from quaternion components
  float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
  return pitch * 57.29578f;
}

// —————— Power Management Functions ——————

void configurePowerManagement() {
  Serial.println("Configuring ESP32 power management...");
  
  // Configure automatic light sleep with BLE-compatible frequencies
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = BLE_CPU_FREQ;  // 80MHz for BLE compatibility
  pm_config.min_freq_mhz = LOW_POWER_CPU_FREQ; // 10MHz minimum for I2C/GPIO
  pm_config.light_sleep_enable = true; // Enable automatic light sleep
  
  esp_err_t ret = esp_pm_configure(&pm_config);
  if (ret == ESP_OK) {
    Serial.println("Power management configured successfully");
  } else {
    Serial.printf("Power management configuration failed: %d\n", ret);
  }
  
  Serial.println("Power management configured for BLE compatibility");
}

void setupIMUWakeup() {
  Serial.println("Setting up IMU wake-on-motion...");
  
  // Configure RTC GPIO for button wake-up
  // M5StickCPlus2 buttons are active LOW (normally HIGH, go LOW when pressed)
  
  rtc_gpio_init(GPIO_NUM_37); // Button A
  rtc_gpio_set_direction(GPIO_NUM_37, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_37);
  rtc_gpio_pulldown_dis(GPIO_NUM_37);
  
  rtc_gpio_init(GPIO_NUM_39); // Button B
  rtc_gpio_set_direction(GPIO_NUM_39, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_39);
  rtc_gpio_pulldown_dis(GPIO_NUM_39);
  
  // Enable wake on Button A press (ext0 - single RTC GPIO pin)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0); // Button A: wake on LOW (button press)
  
  // Enable timer wake for periodic motion checks
  esp_sleep_enable_timer_wakeup(ULP_WAKE_INTERVAL * 1000); // Convert to microseconds
  
  Serial.println("IMU wake-on-motion configured with button and timer wake sources");
}

void updateLEDBreathing() {
  if (!ledBreathingEnabled) {
    ledcWrite(0, 0); // Turn off LED
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Different breathing patterns based on connection state
  float breathingPeriod;
  float maxBrightness = 25.5f; // 10% of 255 (10% brightness)
  
  if (deviceConnected) {
    breathingPeriod = 1500.0f; // 1.5 second breathing when connected
  } else {
    breathingPeriod = 3000.0f; // 3 second breathing when not connected
  }
  
  // Calculate breathing phase (0.0 to 1.0)
  float phase = fmod((currentTime - ledBreathingStartTime), breathingPeriod) / breathingPeriod;
  
  // Create smooth breathing pattern using sine wave
  float brightness = (sin(phase * 2.0f * PI - PI/2.0f) + 1.0f) / 2.0f; // 0.0 to 1.0
  brightness = brightness * maxBrightness; // Scale to max brightness
  
  ledcWrite(0, (int)brightness);
}

void wakePulse() {
  // Quick 50ms pulse at 10% brightness to indicate wake check
  ledcWrite(0, 25); // 10% brightness
  delay(50);
  ledcWrite(0, 0);  // Turn off
}

bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az) {
  // Calculate gyro magnitude (degrees/second)
  float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);
  
  // Calculate accelerometer magnitude (g-force)
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  float accelDelta = abs(accelMagnitude - lastAccelMagnitude);
  
  // Calculate motion deltas with noise filtering
  float gyroDelta = abs(gyroMagnitude - lastGyroMagnitude);
  
  // Apply deadzone/noise filtering
  bool gyroMotion = gyroDelta > MOTION_THRESHOLD;
  bool accelMotion = accelDelta > ACCEL_MOTION_THRESHOLD;
  
  // Update last values with filtering
  float alpha = 0.7f;
  lastGyroMagnitude = alpha * lastGyroMagnitude + (1.0f - alpha) * gyroMagnitude;
  lastAccelMagnitude = alpha * lastAccelMagnitude + (1.0f - alpha) * accelMagnitude;
  
  // Accumulate significant motion for deep sleep decisions
  if (gyroMotion || accelMotion) {
    motionAccumulator += gyroDelta + (accelDelta * 5.0f);
  }
  
  // Reset accumulator every 30 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastMotionAccumulatorReset > 30000) {
    motionAccumulator = 0;
    lastMotionAccumulatorReset = currentTime;
  }
  
  return gyroMotion || accelMotion;
}

void startBLE() {
  Serial.println("Starting BLE...");
  
  // Configure BLE for low power
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  bt_cfg.mode = ESP_BT_MODE_BLE; // BLE only mode
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  
  // Ensure CPU is at 80MHz for BLE
  setCpuFrequencyMhz(BLE_CPU_FREQ);
  
  Serial.println("BLE started, CPU at 80MHz");
}

void stopBLE() {
  Serial.println("Stopping BLE to save power...");
  
  // Stop BLE advertising and disable controller
  BLEDevice::deinit(false);
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  
  Serial.println("BLE stopped");
}

void enterBLEWaitingMode() {
  if (currentPowerMode != POWER_BLE_WAITING) {
    Serial.println("Entering BLE waiting mode");
    
    // Ensure BLE is running and CPU at 80MHz
    setCpuFrequencyMhz(BLE_CPU_FREQ);
    
    // Turn off screen to save power while waiting
    if (screenOn) {
      M5.Display.clear();
      M5.Display.sleep();
      screenOn = false;
    }
    
    currentPowerMode = POWER_BLE_WAITING;
    bleStartTime = millis();
    
    Serial.printf("BLE waiting mode - CPU: %d MHz, Screen: OFF\n", BLE_CPU_FREQ);
  }
}

void enterBLEActiveMode() {
  if (currentPowerMode != POWER_BLE_ACTIVE) {
    Serial.println("Entering BLE active mode");
    
    // Ensure CPU at 80MHz for BLE
    setCpuFrequencyMhz(BLE_CPU_FREQ);
    
    currentPowerMode = POWER_BLE_ACTIVE;
    
    Serial.printf("BLE active mode - CPU: %d MHz\n", BLE_CPU_FREQ);
  }
}

void enterLowPowerMode() {
  if (currentPowerMode != POWER_LOW_POWER) {
    Serial.println("Entering low power mode (no BLE)");
    
    // Stop BLE to save power
    stopBLE();
    
    // Reduce CPU to 10MHz for I2C/GPIO operations
    setCpuFrequencyMhz(LOW_POWER_CPU_FREQ);
    
    // Turn off screen
    if (screenOn) {
      M5.Display.clear();
      M5.Display.sleep();
      screenOn = false;
    }
    
    // Reduce LED brightness further
    ledcWrite(0, 2); // Very dim breathing
    
    currentPowerMode = POWER_LOW_POWER;
    
    Serial.printf("Low power mode - CPU: %d MHz, BLE: OFF\n", LOW_POWER_CPU_FREQ);
  }
}

void enterScreenOffMode() {
  if (screenOn) {
    Serial.println("Turning off screen");
    
    // Turn off display
    M5.Display.clear();
    M5.Display.sleep();
    screenOn = false;
    
    Serial.println("Screen off");
  }
}

void exitScreenOffMode() {
  if (!screenOn) {
    Serial.println("Turning on screen");
    
    // Wake up display
    M5.Display.wakeup();
    M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
    screenOn = true;
    
    Serial.println("Screen on at 5% brightness");
  }
}

void enterULPSleep() {
  // Guard against multiple calls
  if (enteringSleep) {
    Serial.println("enterULPSleep() already in progress, ignoring duplicate call");
    return;
  }
  
  enteringSleep = true;
  Serial.println("=== ENTERING ULP SLEEP MODE ===");
  
  // Show sleep message on screen briefly
  M5.Display.wakeup();
  M5.Display.setBrightness(50);
  M5.Display.clear();
  M5.Display.setRotation(0);
  M5.Display.setTextColor(RED);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 20);
  M5.Display.print("GOING TO");
  M5.Display.setCursor(10, 50);
  M5.Display.print("DEEP SLEEP");
  M5.Display.setCursor(10, 80);
  M5.Display.printf("Timer:%ds", ULP_WAKE_INTERVAL/1000);
  M5.Display.setCursor(10, 110);
  M5.Display.print("Btn A:Wake");
  delay(3000); // Show for 3 seconds
  
  // Turn off everything
  M5.Display.clear();
  M5.Display.sleep();
  ledcWrite(0, 0);
  screenOn = false;
  
  // Stop BLE to save maximum power
  stopBLE();
  
  // Save current state
  prefs.begin("sterzo", false);
  prefs.putBool("wasAsleep", true);
  prefs.putULong("sleepTime", millis());
  prefs.end();
  
  currentPowerMode = POWER_ULP_SLEEP;
  
  // Wake sources are configured in setupIMUWakeup() during setup
  
  // Debug: Show current wake-up configuration
  Serial.printf("Wake-up sources configured:\n");
  Serial.printf("- ext0: GPIO37 (Button A) on LOW\n");
  Serial.printf("- Timer: %d seconds\n", ULP_WAKE_INTERVAL/1000);
  Serial.printf("- Button B/C checked during timer wake\n");
  Serial.printf("- Motion detection via I2C during timer wake\n");
  
  Serial.printf("Going to ULP sleep (wake every %dms via ULP coprocessor)...\n", ULP_WAKE_INTERVAL);
  
  // EXPERIMENTAL: Lower serial baud rate before sleep to test corruption theory
  Serial.flush(); // Ensure all output is sent at current baud rate
  delay(100);
  Serial.end();
  Serial.begin(9600); // Much lower baud rate for low CPU frequency
  Serial.println("Serial switched to 9600 baud for sleep");
  Serial.flush();
  
  // Enter deep sleep with ULP monitoring
  esp_deep_sleep_start();
  
  // This line should never be reached, but reset flag just in case
  enteringSleep = false;
}

void handleWakeupFromSleep() {
  // Reset the sleep guard flag on wake-up
  enteringSleep = false;
  
  // EXPERIMENTAL: Restore normal serial baud rate after wake-up
  Serial.end();
  Serial.begin(115200);
  Serial.println("Serial restored to 115200 baud after wake-up");
  
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  // Always turn on screen and show wake reason prominently
  M5.Display.wakeup();
  M5.Display.setBrightness(50); // Higher brightness for wake display
  M5.Display.clear();
  M5.Display.setRotation(0);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 20);
  M5.Display.print("WAKE UP!");
  
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: {
      Serial.println("Woke up from Button A press (ext0)");
      M5.Display.setCursor(10, 50);
      M5.Display.print("BUTTON A");
      M5.Display.setCursor(10, 80);
      M5.Display.print("(EXT0)");
      ulp_wake_count++;
      lastButtonTime = millis();
      
      // Start in BLE waiting mode after button wake
      startBLE();
      enterBLEWaitingMode();
      break;
    }
    
    case ESP_SLEEP_WAKEUP_EXT1: {
      // Check which GPIO caused the ext1 wake-up
      Serial.println("Woke up from button press (ext1)");
      M5.Display.setCursor(10, 50);
      M5.Display.print("BUTTON");
      M5.Display.setCursor(10, 80);
      M5.Display.print("(EXT1)");
      ulp_wake_count++;
      lastButtonTime = millis();
      
      // Start in BLE waiting mode after button wake
      startBLE();
      enterBLEWaitingMode();
      break;
    }
      
    case ESP_SLEEP_WAKEUP_TIMER: {
      Serial.println("Woke up from ULP timer - checking for motion and buttons");
      M5.Display.setCursor(10, 50);
      M5.Display.print("TIMER");
      M5.Display.setCursor(10, 80);
      M5.Display.printf("Wake #%d", ulp_wake_count + 1);
      ulp_wake_count++;
      wakePulse(); // Quick LED pulse to indicate wake check
      
      // Set CPU to low power for motion check
      setCpuFrequencyMhz(DEEP_SLEEP_CPU_FREQ);
      
      // Check if any button is currently pressed (Button B or others not configured for ext0)
      bool buttonAState = !digitalRead(37); // Button A (active LOW) - for debugging
      bool buttonBPressed = !digitalRead(39); // Button B (active LOW)
      bool buttonCPressed = !digitalRead(35); // Button C (active LOW)
      
      Serial.printf("Button states during timer wake - A:%s B:%s C:%s\n", 
                    buttonAState ? "PRESSED" : "released", 
                    buttonBPressed ? "PRESSED" : "released", 
                    buttonCPressed ? "PRESSED" : "released");
      
      if (buttonBPressed || buttonCPressed) {
        Serial.printf("Button pressed during timer wake - B:%s C:%s\n", 
                      buttonBPressed ? "YES" : "NO", buttonCPressed ? "YES" : "NO");
        M5.Display.setCursor(10, 110);
        M5.Display.printf("BTN %s%s", buttonBPressed ? "B" : "", buttonCPressed ? "C" : "");
        lastButtonTime = millis();
        
        // Show button detection for 3 seconds
        delay(3000);
        
        // Start BLE and enter waiting mode
        startBLE();
        enterBLEWaitingMode();
      } else {
        // No button pressed, check for motion
        float gx, gy, gz, ax, ay, az;
        M5.Imu.getGyro(&gx, &gy, &gz);
        M5.Imu.getAccel(&ax, &ay, &az);
        
        float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
        float accelMag = sqrt(ax*ax + ay*ay + az*az);
        
        M5.Display.setCursor(10, 110);
        M5.Display.printf("G:%.1f A:%.1f", gyroMag, accelMag);
        
        if (detectMotion(gx, gy, gz, ax, ay, az)) {
          Serial.println("Motion detected during ULP wake - starting BLE");
          M5.Display.setCursor(10, 140);
          M5.Display.print("MOTION!");
          ulp_motion_detected = 1;
          lastMotionTime = millis();
          
          // Show motion detection for 3 seconds
          delay(3000);
          
          // Start BLE and enter waiting mode
          startBLE();
          enterBLEWaitingMode();
        } else {
          Serial.println("No motion or button press detected - returning to ULP sleep");
          M5.Display.setCursor(10, 140);
          M5.Display.print("Sleep...");
          ulp_motion_detected = 0;
          
          // Show sleep message for 1 second
          delay(1000);
          M5.Display.clear();
          M5.Display.sleep();
          
          // Return to ULP sleep immediately
          Serial.println("Returning to ULP sleep immediately");
          delay(100); // Brief delay to ensure serial output
          enterULPSleep(); // This will not return
        }
      }
      break;
    }
      
    case ESP_SLEEP_WAKEUP_ULP: {
      Serial.println("Woke up from ULP coprocessor");
      M5.Display.setCursor(10, 50);
      M5.Display.print("ULP");
      M5.Display.setCursor(10, 80);
      M5.Display.print("COPROC");
      ulp_wake_count++;
      // ULP detected something significant
      startBLE();
      enterBLEWaitingMode();
      break;
    }
      
    default: {
      Serial.printf("Woke up for unknown reason: %d\n", wakeup_reason);
      M5.Display.setCursor(10, 50);
      M5.Display.print("UNKNOWN");
      M5.Display.setCursor(10, 80);
      M5.Display.printf("Reason:%d", wakeup_reason);
      // Default to BLE waiting mode
      startBLE();
      enterBLEWaitingMode();
      break;
    }
  }
  
  // Show wake reason for 5 seconds (unless going back to sleep immediately)
  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER || 
      (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && currentPowerMode != POWER_ULP_SLEEP)) {
    delay(5000);
    
    // Set screen back to normal brightness and mark as on
    M5.Display.setBrightness(13); // 5% brightness
    screenOn = true;
    lastButtonTime = millis(); // Reset screen timer
  }
}

void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az) {
  unsigned long currentTime = millis();
  
  // Check for motion
  bool motionDetected = detectMotion(gx, gy, gz, ax, ay, az);
  
  if (motionDetected) {
    lastMotionTime = currentTime;
  }
  
  // Update BLE activity time if connected
  if (deviceConnected) {
    lastBLEActivityTime = currentTime;
    // Switch to BLE active mode when connected
    if (currentPowerMode != POWER_BLE_ACTIVE) {
      enterBLEActiveMode();
    }
  }
  
  // Screen management - turn on screen for 120s after button press
  if (currentTime - lastButtonTime <= SCREEN_ON_TIMEOUT) {
    if (!screenOn) {
      exitScreenOffMode();
    }
  } else {
    if (screenOn) {
      enterScreenOffMode();
    }
  }
  
  // Power mode transitions based on activity and connection state
  if (!isCalibrating) {
    if (deviceConnected) {
      // Stay in BLE active mode when connected
      if (currentPowerMode != POWER_BLE_ACTIVE) {
        enterBLEActiveMode();
      }
    } else {
      // Not connected - manage power modes based on time and activity
      unsigned long timeSinceActivity = min(currentTime - lastMotionTime, currentTime - lastButtonTime);
      unsigned long timeSinceBLEStart = currentTime - bleStartTime;
      
      switch (currentPowerMode) {
        case POWER_BLE_WAITING: {
          // After 5 minutes of no connection, enter low power mode
          if (timeSinceBLEStart > BLE_WAIT_TIMEOUT) {
            enterLowPowerMode();
          }
          break;
        }
          
        case POWER_BLE_ACTIVE: {
          // If disconnected, go back to waiting mode
          enterBLEWaitingMode();
          break;
        }
          
        case POWER_LOW_POWER: {
          // After 2 minutes in low power with no activity, enter ULP sleep
          if (timeSinceActivity > LOW_POWER_TIMEOUT) {
            enterULPSleep();
          }
          break;
        }
          
        case POWER_ULP_SLEEP: {
          // Already in ULP sleep - wake-up handling will manage transitions
          break;
        }
      }
    }
  }
}

// —————— DISPLAY MANAGEMENT SYSTEM ——————

void initializeDisplay() {
  M5.Display.setRotation(0);  // Start in portrait mode
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2);
  M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
  displayLandscape = false;
  currentDisplayMode = DISPLAY_MAIN;
  Serial.println("Display management system initialized");
}

void clearDisplaySafe() {
  M5.Display.clear();
  M5.Display.setTextColor(WHITE); // Reset to default color
}

void setDisplayOrientation(bool landscape) {
  if (landscape != displayLandscape) {
    displayLandscape = landscape;
    M5.Display.setRotation(landscape ? 1 : 0);
    Serial.printf("Display orientation: %s\n", landscape ? "Landscape" : "Portrait");
  }
}

bool shouldUseLandscape(const char* text) {
  // Use landscape for longer text strings
  int textLength = strlen(text);
  return textLength > 12; // Threshold for switching to landscape
}

void drawCenteredText(const char* text, int y, int textSize, uint16_t color) {
  M5.Display.setTextSize(textSize);
  M5.Display.setTextColor(color);
  
  // Calculate text width for centering
  int textWidth = strlen(text) * 6 * textSize; // Approximate character width
  int displayWidth = displayLandscape ? DISPLAY_WIDTH_LANDSCAPE : DISPLAY_WIDTH_PORTRAIT;
  int x = (displayWidth - textWidth) / 2;
  if (x < 0) x = 2; // Minimum margin
  
  M5.Display.setCursor(x, y);
  M5.Display.print(text);
}

void drawStatusBar() {
  int displayWidth = displayLandscape ? DISPLAY_WIDTH_LANDSCAPE : DISPLAY_WIDTH_PORTRAIT;
  
  // Draw status bar background
  M5.Display.fillRect(0, 0, displayWidth, STATUS_BAR_HEIGHT, 0x2104); // Dark gray
  
  // Battery voltage (right side)
  float batteryVoltage = M5.Power.getBatteryVoltage() / 1000.0f;
  M5.Display.setTextSize(1.5);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(displayWidth - 45, 5);
  M5.Display.printf("%.2fV", batteryVoltage);
  
  // Connection status (left side)
  M5.Display.setCursor(2, 5);
  if (deviceConnected) {
    M5.Display.setTextColor(0x07E0); // Green
    M5.Display.print("BLE");
  } else {
    M5.Display.setTextColor(0xFBE0); // Yellow
    M5.Display.print("ADV");
  }
  
  // CPU frequency (center)
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(displayWidth/2 - 30, 5);
  M5.Display.printf("%dMHz", getCpuFrequencyMhz());
}

void showButtonHelp() {
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  int displayWidth = displayLandscape ? DISPLAY_WIDTH_LANDSCAPE : DISPLAY_WIDTH_PORTRAIT;
  int helpY = displayHeight - BUTTON_HELP_HEIGHT;
  
  // Draw help background
  M5.Display.fillRect(0, helpY, displayWidth, BUTTON_HELP_HEIGHT, 0x1082); // Very dark gray
  
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(0xC618); // Light gray
  
  if (displayLandscape) {
    // Landscape layout - more space for button help
    M5.Display.setCursor(2, helpY + 5);
    M5.Display.print("A:Recenter");
    M5.Display.setCursor(80, helpY + 5);
    M5.Display.print("B:Recenter");
    M5.Display.setCursor(160, helpY + 5);
    M5.Display.print("C:Screen");
    M5.Display.setCursor(2, helpY + 15);
    M5.Display.print("Hold A:Cal");
    M5.Display.setCursor(80, helpY + 15);
    M5.Display.print("Hold B:Cal");
    M5.Display.setCursor(160, helpY + 15);
    M5.Display.print("Hold C:Off");
  } else {
    // Portrait layout - compact button help
    M5.Display.setCursor(2, helpY + 5);
    M5.Display.print("A/B:Recenter C:Screen");
    M5.Display.setCursor(2, helpY + 15);
    M5.Display.print("Hold A/B:Cal Hold C:Off");
  }
}

void updateMainDisplay() {
  if (!screenOn || currentDisplayMode != DISPLAY_MAIN) return;
  
  // Check if we need to return from status message
  if (statusMessageEndTime > 0 && millis() > statusMessageEndTime) {
    statusMessageEndTime = 0;
    currentDisplayMode = DISPLAY_MAIN;
  }
  
  // Update only every second for power efficiency
  if (millis() - lastDisplayUpdate < 1000) return;
  lastDisplayUpdate = millis();
  
  clearDisplaySafe();
  
  // Determine if we should use landscape based on content
  bool needLandscape = false;
  
  // Check power status text length
  char powerStatusText[32];
  if (deviceConnected) {
    strcpy(powerStatusText, "CONNECTED");
  } else {
    switch (currentPowerMode) {
      case POWER_BLE_WAITING: {
        unsigned long timeUntilLowPower = (BLE_WAIT_TIMEOUT - (millis() - bleStartTime)) / 1000;
        if (timeUntilLowPower < 60) {
          snprintf(powerStatusText, sizeof(powerStatusText), "LowPwr:%lus", timeUntilLowPower);
        } else {
          strcpy(powerStatusText, "BLE WAIT");
        }
        break;
      }
      case POWER_LOW_POWER: {
        unsigned long timeUntilSleep = (LOW_POWER_TIMEOUT - (millis() - max(lastMotionTime, lastButtonTime))) / 1000;
        if (timeUntilSleep < 60) {
          snprintf(powerStatusText, sizeof(powerStatusText), "Sleep:%lus", timeUntilSleep);
        } else {
          strcpy(powerStatusText, "LOW POWER");
        }
        break;
      }
      case POWER_BLE_ACTIVE:
        strcpy(powerStatusText, "BLE ACTIVE");
        break;
      default:
        strcpy(powerStatusText, "ACTIVE");
        break;
    }
  }
  
  needLandscape = shouldUseLandscape(powerStatusText);
  setDisplayOrientation(needLandscape);
  
  // Draw status bar
  drawStatusBar();
  
  int contentStartY = STATUS_BAR_HEIGHT + 10;
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  
  // Main content area
  if (displayLandscape) {
    // Landscape layout - more horizontal space
    drawCenteredText("STERZO", contentStartY, 2, ORANGE);
    
    // Yaw display with limit indicators
    float rawYaw = getYaw();
    float rel = rawYaw + yawOffset;
    while (rel > 180) rel -= 360;
    while (rel < -180) rel += 360;
    
    float unclamped_rel = rel;
    rel = constrain(rel, -40, 40);
    float bin = round(rel/1.0f)*1.0f;
    if (abs(bin) < 0.1f) bin = 0.0f;
    
    char yawText[32];
    bool flashOn = (millis() / 1000) % 2 == 0;
    if (unclamped_rel >= 40.0f) {
      snprintf(yawText, sizeof(yawText), "Yaw:%.1f%s", rel, flashOn ? ">" : " ");
    } else if (unclamped_rel <= -40.0f) {
      snprintf(yawText, sizeof(yawText), "Yaw:%.1f%s", rel, flashOn ? "<" : " ");
    } else {
      snprintf(yawText, sizeof(yawText), "Yaw:%.1f°", rel);
    }
    
    drawCenteredText(yawText, contentStartY + 25, 2, WHITE);
    
    char binText[16];
    snprintf(binText, sizeof(binText), "Bin:%.0f°", bin);
    drawCenteredText(binText, contentStartY + 50, 2, CYAN);
    
    drawCenteredText(powerStatusText, contentStartY + 75, 1, deviceConnected ? GREEN : YELLOW);
    
  } else {
    // Portrait layout - vertical stacking
    drawCenteredText("STERZO", contentStartY, 2, ORANGE);
    
    // Yaw display
    float rawYaw = getYaw();
    float rel = rawYaw + yawOffset;
    while (rel > 180) rel -= 360;
    while (rel < -180) rel += 360;
    
    float unclamped_rel = rel;
    rel = constrain(rel, -40, 40);
    float bin = round(rel/1.0f)*1.0f;
    if (abs(bin) < 0.1f) bin = 0.0f;
    
    char yawText[20];
    bool flashOn = (millis() / 1000) % 2 == 0;
    if (unclamped_rel >= 40.0f) {
      snprintf(yawText, sizeof(yawText), "%.1f%s", rel, flashOn ? ">" : "°");
    } else if (unclamped_rel <= -40.0f) {
      snprintf(yawText, sizeof(yawText), "%.1f%s", rel, flashOn ? "<" : "°");
    } else {
      snprintf(yawText, sizeof(yawText), "%.1f°", rel);
    }
    
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(10, contentStartY + 30);
    M5.Display.print("Yaw:");
    M5.Display.print(yawText);
    
    M5.Display.setCursor(10, contentStartY + 55);
    M5.Display.printf("Bin:%.0f°", bin);
    
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, contentStartY + 80);
    M5.Display.setTextColor(deviceConnected ? GREEN : YELLOW);
    M5.Display.print(powerStatusText);
    
    // Frequency display
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(10, contentStartY + 100);
    M5.Display.printf("Loop: %dHz", (int)loopfreq);
  }
  
  // Show button help
  showButtonHelp();
}

void showStatusMessage(const char* title, const char* message, uint16_t color, int duration) {
  currentDisplayMode = DISPLAY_STATUS;
  statusMessageEndTime = millis() + duration;
  
  // Determine orientation based on message length
  bool needLandscape = shouldUseLandscape(message) || shouldUseLandscape(title);
  setDisplayOrientation(needLandscape);
  
  clearDisplaySafe();
  
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  int centerY = displayHeight / 2;
  
  drawCenteredText(title, centerY - 20, 2, color);
  drawCenteredText(message, centerY + 10, 1, WHITE);
  
  Serial.printf("Status message: %s - %s\n", title, message);
}

void showProgressBar(const char* title, int progress, int total) {
  currentDisplayMode = DISPLAY_STATUS;
  
  bool needLandscape = shouldUseLandscape(title);
  setDisplayOrientation(needLandscape);
  
  clearDisplaySafe();
  
  int displayWidth = displayLandscape ? DISPLAY_WIDTH_LANDSCAPE : DISPLAY_WIDTH_PORTRAIT;
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  int centerY = displayHeight / 2;
  
  drawCenteredText(title, centerY - 30, 2, WHITE);
  
  // Progress bar
  int barWidth = displayWidth - 20;
  int barHeight = 10;
  int barX = 10;
  int barY = centerY;
  
  // Background
  M5.Display.drawRect(barX, barY, barWidth, barHeight, WHITE);
  
  // Progress fill
  int fillWidth = (progress * barWidth) / total;
  M5.Display.fillRect(barX + 1, barY + 1, fillWidth - 1, barHeight - 2, GREEN);
  
  // Progress text
  char progressText[16];
  snprintf(progressText, sizeof(progressText), "%d%%", (progress * 100) / total);
  drawCenteredText(progressText, centerY + 20, 1, WHITE);
}

void showCalibrationScreen(const char* stage, int progress) {
  currentDisplayMode = DISPLAY_CALIBRATION;
  
  bool needLandscape = shouldUseLandscape(stage);
  setDisplayOrientation(needLandscape);
  
  clearDisplaySafe();
  
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  int centerY = displayHeight / 2;
  
  drawCenteredText("CALIBRATION", centerY - 40, 2, ORANGE);
  drawCenteredText(stage, centerY - 10, 1, WHITE);
  
  if (progress >= 0) {
    showProgressBar("", progress, 100);
  }
}

void showWakeupScreen(const char* reason, const char* details) {
  currentDisplayMode = DISPLAY_WAKEUP;
  
  bool needLandscape = details ? shouldUseLandscape(details) : shouldUseLandscape(reason);
  setDisplayOrientation(needLandscape);
  
  clearDisplaySafe();
  
  int displayHeight = displayLandscape ? DISPLAY_HEIGHT_LANDSCAPE : DISPLAY_HEIGHT_PORTRAIT;
  int centerY = displayHeight / 2;
  
  drawCenteredText("WAKE UP!", centerY - 30, 2, YELLOW);
  drawCenteredText(reason, centerY, 2, WHITE);
  
  if (details) {
    drawCenteredText(details, centerY + 25, 1, CYAN);
  }
}

void showConnectionScreen(bool connected) {
  currentDisplayMode = DISPLAY_CONNECTION;
  statusMessageEndTime = millis() + 3000; // Show for 3 seconds
  
  setDisplayOrientation(false); // Use portrait for connection messages
  
  clearDisplaySafe();
  
  int centerY = DISPLAY_HEIGHT_PORTRAIT / 2;
  
  if (connected) {
    drawCenteredText("BLE", centerY - 20, 2, GREEN);
    drawCenteredText("CONNECTED", centerY + 10, 2, GREEN);
    drawCenteredText("Screen on for 1min", centerY + 40, 1, WHITE);
  } else {
    drawCenteredText("BLE", centerY - 20, 2, RED);
    drawCenteredText("DISCONNECTED", centerY + 10, 2, RED);
    drawCenteredText("Advertising...", centerY + 40, 1, WHITE);
  }
}

// —————— BLE Callbacks ——————
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect (BLEServer* s) override {
    deviceConnected = true;
    lastBLEActivityTime = millis();
    Serial.println("BLE client connected");
    
    // Turn on screen for 1 minute to show connection success
    if (!screenOn) {
      exitScreenOffMode();
    }
    lastButtonTime = millis(); // Reset screen timer for 1 minute display
    
    // Show connection success message using new display system
    showConnectionScreen(true);
    
    // Success beep pattern
    M5.Speaker.tone(800, 100);
    delay(150);
    M5.Speaker.tone(1000, 100);
    delay(150);
    M5.Speaker.tone(1200, 150);
  }
  void onDisconnect (BLEServer* s) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    
    // Show disconnection message if screen is on
    if (screenOn) {
      showConnectionScreen(false);
      
      // Disconnection beep
      M5.Speaker.tone(600, 200);
      delay(300);
      M5.Speaker.tone(400, 200);
    }
  }
};

class char31Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    lastBLEActivityTime = millis(); // Track BLE activity for power management
    
    auto val = c->getValue();
    Serial.print("char31 onWrite: ");
    for (auto &b: val) { Serial.printf("%02X ", b); }
    Serial.println();

    // Zwift handshake
    if (val.size()>=2 && val[0]==0x03 && val[1]==0x10) {
      Serial.println("→ got 0x310, sending initial challenge");
      uint8_t chal[4] = {0x03,0x10,0x12,0x34};
      pChar32->setValue(chal,4);
      pChar32->indicate();
    }
    else if (val.size()>=2 && val[0]==0x03 && val[1]==0x11) {
      Serial.println("→ got 0x311, marking challengeOK");
      challengeOK = true;
      uint8_t resp[4] = {0x03,0x11,0xFF,0xFF};
      pChar32->setValue(resp,4);
      pChar32->indicate();
    }
    else if (val.size()>=6 && val[0]==0x03 && val[1]==0x12) {
      // client has sent seed bytes in val[2..5]
      uint32_t seed = ( (uint32_t)val[5]<<24 ) |
                      ( (uint32_t)val[4]<<16 ) |
                      ( (uint32_t)val[3]<<8  ) |
                      ( (uint32_t)val[2]     );
      uint32_t pwd  = hashed(seed);
      Serial.printf("→ got 0x312, seed=0x%08X, pwd=0x%08X\n", seed, pwd);
      uint8_t res[6];
      res[0]=0x03; res[1]=0x12;
      res[2]= pwd        & 0xFF;
      res[3]=(pwd >>  8) & 0xFF;
      res[4]=(pwd >> 16) & 0xFF;
      res[5]=(pwd >> 24) & 0xFF;
      pChar32->setValue(res,6);
      pChar32->indicate();
    }
    else if (val.size()>=2 && val[0]==0x03 && val[1]==0x13) {
      Serial.println("→ got 0x313, final ACK");
      uint8_t r[3] = {0x03,0x13,0xFF};
      pChar32->setValue(r,3);
      pChar32->indicate();
      challengeOK = true;
    }
  }
};

class char32Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override { Serial.println("char32 onWrite"); }
  void onRead (BLECharacteristic* c) override { Serial.println("char32 onRead");  }
};

class char32Desc2902Callbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* d) override {
    ind32On = true;
    Serial.println("Client ENABLED indications (char32)");
  }
};

// —————— Helpers ——————
void sendSteeringBin(float bin) {
  pChar30->setValue((uint8_t*)&bin, sizeof(bin));
  pChar30->notify();
  Serial.printf("NOTIFY bin: %.0f°\n", bin);
}

// —————— Calibration Functions ——————

// Apply low-pass filter to gyro readings
void filterGyroReadings(float gx, float gy, float gz) {
  filteredGx = gyroFilterAlpha * gx + (1.0f - gyroFilterAlpha) * filteredGx;
  filteredGy = gyroFilterAlpha * gy + (1.0f - gyroFilterAlpha) * filteredGy;
  filteredGz = gyroFilterAlpha * gz + (1.0f - gyroFilterAlpha) * filteredGz;
}

// Check if calibration data exists
bool calibrationExists() {
  // Check if key gyro calibration values exist (using isKey to avoid NVS errors)
  return prefs.isKey("gxb") && prefs.isKey("gyb") && prefs.isKey("gzb");
  }
  
// Quick recenter - just reset offset to make current position = 0
void quickRecenterYaw() {
  Serial.println("Quick recentering yaw...");
  
  // Distinctive beep pattern for quick recenter
  M5.Speaker.tone(800, 50);
  delay(100);
  M5.Speaker.tone(1000, 50);
  
  // Get current raw yaw
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
  // Apply filtering
  filterGyroReadings(gx, gy, gz);
    
  // Update Mahony filter with calibrated data
  MahonyAHRSupdateIMU(
    (filteredGx - gyroBiasX) * DEG_TO_RAD,
    (filteredGy - gyroBiasY) * DEG_TO_RAD,
    (filteredGz - gyroBiasZ) * DEG_TO_RAD,
    ax - accelBiasX,
    ay - accelBiasY,
    az - accelBiasZ,
    currentIMUFrequency
  );
  
  // Get current raw yaw
  float currentRawYaw = getYaw();
  
  if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
    // Set offset to make current position = 0
    yawOffset = -currentRawYaw;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    prefs.putFloat("yoff", yawOffset);
    Serial.printf("Quick recenter - Current raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);
  
    // Store the current raw yaw for drift compensation
    lastMeasuredRawYaw = currentRawYaw;
    lastYawMeasurementTime = millis();
    hasValidYawMeasurement = true;
  
    // Display confirmation if screen is on
    if (screenOn) {
  M5.Display.clear();
      M5.Display.setRotation(0);
  M5.Display.setCursor(0, 25);
      M5.Display.print("Quick Recenter");
  M5.Display.setCursor(0, 40);
      M5.Display.printf("Offset: %.1f°", yawOffset);
    delay(1000);
  }
  
  } else {
    Serial.println("WARNING: Invalid yaw value during quick recenter");
  }
}

// Recenter yaw to current position
void recenterYaw() {
  Serial.println("Recentering yaw...");
  
  // Distinctive beep pattern to indicate recentering
  M5.Speaker.tone(600, 100);
  delay(150);
  M5.Speaker.tone(800, 100);
  delay(150);
  
  // Wait for filter to stabilize
  delay(1000);
  
  // Collect samples for stable average
  const int samples = 50; // 5 seconds at 10Hz
  float yawSum = 0;
  int validSamples = 0;
  
  Serial.println("Collecting yaw samples...");
  
  unsigned long lastSampleTime = micros();
  const unsigned long sampleIntervalMicros = 100000; // 100ms = 10Hz
  
  for (int i = 0; i < samples; i++) {
    // Wait for precise timing
    while (micros() - lastSampleTime < sampleIntervalMicros) {
      M5.update(); // Keep buttons responsive during wait
    }
    lastSampleTime = micros();
    
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
    // Apply filtering
    filterGyroReadings(gx, gy, gz);
    
    // Update Mahony filter with calibrated data
    MahonyAHRSupdateIMU(
      (filteredGx - gyroBiasX) * DEG_TO_RAD,
      (filteredGy - gyroBiasY) * DEG_TO_RAD,
      (filteredGz - gyroBiasZ) * DEG_TO_RAD,
      ax - accelBiasX,
      ay - accelBiasY,
      az - accelBiasZ,
      currentIMUFrequency
    );
    
    // Get current yaw
    float currentYaw = getYaw();
    
    if (!isnan(currentYaw) && !isinf(currentYaw)) {
      yawSum += currentYaw;
      validSamples++;
    }
  }
  
  if (validSamples > 0) {
    float averageYaw = yawSum / validSamples;
    
    // Set the offset as the negative of the current raw yaw
    yawOffset = -averageYaw;
    
    // Normalize offset to [-180, 180] range
    if (yawOffset > 180) yawOffset -= 360;
    else if (yawOffset < -180) yawOffset += 360;
    
    prefs.putFloat("yoff", yawOffset);
    Serial.printf("Yaw recentered - Current: %.1f° → Offset: %.1f°\n", averageYaw, yawOffset);
    
    // Store the last measured raw yaw and timestamp
    lastMeasuredRawYaw = averageYaw;
    lastYawMeasurementTime = millis();
    hasValidYawMeasurement = true;
  } else {
    yawOffset = 0.0f; // Fallback to 0 if no valid samples
    prefs.putFloat("yoff", yawOffset);
    Serial.println("WARNING: No valid yaw samples, using defaults (offset=0)");
  }
  
  // Force a display update if screen is on
  if (screenOn) {
  M5.Display.clear();
    M5.Display.setRotation(0);
  M5.Display.setCursor(0, 25);
  M5.Display.print("Yaw Recentered");
  M5.Display.setCursor(0, 40);
  M5.Display.printf("Offset: %.1f°", yawOffset);
  delay(1000);
}
}

// Auto-calibration on first boot
void performAutoCalibration() {
  Serial.println("=== AUTO CALIBRATION ===");
  
  if (screenOn) {
  M5.Display.clear();
    M5.Display.setRotation(0);
  M5.Display.setCursor(0, 10);
  M5.Display.print("AUTO CALIBRATION");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Keep device still");
  M5.Display.setCursor(0, 40);
  M5.Display.print("3 seconds...");
  }
  
  // Simple auto-calibration: just gyro bias
  performGyroCalibration();
}

// Auto-recenter on boot
void performAutoRecenter() {
  recenterYaw();
}

// Initialize Mahony filter with current device orientation
void initializeMahonyFilter() {
  Serial.println("Initializing Mahony filter with current orientation...");
  
  // Reset quaternion to identity
  initMahonyData();
  
  // Collect IMU data for a few seconds to stabilize the filter
  const int initSamples = 100; // 10 seconds at 10Hz
  
  unsigned long lastSampleTime = micros();
  const unsigned long sampleIntervalMicros = 100000; // 100ms = 10Hz
  
  for (int i = 0; i < initSamples; i++) {
    // Wait for precise timing
    while (micros() - lastSampleTime < sampleIntervalMicros) {
      M5.update(); // Keep buttons responsive during wait
    }
    lastSampleTime = micros();
    
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
    // Apply filtering
    filterGyroReadings(gx, gy, gz);
    
    // Update Mahony filter with calibrated data
    MahonyAHRSupdateIMU(
      (filteredGx - gyroBiasX) * DEG_TO_RAD,
      (filteredGy - gyroBiasY) * DEG_TO_RAD,
      (filteredGz - gyroBiasZ) * DEG_TO_RAD,
      ax - accelBiasX,
      ay - accelBiasY,
      az - accelBiasZ,
      currentIMUFrequency
    );
  }
  
  // Get initial yaw reading
  float initialYaw = getYaw();
  Serial.printf("Mahony filter initialized - Initial yaw: %.1f°\n", initialYaw);
}

// Gyro-only calibration + recenter yaw
void performGyroCalibration() {
  isCalibrating = true;
  currentIMUFrequency = CALIBRATION_IMU_FREQUENCY; // Use 50Hz during calibration
  
  Serial.println("=== STARTING GYRO CALIBRATION ===");
  
  if (screenOn) {
  M5.Display.clear(); 
    M5.Display.setRotation(0);
  M5.Display.setCursor(0, 10);
    M5.Display.print("GYRO CALIBRATION");
  M5.Display.setCursor(0, 25);
    M5.Display.print("Keep device still");
  M5.Display.setCursor(0, 40);
    M5.Display.print("Starting in 3...");
  }
  
  // 3 second countdown
  for (int countdown = 3; countdown > 0; countdown--) {
    if (screenOn) {
      M5.Display.setCursor(0, 55);
      M5.Display.printf("Starting in %d...", countdown);
    }
    delay(1000);
  }
  
  // Reset gyro biases only
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  
  // Collect 100 samples over 2 seconds at 50Hz
  const int samples = 100;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  
  if (screenOn) {
    M5.Display.setCursor(0, 70);
    M5.Display.print("Measuring gyro...");
  }
  
  unsigned long lastSampleTime = micros();
  const unsigned long sampleIntervalMicros = 20000; // 20ms = 50Hz
  
  for (int i = 0; i < samples; i++) {
    // Wait for precise timing
    while (micros() - lastSampleTime < sampleIntervalMicros) {
      M5.update(); // Keep buttons responsive during wait
    }
    lastSampleTime = micros();
    
  float gx, gy, gz, ax, ay, az;
  M5.Imu.getGyro(&gx, &gy, &gz);
  M5.Imu.getAccel(&ax, &ay, &az);
  
    // Apply filtering during calibration
  filterGyroReadings(gx, gy, gz);
  
    sumGx += filteredGx; sumGy += filteredGy; sumGz += filteredGz;
    
    // Update display progress
    if (screenOn && i % 10 == 0) {
      M5.Display.setCursor(0, 85);
      M5.Display.printf("Progress: %d%%", (i * 100) / samples);
    }
  }
  
  // Calculate gyro biases
  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;
  gyroBiasZ = sumGz / samples;
  
  Serial.printf("Gyro calibration complete - Gyro bias: %.3f,%.3f,%.3f\n", 
                gyroBiasX, gyroBiasY, gyroBiasZ);
    
  // Save gyro biases
  prefs.putFloat("gxb", gyroBiasX);
  prefs.putFloat("gyb", gyroBiasY);
  prefs.putFloat("gzb", gyroBiasZ);
    
  // Recenter yaw
  recenterYaw();
  
  if (screenOn) {
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(0, 25);
    M5.Display.print("GYRO CAL DONE");
    M5.Display.setCursor(0, 40);
    M5.Display.print("Yaw recentered");
    delay(1000);
  }
  
  currentIMUFrequency = NORMAL_IMU_FREQUENCY; // Return to 10Hz
  isCalibrating = false;
  Serial.println("=== GYRO CALIBRATION COMPLETE ===");
  }
  
// Full calibration placeholder (simplified for power efficiency)
void setup() {
  Serial.begin(115200);
  Serial.println("=== STERZO STARTUP ===");
  
  // Power management: Set HOLD pin high to maintain power
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  Serial.println("HOLD pin set HIGH to maintain power");
  
  auto cfg = M5.config();
  Serial.println("M5 config created");
  
  M5.begin(cfg);
  Serial.println("M5.begin() completed");
  
  // Configure power management early
  configurePowerManagement();
  
  // Set initial CPU frequency for BLE
  setCpuFrequencyMhz(BLE_CPU_FREQ);
  Serial.printf("CPU frequency set to %d MHz for BLE\n", BLE_CPU_FREQ);
  
  // Initialize display with 5% brightness
  M5.Display.setRotation(0);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2);
  M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
  screenOn = true;
  lastButtonTime = millis(); // Screen starts on
  
  Serial.println("Display configured at 5% brightness");
  
  // Setup LED with PWM for breathing pattern
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);           // PWM channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);       // Attach LED pin to PWM channel 0
  ledBreathingStartTime = millis();
  
  Serial.println("LED configured for breathing pattern");

  // Initialize button GPIO pins
  pinMode(37, INPUT_PULLUP); // Button A
  pinMode(39, INPUT_PULLUP); // Button B  
  pinMode(35, INPUT_PULLUP); // Button C (Power)
  Serial.println("Button GPIO pins configured (37, 39, 35)");
  
  // Check for factory reset request (hold Button A + Button B during startup)
  bool factoryResetRequested = (!digitalRead(37) && !digitalRead(39));
  if (factoryResetRequested) {
    Serial.println("=== FACTORY RESET REQUESTED ===");
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(10, 20);
    M5.Display.print("FACTORY RESET");
    M5.Display.setCursor(10, 40);
    M5.Display.print("Hold buttons");
    M5.Display.setCursor(10, 60);
    M5.Display.print("for 3 seconds");
    
    // Wait 3 seconds while buttons are held
    bool resetConfirmed = true;
    for (int i = 0; i < 30; i++) {
      if (digitalRead(37) || digitalRead(39)) {
        resetConfirmed = false;
        break;
      }
      delay(100);
      
      // Update countdown display
      M5.Display.setCursor(10, 80);
      M5.Display.printf("Countdown: %d", 3 - (i/10));
    }
    
    if (resetConfirmed) {
      Serial.println("Factory reset confirmed - clearing all preferences");
      M5.Display.clear();
      M5.Display.setCursor(10, 20);
      M5.Display.print("CLEARING");
      M5.Display.setCursor(10, 40);
      M5.Display.print("PREFERENCES");
      
      // Clear all stored preferences
      prefs.begin("sterzo", false);
      prefs.clear();
      prefs.end();
      
      M5.Display.setCursor(10, 60);
      M5.Display.print("DONE!");
      M5.Display.setCursor(10, 80);
      M5.Display.print("Restarting...");
      
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Factory reset cancelled");
      M5.Display.clear();
      M5.Display.setCursor(10, 40);
      M5.Display.print("Reset cancelled");
      delay(1000);
    }
  }

  // Initialize Mahony AHRS
  initMahonyData();
  Serial.println("Mahony AHRS initialized");

  // Initialize power management variables
  lastMotionTime = millis();
  lastBLEActivityTime = millis();
  lastMotionAccumulatorReset = millis();
  motionAccumulator = 0.0f;
  currentPowerMode = POWER_BLE_WAITING;
  bleStartTime = millis();
  
  // Check if we woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("=== FRESH BOOT (not from deep sleep) ===");
  } else {
    Serial.printf("=== WOKE FROM DEEP SLEEP (reason: %d) ===\n", wakeup_reason);
    handleWakeupFromSleep();
  }

  // load stored biases + yawOffset
  // Initialize preferences and load calibration data
  Serial.println("Loading calibration data...");
  prefs.begin("sterzo", false);
  
  // Check if calibration exists
  if (!calibrationExists()) {
    // First boot - no calibration exists
    Serial.println("No calibration found - performing auto-calibration");
    performAutoCalibration();
  } else {
    // Calibration exists - load it
    Serial.println("Loading existing calibration data");
    
    // Load gyro bias (these should exist since calibrationExists() passed)
    gyroBiasX  = prefs.getFloat("gxb", 0);
    gyroBiasY  = prefs.getFloat("gyb", 0);
    gyroBiasZ  = prefs.getFloat("gzb", 0);
    
    // Load accel bias only if keys exist (avoid NVS errors)
    accelBiasX = prefs.isKey("axb") ? prefs.getFloat("axb") : 0.0f;
    accelBiasY = prefs.isKey("ayb") ? prefs.getFloat("ayb") : 0.0f;
    accelBiasZ = prefs.isKey("azb") ? prefs.getFloat("azb") : 0.0f;
    
    // Load yaw offset and drift rate only if keys exist
    yawOffset  = prefs.isKey("yoff") ? prefs.getFloat("yoff") : 0.0f;
    yawDriftRate = prefs.isKey("drift") ? prefs.getFloat("drift") : 0.8f;
    
    // Force update drift rate to new default if it's the old weak value
    if (yawDriftRate < 0.2f) {
      yawDriftRate = 0.8f;
      prefs.putFloat("drift", yawDriftRate);
      Serial.println("Updated drift rate from old weak value to 0.8°/s");
    }
    
    Serial.printf("Calibration loaded - Gyro: [%.3f,%.3f,%.3f] Accel: [%.3f,%.3f,%.3f] YawOffset: %.3f° DriftRate: %.2f°/s\n", 
                  gyroBiasX, gyroBiasY, gyroBiasZ, accelBiasX, accelBiasY, accelBiasZ, yawOffset, yawDriftRate);
    
    // Initialize Mahony filter with current orientation
    initializeMahonyFilter();
    
    // Auto-recenter on boot
    Serial.println("Auto-recentering yaw...");
    performAutoRecenter();
  }

  // Initialize BLE system
  Serial.println("Initializing BLE...");
  
  // Start BLE for initial connection attempts
  startBLE();
  
  BLEDevice::init("STERZO");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pSvc = pServer->createService(STERZO_SERVICE_UUID);
  
  pChar14 = pSvc->createCharacteristic(CHAR14_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pChar30 = pSvc->createCharacteristic(CHAR30_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pChar31 = pSvc->createCharacteristic(CHAR31_UUID, BLECharacteristic::PROPERTY_WRITE);
  pChar32 = pSvc->createCharacteristic(CHAR32_UUID, BLECharacteristic::PROPERTY_INDICATE);
  
  p2902_14 = new BLE2902();          pChar14->addDescriptor(p2902_14);
  p2902_30 = new BLE2902(); p2902_30->setNotifications(true); pChar30->addDescriptor(p2902_30);
  p2902_32 = new BLE2902(); p2902_32->setCallbacks(new char32Desc2902Callbacks()); pChar32->addDescriptor(p2902_32);

  pChar31->setCallbacks(new char31Callbacks());
  pChar32->setCallbacks(new char32Callbacks());

  pSvc->start();
  auto adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(STERZO_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  
  // Configure BLE advertising for low power
  adv->setMinInterval(800);  // Longer intervals for power saving
  adv->setMaxInterval(1600);
  
  BLEDevice::startAdvertising();
  
  Serial.println("BLE advertising started - waiting for connection");

  // Setup IMU wake-on-motion
  setupIMUWakeup();

  // Display startup message
  M5.Display.clear();
  M5.Display.setRotation(0);  
  M5.Display.setCursor(10, 20);
  M5.Display.print("STERZO");
  M5.Display.setCursor(10, 50);
  M5.Display.print("Ready!");
  M5.Display.setCursor(10, 80);
  M5.Display.printf("10Hz IMU");
  M5.Display.setCursor(10, 110);
  M5.Display.printf("5%% Screen");
  
  Serial.println("=== STERZO READY ===");
}

void loop() {
  M5.update(); // Update button states

  // Update LED breathing pattern
  updateLEDBreathing();

  // Precise timing for IMU updates - ensure exact frequency
  static unsigned long lastIMUUpdateTime = 0;
  unsigned long currentTime = micros();
  
  // Calculate target interval in microseconds for current IMU frequency
  unsigned long targetIntervalMicros = (unsigned long)(1000000.0f / currentIMUFrequency);
  
  // Check if enough time has passed for the next IMU update
  if (currentTime - lastIMUUpdateTime < targetIntervalMicros) {
    // Not time for next IMU update yet - just update buttons and return
    M5.update();
    return;
  }
  
  // Calculate actual time delta for frequency calculation
  dt = (currentTime - lastIMUUpdateTime) / 1000000.0f;
  lastIMUUpdateTime = currentTime;
  loopfreq = 1.0f / dt; // Calculate actual loop frequency
  
  // Cap frequency display at reasonable limits
  if (loopfreq > currentIMUFrequency * 1.5f) loopfreq = currentIMUFrequency * 1.5f;

  // 1) Read raw IMU, apply bias, fuse
  float gx, gy, gz, ax, ay, az;
  M5.Imu.getGyro(&gx,&gy,&gz);
  M5.Imu.getAccel(&ax,&ay,&az);

  // Apply low-pass filter to gyro readings
  filterGyroReadings(gx, gy, gz);

  // Update power management with current IMU readings
  updatePowerManagement(gx, gy, gz, ax, ay, az);

  // Debug IMU readings every 5 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {
    const char* powerModeStr;
    switch (currentPowerMode) {
      case POWER_BLE_ACTIVE: powerModeStr = "BLE_ACTIVE"; break;
      case POWER_BLE_WAITING: powerModeStr = "BLE_WAITING"; break;
      case POWER_LOW_POWER: powerModeStr = "LOW_POWER"; break;
      case POWER_ULP_SLEEP: powerModeStr = "ULP_SLEEP"; break;
      default: powerModeStr = "UNKNOWN"; break;
    }
    
    Serial.printf("IMU - Gyro: %.2f,%.2f,%.2f Accel: %.2f,%.2f,%.2f Freq: %.1f/%.1fHz PowerMode: %s Screen: %s CPU: %dMHz\n", 
                  gx, gy, gz, ax, ay, az, loopfreq, currentIMUFrequency, powerModeStr,
                  screenOn ? "ON" : "OFF", getCpuFrequencyMhz());
    lastDebugTime = millis();
  }

  // Update Mahony AHRS with calibrated data (convert gyro to radians)
  MahonyAHRSupdateIMU(
    (filteredGx - gyroBiasX) * DEG_TO_RAD,
    (filteredGy - gyroBiasY) * DEG_TO_RAD,
    (filteredGz - gyroBiasZ) * DEG_TO_RAD,
    ax - accelBiasX,
    ay - accelBiasY,
    az - accelBiasZ,
    currentIMUFrequency  // Use current frequency for consistent filter behavior
  );

  // Get yaw in degrees from Mahony AHRS
  float rawYaw = getYaw();  // Already returns degrees in [-180, 180] range
  
  // Check for valid yaw value
  if (isnan(rawYaw) || isinf(rawYaw)) {
    Serial.println("WARNING: Invalid yaw value detected!");
    rawYaw = 0.0f; // Use 0 as fallback
  }
  
  // Calculate relative yaw: raw + offset
  float rel = rawYaw + yawOffset;
  
  // Normalize relative yaw to [-180, 180] range
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Simple centering force - gradually adjust offset to bring rel back to 0
  static unsigned long lastCenteringTime = millis();
  unsigned long centeringTime = millis();
  float deltaTime = (centeringTime - lastCenteringTime) / 1000.0f;
  
  if (deltaTime >= 0.2f) { // Update every 200ms for better responsiveness
    // Apply centering force: if rel is positive, decrease offset; if negative, increase offset
    // Use the configured yawDriftRate instead of hardcoded value
    float centeringAdjustment = -rel * yawDriftRate * deltaTime;
    
    // Limit the adjustment to prevent overcorrection
    // With 0.5°/s rate and 0.2s intervals, max normal adjustment = 0.1°
    centeringAdjustment = constrain(centeringAdjustment, -0.2f, 0.2f);
    
    yawOffset += centeringAdjustment;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    // Debug centering compensation
    if (abs(centeringAdjustment) > 0.01f) {
      //Serial.printf("Centering: rel=%.2f° rate=%.1f°/s adj=%.3f° newOffset=%.2f°\n", 
      //              rel, yawDriftRate, centeringAdjustment, yawOffset);
    }
    
    lastCenteringTime = centeringTime;
  }
  
  // Recalculate relative yaw with updated offset
  rel = rawYaw + yawOffset;
  
  // Normalize relative yaw to [-180, 180] range again
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Clamp to steering range and bin
  rel = constrain(rel, -40, 40);
  float bin = round(rel/1.0f)*1.0f;
  
  // Fix negative zero issue
  if (abs(bin) < 0.1f) {
    bin = 0.0f;
  }

  // Debug yaw calculations every 10 seconds (reduced frequency)
  static unsigned long lastYawDebugTime = 0;
  if (millis() - lastYawDebugTime > 10000) {
    Serial.printf("Status - Yaw: %.1f° Bin: %.0f° BLE: %s\n", 
                  rel, bin, deviceConnected ? "Connected" : "Advertising");
    lastYawDebugTime = millis();
  }

  // notify on true bin‐change
  if (deviceConnected && ind32On && !isnan(lastSentBin) && bin!=lastSentBin) {
    sendSteeringBin(bin);
  }
  lastSentBin = bin;

  // Update display with current status using new display system
  updateMainDisplay();

  // —————— Button Handling ——————
  static unsigned long buttonAStartTime = 0;
  static unsigned long buttonBStartTime = 0;
  static unsigned long buttonCStartTime = 0;
  static bool buttonAWasPressed = false;
  static bool buttonBWasPressed = false;
  static bool buttonCWasPressed = false;
  
  // Button GPIO pins
  const int BUTTON_A_PIN = 37;
  const int BUTTON_B_PIN = 39;
  const int BUTTON_C_PIN = 35;
  
  // Read button states directly from GPIO
  bool buttonAState = !digitalRead(BUTTON_A_PIN); // Inverted logic
  bool buttonBState = !digitalRead(BUTTON_B_PIN); // Inverted logic  
  bool buttonCState = !digitalRead(BUTTON_C_PIN); // Inverted logic
  
  // Button A (front face) - Wake screen when off, recenter when on, Long press (>2s): calibration
  if (buttonAState && !buttonAWasPressed) {
    buttonAStartTime = millis();
    buttonAWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button A pressed");
  } else if (!buttonAState && buttonAWasPressed) {
    unsigned long pressDuration = millis() - buttonAStartTime;
    buttonAWasPressed = false;
    
    if (pressDuration < 500) {
      // Short press behavior depends on screen state
      if (!screenOn) {
        // Screen is off - wake it up
        Serial.println("Button A: Wake screen");
        M5.Speaker.tone(800, 100);
        exitScreenOffMode();
        lastButtonTime = millis(); // Reset screen timer
        
        // Show wake-up message briefly
        showStatusMessage("WAKE UP", "Screen On", WHITE, 1500);
      } else {
        // Screen is on - recenter yaw
        Serial.println("Button A: Quick recenter");
        M5.Speaker.tone(800, 100);
        quickRecenterYaw();
      }
    } else if (pressDuration >= 2000) {
      Serial.println("Button A: Long press - Gyro calibration");
      performGyroCalibration();
    }
  }
  
  // Button B (top/north face) - Single click: quick recenter, Long press (>3s): full calibration
  if (buttonBState && !buttonBWasPressed) {
    buttonBStartTime = millis();
    buttonBWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button B pressed");
  } else if (!buttonBState && buttonBWasPressed) {
    unsigned long pressDuration = millis() - buttonBStartTime;
    buttonBWasPressed = false;
    
    if (pressDuration < 500) {
      Serial.println("Button B: Quick recenter");
      M5.Speaker.tone(1000, 100);
      quickRecenterYaw();
    } else if (pressDuration >= 3000) {
      Serial.println("Button B: Long press - Gyro calibration");
      performGyroCalibration();
    }
  }
  
  // Button C (Power button) - Power management
  if (buttonCState && !buttonCWasPressed) {
    buttonCStartTime = millis();
    buttonCWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button C pressed");
  } else if (!buttonCState && buttonCWasPressed) {
    unsigned long pressDuration = millis() - buttonCStartTime;
    buttonCWasPressed = false;
    
    if (pressDuration < 500) {
      Serial.println("Button C: Wake screen");
      M5.Speaker.tone(1200, 100);
      
      // Wake screen and reset timer
      if (!screenOn) {
        exitScreenOffMode();
      }
      lastButtonTime = millis(); // Reset screen timer
    } else if (pressDuration >= 3000) {
      Serial.println("Button C: Long press - Power off");
      
      if (screenOn) {
      M5.Display.clear();
      M5.Display.setRotation(0);
      M5.Display.setCursor(0, 25);
      M5.Display.print("Powering");
      M5.Display.setCursor(0, 40);
      M5.Display.print("Off...");
      delay(1000);
      }
      
      // Try to power off
      pinMode(4, OUTPUT);
      digitalWrite(4, LOW);
      Serial.println("HOLD pin set LOW for power off");
      delay(2000);
      
      // If we get here, power off failed, so enter deep sleep
      Serial.println("Power off failed, entering deep sleep");
      enterULPSleep();
    }
    }
  }
  
// Utility function for precise timing without blocking
bool waitForInterval(unsigned long &lastTime, unsigned long intervalMicros) {
  unsigned long currentTime = micros();
  if (currentTime - lastTime >= intervalMicros) {
    lastTime = currentTime;
    return true;
  }
  return false;
  }
