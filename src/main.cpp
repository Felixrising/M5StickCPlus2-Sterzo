#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <Wire.h>

// Include the MahonyAHRS library
#include "libs/MahonyAHRS.h"

// —————— BLE UUIDs ——————
#define STERZO_SERVICE_UUID "347b0001-7635-408b-8918-8ff3949ce592"
#define CHAR30_UUID         "347b0030-7635-408b-8918-8ff3949ce592"
#define CHAR31_UUID         "347b0031-7635-408b-8918-8ff3949ce592"
#define CHAR32_UUID         "347b0032-7635-408b-8918-8ff3949ce592"

// —————— Pin Definitions ——————
static const int LED_PIN = 19;    // onboard LED on GPIO19

// —————— RTC Memory Variables ——————
RTC_DATA_ATTR int wakeCount = 0;
RTC_DATA_ATTR int timerWakeCount = 0;
RTC_DATA_ATTR int activityWakeCount = 0;

// Sleep baseline data (gravity vector before sleep)
RTC_DATA_ATTR float sleepGravityX = 0.0f;
RTC_DATA_ATTR float sleepGravityY = 0.0f;
RTC_DATA_ATTR float sleepGravityZ = 0.0f;

// —————— Global Variables ——————
Preferences prefs;

// BLE objects
BLEServer* pServer = nullptr;
BLEService* pSvc = nullptr;
BLECharacteristic* pChar30 = nullptr;
BLECharacteristic* pChar31 = nullptr;
BLECharacteristic* pChar32 = nullptr;

bool deviceConnected = false;
bool ind32On = false;
bool challengeOK = false;

// IMU and orientation
float currentPitch = 0.0f;
float currentRoll = 0.0f;
float currentYaw = 0.0f;
float currentGravityX = 0.0f;
float currentGravityY = 0.0f;
float currentGravityZ = 0.0f;

// Calibration data
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float yawOffset = 0;
float lastSentBin = NAN;

// Activity detection
unsigned long wakeUpTime = 0;
unsigned long lastActivityTime = 0;
bool isInActivityCheckMode = false;
bool wasActive = false;

// Display and LED management
bool screenOn = false;
unsigned long lastDisplayUpdate = 0;
unsigned long ledBreathingStartTime = 0;
bool ledBreathingEnabled = true;
bool forceDisplayRedraw = false; // Flag to force complete redraw after screen clear

// Configuration constants
const float POSITION_THRESHOLD_DEGREES = 1.0f;
const unsigned long ACTIVITY_CHECK_DURATION = 1000; // 1 second to check for activity after wake
const unsigned long ACTIVITY_TIMEOUT = 120000;

// IMU and motion detection configuration
const float IMU_LPF_FREQUENCY_HZ = 10.0f;        // IMU low-pass filter frequency (hardware filtering)
const float MOTION_UPDATE_FREQUENCY_HZ = 10.0f;  // Target frequency for motion/steering updates (software processing)
const float MIN_UPDATE_INTERVAL_MS = 1000.0f / MOTION_UPDATE_FREQUENCY_HZ; // Minimum time between updates (100ms for 10Hz)

// Display update configuration
const float DISPLAY_UPDATE_FREQUENCY_HZ = 10.0f; // Display refresh rate (higher = smoother, more CPU)
const float DISPLAY_UPDATE_INTERVAL_MS = 1000.0f / DISPLAY_UPDATE_FREQUENCY_HZ; // Time between display updates (100ms for 10Hz)

// Note: These frequencies can be set independently for optimal performance:
// - Higher IMU LPF = more filtering, smoother but less responsive
// - Higher MOTION_UPDATE = more frequent processing, more CPU but more responsive  
// - Higher DISPLAY_UPDATE = smoother visual updates, more CPU but better UX
// - Typically set them equal or IMU LPF slightly higher than MOTION_UPDATE

// Yaw drift compensation
const float yawDriftRate = 0.2f;  // degrees per second drift back to center
unsigned long lastCenteringTime = 0;

// Display constants
const int DISPLAY_WIDTH_PORTRAIT = 135;
const int DISPLAY_HEIGHT_PORTRAIT = 240;
const int STATUS_BAR_HEIGHT = 20;
const int BUTTON_HELP_HEIGHT = 25;

// —————— Utility Functions ——————
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

// —————— LED Management ——————
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
    breathingPeriod = 1500.0f; // 1.5 second breathing when connected (faster)
  } else {
    breathingPeriod = 3000.0f; // 3 second breathing when advertising (slower)
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

// —————— Forward Declarations ——————
float getYaw();

// —————— Display Management ——————
void initializeDisplay() {
  M5.Display.setRotation(0);  // Portrait mode
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2);
  M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
  screenOn = true;
  Serial.println("Display initialized at 5% brightness");
}

void clearDisplaySafe() {
  M5.Display.clear();
  M5.Display.setTextColor(WHITE); // Reset to default color
}

void drawCenteredText(const char* text, int y, int textSize = 2, uint16_t color = WHITE) {
  M5.Display.setTextSize(textSize);
  M5.Display.setTextColor(color);
  
  // Calculate text width for centering
  int textWidth = strlen(text) * 6 * textSize; // Approximate character width
  int x = (DISPLAY_WIDTH_PORTRAIT - textWidth) / 2;
  if (x < 0) x = 2; // Minimum margin
  
  M5.Display.setCursor(x, y);
  M5.Display.print(text);
}

void drawStatusBar() {
  // Draw status bar background
  M5.Display.fillRect(0, 0, DISPLAY_WIDTH_PORTRAIT, STATUS_BAR_HEIGHT, 0x2104); // Dark gray
  
  // Battery voltage (right side)
  float batteryVoltage = M5.Power.getBatteryVoltage() / 1000.0f;
  M5.Display.setTextSize(1.5);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(DISPLAY_WIDTH_PORTRAIT - 45, 5);
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
  
  // Wake count (center)
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(DISPLAY_WIDTH_PORTRAIT/2 - 30, 5);
  M5.Display.printf("W:%d", wakeCount);
}

void showButtonHelp() {
  int helpY = DISPLAY_HEIGHT_PORTRAIT - BUTTON_HELP_HEIGHT;
  
  // Draw help background
  M5.Display.fillRect(0, helpY, DISPLAY_WIDTH_PORTRAIT, BUTTON_HELP_HEIGHT, 0x1082); // Very dark gray
  
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(0xC618); // Light gray
  
    // Portrait layout - compact button help
    M5.Display.setCursor(2, helpY + 5);
    M5.Display.print("A/B:Recenter C:Screen");
    M5.Display.setCursor(2, helpY + 15);
    M5.Display.print("Hold A/B:Cal Hold C:Off");
}

void updateMainDisplay() {
  if (!screenOn) return;
  
  // Update at configurable frequency for smooth display
  if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL_MS) return;
  lastDisplayUpdate = millis();
  
  // Static variables to track previous values for selective updates
  static bool firstRun = true;
  static char prevYawText[20] = "";
  static char prevBinText[20] = "";
  static bool prevConnected = false;
  static float prevBatteryVoltage = 0;
  static unsigned long prevTimeUntilSleep = 0;
  static bool prevActivityCheckMode = false;
  static uint16_t prevActivityColor = WHITE;
  static char prevActivityText[30] = "";
  static int prevActivityState = -1;
  
  // Check if we need to force a full redraw
  if (forceDisplayRedraw) {
    firstRun = true;
    forceDisplayRedraw = false;
    // Reset all cached values to force updates
    strcpy(prevYawText, "");
    strcpy(prevBinText, "");
    prevConnected = !deviceConnected; // Force connection status update
    prevBatteryVoltage = 0;
    prevTimeUntilSleep = 0;
    prevActivityCheckMode = !isInActivityCheckMode;
    prevActivityColor = WHITE;
    strcpy(prevActivityText, "");
    prevActivityState = -1;
  }
  
  // Calculate current values
  float rawYaw = getYaw();
  float rel = rawYaw + yawOffset;
  
  // Normalize to [-180, 180]
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  float unclamped_rel = rel;
  rel = constrain(rel, -40, 40);
  float bin = round(rel/1.0f)*1.0f;
  if (abs(bin) < 0.1f) bin = 0.0f;
  
  char yawText[20];
  char binText[20];
  bool flashOn = (millis() / 500) % 2 == 0; // Flash every 500ms for smoother animation
  if (unclamped_rel >= 40.0f) {
    snprintf(yawText, sizeof(yawText), "%.1f%s", rel, flashOn ? ">" : "");
  } else if (unclamped_rel <= -40.0f) {
    snprintf(yawText, sizeof(yawText), "%.1f%s", rel, flashOn ? "<" : "");
  } else {
    snprintf(yawText, sizeof(yawText), "%.1f", rel);
  }
  snprintf(binText, sizeof(binText), "%.0f", bin);
  
  // Layout constants
  int contentStartY = STATUS_BAR_HEIGHT + 10;
  int boxY = contentStartY + 30;
  int boxHeight = 50;
  int boxWidth = 60;
  int leftBoxX = 10;
  int rightBoxX = DISPLAY_WIDTH_PORTRAIT - boxWidth - 10;
  
  // First run - draw everything
  if (firstRun) {
    clearDisplaySafe();
    
    // Draw static elements that rarely change
    drawCenteredText("ZTEERSTICK", contentStartY, 2, ORANGE);
    
    // Draw boxes and divider for Yaw and Bin
    M5.Display.drawRect(leftBoxX, boxY, boxWidth, boxHeight, WHITE);
    M5.Display.drawRect(rightBoxX, boxY, boxWidth, boxHeight, CYAN);
    int centerX = DISPLAY_WIDTH_PORTRAIT / 2;
    M5.Display.drawLine(centerX, boxY, centerX, boxY + boxHeight, WHITE);
    
    // Draw box labels
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(leftBoxX + (boxWidth - 18) / 2, boxY + 5);
    M5.Display.print("YAW");
    
    M5.Display.setTextColor(CYAN);
    M5.Display.setCursor(rightBoxX + (boxWidth - 18) / 2, boxY + 5);
    M5.Display.print("BIN");
    
    showButtonHelp(); // Static button help
    firstRun = false;
  }
  
  // Selective updates - only redraw what changed
  
  // 1. Update Yaw value if changed
  if (strcmp(yawText, prevYawText) != 0) {
    // Clear yaw value area
    M5.Display.fillRect(leftBoxX + 1, boxY + 20, boxWidth - 6, 25, BLACK);
    
    // Draw new yaw value
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    int yawValueWidth = strlen(yawText) * 12;
    int yawValueX = leftBoxX + (boxWidth - yawValueWidth) / 2;
    M5.Display.setCursor(yawValueX, boxY + 25);
    M5.Display.print(yawText);
    
    strcpy(prevYawText, yawText);
  }
  
  // 2. Update Bin value if changed
  if (strcmp(binText, prevBinText) != 0) {
    // Clear bin value area
    M5.Display.fillRect(rightBoxX + 5, boxY + 20, boxWidth - 6, 25, BLACK);
    
    // Draw new bin value
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(CYAN);
    int binValueWidth = strlen(binText) * 12;
    int binValueX = rightBoxX + (boxWidth - binValueWidth) / 2;
    M5.Display.setCursor(binValueX, boxY + 25);
    M5.Display.print(binText);
    
    strcpy(prevBinText, binText);
  }
  
  // 3. Update status bar if battery or connection changed
  float batteryVoltage = M5.Power.getBatteryVoltage() / 1000.0f;
  if (abs(batteryVoltage - prevBatteryVoltage) > 0.01f || deviceConnected != prevConnected) {
    drawStatusBar();
    prevBatteryVoltage = batteryVoltage;
    prevConnected = deviceConnected;
  }
  
  // 4. Update connection status if changed
  if (deviceConnected != prevConnected) {
    // Clear connection status area
    M5.Display.fillRect(10, contentStartY + 95, DISPLAY_WIDTH_PORTRAIT - 20, 15, BLACK);
    
    // Draw new connection status
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, contentStartY + 95);
    if (deviceConnected) {
      M5.Display.setTextColor(GREEN);
      M5.Display.print("ZWIFTING!!!");
    } else {
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("READY TO ROLL");
    }
  }
  
  // 5. Update activity status (check every update for smooth transitions)
  char activityText[30];
  uint16_t activityColor;
  int currentActivityState = 0; // 0=active, 1=idle, 2=sleep countdown, 3=checking
  
  if (isInActivityCheckMode) {
    strcpy(activityText, "Checking position...");
    activityColor = TFT_LIGHTGREY;
    currentActivityState = 3;
  } else {
    unsigned long timeSinceActivity = millis() - lastActivityTime;
    unsigned long timeUntilSleep = (ACTIVITY_TIMEOUT - timeSinceActivity) / 1000;
    
    if (timeSinceActivity < 5000) {
      strcpy(activityText, "A");
      activityColor = GREEN;
      currentActivityState = 0;
      // Ensure screen is at normal brightness when active
      if (M5.Display.getBrightness() != 13) {
        M5.Display.setBrightness(13); // 5% brightness
      }
    } else if (timeSinceActivity < 30000) {
      strcpy(activityText, "I");
      activityColor = YELLOW;
      currentActivityState = 1;
      // Dim screen to 1% when idle
      if (M5.Display.getBrightness() != 1) {
        M5.Display.setBrightness(1); // 1% brightness
      }
    } else {
      snprintf(activityText, sizeof(activityText), "Zzzz:%lus", timeUntilSleep);
      activityColor = TFT_LIGHTGREY;
      currentActivityState = 2;
      // Keep screen dimmed when approaching sleep
      if (M5.Display.getBrightness() != 1) {
        M5.Display.setBrightness(1); // 1% brightness
      }
    }
  }
  
  // Check for major activity state transitions that require full redraw
  bool majorStateTransition = false;
  if (prevActivityState != -1 && prevActivityState != currentActivityState) {
    // Detect transitions that involve significant brightness changes
    if ((prevActivityState == 2 && currentActivityState == 0) ||  // Sleep countdown -> Active
        (prevActivityState == 1 && currentActivityState == 0) ||  // Idle -> Active  
        (prevActivityState == 0 && currentActivityState == 1) ||  // Active -> Idle
        (prevActivityState == 0 && currentActivityState == 2)) {  // Active -> Sleep countdown
      majorStateTransition = true;
      Serial.printf("Major activity state transition: %d -> %d, forcing redraw\n", prevActivityState, currentActivityState);
    }
  }
  
  // Force redraw on major transitions
  if (majorStateTransition && !firstRun) {
    firstRun = true;
    // Reset all cached values to force updates
    strcpy(prevYawText, "");
    strcpy(prevBinText, "");
    prevConnected = !deviceConnected;
    prevBatteryVoltage = 0;
    prevTimeUntilSleep = 0;
    prevActivityCheckMode = !isInActivityCheckMode;
    prevActivityColor = WHITE;
    strcpy(prevActivityText, "");
    prevActivityState = -1;
  }
  
  // Only update activity status if it changed
  if (strcmp(activityText, prevActivityText) != 0 || activityColor != prevActivityColor || 
      isInActivityCheckMode != prevActivityCheckMode || majorStateTransition) {
    // Clear activity status area
    M5.Display.fillRect(10, contentStartY + 170, DISPLAY_WIDTH_PORTRAIT - 20, 15, BLACK);
    
    // Draw new activity status
    M5.Display.setTextSize(1.5);
    M5.Display.setTextColor(activityColor);
    M5.Display.setCursor(10, contentStartY + 170);
    M5.Display.print(activityText);
    
    strcpy(prevActivityText, activityText);
    prevActivityColor = activityColor;
    prevActivityCheckMode = isInActivityCheckMode;
    prevActivityState = currentActivityState;
  }
  
  M5.Display.display();
}

void showStatusMessage(const char* title, const char* message, uint16_t color = WHITE, int duration = 2000) {
  if (!screenOn) return;
  
  clearDisplaySafe();
  
  int centerY = DISPLAY_HEIGHT_PORTRAIT / 2;
  
  drawCenteredText(title, centerY - 20, 2.5, color);
  drawCenteredText(message, centerY + 10, 2, WHITE);
  
  M5.Display.display();
  delay(duration);
  
  // Force a complete redraw of the main display after clearing the screen
  forceDisplayRedraw = true;
  
  Serial.printf("Status message: %s - %s\n", title, message);
}

void showConnectionScreen(bool connected) {
  if (!screenOn) return;
  
  clearDisplaySafe();
  
  int centerY = DISPLAY_HEIGHT_PORTRAIT / 2;
  
  if (connected) {
    drawCenteredText("BLE", centerY - 20, 2, GREEN);
    drawCenteredText("CONNECTED", centerY + 10, 2, GREEN);
    drawCenteredText("Faster LED pulse", centerY + 40, 1, WHITE);
  } else {
    drawCenteredText("BLE", centerY - 20, 2, RED);
    drawCenteredText("DISCONNECTED", centerY + 10, 2, RED);
    drawCenteredText("Slower LED pulse", centerY + 40, 1, WHITE);
  }
  
  M5.Display.display();
  delay(3000);
}

void turnOnScreen() {
  if (!screenOn) {
    Serial.println("Turning on screen");
    M5.Display.wakeup();
    M5.Display.setBrightness(13); // 5% brightness
    screenOn = true;
    initializeDisplay();
    Serial.println("Screen on at 5% brightness");
  }
}

void turnOffScreen() {
  if (screenOn) {
    Serial.println("Turning off screen");
    M5.Display.clear();
    M5.Display.sleep();
    screenOn = false;
    Serial.println("Screen off");
  }
}

// —————— BLE Callbacks ——————
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    lastActivityTime = millis();
    Serial.println("BLE client connected");
    
    // Show connection success message if screen is on
    if (screenOn) {
      showConnectionScreen(true);
    }
    
    // Success beep pattern
    M5.Speaker.tone(800, 100);
    delay(150);
    M5.Speaker.tone(1000, 100);
    delay(150);
    M5.Speaker.tone(1200, 150);
  }
  
  void onDisconnect(BLEServer* s) override {
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
      uint32_t seed = ((uint32_t)val[5]<<24) | ((uint32_t)val[4]<<16) | ((uint32_t)val[3]<<8) | ((uint32_t)val[2]);
      uint32_t pwd = hashed(seed);
      Serial.printf("→ got 0x312, seed=0x%08X, pwd=0x%08X\n", seed, pwd);
      uint8_t res[6];
      res[0]=0x03; res[1]=0x12;
      res[2]= pwd & 0xFF;
      res[3]=(pwd >> 8) & 0xFF;
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

class char32Desc2902Callbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* d) override {
    ind32On = true;
    Serial.println("Client ENABLED indications (char32)");
  }
};

// —————— IMU Functions ——————
void updateGravityVector() {
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
  // Update Mahony AHRS with calibrated data
  MahonyAHRSupdateIMU(
    (gx - gyroBiasX) * DEG_TO_RAD,
    (gy - gyroBiasY) * DEG_TO_RAD,
    (gz - gyroBiasZ) * DEG_TO_RAD,
    ax - accelBiasX,
    ay - accelBiasY,
    az - accelBiasZ,
    &currentPitch, &currentRoll, &currentYaw
  );
  
  // Normalize accelerometer data to get gravity vector
  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  if (magnitude > 0.1f) {
    currentGravityX = ax / magnitude;
    currentGravityY = ay / magnitude;
    currentGravityZ = az / magnitude;
  }
}

bool checkForPositionChange() {
  if (wakeCount <= 1) return false; // No baseline yet
  
  // Static variables to track baseline position for ongoing monitoring
  static float baselineGravityX = 0.0f;
  static float baselineGravityY = 0.0f;
  static float baselineGravityZ = 0.0f;
  static bool baselineSet = false;
  static unsigned long lastPositionCheck = 0;
  
  unsigned long currentTime = millis();
  
  // In activity check mode: compare against sleep position once
  if (isInActivityCheckMode) {
    // Calculate angular difference between sleep and current gravity vectors
    float dotProduct = sleepGravityX * currentGravityX + 
                      sleepGravityY * currentGravityY + 
                      sleepGravityZ * currentGravityZ;
    
    // Clamp dot product to valid range for acos
    dotProduct = constrain(dotProduct, -1.0f, 1.0f);
    
    // Calculate angle in degrees
    float angleDifference = acos(dotProduct) * 180.0f / PI;
    
    Serial.printf("=== WAKE-UP POSITION CHECK ===\n");
    Serial.printf("Sleep gravity: (%.3f, %.3f, %.3f)\n", sleepGravityX, sleepGravityY, sleepGravityZ);
    Serial.printf("Current gravity: (%.3f, %.3f, %.3f)\n", currentGravityX, currentGravityY, currentGravityZ);
    Serial.printf("Angle difference: %.2f degrees (threshold: %.2f)\n", angleDifference, POSITION_THRESHOLD_DEGREES);
    Serial.printf("Position changed since sleep: %s\n", (angleDifference > POSITION_THRESHOLD_DEGREES) ? "YES" : "NO");
    Serial.printf("===============================\n");
    
    // If position changed, set current position as new baseline for ongoing monitoring
    if (angleDifference > POSITION_THRESHOLD_DEGREES) {
      baselineGravityX = currentGravityX;
      baselineGravityY = currentGravityY;
      baselineGravityZ = currentGravityZ;
      baselineSet = true;
      lastPositionCheck = currentTime;
      Serial.println("New baseline position set for ongoing monitoring");
    }
    
    return angleDifference > POSITION_THRESHOLD_DEGREES;
  }
  
  // In normal monitoring mode: compare against recent baseline at 1Hz
  if (!baselineSet) {
    // Set initial baseline if not set
    baselineGravityX = currentGravityX;
    baselineGravityY = currentGravityY;
    baselineGravityZ = currentGravityZ;
    baselineSet = true;
    lastPositionCheck = currentTime;
    Serial.println("Initial baseline position set for monitoring");
    return false;
  }
  
  // Check position at 1Hz rate
  if (currentTime - lastPositionCheck < 1000) {
    return false; // Not time to check yet
  }
  
  lastPositionCheck = currentTime;
  
  // Calculate angular difference between baseline and current gravity vectors
  float dotProduct = baselineGravityX * currentGravityX + 
                    baselineGravityY * currentGravityY + 
                    baselineGravityZ * currentGravityZ;
  
  // Clamp dot product to valid range for acos
  dotProduct = constrain(dotProduct, -1.0f, 1.0f);
  
  // Calculate angle in degrees
  float angleDifference = acos(dotProduct) * 180.0f / PI;
  
  bool positionChanged = angleDifference > POSITION_THRESHOLD_DEGREES;
  
  if (positionChanged) {
    Serial.printf("=== ONGOING POSITION CHECK ===\n");
    Serial.printf("Baseline gravity: (%.3f, %.3f, %.3f)\n", baselineGravityX, baselineGravityY, baselineGravityZ);
    Serial.printf("Current gravity: (%.3f, %.3f, %.3f)\n", currentGravityX, currentGravityY, currentGravityZ);
    Serial.printf("Angle difference: %.2f degrees (threshold: %.2f)\n", angleDifference, POSITION_THRESHOLD_DEGREES);
    Serial.printf("Position changed: YES - updating baseline\n");
    Serial.printf("===============================\n");
    
    // Update baseline to current position for next comparison
    baselineGravityX = currentGravityX;
    baselineGravityY = currentGravityY;
    baselineGravityZ = currentGravityZ;
  }
  
  return positionChanged;
}

bool checkForButtonActivity() {
  static bool lastBtnA = HIGH, lastBtnB = HIGH;
  bool currentBtnA = digitalRead(37);
  bool currentBtnB = digitalRead(39);
  
  bool buttonActivity = false;
  
  if (currentBtnA != lastBtnA) {
    Serial.printf("BUTTON A: %s\n", currentBtnA == LOW ? "PRESSED" : "RELEASED");
    buttonActivity = true;
    lastBtnA = currentBtnA;
  }
  
  if (currentBtnB != lastBtnB) {
    Serial.printf("BUTTON B: %s\n", currentBtnB == LOW ? "PRESSED" : "RELEASED");
    buttonActivity = true;
    lastBtnB = currentBtnB;
  }
  
  return buttonActivity;
}



// —————— Calibration Functions ——————
bool calibrationExists() {
  // Check if key gyro calibration values exist
  prefs.begin("sterzo", true); // Read-only
  bool exists = prefs.isKey("gxb") && prefs.isKey("gyb") && prefs.isKey("gzb");
  prefs.end();
  return exists;
}

void performGyroCalibration() {
  Serial.println("=== STARTING GYRO CALIBRATION ===");
  
  if (screenOn) {
    showStatusMessage("CALIBRATION", "Keep device still", ORANGE, 1000);
  }
  
  // Reset gyro biases
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  
  // Collect 100 samples over 2 seconds
  const int samples = 100;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  
  Serial.println("Collecting gyro samples...");
  
  for (int i = 0; i < samples; i++) {
  float gx, gy, gz, ax, ay, az;
  M5.Imu.getGyro(&gx, &gy, &gz);
  M5.Imu.getAccel(&ax, &ay, &az);
  
    sumGx += gx; 
    sumGy += gy; 
    sumGz += gz;
    
    // Update display progress
    if (screenOn && i % 10 == 0) {
      char progressText[32];
      snprintf(progressText, sizeof(progressText), "Progress: %d%%", (i * 100) / samples);
      showStatusMessage("CALIBRATION", progressText, ORANGE, 50);
    }
    
    delay(20); // 50Hz sampling
  }
  
  // Calculate gyro biases
  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;
  gyroBiasZ = sumGz / samples;
  
  Serial.printf("Gyro calibration complete - Bias: %.3f,%.3f,%.3f\n", 
                gyroBiasX, gyroBiasY, gyroBiasZ);
    
  // Save gyro biases
  prefs.begin("sterzo", false);
  prefs.putFloat("gxb", gyroBiasX);
  prefs.putFloat("gyb", gyroBiasY);
  prefs.putFloat("gzb", gyroBiasZ);
  prefs.end();
  
  if (screenOn) {
    showStatusMessage("CALIBRATION", "Complete!", GREEN, 2000);
  }
  
  Serial.println("=== GYRO CALIBRATION COMPLETE ===");
  }
  
// —————— Button Handling Functions ——————
void quickRecenterYaw() {
  Serial.println("Quick recentering yaw...");
  
  // Distinctive beep pattern for quick recenter
  M5.Speaker.tone(800, 50);
      delay(100);
  M5.Speaker.tone(1000, 50);
  
  // Get current raw yaw and set offset to make current position = 0
  float currentRawYaw = getYaw();
  
  if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
    yawOffset = -currentRawYaw;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    // Save to preferences
    prefs.begin("sterzo", false);
    prefs.putFloat("yoff", yawOffset);
    prefs.end();
    
    Serial.printf("Quick recenter - Current raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);
    
    // Show confirmation if screen is on
    if (screenOn) {
      showStatusMessage("ZEROing", "Yaw to 0", GREEN, 1500);
    }
  } else {
    Serial.println("WARNING: Invalid yaw value during quick recenter");
  }
}

void handleButtonPresses() {
  // Button state tracking
  static unsigned long buttonAStartTime = 0;
  static unsigned long buttonBStartTime = 0;
  static unsigned long buttonCStartTime = 0;
  static bool buttonAWasPressed = false;
  static bool buttonBWasPressed = false;
  static bool buttonCWasPressed = false;
  
  // Read button states (active LOW)
  bool buttonAState = !digitalRead(37); // Button A
  bool buttonBState = !digitalRead(39); // Button B
  bool buttonCState = !digitalRead(35); // Button C
  
  // Button A handling
  if (buttonAState && !buttonAWasPressed) {
    buttonAStartTime = millis();
    buttonAWasPressed = true;
    lastActivityTime = millis(); // Update activity time
    Serial.println("Button A pressed");
  } else if (!buttonAState && buttonAWasPressed) {
    unsigned long pressDuration = millis() - buttonAStartTime;
    buttonAWasPressed = false;
    
    if (pressDuration < 500) {
       // Short press - quick recenter
        Serial.println("Button A: Quick recenter");
        M5.Speaker.tone(800, 100);
        quickRecenterYaw();
       
       // Turn on screen if it's off
       if (!screenOn) {
         turnOnScreen();
      }
    } else if (pressDuration >= 2000) {
       // Long press - gyro calibration
      Serial.println("Button A: Long press - Gyro calibration");
      performGyroCalibration();
    }
  }
  
  // Button B handling
  if (buttonBState && !buttonBWasPressed) {
    buttonBStartTime = millis();
    buttonBWasPressed = true;
    lastActivityTime = millis(); // Update activity time
    Serial.println("Button B pressed");
  } else if (!buttonBState && buttonBWasPressed) {
    unsigned long pressDuration = millis() - buttonBStartTime;
    buttonBWasPressed = false;
    
    if (pressDuration < 500) {
       // Short press - quick recenter
      Serial.println("Button B: Quick recenter");
      M5.Speaker.tone(1000, 100);
      quickRecenterYaw();
       
       // Turn on screen if it's off
       if (!screenOn) {
         turnOnScreen();
       }
     } else if (pressDuration >= 2000) {
       // Long press - gyro calibration
      Serial.println("Button B: Long press - Gyro calibration");
      performGyroCalibration();
    }
  }
  
  // Button C handling
  if (buttonCState && !buttonCWasPressed) {
    buttonCStartTime = millis();
    buttonCWasPressed = true;
    lastActivityTime = millis(); // Update activity time
    Serial.println("Button C pressed");
  } else if (!buttonCState && buttonCWasPressed) {
    unsigned long pressDuration = millis() - buttonCStartTime;
    buttonCWasPressed = false;
    
    if (pressDuration < 500) {
      // Short press - turn on screen
      Serial.println("Button C: Turn on screen");
      M5.Speaker.tone(1200, 100);
      
      if (!screenOn) {
        turnOnScreen();
        showStatusMessage("SCREEN ON", "Display activated", WHITE, 1500);
      }
    }
    // Long press functionality can be added here if needed
  }
}

// —————— Steering Functions ——————
void sendSteeringBin(float bin) {
  if (pChar30 && deviceConnected) {
    pChar30->setValue((uint8_t*)&bin, sizeof(bin));
    pChar30->notify();
    Serial.printf("NOTIFY bin: %.0f°\n", bin);
  }
}

float getYaw() {
  return -currentYaw; // Invert for correct steering direction
}

void updateSteering(unsigned long currentTime) {
  float rawYaw = getYaw();
  
  // Apply yaw drift compensation at loop frequency
  if (lastCenteringTime == 0) {
    lastCenteringTime = currentTime;
  }
  
  float deltaTime = (currentTime - lastCenteringTime) / 1000.0f;
  if ((currentTime - lastCenteringTime) >= MIN_UPDATE_INTERVAL_MS) { // Respect target update frequency
    float rel = rawYaw + yawOffset;
    
    // Normalize to [-180, 180]
    while (rel > 180) rel -= 360;
    while (rel < -180) rel += 360;
    
    // Apply centering force proportional to actual time elapsed
    float centeringAdjustment = -rel * yawDriftRate * deltaTime;
    
    // Prevent overshoot: don't adjust more than the distance to center
    centeringAdjustment = constrain(centeringAdjustment, -abs(rel), abs(rel));
    
    yawOffset += centeringAdjustment;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    lastCenteringTime = currentTime;
    
    // Debug centering compensation occasionally
    static unsigned long lastDebugTime = 0;
    if (abs(centeringAdjustment) > 0.01f && currentTime - lastDebugTime > 5000) {
      Serial.printf("Centering: rel=%.2f° rate=%.1f°/s adj=%.3f° newOffset=%.2f°\n", 
                    rel, yawDriftRate, centeringAdjustment, yawOffset);
      lastDebugTime = currentTime;
    }
  }
  
  // Calculate final relative yaw for steering
  float rel = rawYaw + yawOffset;
  
  // Normalize to [-180, 180]
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Clamp to steering range and bin
  rel = constrain(rel, -40, 40);
  float bin = round(rel/1.0f)*1.0f;
  if (abs(bin) < 0.1f) bin = 0.0f;
  
  // Check for steering activity (bin changes indicate active steering)
  if (!isnan(lastSentBin) && bin != lastSentBin) {
    // Reset activity timer on any steering change (connected or not)
    // Use the same currentTime from updateSteering to avoid timing issues
    lastActivityTime = currentTime;
    Serial.printf("Steering activity detected: bin %.0f° -> %.0f°\n", lastSentBin, bin);
    
    // Send steering data if connected
    if (deviceConnected && challengeOK) {
      sendSteeringBin(bin);
    }
  }
  lastSentBin = bin;
}

// —————— IMU Configuration ——————
uint8_t getLPFRegisterValue(float frequency) {
  // MPU6886 LPF register values for different frequencies
  // Accelerometer: 5.1, 10.2, 21.2, 44.8, 99.0, 218.1, 420.0, 1046 Hz
  // Gyroscope: 5, 10, 20, 41, 92, 176, 250, 3281, 8173 Hz
  if (frequency <= 5.5f) return 0x06;      // ~5Hz
  else if (frequency <= 15.0f) return 0x05; // ~10Hz  
  else if (frequency <= 30.0f) return 0x04; // ~20Hz
  else if (frequency <= 65.0f) return 0x03; // ~41-44Hz
  else if (frequency <= 130.0f) return 0x02; // ~92-99Hz
  else if (frequency <= 200.0f) return 0x01; // ~176-218Hz
  else return 0x00; // ~250Hz+ (minimal filtering)
}

void configureIMU() {
  Serial.printf("Configuring MPU6886 with %.1fHz low-pass filters...\n", IMU_LPF_FREQUENCY_HZ);
  
  uint8_t lpfRegValue = getLPFRegisterValue(IMU_LPF_FREQUENCY_HZ);
  
  // Set accelerometer LPF
  Wire1.beginTransmission(0x68); // MPU6886_ADDRESS
  Wire1.write(0x1D);             // MPU6886_ACCEL_CONFIG2
  Wire1.write(lpfRegValue);      // LPF setting based on frequency
  Wire1.endTransmission();
  delay(1);
  
  // Set gyroscope LPF
  Wire1.beginTransmission(0x68); // MPU6886_ADDRESS  
  Wire1.write(0x1A);             // MPU6886_CONFIG
  Wire1.write(lpfRegValue);      // LPF setting based on frequency
  Wire1.endTransmission();
  delay(1);
  
  // Fix sensor output limitation (recommended by datasheet)
  Wire1.beginTransmission(0x68); // MPU6886_ADDRESS
  Wire1.write(0x69);             // MPU6886_ACCEL_INTEL_CTRL
  Wire1.write(0x02);             // Avoid limiting output to 0x7F7F
  Wire1.endTransmission();
  delay(1);
  
  Serial.printf("IMU configured: %.1fHz LPF (reg=0x%02X), Motion updates at %.1fHz\n", 
                IMU_LPF_FREQUENCY_HZ, lpfRegValue, MOTION_UPDATE_FREQUENCY_HZ);
}

// —————— Sleep Functions ——————
void goToSleep() {
  Serial.println("\n=== Preparing for Deep Sleep ===");
  
  // Update gravity vector one final time before sleep
  updateGravityVector();
  
  // Store current gravity vector as baseline for next wake
  sleepGravityX = currentGravityX;
  sleepGravityY = currentGravityY;
  sleepGravityZ = currentGravityZ;
  
  Serial.printf("Storing sleep gravity vector: %.3f, %.3f, %.3f\n", 
                sleepGravityX, sleepGravityY, sleepGravityZ);
  
  // Turn off screen and LED
  turnOffScreen();
  ledcWrite(0, 0); // Turn off LED
  ledBreathingEnabled = false;
  
  // Stop BLE to save power
  if (pServer) {
    pServer->getAdvertising()->stop();
  }
  BLEDevice::deinit(false);
  
  // Configure GPIO4 HOLD to maintain power during deep sleep
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  gpio_hold_en(GPIO_NUM_4);
  gpio_deep_sleep_hold_en();
  Serial.println("GPIO4 HOLD configured for battery power");
  
  // Configure timer wake-up only
  esp_sleep_enable_timer_wakeup(6000000ULL); // 6 seconds
  Serial.println("Timer wake-up configured for 6 seconds");
  
  Serial.println("Entering deep sleep...");
  Serial.flush();
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

// —————— Setup Function ——————
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== M5StickCPlus2 Simple Sterzo ===");
  
  // Power management: Set HOLD pin high to maintain power
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  Serial.println("HOLD pin set HIGH to maintain power");
  
  // Check wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  // Initialize M5Unified
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Configure IMU for optimal steering performance
  configureIMU();
  
  // Setup LED with PWM for breathing pattern
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);           // PWM channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);       // Attach LED pin to PWM channel 0
  ledBreathingStartTime = millis();
  ledBreathingEnabled = true;
  Serial.println("LED configured for breathing pattern");
  
  wakeCount++;
  wakeUpTime = millis();
  
  // Initialize button pins
  pinMode(37, INPUT_PULLUP); // Button A
  pinMode(39, INPUT_PULLUP); // Button B
  pinMode(35, INPUT_PULLUP); // Button C
  
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    timerWakeCount++;
    isInActivityCheckMode = true;
    lastActivityTime = 0;
    
    // Keep display off for timer wake-ups
    screenOn = false;
    M5.Display.sleep();
    
    // Pulse LED to indicate wake-up
    wakePulse();
    
    Serial.printf("=== Timer Wake #%d ===\n", timerWakeCount);
    Serial.println("Checking for position change...");
    
    // Quick IMU stabilization
    delay(50);
    updateGravityVector();
    delay(10);
    updateGravityVector();
    
  } else {
    // Fresh boot
    Serial.printf("=== Fresh Boot #%d ===\n", wakeCount);
    lastActivityTime = millis();
    isInActivityCheckMode = false;
    
    // Initialize display for fresh boots
    turnOnScreen();
    showStatusMessage("ZTEERSTICK", "Starting...", ORANGE, 2000);
    
    // Full IMU stabilization
    delay(1000);
    for (int i = 0; i < 10; i++) {
      updateGravityVector();
      delay(100);
    }
    
    // Load or perform calibration
    if (!calibrationExists()) {
      // First boot - no calibration exists
      Serial.println("No calibration found - performing auto-calibration");
      showStatusMessage("FIRST BOOT", "Auto-calibrating...", ORANGE, 2000);
      performGyroCalibration();
      
      // Set initial yaw offset to 0
      yawOffset = 0;
      prefs.begin("sterzo", false);
      prefs.putFloat("yoff", yawOffset);
      prefs.end();
    } else {
      // Load existing calibration
      Serial.println("Loading existing calibration data");
      prefs.begin("sterzo", false);
      gyroBiasX = prefs.getFloat("gxb", 0);
      gyroBiasY = prefs.getFloat("gyb", 0);
      gyroBiasZ = prefs.getFloat("gzb", 0);
      yawOffset = prefs.getFloat("yoff", 0);
      prefs.end();
      
      Serial.printf("Loaded calibration - Gyro: [%.3f,%.3f,%.3f] YawOffset: %.3f° DriftRate: %.2f°/s\n", 
                    gyroBiasX, gyroBiasY, gyroBiasZ, yawOffset, yawDriftRate);
    }
    
    // Auto-zero yaw after boot (similar to wake due to activity)
    Serial.println("Auto-zeroing yaw after boot for perfect center alignment");
    showStatusMessage("ZTEERSTICK", "Zeroing yaw", GREEN, 1500);
    delay(500); // Allow IMU to stabilize
    float currentRawYaw = getYaw();
    if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
      yawOffset = -currentRawYaw;
      // Normalize offset to [-180, 180] range
      while (yawOffset > 180) yawOffset -= 360;
      while (yawOffset < -180) yawOffset += 360;
      // Save to preferences
      prefs.begin("sterzo", false);
      prefs.putFloat("yoff", yawOffset);
      prefs.end();
      Serial.printf("Auto-zero on boot - Raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);
      //showStatusMessage("CENTERED!", "Ready to ride!", GREEN, 1500);
    } else {
      Serial.println("Warning: Could not auto-zero yaw - invalid yaw reading");
      showStatusMessage("WARNING", "Yaw zero failed", YELLOW, 1500);
    }
    
    // Initialize BLE
    BLEDevice::init("STERZO");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pSvc = pServer->createService(STERZO_SERVICE_UUID);
    
    pChar30 = pSvc->createCharacteristic(CHAR30_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pChar31 = pSvc->createCharacteristic(CHAR31_UUID, BLECharacteristic::PROPERTY_WRITE);
    pChar32 = pSvc->createCharacteristic(CHAR32_UUID, BLECharacteristic::PROPERTY_INDICATE);
    
    BLE2902* p2902_30 = new BLE2902();
    p2902_30->setNotifications(true);
    pChar30->addDescriptor(p2902_30);
    
    BLE2902* p2902_32 = new BLE2902();
    p2902_32->setCallbacks(new char32Desc2902Callbacks());
    pChar32->addDescriptor(p2902_32);
    
    pChar31->setCallbacks(new char31Callbacks());
    
    pSvc->start();
    
    auto adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(STERZO_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE advertising started");
    
    showStatusMessage("BLE", "Advertising", YELLOW, 2000);
  }
  
  Serial.println("Setup complete");
}

// —————— Main Loop ——————
void loop() {
  unsigned long currentTime = millis();
  
  // Update LED breathing pattern
  updateLEDBreathing();
  
  // Update IMU and gravity vector
  updateGravityVector();
  
  // Handle button presses for recenter/zero yaw functionality
  handleButtonPresses();
  
  // Check for activity (simplified - removed duplicate motion detection)
  // Position change uses dot product method - more reliable than motion rate detection
  bool positionChanged = checkForPositionChange();
  bool buttonActivity = checkForButtonActivity();
  
  // Handle activity detection
  if (positionChanged || buttonActivity) {
    lastActivityTime = currentTime;
    wasActive = true;
    
    if (isInActivityCheckMode) {
      Serial.println("Activity detected during check - Switching to monitoring mode");
      isInActivityCheckMode = false;
      activityWakeCount++;
      
      // Recenter yaw when waking from sleep due to activity
      Serial.println("Auto-recentering yaw on wake from sleep");
      float currentRawYaw = getYaw();
      if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
        yawOffset = -currentRawYaw;
        // Normalize offset to [-180, 180] range
        while (yawOffset > 180) yawOffset -= 360;
        while (yawOffset < -180) yawOffset += 360;
        // Save to preferences
        prefs.begin("sterzo", false);
        prefs.putFloat("yoff", yawOffset);
        prefs.end();
        Serial.printf("Auto-recenter on wake - Raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);
      }
      
      // Turn on display and start BLE
      turnOnScreen();
      showStatusMessage("Waking up..", "Let's Ride!", GREEN, 2000);
      
      // Initialize BLE if not already done
      if (!pServer) {
        BLEDevice::init("STERZO");
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());
        pSvc = pServer->createService(STERZO_SERVICE_UUID);
        
        pChar30 = pSvc->createCharacteristic(CHAR30_UUID, BLECharacteristic::PROPERTY_NOTIFY);
        pChar31 = pSvc->createCharacteristic(CHAR31_UUID, BLECharacteristic::PROPERTY_WRITE);
        pChar32 = pSvc->createCharacteristic(CHAR32_UUID, BLECharacteristic::PROPERTY_INDICATE);
        
        BLE2902* p2902_30 = new BLE2902();
        p2902_30->setNotifications(true);
        pChar30->addDescriptor(p2902_30);
        
        BLE2902* p2902_32 = new BLE2902();
        p2902_32->setCallbacks(new char32Desc2902Callbacks());
        pChar32->addDescriptor(p2902_32);
        
        pChar31->setCallbacks(new char31Callbacks());
        
        pSvc->start();
        
        auto adv = BLEDevice::getAdvertising();
        adv->addServiceUUID(STERZO_SERVICE_UUID);
        adv->setScanResponse(true);
        adv->setMinPreferred(0x06);
        adv->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
        
        Serial.println("BLE advertising restarted");
      }
    }
  }
  
  // Update steering if connected
  updateSteering(currentTime);
  
  // Update display if screen is on
  updateMainDisplay();
  
  // Determine sleep behavior
  bool shouldSleep = false;
  
  if (isInActivityCheckMode) {
    // In activity check mode
    unsigned long checkTime = currentTime - wakeUpTime;
    Serial.printf("Activity check mode: checkTime=%lums, duration=%lums, lastActivity=%lu\n", 
                  checkTime, ACTIVITY_CHECK_DURATION, lastActivityTime);
    
    if (checkTime >= ACTIVITY_CHECK_DURATION) {
      if (lastActivityTime == 0) {
        Serial.println("Position check timeout - No movement detected, returning to sleep");
        shouldSleep = true;
      } else {
        Serial.println("Position change found during check - Switching to monitoring mode");
        isInActivityCheckMode = false;
      }
    }
  } else {
    // In normal monitoring mode
    if (lastActivityTime > 0) {
      unsigned long timeSinceActivity = currentTime - lastActivityTime;
      Serial.printf("Monitoring mode: timeSinceActivity=%lums, timeout=%lums\n", 
                    timeSinceActivity, ACTIVITY_TIMEOUT);
      
      if (timeSinceActivity >= ACTIVITY_TIMEOUT) {
        Serial.println("Activity timeout reached - going to sleep");
        shouldSleep = true;
      }
    }
  }
  
  // Enter sleep if conditions are met
  if (shouldSleep) {
    goToSleep();
  }
  
  // Small delay to prevent excessive CPU usage
  delay(100);
}