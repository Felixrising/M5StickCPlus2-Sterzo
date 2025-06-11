#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#ifdef ESP32
extern HardwareSerial Serial;
#endif

// —————— Function Declarations ——————
void performFullCalibration();
void performGyroCalibration();
void recenterYaw();
void sendSteeringBin(float bin);
float applyYawDriftCompensation(float currentYaw);
void resetYawDrift();
void filterGyroReadings(float gx, float gy, float gz);
bool calibrationExists();
void performAutoCalibration();
void performAutoRecenter();
void performStartupButtonTest();
void initializeMahonyFilter();
// Removed updateContinuousDriftCompensation - using simple centering instead
void quickRecenterYaw();
bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az);
void enterLowPowerMode();
void exitLowPowerMode();
void enterDeepSleep();
bool shouldEnterSleep();
void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az);

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
float lastYawTime = 0;      // last time yaw was processed
float accumulatedYawDrift = 0.0f;
bool yawDriftEnabled = true; // enable/disable drift compensation

// Gyro filtering
float gyroFilterAlpha = 0.1f; // Low-pass filter coefficient (0.0 = no filtering, 1.0 = no change)
float filteredGx = 0, filteredGy = 0, filteredGz = 0;

// Timing for dynamic sample frequency
float dt, preTime;
float loopfreq;

// Display sleep management
unsigned long lastActivityTime = 0;
bool displaySleeping = false;
const unsigned long DISPLAY_SLEEP_TIMEOUT = 120000; // 120 seconds

// Power management and motion-based sleep
unsigned long lastMotionTime = 0;
unsigned long lastBLEActivityTime = 0;
bool deviceInSleep = false;
const unsigned long MOTION_SLEEP_TIMEOUT = 120000; // 120 seconds of no motion
const float MOTION_THRESHOLD = 0.5f; // degrees/second for gyro motion detection
const float ACCEL_MOTION_THRESHOLD = 0.1f; // g-force for accelerometer motion detection
float lastAccelMagnitude = 0.0f;
float lastGyroMagnitude = 0.0f;
bool lowPowerMode = false;
int normalCpuFreq = 240; // MHz
int lowPowerCpuFreq = 80; // MHz

// IMU sampling frequency management
float targetIMUFrequency = 50.0f; // Normal: 50Hz (plenty for steering, saves power)
float lowPowerIMUFrequency = 10.0f; // Low power: 10Hz (minimal for motion detection)
float currentIMUFrequency = 50.0f;

// Store the last measured raw yaw and timestamp from recentering
float lastMeasuredRawYaw = 0.0f;
unsigned long lastYawMeasurementTime = 0;
bool hasValidYawMeasurement = false;

  // Removed complex continuous drift compensation - using simple centering instead

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

// —————— BLE Callbacks ——————
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect (BLEServer* s) override {
    deviceConnected = true;
    lastBLEActivityTime = millis();
    Serial.println("BLE client connected");
    
    // Exit low power mode when BLE connects
    if (lowPowerMode) {
      exitLowPowerMode();
    }
  }
  void onDisconnect (BLEServer* s) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
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

// Full calibration: both accelerometer and gyro + recenter yaw
void performFullCalibration() {
  isCalibrating = true;
  Serial.println("=== STARTING FULL CALIBRATION ===");
  
  // Step 1: Gyro calibration (device still)
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 10);
  M5.Display.print("GYRO CALIBRATION");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Keep device still");
  M5.Display.setCursor(0, 40);
  M5.Display.print("Starting in 5...");
  
  // 5 second countdown with instruction
  for (int countdown = 5; countdown > 0; countdown--) {
    M5.Display.setCursor(0, 55);
    M5.Display.printf("Starting in %d...", countdown);
    delay(1000);
  }
  
  // 3 beeps over 3 seconds before measurement
  M5.Display.setCursor(0, 55);
  M5.Display.print("3 beeps then measure...");
  for (int beep = 3; beep > 0; beep--) {
    M5.Speaker.tone(800, 200);
    M5.Display.setCursor(0, 70);
    M5.Display.printf("Beep %d...", beep);
    delay(1000);
  }
  
  // Reset biases
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  accelBiasX = accelBiasY = accelBiasZ = 0;
  
  // Collect 300 samples over 3 seconds for gyro
  const int gyroSamples = 3000;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  
  M5.Display.setCursor(0, 70);
  M5.Display.print("Measuring gyro...");
  
  for (int i = 0; i < gyroSamples; i++) {
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
    sumGx += gx; sumGy += gy; sumGz += gz;
    
    // Update display progress with progress bar
    if (i % 100 == 0) {
      int progressPercent = (i * 100) / gyroSamples;
      M5.Display.setCursor(0, 85);
      M5.Display.printf("Gyro: %d%%", progressPercent);
      
      // Simple progress bar
      M5.Display.setCursor(0, 100);
      String progressBar = "[";
      int barLength = progressPercent / 5; // 20 character bar
      for (int j = 0; j < 20; j++) {
        if (j < barLength) progressBar += "=";
        else progressBar += " ";
      }
      progressBar += "]";
      M5.Display.print(progressBar);
    }
    
    delay(1);
  }
  
  // Calculate gyro biases
  gyroBiasX = sumGx / gyroSamples;
  gyroBiasY = sumGy / gyroSamples;
  gyroBiasZ = sumGz / gyroSamples;
  
  Serial.printf("Gyro calibration complete - Gyro bias: %.3f,%.3f,%.3f\n", 
                gyroBiasX, gyroBiasY, gyroBiasZ);
  
  // 1 beep when gyro calibration complete
  M5.Speaker.tone(1000, 200);
  M5.Display.setCursor(0, 100);
  M5.Display.print("Gyro calibration done!");
  delay(1000);
  
  // Step 2: Accelerometer calibration (6-point calibration)
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 10);
  M5.Display.print("ACCEL CALIBRATION");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Place on each face");
  M5.Display.setCursor(0, 40);
  M5.Display.print("Hold 3s each");
  M5.Display.setCursor(0, 55);
  M5.Display.print("Starting in 5...");
  
  // 5 second countdown before starting
  for (int countdown = 5; countdown > 0; countdown--) {
    M5.Display.setCursor(0, 70);
    M5.Display.printf("Starting in %d...", countdown);
    delay(1000);
  }
  
  // 6-point calibration: +X, -X, +Y, -Y, +Z, -Z
  const char* positions[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  const char* instructions[] = {
    "Screen UP",      // +X
    "Screen DOWN",    // -X  
    "USB port UP",    // +Y
    "USB port DOWN",  // -Y
    "Button side UP", // +Z
    "Button side DOWN" // -Z
  };
  float accelReadings[6][3]; // [position][x,y,z]
  
  for (int pos = 0; pos < 6; pos++) {
    M5.Display.clear();
    M5.Display.setRotation(1);
    M5.Display.setCursor(0, 10);
    M5.Display.print("ACCEL CALIBRATION");
    M5.Display.setCursor(0, 25);
    M5.Display.printf("Position %d/6: %s", pos + 1, positions[pos]);
    M5.Display.setCursor(0, 40);
    M5.Display.print(instructions[pos]);
    M5.Display.setCursor(0, 55);
    M5.Display.print("Starting in 5...");
    
    // 5 second countdown with instruction
    for (int countdown = 5; countdown > 0; countdown--) {
      M5.Display.setCursor(0, 70);
      M5.Display.printf("Starting in %d...", countdown);
      delay(1000);
    }
    
    // 3 beeps over 3 seconds before measurement
    M5.Display.setCursor(0, 70);
    M5.Display.print("3 beeps then measure...");
    for (int beep = 3; beep > 0; beep--) {
      M5.Speaker.tone(1200, 200);
      M5.Display.setCursor(0, 85);
      M5.Display.printf("Beep %d...", beep);
      delay(1000);
    }
    
    // Collect samples for this position
    const int accelSamples = 300; // 3 seconds at 100Hz
    float sumAx = 0, sumAy = 0, sumAz = 0;
    
    M5.Display.setCursor(0, 85);
    M5.Display.print("Measuring...");
    
    for (int i = 0; i < accelSamples; i++) {
      float gx, gy, gz, ax, ay, az;
      M5.Imu.getGyro(&gx, &gy, &gz);
      M5.Imu.getAccel(&ax, &ay, &az);
      
      sumAx += ax; sumAy += ay; sumAz += az;
      
      // Update display progress every 50 samples (0.5 seconds)
      if (i % 50 == 0) {
        M5.Display.setCursor(0, 100);
        M5.Display.printf("Progress: %d%%", (i * 100) / accelSamples);
      }
      
      delay(10);
    }
    
    // Store average reading for this position
    accelReadings[pos][0] = sumAx / accelSamples;
    accelReadings[pos][1] = sumAy / accelSamples;
    accelReadings[pos][2] = sumAz / accelSamples;
    
    Serial.printf("Position %s (%s): %.3f,%.3f,%.3f\n", 
                  positions[pos], instructions[pos], accelReadings[pos][0], accelReadings[pos][1], accelReadings[pos][2]);
    
    // 1 beep when position complete
    M5.Speaker.tone(1000, 200);
    M5.Display.setCursor(0, 115);
    M5.Display.print("Position complete!");
    delay(1000);
  }
  
  // Calculate accelerometer biases from 6-point calibration
  // For a properly calibrated accelerometer, each axis should read ±1g when facing that direction
  accelBiasX = (accelReadings[0][0] + accelReadings[1][0]) / 2.0f; // Average of +X and -X
  accelBiasY = (accelReadings[2][1] + accelReadings[3][1]) / 2.0f; // Average of +Y and -Y
  accelBiasZ = (accelReadings[4][2] + accelReadings[5][2]) / 2.0f; // Average of +Z and -Z
  
  // Calculate scale factors (should be close to 1.0)
  float scaleX = (accelReadings[0][0] - accelReadings[1][0]) / 2.0f;
  float scaleY = (accelReadings[2][1] - accelReadings[3][1]) / 2.0f;
  float scaleZ = (accelReadings[4][2] - accelReadings[5][2]) / 2.0f;
  
  Serial.printf("Accel calibration complete - Bias: %.3f,%.3f,%.3f Scale: %.3f,%.3f,%.3f\n", 
                accelBiasX, accelBiasY, accelBiasZ, scaleX, scaleY, scaleZ);
  
  // Save biases
  prefs.putFloat("gxb", gyroBiasX);
  prefs.putFloat("gyb", gyroBiasY);
  prefs.putFloat("gzb", gyroBiasZ);
  prefs.putFloat("axb", accelBiasX);
  prefs.putFloat("ayb", accelBiasY);
  prefs.putFloat("azb", accelBiasZ);
  prefs.putFloat("drift", yawDriftRate);
  
  // Recenter yaw
  recenterYaw();
  
  // Final completion beep sequence
  M5.Speaker.tone(800, 200);
  delay(200);
  M5.Speaker.tone(1000, 200);
  delay(200);
  M5.Speaker.tone(1200, 400);
  
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 25);
  M5.Display.print("FULL CAL DONE");
  M5.Display.setCursor(0, 40);
  M5.Display.print("Yaw recentered");
  Serial.println("=== FULL CALIBRATION COMPLETE ===");
  delay(2000);
  
  isCalibrating = false;
}

// Gyro-only calibration + recenter yaw
void performGyroCalibration() {
  isCalibrating = true;
  Serial.println("=== STARTING GYRO CALIBRATION ===");
  
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 10);
  M5.Display.print("GYRO CALIBRATION");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Keep device still");
  M5.Display.setCursor(0, 40);
  M5.Display.print("Starting in 5...");
  
  // 5 second countdown with instruction
  for (int countdown = 5; countdown > 0; countdown--) {
    M5.Display.setCursor(0, 55);
    M5.Display.printf("Starting in %d...", countdown);
    delay(1000);
  }
  
  // 3 beeps over 3 seconds before measurement
  M5.Display.setCursor(0, 55);
  M5.Display.print("3 beeps then measure...");
  for (int beep = 3; beep > 0; beep--) {
    M5.Speaker.tone(800, 200);
    M5.Display.setCursor(0, 70);
    M5.Display.printf("Beep %d...", beep);
    delay(1000);
  }
  
  // Reset gyro biases only
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  
  // Collect 200 samples over 2 seconds
  const int samples = 200;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  
  M5.Display.setCursor(0, 70);
  M5.Display.print("Measuring gyro...");
  
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
    // Apply filtering during calibration
    filterGyroReadings(gx, gy, gz);
    
    sumGx += filteredGx; sumGy += filteredGy; sumGz += filteredGz;
    
    // Update display progress
    if (i % 20 == 0) {
      M5.Display.setCursor(0, 85);
      M5.Display.printf("Progress: %d%%", (i * 100) / samples);
    }
    
    delay(10);
  }
  
  // Calculate gyro biases
  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;
  gyroBiasZ = sumGz / samples;
  
  Serial.printf("Gyro calibration complete - Gyro bias: %.3f,%.3f,%.3f\n", 
                gyroBiasX, gyroBiasY, gyroBiasZ);
  
  // 1 beep when gyro calibration complete
  M5.Speaker.tone(1000, 200);
  M5.Display.setCursor(0, 100);
  M5.Display.print("Gyro calibration done!");
  delay(1000);
  
  // Save gyro biases
  prefs.putFloat("gxb", gyroBiasX);
  prefs.putFloat("gyb", gyroBiasY);
  prefs.putFloat("gzb", gyroBiasZ);
  
  // Recenter yaw
  recenterYaw();
  
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 25);
  M5.Display.print("GYRO CAL DONE");
  M5.Display.setCursor(0, 40);
  M5.Display.print("Yaw recentered");
  Serial.println("=== GYRO CALIBRATION COMPLETE ===");
  delay(1000);
  
  isCalibrating = false;
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
  
  // Collect 10 seconds of yaw readings to get a stable average and detect drift
  const int samples = 200; // 200 samples over 10 seconds = 20Hz sampling rate
  float yawSum = 0;
  int validSamples = 0;
  float minYaw = 999.0f;
  float maxYaw = -999.0f;
  float firstYaw = 0.0f;
  float lastYaw = 0.0f;
  
  Serial.println("Collecting 10 seconds of yaw samples...");
  
  for (int i = 0; i < samples; i++) {
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
      20.0f // Use 20Hz for sampling
    );
    
    // Get current yaw
    float currentYaw = getYaw();
    
    if (!isnan(currentYaw) && !isinf(currentYaw)) {
      yawSum += currentYaw;
      validSamples++;
      
      // Track min/max for drift detection
      if (currentYaw < minYaw) minYaw = currentYaw;
      if (currentYaw > maxYaw) maxYaw = currentYaw;
      
      // Store first and last values
      if (validSamples == 1) firstYaw = currentYaw;
      lastYaw = currentYaw;
      
      // Print progress every 20 samples (every second)
      if (i % 20 == 0) {
        Serial.printf("Sample %d/600: %.1f° (%.1fs elapsed)\n", i, currentYaw, i * 0.05f);
      }
    }
    
    delay(50); // 50ms between samples = 20Hz
  }
  
  if (validSamples > 0) {
    float averageYaw = yawSum / validSamples;
    float yawRange = maxYaw - minYaw;
    float yawDrift = lastYaw - firstYaw;
    float driftRatePerSecond = yawDrift / 10.0f; // 10 seconds of sampling
    
    Serial.printf("Recentering - Average yaw from %d samples: %.1f°\n", validSamples, averageYaw);
    Serial.printf("Yaw range: %.1f° (min: %.1f°, max: %.1f°)\n", yawRange, minYaw, maxYaw);
    Serial.printf("Yaw drift over 30s: %.1f° (%.3f°/s)\n", yawDrift, driftRatePerSecond);
    
    // Set the offset as the negative of the current raw yaw
    // This way: rel = raw + offset = raw + (-raw) = 0 when recentered
    yawOffset = -averageYaw;
    
    // Normalize offset to [-180, 180] range
    if (yawOffset > 180) yawOffset -= 360;
    else if (yawOffset < -180) yawOffset += 360;
    
    // Update drift compensation rate based on measured drift
    if (abs(driftRatePerSecond) > 0.01f) { // Only update if significant drift detected
      // Set drift compensation rate to counteract the measured drift
      yawDriftRate = abs(driftRatePerSecond) * 1.5f; // 1.5x the measured drift for safety margin
      Serial.printf("Updating drift compensation rate to %.3f°/s (measured drift: %.3f°/s)\n", 
                    yawDriftRate, driftRatePerSecond);
    } else {
      // No significant drift detected, use default rate
      yawDriftRate = 0.5f; // Default gentle centering
      Serial.printf("No significant drift detected, using default rate: %.1f°/s\n", yawDriftRate);
    }
    
    prefs.putFloat("yoff", yawOffset);
    prefs.putFloat("drift", yawDriftRate);
    Serial.printf("Yaw recentered - Average: %.1f° Offset: %.1f° DriftRate: %.3f°/s\n", 
                  averageYaw, yawOffset, yawDriftRate);
    
    // Store the last measured raw yaw and timestamp for drift compensation
    lastMeasuredRawYaw = averageYaw;
    lastYawMeasurementTime = millis();
    hasValidYawMeasurement = true;
    Serial.printf("Stored measured raw yaw: %.1f° at time %lu\n", lastMeasuredRawYaw, lastYawMeasurementTime);
    
    Serial.println("Yaw recentering completed");
    
    // Warn if significant drift was detected
    if (abs(yawDrift) > 1.0f) {
      Serial.printf("WARNING: Significant yaw drift detected (%.1f° over 30s)\n", yawDrift);
    }
  } else {
    yawOffset = 0.0f; // Fallback to 0 if no valid samples
    yawDriftRate = 0.5f; // Default drift rate
    prefs.putFloat("yoff", yawOffset);
    prefs.putFloat("drift", yawDriftRate);
    Serial.println("WARNING: No valid yaw samples, using defaults (offset=0, drift=0.5°/s)");
  }
  
  // Reset drift compensation when recentering
  resetYawDrift();
  
  // Force a display update to show the new yaw
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 25);
  M5.Display.print("Yaw Recentered");
  M5.Display.setCursor(0, 40);
  M5.Display.printf("Offset: %.1f°", yawOffset);
  delay(1000);
}

// Apply yaw drift compensation to gradually return to center
float applyYawDriftCompensation(float currentYaw) {
  if (!yawDriftEnabled) {
    return currentYaw;
  }
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastYawTime) / 1000.0f; // Convert to seconds
  
  if (deltaTime > 0) {
    // Calculate how much drift to apply (0.5 degrees per second)
    float driftAmount = yawDriftRate * deltaTime;
    float originalYaw = currentYaw;
    
    // Debug drift calculation every 2 seconds
    static unsigned long lastDriftDebugTime = 0;
    if (millis() - lastDriftDebugTime > 2000) {
      Serial.printf("DRIFT DEBUG - Rate: %.2f°/s, DeltaTime: %.3fs, Amount: %.3f°, Before: %.1f°", 
                    yawDriftRate, deltaTime, driftAmount, originalYaw);
      lastDriftDebugTime = millis();
    }
    
    // Apply gentle centering pressure towards 0 degrees
    if (currentYaw > 0) {
      currentYaw -= driftAmount;
      if (currentYaw < 0) currentYaw = 0; // Don't overshoot
    } else if (currentYaw < 0) {
      currentYaw += driftAmount;
      if (currentYaw > 0) currentYaw = 0; // Don't overshoot
    }
    
    // Debug the after value
    static unsigned long lastAfterDebugTime = 0;
    if (millis() - lastAfterDebugTime > 2000) {
      Serial.printf(", After: %.1f°, Change: %.3f°\n", currentYaw, originalYaw - currentYaw);
      lastAfterDebugTime = millis();
    }
    
    accumulatedYawDrift += driftAmount;
  }
  
  // Always update the time, even if deltaTime was 0
  lastYawTime = currentTime;
  
  return currentYaw;
}

// Reset drift compensation (called when recentering)
void resetYawDrift() {
  accumulatedYawDrift = 0.0f;
  lastYawTime = millis();
  
  // Reset measurement tracking
  hasValidYawMeasurement = false;
  lastMeasuredRawYaw = 0.0f;
  lastYawMeasurementTime = 0;
  
  Serial.println("Yaw drift compensation reset");
}

// Removed complex continuous drift compensation function - using simple centering instead

// Apply low-pass filter to gyro readings
void filterGyroReadings(float gx, float gy, float gz) {
  filteredGx = gyroFilterAlpha * gx + (1.0f - gyroFilterAlpha) * filteredGx;
  filteredGy = gyroFilterAlpha * gy + (1.0f - gyroFilterAlpha) * filteredGy;
  filteredGz = gyroFilterAlpha * gz + (1.0f - gyroFilterAlpha) * filteredGz;
}

// Check if calibration data exists
bool calibrationExists() {
  return prefs.getFloat("gxb", NAN) != NAN && 
         prefs.getFloat("gyb", NAN) != NAN && 
         prefs.getFloat("gzb", NAN) != NAN &&
         prefs.getFloat("axb", NAN) != NAN && 
         prefs.getFloat("ayb", NAN) != NAN && 
         prefs.getFloat("azb", NAN) != NAN;
}

// Auto-calibration on first boot
void performAutoCalibration() {
  Serial.println("=== AUTO CALIBRATION ===");
  
  M5.Display.clear();
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 10);
  M5.Display.print("AUTO CALIBRATION");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Keep device still");
  M5.Display.setCursor(0, 40);
  M5.Display.print("3 seconds...");
  
  // Simple auto-calibration: just gyro bias
  performGyroCalibration();
}

// Auto-recenter on boot
void performAutoRecenter() {
  Serial.println("=== AUTO RECENTER ===");
  recenterYaw();
}

// Initialize Mahony filter with current device orientation
void initializeMahonyFilter() {
  Serial.println("Initializing Mahony filter with current orientation...");
  
  // Reset quaternion to identity
  initMahonyData();
  
  // Collect IMU data for a few seconds to stabilize the filter
  const int initSamples = 200; // 2 seconds at 100Hz
  
  for (int i = 0; i < initSamples; i++) {
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
      100.0f // Use fixed 100Hz for initialization
    );
    
    delay(10);
  }
  
  // Get initial yaw reading
  float initialYaw = getYaw();
  Serial.printf("Mahony filter initialized - Initial yaw: %.1f°\n", initialYaw);
}

// Startup button test
void performStartupButtonTest() {
  Serial.println("=== STARTUP BUTTON TEST ===");
  M5.Display.clear(); 
  M5.Display.setRotation(1);
  M5.Display.setCursor(0, 10);
  M5.Display.print("BUTTON TEST");
  M5.Display.setCursor(0, 25);
  M5.Display.print("Press each button");
  M5.Display.setCursor(0, 40);
  M5.Display.print("to test...");
  
  unsigned long startTime = millis();
  bool btnATested = false, btnBTested = false;
  
  while (millis() - startTime < 10000) { // 10 second test
    M5.update();
    
    if (M5.BtnA.wasClicked() && !btnATested) {
      Serial.println("Button A test - PASSED");
      M5.Display.setCursor(0, 55);
      M5.Display.print("Button A: OK");
      M5.Display.setCursor(0, 70);
      M5.Display.print("(Wake/Recenter)");
      M5.Speaker.tone(800, 200);
      btnATested = true;
    }
    
    if (M5.BtnB.wasClicked() && !btnBTested) {
      Serial.println("Button B test - PASSED");
      M5.Display.setCursor(0, 85);
      M5.Display.print("Button B: OK");
      M5.Display.setCursor(0, 100);
      M5.Display.print("(Recenter/Cal)");
      M5.Speaker.tone(1000, 200);
      btnBTested = true;
    }
    
    delay(10);
  }
  
  Serial.printf("Button test complete - A:%s B:%s\n", 
                btnATested ? "PASS" : "FAIL", 
                btnBTested ? "PASS" : "FAIL");
  
  M5.Display.setCursor(0, 115);
  M5.Display.print("Test complete");
  delay(2000);
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
    100.0f
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
    
    // Reset completed    
    // Display confirmation
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(0, 25);
    M5.Display.print("Quick Recenter");
    M5.Display.setCursor(0, 40);
    M5.Display.printf("Offset: %.1f°", yawOffset);
    delay(1000);
    
  } else {
    Serial.println("WARNING: Invalid yaw value during quick recenter");
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(0, 25);
    M5.Display.print("Recentering");
    M5.Display.setCursor(0, 40);
    M5.Display.print("Failed");
    delay(1000);
  }
}

// —————— Power Management Functions ——————

bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az) {
  // Calculate gyro magnitude (degrees/second)
  float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);
  
  // Calculate accelerometer magnitude (g-force)
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  
  // Check for significant motion
  bool gyroMotion = abs(gyroMagnitude - lastGyroMagnitude) > MOTION_THRESHOLD;
  bool accelMotion = abs(accelMagnitude - lastAccelMagnitude) > ACCEL_MOTION_THRESHOLD;
  
  // Update last values
  lastGyroMagnitude = gyroMagnitude;
  lastAccelMagnitude = accelMagnitude;
  
  return gyroMotion || accelMotion;
}

void enterLowPowerMode() {
  if (!lowPowerMode) {
    Serial.println("Entering low power mode");
    setCpuFrequencyMhz(lowPowerCpuFreq);
    
    // Reduce LED brightness further
    ledcWrite(0, 8); // ~3% brightness
    
    // Reduce BLE advertising interval
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setMinInterval(800); // Increase interval to save power
    pAdvertising->setMaxInterval(1600);
    
    // Reduce IMU sampling frequency
    currentIMUFrequency = lowPowerIMUFrequency;
    
    lowPowerMode = true;
    Serial.printf("CPU frequency reduced to %d MHz, IMU frequency to %.1f Hz\n", lowPowerCpuFreq, currentIMUFrequency);
  }
}

void exitLowPowerMode() {
  if (lowPowerMode) {
    Serial.println("Exiting low power mode");
    setCpuFrequencyMhz(normalCpuFreq);
    
    // Restore LED brightness
    ledcWrite(0, 32); // Back to 12.5% brightness
    
    // Restore normal BLE advertising interval
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setMinInterval(100);
    pAdvertising->setMaxInterval(200);
    
    // Restore normal IMU sampling frequency
    currentIMUFrequency = targetIMUFrequency;
    
    lowPowerMode = false;
    Serial.printf("CPU frequency restored to %d MHz, IMU frequency to %.1f Hz\n", normalCpuFreq, currentIMUFrequency);
  }
}

void enterDeepSleep() {
  Serial.println("=== ENTERING SLEEP MODE ===");
  
  // Display sleep message
  M5.Display.clear();
  M5.Display.setRotation(0);
  M5.Display.setCursor(10, 25);
  M5.Display.print("SLEEP MODE");
  M5.Display.setCursor(10, 50);
  M5.Display.print("Press Button A");
  M5.Display.setCursor(10, 75);
  M5.Display.print("to wake up");
  M5.Display.setCursor(10, 100);
  M5.Display.print("Auto wake: 30s");
  delay(2000);
  
  // Turn off LED
  ledcWrite(0, 0);
  
  // Save current state
  prefs.begin("sterzo", false);
  prefs.putBool("wasAsleep", true);
  prefs.end();
  
  Serial.println("Going to timer sleep (30 seconds)...");
  
  // Use M5StickCPlus2 timer sleep - wakes up automatically after 30 seconds
  // This is more power efficient than deep sleep for short periods
  // and automatically handles wake-up without complex GPIO configuration
  M5.Power.timerSleep(30); // Sleep for 30 seconds, then auto-wake to check for motion
}

bool shouldEnterSleep() {
  unsigned long currentTime = millis();
  
  // Don't sleep if BLE is connected
  if (deviceConnected) {
    lastBLEActivityTime = currentTime;
    return false;
  }
  
  // Don't sleep if currently calibrating
  if (isCalibrating) {
    return false;
  }
  
  // Check if enough time has passed since last motion
  return (currentTime - lastMotionTime) > MOTION_SLEEP_TIMEOUT;
}

void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az) {
  unsigned long currentTime = millis();
  
  // Check for motion
  if (detectMotion(gx, gy, gz, ax, ay, az)) {
    lastMotionTime = currentTime;
    lastActivityTime = currentTime; // Also update display activity
    
    // Exit low power mode if we were in it
    if (lowPowerMode) {
      exitLowPowerMode();
    }
  }
  
  // Update BLE activity time if connected
  if (deviceConnected) {
    lastBLEActivityTime = currentTime;
  }
  
  // Enter low power mode if no motion for 30 seconds and not connected
  if (!deviceConnected && !lowPowerMode && 
      (currentTime - lastMotionTime) > 30000) {
    enterLowPowerMode();
  }
  
  // Check if we should enter deep sleep
  if (shouldEnterSleep()) {
    enterDeepSleep();
  }
}

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
  
  // Enable verbose logging for M5Unified
  M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_VERBOSE);
  
  M5.Display.setRotation(0);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);  // Use smaller font
  M5.Display.setTextSize(2);
  
  Serial.println("Display configured");
  
  // Setup LED with PWM for 25% brightness
  pinMode(LED_PIN, OUTPUT);
  
  // Configure PWM for LED brightness control
  // ESP32 PWM: 8-bit resolution (0-255), 5000Hz frequency
  ledcSetup(0, 5000, 8);           // PWM channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);       // Attach LED pin to PWM channel 0
  ledcWrite(0, 32);                // Set to 25% brightness (64/255 = 25%)
  
  Serial.println("LED pin configured at 25% brightness");

  // Initialize button GPIO pins
  pinMode(37, INPUT_PULLUP); // Button A
  pinMode(39, INPUT_PULLUP); // Button B  
  pinMode(35, INPUT_PULLUP); // Button C (Power)
  Serial.println("Button GPIO pins configured (37, 39, 35)");

  // Initialize Mahony AHRS
  initMahonyData();
  Serial.println("Mahony AHRS initialized");

  // load stored biases + yawOffset
  Serial.println("Initializing preferences...");
  prefs.begin("sterzo", false);
  
  // Check if calibration exists
  if (!calibrationExists()) {
    // First boot - no calibration exists
    Serial.println("No calibration found - performing auto-calibration");
    performAutoCalibration();
  } else {
    // Calibration exists - load it and auto-recenter
    Serial.println("Calibration found - loading and auto-recentering");
    gyroBiasX  = prefs.getFloat("gxb",0);
    gyroBiasY  = prefs.getFloat("gyb",0);
    gyroBiasZ  = prefs.getFloat("gzb",0);
    accelBiasX = prefs.getFloat("axb",0);
    accelBiasY = prefs.getFloat("ayb",0);
    accelBiasZ = prefs.getFloat("azb",0);
    yawOffset  = prefs.getFloat("yoff",0);
    yawDriftRate = prefs.getFloat("drift", 0.5f);
    
    // Force higher drift rate for better centering
    yawDriftRate = 0.5f; // 0.5 degrees per second for more noticeable centering
    Serial.printf("Forced drift rate to %.2f°/s for better centering\n", yawDriftRate);
    
    Serial.printf("Loaded biases - Gyro: %.3f,%.3f,%.3f Accel: %.3f,%.3f,%.3f YawOffset: %.3f DriftRate: %.2f\n", 
                  gyroBiasX, gyroBiasY, gyroBiasZ, accelBiasX, accelBiasY, accelBiasZ, yawOffset, yawDriftRate);
    
    // Initialize Mahony filter with current orientation
    initializeMahonyFilter();
    
    // Auto-recenter on boot
    performAutoRecenter();
  }

  // BLE init
  Serial.println("Initializing BLE...");
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
  BLEDevice::startAdvertising();
  
  Serial.println("BLE advertising started");

  // Initialize power management
  lastMotionTime = millis();
  lastBLEActivityTime = millis();
  lastActivityTime = millis();
  
  // Check if we woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.printf("Woke up from deep sleep, reason: %d\n", wakeup_reason);
    
    // Clear the sleep flag
    prefs.begin("sterzo", false);
    prefs.putBool("wasAsleep", false);
    prefs.end();
  }
  
  Serial.println("Power management initialized");

  // Display startup message
  M5.Display.clear();
  M5.Display.setRotation(0);  
  M5.Display.setCursor(10, 20);
  M5.Display.print("STERZO");
  M5.Display.setCursor(10, 50);
  M5.Display.print("Ready!");
  
  Serial.println("=== STERZO READY ===");
}

// —————— loop() ——————
void loop() {
  M5.update(); // Update button states

  // Calculate dynamic sample frequency with proper limiting
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = micros();
  dt = (currentTime - lastUpdateTime) / 1000000.0f; // Calculate time delta in seconds
  
  // Adaptive frequency limiting based on power mode
  float minInterval = 1.0f / currentIMUFrequency; // Calculate minimum interval for current frequency
  if (dt < minInterval) {
    // Calculate delay needed to maintain target frequency
    int delayMicros = (int)((minInterval - dt) * 1000000.0f);
    if (delayMicros > 0 && delayMicros < 50000) { // Cap delay at 50ms
      delayMicroseconds(delayMicros);
    }
    M5.update(); // Update buttons even during delay
    return;
  }
  
  lastUpdateTime = currentTime;
  loopfreq = 1.0f / dt; // Calculate loop frequency
  
  // Cap frequency at reasonable limits for debugging
  if (loopfreq > currentIMUFrequency * 1.5f) loopfreq = currentIMUFrequency * 1.5f;

  // 1) Read raw IMU, apply bias, fuse
  float gx, gy, gz, ax, ay, az;
  M5.Imu.getGyro(&gx,&gy,&gz);
  M5.Imu.getAccel(&ax,&ay,&az);

  // Apply low-pass filter to gyro readings
  filterGyroReadings(gx, gy, gz);

  // Update power management with current IMU readings
  updatePowerManagement(gx, gy, gz, ax, ay, az);

  // Debug IMU readings every 2 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 2000) {
    Serial.printf("IMU - Gyro: %.2f,%.2f,%.2f Accel: %.2f,%.2f,%.2f Freq: %.1f/%.1fHz PowerMode: %s\n", 
                  gx, gy, gz, ax, ay, az, loopfreq, currentIMUFrequency, 
                  lowPowerMode ? "LOW" : "NORMAL");
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
    currentIMUFrequency  // Use target frequency for consistent filter behavior
  );

  // Get yaw in degrees from Mahony AHRS (this is the "raw" value with bias compensation)
  float rawYaw = getYaw();  // Already returns degrees in [-180, 180] range
  
  // Check for valid yaw value
  if (isnan(rawYaw) || isinf(rawYaw)) {
    Serial.println("WARNING: Invalid yaw value detected!");
    rawYaw = 0.0f; // Use 0 as fallback
  }
  
  // Apply yaw drift compensation (this was causing the bug - commenting out)
  // rawYaw = applyYawDriftCompensation(rawYaw);
  
  // Calculate expected raw yaw for drift tracking (but don't override actual rawYaw)
  float expectedRawYaw = rawYaw;
  if (hasValidYawMeasurement) {
    float timeSinceMeasurement = (millis() - lastYawMeasurementTime) / 1000.0f;
    float expectedDrift = yawDriftRate * timeSinceMeasurement;
    expectedRawYaw = lastMeasuredRawYaw + expectedDrift;
    
    // Debug drift calculation every 2 seconds
    static unsigned long lastDriftCalcDebugTime = 0;
    if (millis() - lastDriftCalcDebugTime > 2000) {
      Serial.printf("DRIFT CALC - LastMeasured: %.1f°, TimeSince: %.1fs, ExpectedDrift: %.3f°, ExpectedRaw: %.1f°, ActualRaw: %.1f°\n", 
                    lastMeasuredRawYaw, timeSinceMeasurement, expectedDrift, expectedRawYaw, rawYaw);
      lastDriftCalcDebugTime = millis();
    }
  }
  
  // Use actual raw yaw (not expected) - this fixes the bug!
  // rawYaw = expectedRawYaw;  // REMOVED - this was causing yaw to stop updating
  
  // Calculate relative yaw: raw + offset (offset is negative of the learned center)
  float rel = rawYaw + yawOffset;
  
  // Normalize relative yaw to [-180, 180] range
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Simple centering force - gradually adjust offset to bring rel back to 0
  static unsigned long lastCenteringTime = millis();
  unsigned long centeringTime = millis();
  float deltaTime = (centeringTime - lastCenteringTime) / 1000.0f;
  
  if (deltaTime >= 0.1f) { // Update every 100ms
    // Apply centering force: if rel is positive, decrease offset; if negative, increase offset
    float centeringRate = 0.2f; // degrees per second centering force
    float centeringAdjustment = -rel * centeringRate * deltaTime;
    
    // Limit the adjustment to prevent overcorrection
    centeringAdjustment = constrain(centeringAdjustment, -0.1f, 0.1f);
    
    yawOffset += centeringAdjustment;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    // Debug centering every 3 seconds
    static unsigned long lastCenteringDebugTime = 0;
    if (millis() - lastCenteringDebugTime > 3000) {
      Serial.printf("CENTERING - RelYaw: %.2f°, CenteringAdj: %.3f°, NewOffset: %.2f°\n", 
                    rel, centeringAdjustment, yawOffset);
      lastCenteringDebugTime = millis();
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
  
  // Fix negative zero issue - if bin is very close to 0, make it positive 0
  if (abs(bin) < 0.1f) {
    bin = 0.0f; // Ensure it's positive zero for values very close to 0
  }

  // Debug yaw calculations every 2 seconds
  static unsigned long lastYawDebugTime = 0;
  if (millis() - lastYawDebugTime > 2000) {
    Serial.printf("Yaw calc - Raw: %.1f° Offset: %.1f° Rel: %.1f° Bin: %.0f° Drift: %.1f° Connected: %s Ind32: %s\n", 
                  rawYaw, yawOffset, rel, bin, accumulatedYawDrift, deviceConnected ? "YES" : "NO", ind32On ? "YES" : "NO");
    lastYawDebugTime = millis();
  }

  // notify on true bin‐change
  if (deviceConnected && ind32On && !isnan(lastSentBin) && bin!=lastSentBin) {
    sendSteeringBin(bin);
  }
  lastSentBin = bin;

  // Update display with current status
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 500) {
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(10, 10);
    M5.Display.print("STERZO");
    
    // Display yaw with limit indicators
    M5.Display.setCursor(10, 35);
    
    // Check if we're at the limits before clamping
    float unclamped_rel = rawYaw + yawOffset;
    while (unclamped_rel > 180) unclamped_rel -= 360;
    while (unclamped_rel < -180) unclamped_rel += 360;
    
    // Create flashing effect for limit indicators (flash every 500ms)
    bool flashOn = (millis() / 500) % 2 == 0;
    
    if (unclamped_rel >= 40.0f) {
      // At or beyond positive limit
      M5.Display.printf("Yaw:%.1f%s", rel, flashOn ? ">" : " ");
    } else if (unclamped_rel <= -40.0f) {
      // At or beyond negative limit  
      M5.Display.printf("Yaw:%.1f%s", rel, flashOn ? "<" : " ");
    } else {
      // Within normal range
      M5.Display.printf("Yaw:%.1f", rel);
    }
    
    M5.Display.setCursor(10, 60);
    M5.Display.printf("Bin:%.0f", bin);
    M5.Display.setCursor(10, 85);
    M5.Display.printf("BLE:%s", deviceConnected ? "ON" : "ADV");
    
    // Show power management status
    M5.Display.setCursor(10, 110);
    if (displaySleeping) {
      M5.Display.print("DISP SLEEP");
    } else if (lowPowerMode) {
      M5.Display.print("LOW POWER");
    } else {
      // Show motion sleep countdown when approaching sleep
      unsigned long timeUntilMotionSleep = (MOTION_SLEEP_TIMEOUT - (millis() - lastMotionTime)) / 1000;
      if (!deviceConnected && timeUntilMotionSleep < 60) {
        M5.Display.printf("Sleep:%ds", timeUntilMotionSleep);
      } else {
        // Show display sleep countdown
        unsigned long timeUntilDisplaySleep = (DISPLAY_SLEEP_TIMEOUT - (millis() - lastActivityTime)) / 1000;
        if (timeUntilDisplaySleep < 60) {
          M5.Display.printf("Disp:%ds", timeUntilDisplaySleep);
        }
      }
    }
    
    // Bottom row: Refresh rate (left) and Battery voltage (right)
    M5.Display.setCursor(10, 224);
    M5.Display.printf("%dHz", (int)loopfreq);  // Refresh rate as integer Hz on bottom left
    
    // Get battery voltage and display on bottom right
    float batteryVoltage = M5.Power.getBatteryVoltage() / 1000.0f; // Convert mV to V
    M5.Display.setCursor(70, 224);  // Position on right side (adjust based on display width)
    M5.Display.printf("%.2fV", batteryVoltage);  // Battery voltage on bottom right
    
    lastDisplayUpdate = millis();
  }

  // —————— Direct GPIO Button Handling ——————
  static unsigned long buttonAStartTime = 0;
  static unsigned long buttonBStartTime = 0;
  static unsigned long buttonCStartTime = 0;
  static bool buttonAWasPressed = false;
  static bool buttonBWasPressed = false;
  static bool buttonCWasPressed = false;
  
  // Button press counters for debugging
  static int btnAPressCount = 0;
  static int btnBPressCount = 0;
  static int btnCPressCount = 0;
  
  // Button GPIO pins (from M5StickCPlus2 Board Notes)
  const int BUTTON_A_PIN = 37;
  const int BUTTON_B_PIN = 39;
  const int BUTTON_C_PIN = 35;
  
  // Read button states directly from GPIO
  bool buttonAState = !digitalRead(BUTTON_A_PIN); // Inverted logic
  bool buttonBState = !digitalRead(BUTTON_B_PIN); // Inverted logic  
  bool buttonCState = !digitalRead(BUTTON_C_PIN); // Inverted logic
  
  // Debug button states every 5 seconds
  static unsigned long lastButtonDebugTime = 0;
  if (millis() - lastButtonDebugTime > 5000) {
    Serial.printf("GPIO Button states - A(GPIO37): %s B(GPIO39): %s C(GPIO35): %s\n", 
                  buttonAState ? "PRESSED" : "released",
                  buttonBState ? "PRESSED" : "released", 
                  buttonCState ? "PRESSED" : "released");
    Serial.printf("Button press counts - A:%d B:%d C:%d\n", 
                  btnAPressCount, btnBPressCount, btnCPressCount);
    lastButtonDebugTime = millis();
  }
  
  // Button A (front face) - Single click: wake display, Long press (>2s): recenter yaw
  if (buttonAState && !buttonAWasPressed) {
    // Button A just pressed
    buttonAStartTime = millis();
    buttonAWasPressed = true;
    Serial.println("*** Button A PRESSED - Starting timer ***");
  } else if (!buttonAState && buttonAWasPressed) {
    // Button A just released
    unsigned long pressDuration = millis() - buttonAStartTime;
    buttonAWasPressed = false;
    
    Serial.printf("*** Button A RELEASED after %dms ***\n", pressDuration);
    
    // Short press (< 500ms): wake display
    if (pressDuration < 500) {
      btnAPressCount++;
      Serial.printf("*** Button A SHORT PRESS (count: %d) - Quick recenter yaw ***\n", btnAPressCount);
      M5.Speaker.tone(800, 100);
      
      // Quick recenter yaw immediately
      quickRecenterYaw();
    }
    // Long press (2s+): full calibration
    else if (pressDuration >= 2000) {
      Serial.println("*** Button A LONG PRESS - Full yaw calibration ***");
      if (!isCalibrating) {
        M5.Display.clear(); 
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Stabilizing");
        M5.Display.setCursor(0, 40);
        M5.Display.print("2 seconds...");
      }
      
      // Wait for device to stabilize before calibration
      delay(2000);
      
      if (!isCalibrating) {
        M5.Display.clear();
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Full Calibration");
        M5.Display.setCursor(0, 40);
        M5.Display.print("10 seconds...");
      }
      recenterYaw();
      if (!isCalibrating) {
        M5.Display.clear();
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Calibration");
        M5.Display.setCursor(0, 40);
        M5.Display.print("Complete");
        delay(800);
      }
    }
  }
  
  // Button B (top/north face) - Single click: recenter yaw, Long press (>3s): full calibration
  if (buttonBState && !buttonBWasPressed) {
    // Button B just pressed
    buttonBStartTime = millis();
    buttonBWasPressed = true;
    Serial.println("*** Button B PRESSED - Starting timer ***");
  } else if (!buttonBState && buttonBWasPressed) {
    // Button B just released
    unsigned long pressDuration = millis() - buttonBStartTime;
    buttonBWasPressed = false;
    
    Serial.printf("*** Button B RELEASED after %dms ***\n", pressDuration);
    
    // Short press (< 500ms): quick recenter yaw
    if (pressDuration < 500) {
      btnBPressCount++;
      Serial.printf("*** Button B SHORT PRESS (count: %d) - Quick recenter yaw ***\n", btnBPressCount);
      M5.Speaker.tone(1000, 100);
      
      // Quick recenter yaw immediately
      quickRecenterYaw();
    }
    // Long press (3s+): full calibration
    else if (pressDuration >= 3000) {
      Serial.println("*** Button B LONG PRESS - Full yaw calibration ***");
      if (!isCalibrating) {
        M5.Display.clear();
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Stabilizing");
        M5.Display.setCursor(0, 40);
        M5.Display.print("2 seconds...");
      }
      
      // Wait for device to stabilize before calibration
      delay(2000);
      
      if (!isCalibrating) {
        M5.Display.clear();
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Full Calibration");
        M5.Display.setCursor(0, 40);
        M5.Display.print("10 seconds...");
      }
      recenterYaw();
      if (!isCalibrating) {
        M5.Display.clear();
        M5.Display.setRotation(1);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Calibration");
        M5.Display.setCursor(0, 40);
        M5.Display.print("Complete");
        delay(800);
      }
    }
  }
  
  // Button C (Power button) - Power management
  if (buttonCState && !buttonCWasPressed) {
    // Button C just pressed
    buttonCStartTime = millis();
    buttonCWasPressed = true;
    Serial.println("*** Button C PRESSED - Starting timer ***");
  } else if (!buttonCState && buttonCWasPressed) {
    // Button C just released
    unsigned long pressDuration = millis() - buttonCStartTime;
    buttonCWasPressed = false;
    
    Serial.printf("*** Button C RELEASED after %dms ***\n", pressDuration);
    
    // Short press (< 500ms): wake display
    if (pressDuration < 500) {
      btnCPressCount++;
      Serial.printf("*** Button C SHORT PRESS (count: %d) ***\n", btnCPressCount);
      M5.Speaker.tone(1200, 100);
      
      // Wake display if sleeping
      if (displaySleeping) {
        M5.Display.wakeup();
        displaySleeping = false;
        lastActivityTime = millis();
        Serial.println("Display woken by Button C");
      }
    }
    // Long press (3s+): power off or sleep (reduced from 6s)
    else if (pressDuration >= 3000) {
      Serial.println("*** Button C LONG PRESS - Power management ***");
      
      M5.Display.clear();
      M5.Display.setRotation(0);
      M5.Display.setCursor(0, 25);
      M5.Display.print("Powering");
      M5.Display.setCursor(0, 40);
      M5.Display.print("Off...");
      delay(1000);
      
      // Always try to power off first
      pinMode(4, OUTPUT);
      digitalWrite(4, LOW);
      Serial.println("HOLD pin set LOW for power off");
      delay(2000);
      
      // If we get here, power off failed, so enter deep sleep
      Serial.println("Power off failed, entering deep sleep");
      M5.Display.clear();
      M5.Display.setRotation(0);
      M5.Display.setCursor(0, 25);
      M5.Display.print("Entering");
      M5.Display.setCursor(0, 40);
      M5.Display.print("Deep Sleep");
      delay(1000);
      
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // Wake on button C press
      esp_deep_sleep_start();
    }
  }
  
  // Update activity timer on any movement or button press
  if (abs(rel) > 1.0f || buttonAWasPressed || buttonBWasPressed || buttonCWasPressed) {
    lastActivityTime = millis();
  }
  
  // Display sleep management
  if (millis() - lastActivityTime > DISPLAY_SLEEP_TIMEOUT && !displaySleeping) {
    displaySleeping = true;
    M5.Display.sleep();
    Serial.println("Display going to sleep");
  } else if (millis() - lastActivityTime <= DISPLAY_SLEEP_TIMEOUT && displaySleeping) {
    displaySleeping = false;
    M5.Display.wakeup();
    Serial.println("Display woken");
  }
} 