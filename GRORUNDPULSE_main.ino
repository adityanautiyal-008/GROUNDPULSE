
/*
 * ══════════════════════════════════════════════════════════════════════════════
 * GROUND PULSE v3.0 — Multi-Sensor Life Detection System
 * ESP32 Firmware for Real-Time Seismic/Acoustic Detection
 * ══════════════════════════════════════════════════════════════════════════════
 * 
 * HARDWARE CONFIGURATION:
 * ─────────────────────────────────────────────────────────────────────────────
 * Controller:    ESP32 DevKit
 * ADC:           ADS1115 (16-bit) via I2C
 * IMU:           MPU6050 via I2C (shared bus)
 * Power:         2S Li-ion (7.4V nom) → Buck converter → 5V to ESP32 Vin
 * 
 * I2C BUS (GPIO21=SDA, GPIO22=SCL):
 *   - ADS1115 Address: 0x48 (default)
 *   - MPU6050 Address: 0x68 (default)
 * 
 * ADS1115 CHANNEL MAPPING (FIXED - DO NOT CHANGE):
 *   A0 → Piezo Channel 1 (via LM358 ×11 gain) — Also used for rawADC
 *   A1 → Piezo Channel 2 (via LM358 ×11 gain)
 *   A2 → UNUSED
 *   A3 → KY-038 Microphone (via 0.1µF AC coupling capacitor)
 * 
 * GPIO MAPPING:
 *   GPIO21 → I2C SDA
 *   GPIO22 → I2C SCL
 *   GPIO25 → Alert LED (active HIGH)
 *   GPIO26 → Buzzer (active HIGH)
 *   GPIO34 → Battery voltage monitor (via 2:1 resistor divider)
 * 
 * JSON OUTPUT SCHEMA (matches dashboard expectations):
 * {
 *   "piezo": 0-32767,      // Combined piezo intensity (A0+A1 max), scaled
 *   "mic": 0-32767,        // Microphone level from A3, scaled
 *   "motion": float,       // Acceleration magnitude in m/s²
 *   "score": 0-7,          // Fusion detection score
 *   "rawADC": 0-32767,     // RAW ADS1115 A0 reading ONLY (unprocessed)
 *   "battery": float,      // Battery voltage in V
 *   "status": bool,        // Detection active flag
 *   "confidence": 0-100    // Weighted confidence percentage
 * }
 * 
 * IMPORTANT: rawADC is STRICTLY the unmodified A0 reading.
 *            It is NOT averaged, filtered, or combined with any other channel.
 * 
 * ══════════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <MPU6050_light.h>

// ══════════════════════════════════════════════════════════════════════════════
// COMPILE-TIME CONFIGURATION
// ══════════════════════════════════════════════════════════════════════════════

// Set to true to enable Firebase publishing (requires WiFi credentials)
#define ENABLE_FIREBASE false

#if ENABLE_FIREBASE
  #include <WiFi.h>
  #include <Firebase_ESP_Client.h>
  #include "addons/TokenHelper.h"
  #include "addons/RTDBHelper.h"
  
  // WiFi credentials (fill in your values)
  #define WIFI_SSID "YOUR_WIFI_SSID"
  #define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
  
  // Firebase credentials (fill in your values)
  #define FIREBASE_API_KEY "YOUR_FIREBASE_API_KEY"
  #define FIREBASE_DATABASE_URL "YOUR_FIREBASE_DATABASE_URL"
  #define FIREBASE_PROJECT_ID "YOUR_PROJECT_ID"
#endif

// ══════════════════════════════════════════════════════════════════════════════
// HARDWARE PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════════════════════

#define PIN_SDA           21    // I2C Data
#define PIN_SCL           22    // I2C Clock
#define PIN_LED           25    // Alert LED
#define PIN_BUZZER        26    // Alert Buzzer
#define PIN_BATTERY       34    // Battery ADC (GPIO34 = ADC1_CH6)

// I2C Addresses
#define ADS1115_ADDRESS   0x48
#define MPU6050_ADDRESS   0x68

// ══════════════════════════════════════════════════════════════════════════════
// TIMING CONFIGURATION (non-blocking)
// ══════════════════════════════════════════════════════════════════════════════

#define SAMPLE_INTERVAL_MS    20    // 50 Hz sampling rate
#define PUBLISH_INTERVAL_MS   100   // 10 Hz publishing rate
#define BATTERY_INTERVAL_MS   1000  // 1 Hz battery check
#define WIFI_RETRY_INTERVAL   10000 // 10s WiFi reconnect interval

// ══════════════════════════════════════════════════════════════════════════════
// SIGNAL PROCESSING CONFIGURATION
// ══════════════════════════════════════════════════════════════════════════════

#define FILTER_WINDOW_SIZE    5     // Moving average window

// ADS1115 has 16-bit signed output, but in single-ended mode: 0 to 32767
#define ADS_MAX_VALUE         32767
#define NORMALIZED_MAX        100

// ══════════════════════════════════════════════════════════════════════════════
// DETECTION THRESHOLDS
// ══════════════════════════════════════════════════════════════════════════════

#define PIEZO_THRESHOLD       20    // Normalized 0-100 scale
#define MIC_THRESHOLD         15    // Normalized 0-100 scale
#define MOTION_DELTA_THRESH   0.1f  // m/s² deviation from gravity

// Confidence weights (must sum to 100)
#define WEIGHT_PIEZO          40
#define WEIGHT_MIC            30
#define WEIGHT_MOTION         30

// Score point allocations (max total = 7)
#define SCORE_PIEZO_MAX       3
#define SCORE_MIC_MAX         2
#define SCORE_MOTION_MAX      2

// Confidence cap when only piezo is active (prevents false positives)
#define SINGLE_SENSOR_CAP     15.0f

// ══════════════════════════════════════════════════════════════════════════════
// BATTERY MONITORING
// ══════════════════════════════════════════════════════════════════════════════

// 2S Li-ion: 8.4V max, 7.4V nominal, 6.0V cutoff
// Resistor divider ratio: 2:1 (e.g., 100K + 100K)
#define BATTERY_DIVIDER_RATIO 2.0f
#define BATTERY_LOW_THRESHOLD 6.8f   // Low battery warning threshold
#define ESP32_ADC_MAX         4095   // 12-bit ADC
#define ESP32_ADC_VREF        3.3f   // Reference voltage

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ══════════════════════════════════════════════════════════════════════════════

Adafruit_ADS1115 ads;
MPU6050 mpu(Wire);

#if ENABLE_FIREBASE
  FirebaseData fbdo;
  FirebaseAuth auth;
  FirebaseConfig config;
  bool firebaseReady = false;
  unsigned long lastWiFiRetry = 0;
#endif

// ══════════════════════════════════════════════════════════════════════════════
// SENSOR DATA STRUCTURES
// ══════════════════════════════════════════════════════════════════════════════

// Raw ADC readings (before filtering)
struct RawReadings {
  int16_t piezoA0;        // ADS1115 Channel A0 - Piezo 1
  int16_t piezoA1;        // ADS1115 Channel A1 - Piezo 2
  int16_t micA3;          // ADS1115 Channel A3 - Microphone
  float accelX, accelY, accelZ;  // MPU6050 acceleration (m/s²)
  float batteryVoltage;   // Battery voltage (V)
};

// Filtered/processed readings
struct ProcessedReadings {
  float piezoNorm;        // Combined piezo (0-100)
  float micNorm;          // Mic level (0-100)
  float motionMag;        // Acceleration magnitude (m/s²)
  float motionDelta;      // Deviation from gravity (m/s²)
};

// Detection results
struct DetectionResult {
  bool piezoActive;
  bool micActive;
  bool motionActive;
  bool detected;          // Multi-sensor detection flag
  int score;              // 0-7 fusion score
  float confidence;       // 0-100 weighted confidence
};

// Moving average filter buffers
struct FilterBuffer {
  float piezoA0[FILTER_WINDOW_SIZE];
  float piezoA1[FILTER_WINDOW_SIZE];
  float mic[FILTER_WINDOW_SIZE];
  float motion[FILTER_WINDOW_SIZE];
  int index;
  bool filled;
};

// Global state
RawReadings rawData;
ProcessedReadings procData;
DetectionResult detection;
FilterBuffer filters;

// Timing state (non-blocking)
unsigned long lastSampleTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastBatteryTime = 0;

// Sensor status flags
bool adsInitialized = false;
bool mpuInitialized = false;
bool lowBattery = false;

// ══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Normalize ADS1115 reading (0-32767) to 0-100 scale
 */
float normalizeADS(int16_t value) {
  if (value < 0) value = 0;
  return ((float)value / ADS_MAX_VALUE) * NORMALIZED_MAX;
}

/**
 * Scale normalized value (0-100) back to ADS range (0-32767)
 * Used for JSON output compatibility with dashboard
 */
int16_t scaleToADSRange(float normalized) {
  return (int16_t)constrain((normalized / NORMALIZED_MAX) * ADS_MAX_VALUE, 0, ADS_MAX_VALUE);
}

/**
 * Clamp float to range
 */
float clampf(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// ══════════════════════════════════════════════════════════════════════════════
// SENSOR INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Initialize all sensors and peripherals
 */
void initSensors() {
  Serial.println(F("[INIT] Starting sensor initialization..."));
  
  // Initialize I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  Serial.println(F("[INIT] I2C bus initialized (SDA=GPIO21, SCL=GPIO22)"));
  
  // Initialize ADS1115
  Serial.print(F("[INIT] Initializing ADS1115 at 0x"));
  Serial.print(ADS1115_ADDRESS, HEX);
  Serial.print(F("... "));
  
  if (ads.begin(ADS1115_ADDRESS)) {
    adsInitialized = true;
    ads.setGain(GAIN_ONE);        // ±4.096V range
    ads.setDataRate(RATE_ADS1115_250SPS);  // 250 samples/sec
    Serial.println(F("OK"));
    Serial.println(F("[INIT] ADS1115 Channel Map:"));
    Serial.println(F("       A0 -> Piezo Ch1 (LM358) -- also rawADC"));
    Serial.println(F("       A1 -> Piezo Ch2 (LM358)"));
    Serial.println(F("       A2 -> UNUSED"));
    Serial.println(F("       A3 -> KY-038 Microphone"));
  } else {
    Serial.println(F("FAILED!"));
    Serial.println(F("[ERROR] ADS1115 not found! Check wiring."));
  }
  
  // Initialize MPU6050
  Serial.print(F("[INIT] Initializing MPU6050 at 0x"));
  Serial.print(MPU6050_ADDRESS, HEX);
  Serial.print(F("... "));
  
  byte mpuStatus = mpu.begin();
  if (mpuStatus == 0) {
    mpuInitialized = true;
    Serial.println(F("OK"));
    Serial.println(F("[INIT] Calibrating MPU6050 gyroscope (keep device still)..."));
    mpu.calcOffsets();
    Serial.println(F("[INIT] MPU6050 calibration complete"));
  } else {
    Serial.print(F("FAILED! Error code: "));
    Serial.println(mpuStatus);
  }
  
  // Initialize GPIO outputs
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  Serial.println(F("[INIT] GPIO outputs configured (LED=GPIO25, Buzzer=GPIO26)"));
  
  // Initialize battery ADC
  analogReadResolution(12);  // 12-bit resolution
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  Serial.println(F("[INIT] Battery monitor configured (GPIO34, 2:1 divider)"));
  
  // Initialize filter buffers
  memset(&filters, 0, sizeof(filters));
  filters.index = 0;
  filters.filled = false;
  
  Serial.println(F("[INIT] Sensor initialization complete"));
  Serial.println(F("══════════════════════════════════════════════════════════"));
}

// ══════════════════════════════════════════════════════════════════════════════
// FIREBASE INITIALIZATION (conditional)
// ══════════════════════════════════════════════════════════════════════════════

#if ENABLE_FIREBASE
void initFirebase() {
  Serial.println(F("[WIFI] Connecting to WiFi..."));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print(F("[WIFI] Connected! IP: "));
    Serial.println(WiFi.localIP());
    
    // Configure Firebase
    config.api_key = FIREBASE_API_KEY;
    config.database_url = FIREBASE_DATABASE_URL;
    config.token_status_callback = tokenStatusCallback;
    
    // Anonymous auth
    Firebase.signUp(&config, &auth, "", "");
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    
    firebaseReady = true;
    Serial.println(F("[FIREBASE] Initialized successfully"));
  } else {
    Serial.println();
    Serial.println(F("[WIFI] Connection failed. Continuing without Firebase."));
  }
}

void checkWiFiReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    firebaseReady = false;
    unsigned long now = millis();
    if (now - lastWiFiRetry >= WIFI_RETRY_INTERVAL) {
      lastWiFiRetry = now;
      Serial.println(F("[WIFI] Attempting reconnection..."));
      WiFi.reconnect();
    }
  } else if (!firebaseReady) {
    firebaseReady = true;
    Serial.println(F("[WIFI] Reconnected!"));
  }
}
#endif

// ══════════════════════════════════════════════════════════════════════════════
// SENSOR READING FUNCTIONS
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Read all ADS1115 channels
 * Channel mapping:
 *   A0 -> Piezo Channel 1 (also used for rawADC - UNPROCESSED)
 *   A1 -> Piezo Channel 2
 *   A3 -> KY-038 Microphone
 */
void readADS1115() {
  if (!adsInitialized) {
    rawData.piezoA0 = 0;
    rawData.piezoA1 = 0;
    rawData.micA3 = 0;
    return;
  }
  
  // Read Piezo Channel 1 (A0) - This is also the rawADC value
  rawData.piezoA0 = ads.readADC_SingleEnded(0);
  
  // Read Piezo Channel 2 (A1)
  rawData.piezoA1 = ads.readADC_SingleEnded(1);
  
  // Read Microphone (A3) - Note: A2 is unused
  rawData.micA3 = ads.readADC_SingleEnded(3);
  
  // Ensure non-negative values
  if (rawData.piezoA0 < 0) rawData.piezoA0 = 0;
  if (rawData.piezoA1 < 0) rawData.piezoA1 = 0;
  if (rawData.micA3 < 0) rawData.micA3 = 0;
}

/**
 * Read MPU6050 accelerometer data
 * Returns acceleration magnitude in m/s²
 */
void readMPU6050() {
  if (!mpuInitialized) {
    rawData.accelX = 0;
    rawData.accelY = 0;
    rawData.accelZ = 9.81f;  // Default to gravity
    return;
  }
  
  mpu.update();
  
  // MPU6050_light returns acceleration in g, convert to m/s²
  rawData.accelX = mpu.getAccX() * 9.81f;
  rawData.accelY = mpu.getAccY() * 9.81f;
  rawData.accelZ = mpu.getAccZ() * 9.81f;
}

/**
 * Read battery voltage via GPIO34 ADC
 * Uses averaging for stability
 */
void readBattery() {
  const int NUM_SAMPLES = 10;
  long sum = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(PIN_BATTERY);
  }
  
  float avgReading = (float)sum / NUM_SAMPLES;
  
  // Convert ADC reading to voltage
  // ADC reading -> ESP32 pin voltage -> actual battery voltage
  float pinVoltage = (avgReading / ESP32_ADC_MAX) * ESP32_ADC_VREF;
  rawData.batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;
  
  // Check low battery condition
  lowBattery = (rawData.batteryVoltage < BATTERY_LOW_THRESHOLD);
}

// ══════════════════════════════════════════════════════════════════════════════
// SIGNAL FILTERING
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Add value to moving average buffer and compute average
 */
float updateMovingAverage(float* buffer, float newValue) {
  buffer[filters.index] = newValue;
  
  float sum = 0;
  int count = filters.filled ? FILTER_WINDOW_SIZE : (filters.index + 1);
  for (int i = 0; i < count; i++) {
    sum += buffer[i];
  }
  return sum / count;
}

/**
 * Update all filter buffers with new readings
 */
void updateFilters() {
  // Normalize raw readings to 0-100 scale
  float piezoA0Norm = normalizeADS(rawData.piezoA0);
  float piezoA1Norm = normalizeADS(rawData.piezoA1);
  float micNorm = normalizeADS(rawData.micA3);
  
  // Compute acceleration magnitude
  float accelMag = sqrtf(
    rawData.accelX * rawData.accelX +
    rawData.accelY * rawData.accelY +
    rawData.accelZ * rawData.accelZ
  );
  
  // Apply moving average filters
  float filteredA0 = updateMovingAverage(filters.piezoA0, piezoA0Norm);
  float filteredA1 = updateMovingAverage(filters.piezoA1, piezoA1Norm);
  float filteredMic = updateMovingAverage(filters.mic, micNorm);
  float filteredMotion = updateMovingAverage(filters.motion, accelMag);
  
  // Advance filter index
  filters.index = (filters.index + 1) % FILTER_WINDOW_SIZE;
  if (filters.index == 0) filters.filled = true;
  
  // Combine piezo channels using MAX (captures strongest signal)
  // Documentation: Using MAX instead of average to detect any significant piezo activity
  procData.piezoNorm = fmaxf(filteredA0, filteredA1);
  
  // Store filtered mic value
  procData.micNorm = filteredMic;
  
  // Store motion data
  procData.motionMag = filteredMotion;
  procData.motionDelta = fabsf(filteredMotion - 9.81f);
}

// ══════════════════════════════════════════════════════════════════════════════
// DETECTION LOGIC
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Compute multi-sensor detection with confidence scoring
 * 
 * Detection Logic:
 *   - Piezo must be active (>=20 normalized)
 *   - AND at least one of: Mic active (>=15) OR Motion active (>=0.1 m/s² delta)
 *   
 * Confidence Calculation:
 *   - Piezo contributes 40%
 *   - Mic contributes 30%
 *   - Motion contributes 30%
 *   - If only piezo active: cap at 15% to avoid false positives
 *   
 * Score Calculation (0-7):
 *   - Piezo: 0-3 points based on intensity
 *   - Mic: 0-2 points based on level
 *   - Motion: 0-2 points based on delta
 */
void computeDetection() {
  // Check individual sensor activation
  detection.piezoActive = (procData.piezoNorm >= PIEZO_THRESHOLD);
  detection.micActive = (procData.micNorm >= MIC_THRESHOLD);
  detection.motionActive = (procData.motionDelta >= MOTION_DELTA_THRESH);
  
  // Multi-sensor detection: piezo AND (mic OR motion)
  detection.detected = detection.piezoActive && 
                       (detection.micActive || detection.motionActive);
  
  // Calculate confidence score
  float confidence = 0.0f;
  
  if (detection.detected) {
    // Full weighted confidence calculation
    confidence += (procData.piezoNorm / NORMALIZED_MAX) * WEIGHT_PIEZO;
    
    if (detection.micActive) {
      confidence += (procData.micNorm / NORMALIZED_MAX) * WEIGHT_MIC;
    }
    
    if (detection.motionActive) {
      // Scale motion delta contribution (max at 1.0 m/s² delta)
      float motionContrib = fminf(procData.motionDelta / 1.0f, 1.0f);
      confidence += motionContrib * WEIGHT_MOTION;
    }
  } else if (detection.piezoActive) {
    // Only piezo active - cap confidence to avoid false positives
    confidence = (procData.piezoNorm / NORMALIZED_MAX) * SINGLE_SENSOR_CAP;
  }
  
  detection.confidence = clampf(confidence, 0.0f, 100.0f);
  
  // Calculate integer score (0-7)
  int score = 0;
  
  // Piezo contribution (0-3 points)
  if (procData.piezoNorm >= 20) score += 1;
  if (procData.piezoNorm >= 50) score += 1;
  if (procData.piezoNorm >= 80) score += 1;
  
  // Mic contribution (0-2 points)
  if (procData.micNorm >= 15) score += 1;
  if (procData.micNorm >= 50) score += 1;
  
  // Motion contribution (0-2 points)
  if (procData.motionDelta >= 0.1f) score += 1;
  if (procData.motionDelta >= 0.5f) score += 1;
  
  detection.score = constrain(score, 0, 7);
  
  // Update alert outputs
  if (detection.detected && detection.confidence >= 60.0f) {
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// JSON OUTPUT
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Build JSON string for output
 * 
 * IMPORTANT FIELD MAPPING:
 *   "piezo"   -> Combined piezo intensity (A0+A1 max), scaled to 0-32767
 *   "mic"     -> Microphone from A3, scaled to 0-32767
 *   "rawADC"  -> STRICTLY ADS1115 A0 only, UNPROCESSED, UNFILTERED
 *   "motion"  -> Acceleration magnitude in m/s²
 *   "score"   -> Integer fusion score 0-7
 *   "battery" -> Battery voltage in V
 *   "status"  -> Detection active boolean
 *   "confidence" -> Weighted confidence 0-100
 */
String buildJson() {
  // Scale normalized values back to ADS range for dashboard compatibility
  int16_t piezoScaled = scaleToADSRange(procData.piezoNorm);
  int16_t micScaled = scaleToADSRange(procData.micNorm);
  
  // rawADC is STRICTLY the unmodified A0 reading - NOT filtered or combined
  int16_t rawADC = rawData.piezoA0;
  
  String json = "{";
  json += "\"piezo\":" + String(piezoScaled) + ",";
  json += "\"mic\":" + String(micScaled) + ",";
  json += "\"motion\":" + String(procData.motionMag, 2) + ",";
  json += "\"score\":" + String(detection.score) + ",";
  json += "\"rawADC\":" + String(rawADC) + ",";  // A0 ONLY, unprocessed
  json += "\"battery\":" + String(rawData.batteryVoltage, 2) + ",";
  json += "\"status\":" + String(detection.detected ? "true" : "false") + ",";
  json += "\"confidence\":" + String(detection.confidence, 1);
  json += "}";
  
  return json;
}

// ══════════════════════════════════════════════════════════════════════════════
// DATA PUBLISHING
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Publish data over USB Serial (always enabled)
 * Outputs one JSON object per line at ~10 Hz
 */
void publishSerial() {
  String json = buildJson();
  Serial.println(json);
}

#if ENABLE_FIREBASE
/**
 * Publish data to Firebase Realtime Database
 * Non-blocking, resilient to connection drops
 */
void publishFirebase() {
  if (!firebaseReady || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  if (!Firebase.ready()) {
    return;
  }
  
  // Publish to /sensor path
  FirebaseJson fbJson;
  
  fbJson.set("piezo", scaleToADSRange(procData.piezoNorm));
  fbJson.set("mic", scaleToADSRange(procData.micNorm));
  fbJson.set("motion", procData.motionMag);
  fbJson.set("score", detection.score);
  fbJson.set("rawADC", (int)rawData.piezoA0);  // A0 only, unprocessed
  fbJson.set("battery", rawData.batteryVoltage);
  fbJson.set("status", detection.detected);
  fbJson.set("confidence", detection.confidence);
  fbJson.set("timestamp/.sv", "timestamp");  // Server timestamp
  
  // Non-blocking set
  if (!Firebase.RTDB.setJSONAsync(&fbdo, "/sensor", &fbJson)) {
    // Silently ignore errors to maintain non-blocking operation
  }
}
#endif

// ══════════════════════════════════════════════════════════════════════════════
// MAIN SCHEDULER
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Non-blocking loop scheduler
 * Manages all timing for sampling and publishing
 */
void loopScheduler() {
  unsigned long currentTime = millis();
  
  // Sensor sampling at 50 Hz (every 20ms)
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;
    
    // Read all sensors
    readADS1115();
    readMPU6050();
    
    // Update filters and compute detection
    updateFilters();
    computeDetection();
  }
  
  // Battery check at 1 Hz (every 1000ms)
  if (currentTime - lastBatteryTime >= BATTERY_INTERVAL_MS) {
    lastBatteryTime = currentTime;
    readBattery();
    
    // Low battery warning (brief LED flash)
    if (lowBattery && !detection.detected) {
      digitalWrite(PIN_LED, HIGH);
      delayMicroseconds(50000);  // 50ms flash
      digitalWrite(PIN_LED, LOW);
    }
  }
  
  // Data publishing at 10 Hz (every 100ms)
  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    lastPublishTime = currentTime;
    
    publishSerial();
    
    #if ENABLE_FIREBASE
      publishFirebase();
    #endif
  }
  
  #if ENABLE_FIREBASE
    // Check WiFi reconnection in background
    checkWiFiReconnect();
  #endif
}

// ══════════════════════════════════════════════════════════════════════════════
// ARDUINO ENTRY POINTS
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize Serial for USB communication
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for Serial (with timeout)
  }
  
  Serial.println();
  Serial.println(F("══════════════════════════════════════════════════════════"));
  Serial.println(F("   GROUND PULSE v3.0 - Life Detection System"));
  Serial.println(F("   ESP32 Firmware"));
  Serial.println(F("══════════════════════════════════════════════════════════"));
  Serial.println();
  
  // Initialize all sensors
  initSensors();
  
  #if ENABLE_FIREBASE
    initFirebase();
  #endif
  
  // Initial battery reading
  readBattery();
  
  Serial.println();
  Serial.println(F("[READY] System operational. Publishing JSON at 10 Hz..."));
  Serial.println(F("[INFO] JSON Schema:"));
  Serial.println(F("  piezo    -> Combined A0+A1 (max), scaled 0-32767"));
  Serial.println(F("  mic      -> A3 microphone, scaled 0-32767"));
  Serial.println(F("  rawADC   -> A0 ONLY (Piezo Ch1), unprocessed 0-32767"));
  Serial.println(F("  motion   -> Acceleration magnitude (m/s^2)"));
  Serial.println(F("  score    -> Fusion score 0-7"));
  Serial.println(F("  battery  -> Battery voltage (V)"));
  Serial.println(F("  status   -> Detection active (bool)"));
  Serial.println(F("  confidence -> Weighted confidence 0-100"));
  Serial.println(F("══════════════════════════════════════════════════════════"));
  Serial.println();
  
  // Initialize timing
  lastSampleTime = millis();
  lastPublishTime = millis();
  lastBatteryTime = millis();
}

void loop() {
  loopScheduler();
}
FIRMWARE_EOF 
