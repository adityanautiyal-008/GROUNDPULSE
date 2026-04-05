/*
 * ╔══════════════════════════════════════════════════════════════╗
 *   GroundPulse — MAIN CONTROLLER (Field Unit)
 *   ESP32 DevKit V1
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * CORE PRINCIPLE:
 *  - Piezo-based seismic bio-vibration detection (PRIMARY)
 *  - Conditional acoustic confirmation (tapping / response)
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <arduinoFFT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_task_wdt.h"

// ─────────────────────────────────────────────
// SYSTEM CONFIG
// ─────────────────────────────────────────────
#define DEVICE_NAME        "GROUNDPULSE-01"
#define LORA_FREQUENCY     433E6
#define WDT_TIMEOUT_SEC    30

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────
#define PIEZO_PIN          34      // Piezo + LM358 (ADC)
#define MIC_PIN            32      // MAX9814 or similar mic (ADC)
#define BATTERY_PIN        35      // Battery divider
#define LED_PIN            4
#define BUZZER_PIN         13

#define LORA_SCK           18
#define LORA_MISO          19
#define LORA_MOSI          23
#define LORA_CS            5
#define LORA_RST           14
#define LORA_DIO0          2

// ─────────────────────────────────────────────
// OLED
// ─────────────────────────────────────────────
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64
#define OLED_ADDR          0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────
// FFT CONFIG
// ─────────────────────────────────────────────
#define FFT_SAMPLES        256
#define SAMPLING_FREQ      50
#define SAMPLE_PERIOD_US   (1000000 / SAMPLING_FREQ)

// ─────────────────────────────────────────────
// BIOLOGICAL BANDS
// ─────────────────────────────────────────────
#define BREATH_MIN_HZ      0.10
#define BREATH_MAX_HZ      0.50
#define HEART_MIN_HZ       0.80
#define HEART_MAX_HZ       3.00
#define PROMINENCE_MIN     2.5

#define ALERT_THRESHOLD    60

// ─────────────────────────────────────────────
// BATTERY
// ─────────────────────────────────────────────
#define BAT_FULL_V         4.20
#define BAT_EMPTY_V        3.00
#define BAT_DIV_RATIO      2.0
#define ADC_REF            3.3
#define ADC_MAX            4095.0

// ─────────────────────────────────────────────
// FFT BUFFERS
// ─────────────────────────────────────────────
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);

// ─────────────────────────────────────────────
// SHARED STATE
// ─────────────────────────────────────────────
volatile float g_freq          = 0;
volatile float g_seismicEnergy = 0;
volatile bool  g_acousticEvent = false;
volatile int   g_score         = 0;
volatile bool  g_human         = false;
volatile int   g_batteryPct    = 100;

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
int packetID = 0;

// ─────────────────────────────────────────────
// TASK HANDLES
// ─────────────────────────────────────────────
TaskHandle_t hSense;
TaskHandle_t hDisplay;

// ─────────────────────────────────────────────
// FORWARD DECL
// ─────────────────────────────────────────────
void Task_SenseTransmit(void *);
void Task_DisplayAlert(void *);
float readBatteryVoltage();
int batteryPercent(float v);
int calculateConfidence(float f, float pr, bool acoustic);
void beepAlert(int t);

// ══════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  analogReadResolution(12);

  Wire.begin(21, 22);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("GROUND");
  display.println("PULSE");
  display.display();
  delay(2000);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  LoRa.begin(LORA_FREQUENCY);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();

  xTaskCreatePinnedToCore(Task_SenseTransmit, "Sense", 10000, NULL, 2, &hSense, 0);
  xTaskCreatePinnedToCore(Task_DisplayAlert,  "Display", 4096,  NULL, 1, &hDisplay, 1);

  beepAlert(2);
}

// ══════════════════════════════════════════════
// LOOP (watchdog only)
// ══════════════════════════════════════════════
void loop() {
  esp_task_wdt_reset();
  delay(1000);
}

// ══════════════════════════════════════════════
// CORE 0 — SENSE + FFT + LORA
// ══════════════════════════════════════════════
void Task_SenseTransmit(void *) {

  while (true) {

    // ---- PIEZO SAMPLING ----
    unsigned long t = micros();
    for (int i = 0; i < FFT_SAMPLES; i++) {
      while (micros() < t);
      vReal[i] = analogRead(PIEZO_PIN);
      vImag[i] = 0;
      t += SAMPLE_PERIOD_US;
    }

    // Remove DC offset
    double mean = 0;
    for (int i = 0; i < FFT_SAMPLES; i++) mean += vReal[i];
    mean /= FFT_SAMPLES;
    for (int i = 0; i < FFT_SAMPLES; i++) vReal[i] -= mean;

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double maxMag = 0, noise = 0;
    int peakBin = 0, bins = 0;
    double res = (double)SAMPLING_FREQ / FFT_SAMPLES;

    for (int i = 1; i < FFT_SAMPLES / 2; i++) {
      double f = i * res;
      if (f >= 0.1 && f <= 5.0) {
        if (vReal[i] > maxMag) { maxMag = vReal[i]; peakBin = i; }
        noise += vReal[i];
        bins++;
      }
    }

    noise /= (bins ? bins : 1);
    float prominence = maxMag / (noise ? noise : 1);
    float freq = peakBin * res;

    // ---- MICROPHONE EVENT (threshold) ----
    int micRaw = analogRead(MIC_PIN);
    bool acoustic = micRaw > 2500;   // adjust experimentally

    // ---- BATTERY ----
    float bv = readBatteryVoltage();
    int bp  = batteryPercent(bv);

    // ---- SCORE ----
    int score = calculateConfidence(freq, prominence, acoustic);
    bool human = score >= ALERT_THRESHOLD;

    portENTER_CRITICAL(&dataMux);
    g_freq = freq;
    g_seismicEnergy = prominence;
    g_acousticEvent = acoustic;
    g_score = score;
    g_human = human;
    g_batteryPct = bp;
    portEXIT_CRITICAL(&dataMux);

    // ---- LORA TX ----
    String pkt = "GP," +
      String(score) + "," +
      String(freq, 2) + "," +
      String(prominence, 4) + "," +
      String(acoustic ? 1 : 0) + "," +
      String(human ? 1 : 0) + "," +
      String(bp) + "," +
      String(packetID++);

    LoRa.beginPacket();
    LoRa.print(pkt);
    LoRa.endPacket(true);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ══════════════════════════════════════════════
// CORE 1 — DISPLAY & ALERT
// ══════════════════════════════════════════════
void Task_DisplayAlert(void *) {
  bool lastHuman = false;

  while (true) {
    portENTER_CRITICAL(&dataMux);
    int score = g_score;
    float f = g_freq;
    int bp = g_batteryPct;
    bool human = g_human;
    portEXIT_CRITICAL(&dataMux);

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Score: "); display.print(score); display.print("%");
    display.setCursor(0, 12);
    display.printf("Freq: %.2f Hz", f);
    display.setCursor(0, 24);
    display.printf("Bat : %d%%", bp);

    if (human) {
      if (!lastHuman) beepAlert(3);
      digitalWrite(LED_PIN, HIGH);
      display.setCursor(0, 42);
      display.print(">>> HUMAN <<<");
    } else {
      digitalWrite(LED_PIN, LOW);
      display.setCursor(0, 42);
      display.print("Scanning...");
    }

    display.display();
    lastHuman = human;
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ══════════════════════════════════════════════
// CONFIDENCE SCORING
// ══════════════════════════════════════════════
int calculateConfidence(float f, float pr, bool acoustic) {
  int s = 0;

  if (f >= HEART_MIN_HZ && f <= HEART_MAX_HZ) s += 40;
  else if (f >= BREATH_MIN_HZ && f <= BREATH_MAX_HZ) s += 30;
  else if (f >= 0.1 && f <= 5.0) s += 10;

  if (pr >= 5.0) s += 25;
  else if (pr >= PROMINENCE_MIN) s += pr * 5;

  if (acoustic) s += 10;

  return constrain(s, 0, 100);
}

// ══════════════════════════════════════════════
// BATTERY HELPERS
// ══════════════════════════════════════════════
float readBatteryVoltage() {
  float raw = analogRead(BATTERY_PIN);
  float v = (raw / ADC_MAX) * ADC_REF * BAT_DIV_RATIO;
  return v;
}

int batteryPercent(float v) {
  return constrain(
    (int)((v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0),
    0, 100
  );
}

// ══════════════════════════════════════════════
void beepAlert(int t) {
  for (int i = 0; i < t; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}
