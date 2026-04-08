/*
╔══════════════════════════════════════════════════════════════╗
║                GroundPulse — MAIN CONTROLLER                 ║
║        Subsurface Human Presence Detection System            ║
║              (Single-Box Field Unit)                         ║
╚══════════════════════════════════════════════════════════════╝


## SENSING STRATEGY (IMPORTANT)

   ### PRIMARY SENSOR — PIEZO VIBRATION (Always Active)
      ✔ Four piezoelectric discs connected in parallel  
      ✔ Amplified using LM358 op-amp  
      ✔ Detects:
         - Breathing-induced micro-movements  
         - Involuntary body motion transmitted through debris  
      This sensor contributes **70% weight** to the confidence score.

---

### SECONDARY SENSOR — MICROPHONE (Support Only)
   ✔ Used only to detect intentional human response (tapping, sound)  
   ✔ Never allowed to trigger detection on its own  
   ✔ Helps confirm ambiguous vibration signals  
   This sensor contributes **30% weight** to the confidence score.

---

## OLED DISPLAY OPERATION

### Normal State
- Displays:
  - Piezo signal percentage
  - Microphone signal percentage
  - Overall confidence percentage
- Shows message:
   Scanning...

### Detection State
- Displays:
   HUMAN DETECTED
      - Green LED turns ON
      - Buzzer is activated
      - Detection confidence is shown

---

## ALERT LOGIC

| Condition | OLED | LED | Buzzer |
|--------|------|-----|-------|
| No significant signal | “Scanning…” | 🔴 Red | OFF |
| Weak / noisy signal | % shown | 🔴 Red | OFF |
| Confidence ≥ threshold | “HUMAN DETECTED” | 🟢 Green | ON |

---

## CONFIDENCE CALCULATION

Sensor values are normalized into percentages:

   Piezo Signal  → 0–100%
   Mic Signal    → 0–100%

Final confidence is calculated as:
         Confidence = (0.7 × Piezo%) + (0.3 × Mic%)

   A detection is triggered when:
            Confidence ≥ 60%

---

Where:
- `confidence` → Combined detection confidence (0–100)
- `piezoPercent` → Piezo vibration strength
- `micPercent` → Microphone activity level
- `detectedFlag` → 1 = detected, 0 = scanning

---

## HARDWARE PLATFORM
- ESP32 DevKit V1 (ESP32‑WROOM‑32)
- 4× Piezoelectric discs + LM358 amplifier
- Microphone module (analog output)
- MPU6050 MEMS accelerometer (reference sensor)
- SSD1306 0.96” OLED display
- LoRa module S‑14432 (433 MHz)
- Dual 18650 Li‑ion battery pack
- Dual regulated outputs (5 V & 3.3 V)
- LEDs and buzzer

---
*/
   

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================================================
// PIN DEFINITIONS
// ==================================================
#define PIEZO_PIN      35
#define MIC_PIN        34

#define LED_GREEN      16
#define LED_RED        17
#define BUZZER_PIN     25

#define LORA_SCK       18
#define LORA_MISO      19
#define LORA_MOSI      23
#define LORA_CS        5
#define LORA_RST       14
#define LORA_DIO0      26

// ==================================================
// OLED CONFIG
// ==================================================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_ADDR      0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==================================================
// LORA CONFIG
// ==================================================
#define LORA_FREQ      433E6

// ==================================================
// THRESHOLDS (TUNE THESE)
// ==================================================
#define PIEZO_MIN      300
#define PIEZO_MAX      3500

#define MIC_MIN        200
#define MIC_MAX        3500

#define DETECT_LIMIT   60     // % threshold

// ==================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);

  // OLED
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 25);
  display.println("Ground");
  display.println("Pulse");
  display.display();
  delay(2000);

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  LoRa.begin(LORA_FREQ);
  LoRa.enableCrc();

  beep(2);
}

// ==================================================
void loop() {

  int piezoRaw = analogRead(PIEZO_PIN);
  int micRaw   = analogRead(MIC_PIN);

  // Convert to percentage
  int piezoPct = map(constrain(piezoRaw, PIEZO_MIN, PIEZO_MAX),
                     PIEZO_MIN, PIEZO_MAX, 0, 100);

  int micPct   = map(constrain(micRaw, MIC_MIN, MIC_MAX),
                     MIC_MIN, MIC_MAX, 0, 100);

  // Weighting: Piezo = 70%, Mic = 30%
  int confidence = (piezoPct * 0.7) + (micPct * 0.3);

  bool detected = confidence >= DETECT_LIMIT;

  // OLED DISPLAY
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("GroundPulse Scan");

  display.setCursor(0, 15);
  display.print("Piezo: "); display.print(piezoPct); display.println("%");

  display.setCursor(0, 25);
  display.print("Mic  : "); display.print(micPct); display.println("%");

  display.setCursor(0, 40);
  display.print("CONF : "); display.print(confidence); display.println("%");

  // OUTPUT LOGIC
  if (detected) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER_PIN, HIGH);

    display.setCursor(0, 55);
    display.print("HUMAN DETECTED");
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER_PIN, LOW);

    display.setCursor(0, 55);
    display.print("Scanning...");
  }

  display.display();

  // LoRa TX
  LoRa.beginPacket();
  LoRa.print("GP,");
  LoRa.print(confidence);
  LoRa.print(",");
  LoRa.print(piezoPct);
  LoRa.print(",");
  LoRa.print(micPct);
  LoRa.print(",");
  LoRa.print(detected ? 1 : 0);
  LoRa.endPacket(true);

  delay(500);
}

// ==================================================
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);
    delay(120);
  }
}
