# 🌍 GroundPulse

**GroundPulse** is an ESP32‑based **subsurface human presence detection system** that identifies biological activity underground by fusing **seismic vibrations, micro‑motion, and CO₂ concentration data**.  
It performs **real‑time frequency‑domain analysis (FFT)** and transmits detection data wirelessly using **LoRa**, while providing live feedback on an OLED display.

> Built as an **all‑in‑one, deploy‑and‑forget unit** for applications like rescue operations, underground monitoring, and experimental life‑detection systems.

***

## ✨ Features

*   ✅ **Dual‑core ESP32 firmware**
*   ✅ **Real‑time FFT‑based seismic analysis**
*   ✅ **Human breathing & heartbeat frequency detection**
*   ✅ **CO₂ rise detection (respiration‑based)**
*   ✅ **Micro‑vibration sensing via accelerometer**
*   ✅ **Multi‑sensor confidence scoring (0–100%)**
*   ✅ **OLED live status & alert display**
*   ✅ **Long‑range LoRa wireless telemetry**
*   ✅ **Solar‑charged battery monitoring**
*   ✅ **Watchdog‑protected field reliability**

***

## 🧠 How It Works

GroundPulse combines **three independent indicators** of human presence:

| Sensor            | What it Detects                          |
| ----------------- | ---------------------------------------- |
| **Piezo + LM358** | Ground vibrations (breathing, heartbeat) |
| **ADXL345**       | Micro‑movements and tremors              |
| **MH‑Z19B CO₂**   | CO₂ rise from human respiration          |

The seismic signal is sampled at **50 Hz**, processed using a **256‑point FFT**, and analyzed in the **0.1–5 Hz biological band**.  
All sensor readings are fused into a **confidence score (0–100%)**, triggering alerts when the threshold is exceeded.

***

## 📊 Detection Confidence Logic

**Score Composition:**

*   **Frequency match** (breathing / heartbeat) → up to 40%
*   **FFT prominence (signal‑to‑noise)** → up to 25%
*   **CO₂ rise above baseline** → up to 25%
*   **Micro‑vibration magnitude** → up to 10%

### Status Levels

*   **0–30%** → Environmental noise
*   **30–60%** → Weak / uncertain signal
*   **60–80%** → Possible human presence ⚠️
*   **80–100%** → Strong human signature 🚨

***

## 🖥 Hardware Used

*   ESP32 DevKit V1
*   Piezo disc + LM358 amplifier
*   ADXL345 3‑axis accelerometer
*   MH‑Z19B NDIR CO₂ sensor
*   SSD1306 128×64 OLED
*   SX127x LoRa module (433 MHz)
*   18650 Li‑ion battery (solar‑charge capable)
*   LED + Buzzer for alerts

***

## 🔌 Pin Connections

| Component              | ESP32 Pin        |
| ---------------------- | ---------------- |
| Piezo (ADC)            | GPIO34           |
| Battery Divider        | GPIO35           |
| ADXL345 SDA / SCL      | GPIO21 / GPIO22  |
| MH‑Z19B RX / TX        | GPIO16 / GPIO17  |
| OLED SDA / SCL         | GPIO21 / GPIO22  |
| LoRa SCK / MISO / MOSI | GPIO18 / 19 / 23 |
| LoRa NSS               | GPIO5            |
| LoRa RST               | GPIO14           |
| LoRa DIO0              | GPIO2            |
| Alert LED              | GPIO4            |
| Buzzer                 | GPIO13           |

> Battery sensing uses a **100kΩ + 100kΩ voltage divider**.

***

## 📡 LoRa Data Format

Each packet sent over LoRa follows:

    GP,<score>,<freq>,<co2_delta>,<accel>,<human>,<battery%>,<packet_id>

**Example:**

    GP,72,1.23,34,0.018,1,82,145

***

## 🖥 OLED Display Modes

### Normal Scan Mode

*   Confidence percentage
*   Dominant frequency (Hz)
*   CO₂ delta from baseline
*   Battery voltage & percentage
*   Scan status indicator

### Alert Mode

*   Flashing border
*   **“HUMAN DETECTED”** message
*   Buzzer + LED activation
*   Live detection metrics

***

## ⚙️ Software & Libraries

Install the following using **Arduino Library Manager**:

*   `arduinoFFT` – FFT processing
*   `LoRa` – SX127x radio
*   `Adafruit ADXL345 Unified`
*   `Adafruit GFX`
*   `Adafruit SSD1306`
*   `Adafruit Unified Sensor`
*   `MHZ19`

***

## 🔐 Reliability & Safety

*   ESP32 **watchdog timer** prevents system lockups
*   CO₂ **auto‑calibration disabled** (safe for enclosed spaces)
*   Invalid sensor readings automatically ignored
*   Battery voltage protected and monitored

***

## 🧪 Getting Started

1.  Assemble hardware as per pin table
2.  Install required Arduino libraries
3.  Upload firmware to ESP32
4.  Power using battery or solar source
5.  Wait for CO₂ warm‑up (\~60 seconds)
6.  Monitor OLED or receive packets via LoRa


***

## 👨‍💻 Authors

*   **Aditya Nautiyal**
*   **Aman Payal**
*   **Ayush Panwar**
*   **Aditya Shah**

    ~~Team **GroundPulse**


***
