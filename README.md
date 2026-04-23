# 🌍 GroundPulse v3.0
### Multi‑Sensor Subsurface Human Presence Detection System

GroundPulse is an **ESP32‑based, battery‑powered, multi‑sensor life‑detection system** designed to identify **probable living human presence beneath debris** after earthquakes, collapses, and disasters.

The system detects **biomechanical vibrations**, **intentional acoustic responses**, and **motion context**, and fuses these signals into a **confidence‑based detection score**. Real‑time telemetry is streamed to a **web dashboard** via USB Serial (and optionally Firebase), while long‑range LoRa transmission enables field deployment.

> **NO CAMERA · NO CO₂ · NO INTERNET REQUIRED IN FIELD MODE**

---

## ✅ System Highlights

- 🔹 Primary seismic sensing using **piezoelectric discs**
- 🔹 Analog signal amplification via **LM358 dual op‑amp**
- 🔹 High‑resolution sampling using **ADS1115 (16‑bit ADC)**
- 🔹 Motion‑aware noise rejection using **MPU6050 IMU**
- 🔹 Confidence‑based multi‑sensor fusion
- 🔹 Real‑time **HTML dashboard** (USB Serial / Firebase)
- 🔹 Long‑range **LoRa (433 MHz)** telemetry
- 🔹 Rechargeable **2‑cell Li‑ion power system**
- 🔹 Designed for **realistic SAR scenarios**

---

## 🧠 Detection Philosophy

GroundPulse follows a **professional search‑and‑rescue sensing strategy**:

### 🟡 Primary Sensor — Seismic (Always Active)
- Four piezoelectric discs coupled to ground
- Detects:
  - Breathing‑induced micro‑movements
  - Involuntary body motion
- Signals amplified via LM358
- Contributes **40%** to confidence

### 🔵 Secondary Sensor — Acoustic (Support Only)
- Analog microphone (KY‑038 / similar)
- Used only for **intentional human response**
- Never triggers detection alone
- Contributes **30%** to confidence

### 🟢 Context Sensor — Motion Reference
- MPU6050 accelerometer
- Suppresses false positives from:
  - Rescue activity
  - Structural vibration
- Contributes **30%** to confidence

**Detection Rule**  
    Piezo AND (Microphone OR Motion)

---

## 📊 Confidence Scoring

All sensor values are normalized to a **0–100 scale**:


Confidence =
0.40 × Piezo +
0.30 × Microphone +
0.30 × Motion


### Anti‑False‑Positive Rule
If **only piezo** is active:
  Confidence ≤ 15%

  
This prevents environmental noise from triggering alerts.

---

## 🚨 Alert Logic

| Condition | System State |
|---------|-------------|
| No signal | Scanning mode |
| Weak / unstable signal | Low confidence |
| Confidence ≥ 60% | ✅ Human presence detected |

When detected:
- 🟢 LED ON
- 🔔 Buzzer ON
- 🚨 Dashboard alert banner
- 📡 LoRa packet transmitted

---

## 🖥 Live Dashboard (HTML)

GroundPulse includes a **real‑time dashboard** that visualizes:

- Confidence arc
- Piezo, mic, and motion gauges
- Raw ADC waveform
- Event log
- Battery voltage & status
- System heartbeat

### Communication Modes
- ✅ USB Serial (Web Serial API)
- ✅ Firebase Realtime Database (optional)

> Dashboard file:  
> `groundpulse_dashboard_v3.html`

---

## 📡 Telemetry Data Format (JSON)

The ESP32 streams one JSON object per line:

```json
{
  "piezo": 15432,
  "mic": 4231,
  "motion": 9.86,
  "score": 5,
  "rawADC": 14820,
  "battery": 7.42,
  "status": true,
  "confidence": 78.5
}
```

📌 Project Status
✅ Hardware architecture finalized
✅ Signal fusion implemented
✅ Dashboard integration complete
✅ Field‑ready prototype
✅ Viva‑defensible design

🎯 Future Improvements

Directional ground coupling
Adaptive thresholding
Multi‑node LoRa mesh
Geophone integration
Field casing & ruggedization


👨‍💻 Authors
Team Maverics.exe

Aditya Nautiyal
Aman Payal
Ayush Panwar
Aditya Shah


“Designed to listen where eyes cannot see.”
