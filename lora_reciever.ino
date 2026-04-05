/*
 * ╔══════════════════════════════════════════════════════════════╗
 *   GroundPulse — LoRa Receiver (Base Station)
 *   ESP32 DevKit V1 + SX1278
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * Packet format:
 *   GP,<score>,<freq>,<seismic>,<acoustic>,<human>,<battery>,<packetID>
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ──────────────────────────────────────────────────────────────
// CONFIGURATION
// ──────────────────────────────────────────────────────────────
#define LORA_FREQUENCY  433E6   // MUST match transmitter

// LoRa pins
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  2

// ──────────────────────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("╔══════════════════════════════╗");
  Serial.println("║  GroundPulse LoRa Receiver   ║");
  Serial.println("╚══════════════════════════════╝");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[ERROR] LoRa init failed!");
    while (true) delay(100);
  }

  // RF parameters (MUST match transmitter)
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();

  Serial.println("[OK] LoRa receiver ready.");
  Serial.println("Waiting for packets...\n");
}

// ──────────────────────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────────────────────
void loop() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String packet = "";
  while (LoRa.available()) {
    packet += (char)LoRa.read();
  }

  Serial.println("================================");
  Serial.print("RAW: ");
  Serial.println(packet);

  parsePacket(packet);

  Serial.printf("RSSI: %d dBm | SNR: %.1f dB\n",
                LoRa.packetRssi(),
                LoRa.packetSnr());
  Serial.println("================================\n");
}

// ──────────────────────────────────────────────────────────────
// PACKET PARSER
// ──────────────────────────────────────────────────────────────
void parsePacket(String pkt) {
  if (!pkt.startsWith("GP,")) {
    Serial.println("[WARN] Invalid packet format.");
    return;
  }

  pkt.remove(0, 3);  // remove "GP,"

  float score           = nextValue(pkt).toFloat();
  float freq            = nextValue(pkt).toFloat();
  float seismicEnergy   = nextValue(pkt).toFloat();
  int   acousticEvent   = nextValue(pkt).toInt();
  int   humanFlag       = nextValue(pkt).toInt();
  int   battery         = nextValue(pkt).toInt();
  int   packetID        = pkt.toInt();

  Serial.printf("Confidence      : %.0f %%\n", score);
  Serial.printf("Bio Frequency   : %.2f Hz\n", freq);
  Serial.printf("Seismic Energy  : %.4f\n", seismicEnergy);
  Serial.printf("Acoustic Event  : %s\n", acousticEvent ? "DETECTED" : "NONE");
  Serial.printf("Human Flag      : %s\n", humanFlag ? "YES" : "NO");
  Serial.printf("Battery Level   : %d %%\n", battery);
  Serial.printf("Packet ID       : %d\n", packetID);

  if (humanFlag) {
    Serial.println("🚨 HIGH PROBABILITY HUMAN PRESENCE 🚨");
  }
}

// ──────────────────────────────────────────────────────────────
// CSV HELPER
// ──────────────────────────────────────────────────────────────
String nextValue(String &data) {
  int comma = data.indexOf(',');
  String value;

  if (comma == -1) {
    value = data;
    data = "";
  } else {
    value = data.substring(0, comma);
    data = data.substring(comma + 1);
  }
  return value;
}
