#include <Wire.h>
#include <U8x8lib.h>
#include <SoftwareSerial.h>

// LAB SECURITY - Arduino Uno hardware controller
// ESP32-CAM GPIO14 TX -> Uno D10 RX
// Uno D11 TX -> ESP32-CAM GPIO13 RX through a 5V-to-3.3V level shifter/divider
// Connect grounds together. Never connect Uno D11 directly to ESP32 GPIO13.

const byte IR_PIN = 2;
const byte GREEN_LED_PIN = 4;
const byte RED_LED_PIN = 5;
const byte BUZZER_PIN = 6;
const byte VIBRATION_PIN = 7;
const byte ESP_RX_PIN = 10;
const byte ESP_TX_PIN = 11;

const int IR_NEAR_LEVEL = LOW;       // Change to HIGH if your IR module is active-HIGH
const int VIB_ACTIVE_LEVEL = HIGH;   // Change to LOW if your vibration module is active-LOW
const unsigned long VIBRATION_HOLD_MS = 2000;
const unsigned long SENSOR_SEND_MS = 500;
const unsigned long ACCESS_DISPLAY_MS = 5000;
const unsigned long ACCESS_LED_MS = 3000;

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN); // RX, TX
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

int tamperStatus = 0;
int vibrationStatus = 0;
int systemState = 0; // 0 normal, 1 tamper, 2 vibration, 3 both
int previousSensorAlarm = 0;
unsigned long vibrationStart = 0;
unsigned long lastSensorSend = 0;
unsigned long accessDisplayUntil = 0;
unsigned long accessLedUntil = 0;
bool accessEventActive = false;
bool lastAccessAllowed = false;
float lastMatch = 0.0f;
int lastReference = 0;
String cameraStatus = "WAITING";
String fsStatus = "WAITING";
int refsLoaded = 0;

void showNormalScreen() {
  oled.clear();
  oled.drawString(0, 0, "LAB SECURITY");
  oled.drawString(0, 2, "CAM:");
  oled.drawString(5, 2, cameraStatus.c_str());
  oled.drawString(0, 3, "REFS:");
  char refs[12];
  snprintf(refs, sizeof(refs), "%d/3", refsLoaded);
  oled.drawString(6, 3, refs);
  oled.drawString(0, 5, tamperStatus ? "TAMPER ALERT" : "TAMPER: NORMAL");
  oled.drawString(0, 6, vibrationStatus ? "VIBRATION ALERT" : "VIB: NORMAL");
  oled.drawString(0, 7, systemState == 0 ? "SYSTEM READY" : "CHECK SECURITY");
}

void showAccessScreen(bool allowed, float match, int referenceNumber) {
  oled.clear();
  oled.drawString(0, 0, "LAB SECURITY");
  if (allowed) {
    oled.drawString(0, 2, "ACCESS ALLOWED");
  } else {
    oled.drawString(0, 2, "ACCESS DENIED");
  }
  char line[17];
  snprintf(line, sizeof(line), "MATCH: %5.1f%%", match);
  oled.drawString(0, 4, line);
  snprintf(line, sizeof(line), "REFERENCE: %d", referenceNumber);
  oled.drawString(0, 5, line);
  oled.drawString(0, 7, "THRESHOLD: >69%");
}

void beepAllowed() {
  tone(BUZZER_PIN, 880, 120);
  delay(160);
  tone(BUZZER_PIN, 1175, 150);
  delay(180);
  noTone(BUZZER_PIN);
}

void beepDenied() {
  tone(BUZZER_PIN, 330, 250);
  delay(300);
  tone(BUZZER_PIN, 262, 350);
  delay(400);
  tone(BUZZER_PIN, 220, 450);
  delay(500);
  noTone(BUZZER_PIN);
}

void handleAccessEvent(char *line) {
  char *token = strtok(line, ","); // A
  token = strtok(NULL, ",");      // ALLOWED / DENIED
  if (!token) return;
  bool allowed = (strcmp(token, "ALLOWED") == 0);

  token = strtok(NULL, ",");
  float match = token ? atof(token) : 0.0f;
  token = strtok(NULL, ",");
  int referenceNumber = token ? atoi(token) : 0;

  lastAccessAllowed = allowed;
  lastMatch = match;
  lastReference = referenceNumber;
  accessEventActive = true;
  accessDisplayUntil = millis() + ACCESS_DISPLAY_MS;
  accessLedUntil = millis() + ACCESS_LED_MS;
  showAccessScreen(allowed, match, referenceNumber);

  // Physical alert feedback: allowed = green + two high beeps;
  // denied = red + three low beeps.
  digitalWrite(GREEN_LED_PIN, allowed ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, allowed ? LOW : HIGH);
  if (allowed) beepAllowed(); else beepDenied();
}

void handleEspMessage(char *line) {
  if (strncmp(line, "A,", 2) == 0) {
    handleAccessEvent(line);
    return;
  }

  // ESP32 status format: C,CAMERA_OK,FS_OK,3,192.168.x.x
  if (strncmp(line, "C,", 2) == 0) {
    char *token = strtok(line, ",");
    token = strtok(NULL, ",");
    if (token) cameraStatus = token;
    token = strtok(NULL, ",");
    if (token) fsStatus = token;
    token = strtok(NULL, ",");
    if (token) refsLoaded = atoi(token);
    if (!accessEventActive || (long)(millis() - accessDisplayUntil) >= 0) showNormalScreen();
  }
}

void readEspSerial() {
  static char line[96];
  static byte index = 0;
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[index] = '\0';
      if (index > 0) handleEspMessage(line);
      index = 0;
    } else if (index < sizeof(line) - 1) {
      line[index++] = c;
    } else {
      index = 0;
    }
  }
}

void updateSensors() {
  int irValue = digitalRead(IR_PIN);
  int nearDetected = (irValue == IR_NEAR_LEVEL);
  tamperStatus = nearDetected ? 0 : 1; // Missing expected object = tamper

  int vibrationRaw = digitalRead(VIBRATION_PIN);
  if (vibrationRaw == VIB_ACTIVE_LEVEL) {
    if (vibrationStart == 0) vibrationStart = millis();
    vibrationStatus = (millis() - vibrationStart >= VIBRATION_HOLD_MS) ? 1 : 0;
  } else {
    vibrationStart = 0;
    vibrationStatus = 0;
  }

  systemState = (tamperStatus ? 1 : 0) | (vibrationStatus ? 2 : 0);
  int sensorAlarm = systemState != 0;

  if (sensorAlarm && !previousSensorAlarm) {
    tone(BUZZER_PIN, 900);
  } else if (!sensorAlarm && previousSensorAlarm) {
    noTone(BUZZER_PIN);
  }
  previousSensorAlarm = sensorAlarm;

  if (sensorAlarm) {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, (millis() / 250) % 2 ? HIGH : LOW);
  } else if ((long)(millis() - accessLedUntil) < 0) {
    digitalWrite(GREEN_LED_PIN, lastAccessAllowed ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, lastAccessAllowed ? LOW : HIGH);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH); // system ready
  }

  if (millis() - lastSensorSend >= SENSOR_SEND_MS) {
    lastSensorSend = millis();
    espSerial.print("S,");
    espSerial.print(tamperStatus);
    espSerial.print(',');
    espSerial.print(vibrationStatus);
    espSerial.print(',');
    espSerial.println(systemState);
  }

  if (accessEventActive && (long)(millis() - accessDisplayUntil) >= 0) {
    accessEventActive = false;
    showNormalScreen();
  } else if (!accessEventActive && (millis() % 2000 < 30)) {
    showNormalScreen();
  }
}

void setup() {
  pinMode(IR_PIN, INPUT);
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  noTone(BUZZER_PIN);

  espSerial.begin(9600);
  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.clear();
  oled.drawString(0, 0, "LAB SECURITY");
  oled.drawString(0, 3, "STARTING...");
  delay(500);
  showNormalScreen();
}

void loop() {
  readEspSerial();
  updateSensors();
  delay(5);
}
