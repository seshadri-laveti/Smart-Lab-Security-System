# 🔐 Smart Laboratory Security System

**An IoT-Based Laboratory Security and Monitoring System Using Arduino UNO and ESP32-CAM**

## 📌 About the Project

The Smart Laboratory Security System is an IoT-based security solution designed to monitor laboratory activities, detect unauthorized access attempts, and identify potential physical disturbances.

The system integrates Arduino UNO and ESP32-CAM for sensor monitoring, image comparison, security alerts, and web-based monitoring.

## 🛠️ Hardware Components

- Arduino UNO
- ESP32-CAM (AI Thinker)
- GC2145 Camera Sensor
- IR Sensor
- Vibration Sensor
- 0.96-inch OLED Display (SSD1306)
- Green LED
- Red LED
- Buzzer
- Resistors and jumper wires

## ⚙️ Working Principle

1. ESP32-CAM captures images.
2. Captured images are compared with three stored reference images.
3. Access is allowed when the highest pixel similarity score exceeds 69%.
4. Arduino UNO receives the access decision.
5. LEDs, buzzer and OLED provide physical feedback.
6. IR and vibration sensors continuously monitor laboratory security.
7. ESP32-CAM hosts a web dashboard for monitoring.

## 🔌 Circuit Diagram

![Circuit Diagram](Circuit_Diagram/Lab_Security_Circuit.png)

## 💻 Software Requirements

- Arduino IDE
- ESP32 Board Package
- U8x8lib
- SoftwareSerial
- LittleFS
- ESP32 Camera Library
- WiFi and WebServer Libraries


## 🚀 Future Enhancements

- Improved face recognition.
- Real-time security notifications.
- Access history and event logging.
- Advanced laboratory monitoring.
- Cloud-based data storage.

## 🌐 Project Domain

Internet of Things | Embedded Systems | Image Processing | Smart Security

## 📜 License

MIT License (if included in this repository).

---

**Smart Laboratory Security System**

Arduino UNO + ESP32-CAM | IoT | Embedded Systems
