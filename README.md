# 🌡️ Solution & Incubator Multi-Arduino Control System

> A multi-Arduino based CO₂ and temperature control system with real-time monitoring and serial communication.

This project involves three Arduino microcontrollers working in harmony:
- One controls the **Solution Box** (CO₂ + temperature)
- One manages the **Incubator** (temperature)
- One acts as a **central display node**, collecting data via UART and visualizing it on an LCD

---

## 🧠 System Overview

### 🧪 Solution Box Arduino
- Measures and controls CO₂ levels and temperature
- Controls pumps/valves using PWM or digital pins
- Outputs sensor data via `Serial` to LCD Arduino

### 🐣 Incubator Arduino
- Controls temperature in an incubation chamber
- Outputs temperature data over `Serial`

### 🖥️ LCD Arduino (Central Display)
- Receives serial data from both other Arduinos
- Parses incoming data and displays on a 16x2 or 20x4 LCD
- Real-time user interface for monitoring

---

## 🚀 Features

- 🔁 Multi-Arduino serial communication (UART)
- 🔧 Real-time sensor readings from separate modules
- 🧩 Modular design (plug-and-play structure)
- 💬 LCD display interface (I2C or parallel)
- 📡 Scalable to more sensors or Arduinos

---

## 🔌 Hardware Used

| Component             | Description                        |
|----------------------|------------------------------------|
| Arduino Mega / Uno   | 3 units used for modular control   |
| SHT20 or similar     | For temperature sensing            |
| CO₂ sensor (NDIR)    | For Solution Box                   |
| 16x2 or 20x4 LCD     | With I2C Backpack (recommended)    |
| L298N / Relay Module | For valve / actuator control       |
| UART communication   | Between boards                     |
| 12V Power Supply     | For actuators and sensors          |

---

## 📁 File Structure

Incubator_System/
│
├── incubator_CO2_temp_Control.ino       → Solution Box'un ana Arduino kodu
├── dataTransfer.py                      → LCD data transfer.
├── system_diagram.png                   → Fritzing bağlantı şeması (L298N)
├── lcd_DisplayDatas.ino/                → Display upcoming datas in LCD.

---

📷 System Wiring Overview:
![Wiring Diagram](docs/l298n-arduino_bb.png)

*This diagram illustrates the connections for the pump control system using the L298N module.*


---
## 👤 Author

**Hüseyin Bertan Acar**  
📧 bertan_acr@hotmail.com  
📍 İzmir / Turkey  
🔗 [LinkedIn](https://www.linkedin.com/in/huseyin-bertan-acar/)

---
📄 License:
MIT License – free to use for personal or academic purposes.  
Commercial use allowed with proper attribution.

