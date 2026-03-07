# ESP32 Smart Pump Control (Premium Version)

An advanced Automatic Pump Controller with Voltage Protection, Dry-Run Safety, and Cloud Dashboard integration.

## 🚀 One-Click Web Installer
You can install the firmware directly from your browser:
**[👉 Launch Web Installer](https://aungmoekhine.github.io/smart_water_pump-control/)**
*(Note: Replace the link above with your actual GitHub Pages URL once enabled)*

## 📑 Project Features
- **Automatic Water Level Control:** Uses ultrasonic sensors for precision.
- **Voltage Guard:** Protection against high (>250V) and low (<170V) voltage.
- **Dry-Run Safety:** Automatically stops the pump if no water flow is detected.
- **Dual Dashboards:** Local Web Dashboard (via IP) and Cloud Dashboard (MQTT).
- **TFT Display:** Real-time monitoring on the device hardware.

## 📂 Repository Structure
- `index.html`: The main Web Installer page (GitHub Pages).
- `ESP32_Auto_Pump_Control.ino`: Complete source code for Arduino IDE.
- `User_Guide.md`: Detailed setup and hardware wiring instructions.
- `cloud_control.html`: Standalone Cloud Dashboard interface.
- `Star_Cloud_App.bat`: Desktop launcher for the cloud app.
- `OTA_Ready_Files/`: Files for Over-The-Air firmware updates.
- `ESP32_Auto_Pump_Control.ino.merged.bin`: The pre-compiled firmware for the installer.

## 🛠️ Installation Requirements
- **Hardware:** ESP32-S3 (N16R8 recommended).
- **Driver:** Make sure you have the CP210x or CH343 drivers installed for your ESP32.
- **Browser:** Use Chrome, Edge, or Opera (support WebSerial).

## 📄 License
This project is provided for personal and educational use. Developed by **Aung Moe Khine**.



