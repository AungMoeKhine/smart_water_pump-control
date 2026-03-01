# Smart Pump Control - User Guide

Welcome to your **Smart Pump Control System**! This guide will help you set up, configure, and manage your automatic pump controller.

## Table of Contents
1. [Initial Setup](#initial-setup)
2. [Hardware Overview](#hardware-overview)
3. [TFT Dashboard](#tft-dashboard)
4. [Web & Cloud Dashboards](#web--cloud-dashboards)
5. [Safety Features & Logic](#safety-features--logic)
6. [Troubleshooting](#troubleshooting)

---

## Initial Setup

When you first power on the device, it needs to connect to your WiFi network.

1. **Power On** the device.
2. If it cannot connect to a known network, it will enter **Setup Mode**.
   - The screen will display: **"Auto-Pump-Config"** as the AP Name.
3. On your phone or computer, search for the WiFi network:
   > **`Auto-Pump-Config`** (Password: `12345678`)
4. Once connected, open a browser and go to:
   > **`192.168.4.1`**
5. Go to **Settings** and enter your home WiFi details.
6. Set your **Tank Height** and **Voltage Thresholds** as needed.
7. Click **Save & Reboot**.

---

## Hardware Overview

*   **Controller:** ESP32-S3.
*   **Sensors:** 
    *   Ultrasonic Sensor (for water level).
    *   Flow Sensor (for dry-run protection).
    *   Voltage Sensor (for system protection).
*   **Actuators:** Motor Relay & Buzzer for alarms.
*   **Visuals:** TFT Color Touchscreen & RGB System Status LED.

---

## TFT Dashboard

*   **Water Level:** Central display showing percentage and visual fill.
*   **Voltage:** Real-time RMS voltage reading.
*   **Pump Status:** Large ON/OFF indicator.
*   **System Info:** Displays messages like "FLOW_DETECTED", "FLOW_CHECKING", or "SYSTEM_STANDBY".
*   **LED Indicators:**
    *   **Green:** Connected to WiFi & Cloud.
    *   **Cyan:** Connected to WiFi only.
    *   **Orange:** Pump is Running.
    *   **Flashing Red:** Alarm/Error (Check screen for details).
    *   **Pulsing Blue:** Searching for WiFi/AP Mode.

---

## Web & Cloud Dashboards

Access your pump status from anywhere using the same local network or via the Cloud.

### Local Web Portal
1. Open a browser on your local network.
2. Enter the device's **IP Address** (shown on the TFT boot screen).
3. **Features:** Live monitoring, Manual pump toggle, and Device settings.

### Cloud Dashboard (`cloud_control.html`)
1. Open `cloud_control.html` on your PC.
2. Enter your **Device ID** (found at the bottom of the local web portal or TFT screen).
3. **Features:** Remote control from anywhere with internet. Use `Star_Cloud_App.bat` for a standalone app experience.

---

## Safety Features & Logic

*   **Automatic Pumping:** Starts when level < 20% and stops when level reaches "FULL" (set by Tank Height).
*   **Voltage Protection:** Automatically stops the pump if voltage is too high (>250V) or too low (<170V).
*   **Dry-Run Protection:** If the pump is ON but no flow is detected within 60s, it enters **DRY_RUN_ALARM**.
    *   After 1 minute of alarm, it enters **DRY_RUN_LOCKED**.
    *   **How to Reset:** Use the "Reset Alarm" button on the Dashboard.
*   **Sensor Error:** If the ultrasonic sensor fails, the buzzer will sound until silenced or fixed.

---

## Troubleshooting

### "PUMP: OFF" (Blocked Action)
The system will block manual starts if:
*   Voltage is out of safe range.
*   The tank is already full.
*   There is a sensor error.

### "DRY_RUN_ERROR"
*   Check if the water source has run out.
*   Check for leaks or pipe blockages.
*   Check if the flow sensor is working correctly.

---
*Created by Aung Moe Khine*
