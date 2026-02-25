# Step Detection System 🚶‍♂️📟

A wearable **ankle-mounted step detection system** that accurately counts human walking steps using **multi-sensor fusion**.  
The system combines **motion data** (accelerometer + gyroscope) and **ground contact data** (force sensor) with a **Finite State Machine (FSM)** and **anti-ghost-step validation logic** to eliminate false step detection caused by noise, vibrations, or static movements.

This project was developed as part of the **Logic Design Project – CO3091** at **Ho Chi Minh City University of Technology (HCMUT-OISP)**.

---

## 🎯 Project Objectives

- Accurately detect real walking steps
- Eliminate false detections ("ghost steps")
- Combine pressure + motion sensing for reliability
- Support real-time monitoring and data logging
- Design a wearable, compact, low-power device

---

## 🧠 Core Concepts

### Sensor Fusion
The system uses:
- **MPU6050 IMU** → Acceleration + Gyroscope data
- **FSR Pressure Sensor** → Ground contact detection

A step is only counted when **both motion and pressure conditions** are satisfied.

---

## ⚙️ Detection Logic

### Finite State Machine (FSM)

The gait cycle is modeled using 4 phases:

1. **Flat (Stance)** – Foot on ground  
2. **Lift (Pre-Swing)** – Foot leaving ground  
3. **Swing** – Foot in air (motion tracking phase)  
4. **Strike (Impact)** – Foot hits ground  

Only when the full sequence is completed is a step considered valid.

---

## 🛡️ Anti-Ghost Step Algorithm

A step is counted only if:

- **Horizontal acceleration threshold is exceeded**
- **Gyroscope rotation threshold is exceeded**
- **Pressure contact is confirmed**

This prevents false steps from:
- Foot tapping
- Stomping in place
- Vibrations
- Random movements
- Device shaking

---

## 🧩 Hardware Components

- Arduino Nano  
- MPU6050 (Accelerometer + Gyroscope)  
- FSR Force Sensitive Resistor  
- HC-06 Bluetooth Module  
- Micro SD Card Module  
- TP4056 Charging Module  
- MT3608 Boost Converter  
- 2× 18650 Li-ion Batteries  
- LEDs + Push Button  

---

## 📦 Features

✅ Real-time step detection  
✅ Multi-sensor fusion  
✅ FSM gait modeling  
✅ Anti-noise filtering  
✅ Anti-ghost-step validation  
✅ Bluetooth data transmission  
✅ SD card data logging  
✅ Wearable ankle-mounted design  
✅ Low power consumption  
✅ High detection accuracy (>99%)  

---

## 📁 Repository Structure

```text
📦 Step-Detection-System
 ┣ 📄 SOURCE_CODE.ino        # Arduino firmware
 ┣ 📄 Step Detection.pdf     # Full project report
 ┣ 📄 README.md              # Project documentation
