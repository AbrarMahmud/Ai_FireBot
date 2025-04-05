# 🔥 Fire-Bot: FOMO-vision model based autonomous robot for fire detection and suppression

This project presents the design and implementation of **Fire-Bot**, an image processing-based autonomous firefighting robot. The system addresses the complex challenge of real-time fire detection and response using a low-powered microcontroller, onboard stereoscopic cameras, PID-controlled motion, and integrated IMU feedback for precise mobility. The solution brings together **machine learning**, **control systems**, **real-time image processing**, and **robotics** in a single cohesive system.

---
<div align="center">
  <img src="https://github.com/AbrarMahmud/Ai_FireBot/blob/master/source%20code/strutural_view.jpeg" alt="github-small" width="80%">
</div>


## 🚀 Project Highlights

- **Real-Time Fire Detection** using an **optimized ML model** deployed on a **248kB microcontroller**.
- **Autonomous Navigation** with **discrete-time PID control** coded from scratch.
- **Stereoscopic Distance Estimation** using dual ESP32-CAMs with custom frame synchronization.
- **IMU Feedback Integration** using **MPU6050 DMP** for orientation correction.
- **Custom Communication Protocol** for transmitting image data with structured identifiers (camera ID + frame number).
- Efficient deployment of **FOMO (Fast Object Detection Model)** on edge hardware.

---

## 📂 Repository Structure

<pre>
Fire-Bot/
├── report/
│   └── Fire-Bot_Project_Report.pdf  # Contains detailed documentation, methodology, and experimental analysis
└── source code/
    ├── Camera/
    │   ├── esp32_camera_one/
    │   │   └── camera_one_code.ino  # ESP32-CAM node 1 with object detection
    │   ├── esp32_camera_two/
    │   │   └── camera_two_code.ino  # ESP32-CAM node 2 with stereo calibration
    │   └── src/
    │       └── fire_detection_fomo.tflite  # Optimized TFLite ML model for fire detection
    ├── Rover/
    │   ├── Rover.ino                # Main rover code: diff drive + stereo camera manager + IMU PID feedback
    │   └── pins_me.h                # Pin mapping and hardware definitions

</pre>

## 🧠 Machine Learning Model

- Model: [**FOMO** (Fast Object Detection Model)](https://docs.edgeimpulse.com/docs/edge-impulse-studio/learning-blocks/object-detection/fomo-object-detection-for-constrained-devices)
- Optimized and quantized for deployment on **ESP32-CAM** (approx. 248kB footprint)
- Detects fire/flame regions in real-time under diverse lighting conditions

---

## ⚙️ Hardware Used

- 2 × **ESP32-CAM** modules for stereoscopic imaging
- **MPU6050** with DMP support for orientation data
- Custom-built **Differential Drive Rover** with geared motors
- Power-efficient **ESP-32 Dev microcontroller** (no external GPU/CPU needed)

---

## 🔄 Control System

- A **discrete-time PID controller** built from scratch
- Corrects for orientation drift using MPU6050 DMP data
- Ensures stable and accurate movement towards detected fire source

---

## 🧪 Data Handling & Communication

- Custom image frame packet includes:
  - `Camera ID`
  - `Frame Number`
- Data transmitted over **ESP-Now Data Transmission Protocol**
- Stereo vision logic aligns frames from both cameras to estimate fire distance

---

## 📚 How to Run

### Steps

1. **Flash ESP32-CAMs**:
    - Upload `esp32_camera_one` to one ESP32-CAM
    - Upload `esp32_camera_two` to the second ESP32-CAM

2. **Flash the Rover**:
    - Upload `Rover.ino` to the rover's MCU
    - Ensure correct IMU pin mapping via `pins_me.h`

3. **Connect Hardware**:
    - Mount and align cameras for stereoscopic vision
    - Power up the system

4. **Test & Debug**:
    - Use Serial Monitor to view distance estimates and PID outputs
    - Tune PID gains as needed

---

## 📄 Report

You can find the full project documentation including:

- System architecture
- ML model training & optimization
- PID tuning
- Stereo calibration
- Experimental results

In the [`report/Fire-Bot_Project_Report.pdf`](./report/Fire-Bot_Project_Report.pdf) file.

---

## 💡 Future Improvements

- Thermal camera integration
- SLAM-based navigation
- Wireless streaming of camera data

---

## 🤝 Contributing

Feel free to open issues or contribute to the codebase!

```bash
git clone https://github.com/AbrarMahmud/Ai_FireBot.git



