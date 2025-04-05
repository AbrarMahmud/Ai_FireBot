# 🔥 Fire-Bot: Image Processing-Based Fire Detection and Response Robot

This project presents the design and implementation of **Fire-Bot**, an image processing-based autonomous firefighting robot. The system addresses the complex challenge of real-time fire detection and response using a low-powered microcontroller, onboard stereoscopic cameras, PID-controlled motion, and integrated IMU feedback for precise mobility. The solution brings together **machine learning**, **control systems**, **real-time image processing**, and **robotics** in a single cohesive system.

---

## 🚀 Project Highlights

- 🔍 **Real-Time Fire Detection** using an **optimized ML model** deployed on a **248kB microcontroller**.
- 🤖 **Autonomous Navigation** with **discrete-time PID control** coded from scratch.
- 🎥 **Stereoscopic Distance Estimation** using dual ESP32-CAMs with custom frame synchronization.
- 🧭 **IMU Feedback Integration** using **MPU6050 DMP** for orientation correction.
- 🔗 **Custom Communication Protocol** for transmitting image data with structured identifiers (camera ID + frame number).
- 🧠 Efficient deployment of **FOMO (Fast Object Detection Model)** on edge hardware.

---

## 📂 Repository Structure

```bash
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
