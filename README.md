# project-WAVe: Vibroacoustic Measurement System for Flapping-Wing Drones

## Overview
**project-WAVe** (Wing Acoustic & Vibroacoustic Experiment) is a custom wireless measurement system designed for flapping-wing drones. This repository contains the hardware design, firmware, and software for a high-fidelity data acquisition platform, specifically tailored as a lightweight expansion deck for the [Flapper Nimble+](https://flapper-drones.com/product/flapper-nimble/) platform.

It serves as a foundation for collecting large-scale vibroacoustic datasets required to develop future machine learning-based Fault Detection and Diagnosis (FDD) algorithms.

## Key Features
* **Miniaturized Custom Hardware**: A custom 4-layer PCB (25.2 x 27.3 mm) designed to fit the standard Bitcraze Crazyflie expansion deck footprint.
* **High-Fidelity Inertial Tracking**: LSM6DS3TR-C 6-DOF IMU capturing 3-axis acceleration and angular velocity via SPI.
* **Precision Acoustic Acquisition**: INMP441 digital MEMS microphone via I2S for 24-bit resolution acoustic recordings.
* **Real-Time Wireless Streaming**: ESP32-C3 microcontroller actings as a TCP server, streaming data over 2.45 GHz Wi-Fi.
* **Concurrent Processing**: Efficient use of DMA, dual-buffers, and hardware interrupts for simultaneous data collection and transmission.

## Repository Structure
The project is organized into three technical domains:

### [WAVe-PCB](./WAVe-PCB)
Contains the electronics design files (KiCad).
* **MCU**: ESP32-C3FH4 SoC.
* **Sensors**: LSM6DS3TR-C IMU & INMP441 Microphone.
* **Power**: TPS566238RQFR buck converter for stable 3.3V delivery.

### [data-collection](./data-collection)
Embedded firmware developed using PlatformIO and the ESP-IDF framework.
* Manages low-level task scheduling (FreeRTOS).
* Handles interrupt-driven sensor readings and network transmission.
* Optimized for high-frequency IMU and acoustic data streaming.

### [data-processing](./data-processing)
Python client for real-time data reception and analysis.
* Utilizes TCP sockets to receive high-bandwidth data frames.
* Implements SciPy for real-time parsing, FFT, and STFT spectrogram visualizations.
* Specifically adapted for capturing 3-second IMU frames for diagnostic analysis.

## Experimental Validation
The system has been successfully validated through:
1. **Bench Testing**: Verification of sensor fidelity and wireless throughput.
2. **In-Flight Trials**: Successful capture of distinct cyclostationary vibroacoustic signatures during hover and forward flight maneuvers.
3. **Decoupled Architecture**: High-bandwidth data handling without introducing latency to the drone's primary flight controller.

## Authors
* [Marco Wiktor Santoro](https://github.com/Wik19)
* [Olaf Bukowski](https://github.com/olaf-bukowski)
