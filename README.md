# Python ESC Test Bench

## Overview

This project is a comprehensive test stand for Electronic Speed Controllers (ESCs), developed in Python. It features a web-based graphical user interface (GUI) built with the **NiceGUI** framework, allowing for real-time control and monitoring of an ESC connected to a Raspberry Pi.

The application is designed to be a versatile tool for testing and configuring ESCs used in drones, RC cars, and other robotics projects. It supports multiple communication protocols, including standard **PWM** (50Hz and 490Hz) and the digital **DShot300** protocol. The core logic is asynchronous and thread-safe, ensuring responsive control and safe hardware interaction through the use of the `pigpio` library.

---

## Features

* **Web-Based User Interface**: An intuitive GUI built with NiceGUI for easy operation from any web browser on the local network.
* **Multi-Protocol Support**:
    * **PWM**: Standard servo pulses (50Hz).
    * **PWM490**: Higher frequency PWM suitable for some ESCs.
    * **DShot300**: High-speed, robust digital protocol.
* **Real-Time Control**: Arm, disarm, start, stop, and pause test sequences with interactive buttons.
* **Configurable Test Parameters**: Easily adjust test duration, PWM pulse widths (idle, min, max), DShot throttle values, and signal ramp-up/down rates.
* **Safety First**:
    * System requires an explicit "ARM" action before starting the motor.
    * Includes an "EMERGENCY STOP" button for immediate shutdown.
    * Handles `pigpio` daemon disconnections gracefully with automatic emergency stop and reconnection attempts.
* **Asynchronous & Thread-Safe**: Utilizes a dedicated worker thread (`PigpioWorker`) to serialize all `pigpio` library calls, preventing race conditions and ensuring stable GPIO control.
* **DShot Waveform Generation**: Custom DShot frame generation using `pigpio`'s advanced waveform capabilities for precise timing.

---

## Technology Stack

* **Language**: Python
* **GUI Framework**: NiceGUI
* **GPIO Library**: pigpio
* **Hardware Platform**: Raspberry Pi

---

## Hardware Requirements

* A Raspberry Pi (any model with GPIO pins).
* An Electronic Speed Controller (ESC).
* A brushless motor compatible with the ESC.
* A suitable power supply for the ESC and motor.
* Basic wiring components (breadboard, jumper wires).

---

## Installation and Setup

### 1. Install pigpio

The `pigpio` library and daemon are required for hardware control.

```bash
# Install the library
sudo apt-get update
sudo apt-get install pigpio python3-pigpio

# Start the daemon with 1µs sampling rate (required for DShot)
sudo pigpiod -s 1
