# 🤖 4-Wheel Mecanum Drive System  
*A Complete Implementation Guide*

---

## 📘 Overview

This repository documents the design, logic, and implementation of a **4-Wheel Mecanum Drive System** – enabling omnidirectional motion for robotics applications.  
It covers everything from **kinematics derivation** and **field-oriented control** to **PID rotation stabilization** and **UART-based communication**.

If it makes your robot **move, rotate, or drift perfectly sideways**, this is where it’s explained.

---

## 📑 Table of Contents

1. [Introduction](#1-introduction)
2. [System Overview](#2-system-overview)
3. [Communication Architecture](#3-communication-architecture)
4. [Hardware Description](#4-hardware-description)
5. [Mecanum Drive Kinematics](#5-mecanum-drive-kinematics)
6. [Local Frame Control](#6-local-frame-control)
7. [Global (Field-Oriented) Control](#7-global-field-oriented-control)
8. [PID Control for Rotation](#8-pid-control-for-rotation)
9. [Integration Flow](#9-integration-flow)
10. [UART Communication Design](#10-uart-communication-design)
11. [Common Issues & Fixes](#11-common-issues--fixes)
12. [Rejected Implementations](#12-rejected-implementations)
13. [Final Equations](#13-final-equations-clean-summary)
14. [Appendices](#14-appendices)
15. [Maintainers & License](#15-maintainers--license)
- [📚 Resources](#-resources)

---

## 1. Introduction

### 🔹 What is a Mecanum Drive?

A **Mecanum drive** uses specially angled wheels (usually 45° rollers) that allow movement in **any direction** — forward, sideways, or diagonal — without changing orientation.  
Each wheel contributes a unique motion vector, combining to give **full holonomic control**.

### 🔹 Why It’s Used

- Enables **precise omnidirectional movement**.  
- Ideal for **Robocon, FRC, and warehouse bots**.  
- Simplifies navigation since **translation and rotation are decoupled**.

### 🔹 Real-World Applications

- Amazon Kiva robots  
- RoboCup & FRC competition bots  
- Hospital autonomous trolleys  
- Defense/inspection robots

---

## 2. System Overview

### 🔹 Hardware Components

| Component                   | Role                                                          |
|-----------------------------|---------------------------------------------------------------|
| **ESP32**                   | Reads PS4 controller input and sends data to Teensy via UART. |
| **Teensy 4.1 / Arduino**    | Main controller for motion kinematics and PID control.        |
| **BNO055 IMU**              | Provides absolute orientation (yaw angle θ).                  |
| **BTS7960 Drivers (x4)**    | High-current motor drivers.                                   |
| **PS4 Controller**          | Wireless joystick interface.                                  |

*The major Difference betweent the Teensy 4.1 and Arduino, apart from the obvious speed gap, being the Communication flow
Since Teensy 4.1 allows multiple I2C ports, the communication between ESP32 and Teensy 4.1 can be I2C instead of UART which is slightly simpler to implement and faster2*

### 🔹 Communication Flow

```plaintext
[PS4 Controller]
        ↓ (Bluetooth)
      [ESP32]
        ↓ (UART)
      [Teensy MCU]
        ↓ (PWM)
   [BTS7960 Drivers]
        ↓
     [Motors]
        ↑
     [BNO055 IMU Feedback]
````

---

## 3. Communication Architecture

### 🔹 UART Data Exchange

The **ESP32** transmits joystick data to the Teensy using a structured packet.

**Example Packet Format:**

```
[START_BYTE][Vx][Vy][L2][R2][END_BYTE]
```

Each field is `int8_t` (−127 to +127) for efficient transmission.

**Baud Rate:** 115200
**Protocol:** Start/End markers for reliable framing.

---

## 4. Hardware Description

| Module      | Description                                          |
| ----------- | ---------------------------------------------------- |
| **ESP32**   | Reads joystick input via Bluetooth, sends UART data. |
| **Teensy**  | Computes kinematics and motor PWM signals.           |
| **BNO055**  | Provides orientation (yaw angle θ).                  |
| **BTS7960** | Dual-channel motor driver for each wheel.            |
| **Motors**  | 4 Mecanum wheels with 45° rollers.                   |

---

## 5. Mecanum Drive Kinematics

### 🔹 Geometry & Axes

Let:

* `L` = half the robot length
* `W` = half the robot width
* `r` = wheel radius

### 🔹 Wheel Layout

```
      FRONT
   [FL]   [FR]
     \     /
      \   /
       \ /
       / \
      /   \
   [RL]   [RR]
      BACK
```

### 🔹 Inverse Kinematics

Robot velocity vector:

```
[ Vx ]
[ Vy ]
[ ω  ]
```

Wheel speed matrix:

```
[ ω_FL ]   1/r * [  1  -1  -(L+W) ] [ Vx ]
[ ω_FR ] =       [  1   1   (L+W) ] [ Vy ]
[ ω_RL ]         [  1   1  -(L+W) ] [ ω  ]
[ ω_RR ]         [  1  -1   (L+W) ]
```

---

## 6. Local Frame Control

### 🔹 Inputs

* Joystick X → `Vy` (sideways)
* Joystick Y → `Vx` (forward/backward)
* L2/R2 → `ω` (rotation)

### 🔹 Wheel Speed Equations

```cpp
FL = (Vx - Vy - ω*(L + W)) / r;
FR = (Vx + Vy + ω*(L + W)) / r;
RL = (Vx + Vy - ω*(L + W)) / r;
RR = (Vx - Vy + ω*(L + W)) / r;
```

### 🔹 Normalization

```cpp
maxVal = max(|FL|, |FR|, |RL|, |RR|);
if (maxVal > 1) {
  FL /= maxVal; FR /= maxVal;
  RL /= maxVal; RR /= maxVal;
}
```

---

## 7. Global (Field-Oriented) Control

### 🔹 Concept

Field-oriented control aligns robot movement to a **global reference frame** instead of the robot’s current heading.

### 🔹 Transformation

Let θ = yaw angle from the IMU.

```
VxL =  VxG * cosθ + VyG * sinθ
VyL = -VxG * sinθ + VyG * cosθ
```

This corrects for robot orientation when the driver moves “forward”.

---

## 8. PID Control for Rotation

### 🔹 Purpose

Maintains heading when no manual rotation is applied.

### 🔹 Formula

```
ω_PID = Kp * e + Kd * (de/dt)
```

Where `e = targetAngle - currentAngle`

### 🔹 Implementation

```cpp
error = targetAngle - currentAngle;
if (error > 180) error -= 360;
if (error < -180) error += 360;

derivative = (error - prevError) / dt;
omega_correction = Kp * error + Kd * derivative;
prevError = error;
```

---

## 9. Integration Flow

```
PS4 → ESP32 (UART) → Teensy
      ↓
   Parse Data
      ↓
Field-Oriented Conversion (θ)
      ↓
PID Correction (ω)
      ↓
Mecanum Kinematics
      ↓
PWM Output → BTS7960 → Motors
      ↑
     BNO055 Feedback
```

**Loop Frequency:** 100 Hz
**IMU Update:** 50–100 Hz
**UART Baud:** 115200

---

## 10. UART Communication Design

| Field      | Description      |
| ---------- | ---------------- |
| `Vx`       | Forward/Backward |
| `Vy`       | Sideways         |
| `L2`       | CCW Rotation     |
| `R2`       | CW Rotation      |
| Start/End  | Frame delimiters |

**ESP32 Transmission Example:**

```cpp
uint8_t packet[6] = {0xAA, Vy, Vx, L2, R2, 0x55};
Serial.write(packet, 6);
```

**Teensy Parsing Example:**

```cpp
if (Serial1.available() >= 6) {
  if (Serial1.read() == 0xAA) {
     Vy = Serial1.read();
     Vx = Serial1.read();
     L2 = Serial1.read();
     R2 = Serial1.read();
     if (Serial1.read() == 0x55) valid = true;
  }
}
```

---

## 11. Common Issues & Fixes

| Issue           | Cause                         | Fix                            |
| --------------- | ----------------------------- | ------------------------------ |
| Random motion   | Sign error in rotation matrix | Correct sin/cos signs          |
| Angle flipping  | IMU wraparound                | Normalize (−180° ↔ +180°)      |
| Lag             | IMU delay                     | Apply smoothing filter         |
| UART corruption | Noise/sync loss               | Use start/end bytes + checksum |
| Motor reversed  | Polarity mismatch             | Swap motor terminals           |

---

## 12. Rejected Implementations

| Rejected Method         | Reason                                |
| ----------------------- | ------------------------------------- |
| `θ = 90 - currentAngle` | Introduced phase error                |
| Global PID              | Caused instability during translation |
| Mixed frame control     | Inconsistent motion vectors           |

---

## 13. Final Equations (Clean Summary)

**Field-Oriented Conversion**

```
VxL =  VxG * cosθ + VyG * sinθ
VyL = -VxG * sinθ + VyG * cosθ
```

**Wheel Speeds**

```
ω_FL = VxL - VyL - ω(L+W)
ω_FR = VxL + VyL + ω(L+W)
ω_RL = VxL + VyL - ω(L+W)
ω_RR = VxL - VyL + ω(L+W)
```

**PID Correction**

```
ω_final = ω_input + ω_PID
```

---

## 14. Appendices

**📂 Datasheets**

* [BNO055 IMU](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
* [BTS7960 Motor Driver](https://components101.com/modules/bts7960-43a-motor-driver-module)

**📚 Libraries**

* `Adafruit_BNO055.h`
* `PS4Controller.h`
* `HardwareSerial.h`

**📖 References**

* FRC Whitepaper: *Mecanum Wheel Kinematics*
* ROS `mecanum_drive_controller` package

---

## 15. Maintainers & License
Use, modify, and share responsibly.

**License:** MIT
**Author:** [Atreya Rahegaonkar]
**Last Updated:** October 2025

---

## 📚 Resources

### 🧰 Tutorials
- [HashInclude Electronics – Mecanum Wheel Control](https://www.youtube.com/watch?v=dRysvxQfVDw&ab_channel=hashincludeelectronics)  
- [Rachel DeBarros – Stepper Motor Basics](https://www.youtube.com/watch?v=EEViXFoSzww&ab_channel=RachelDeBarros)  
- [Mecanum Wheel Robot – YouTube Tutorial](https://youtu.be/ibbc6kkuc_c)  
- [Instructables – How to Make a Mecanum Wheel Robot](https://www.instructables.com/How-to-Make-Mecanum-Wheel-Robot-and-Program-It-Cor/)

### ⚙️ Mecanum Drive References
- [Gavin Ford – Mecanum Wheel Explanation](https://www.youtube.com/watch?v=gnSW2QpkGXQ&ab_channel=GavinFord)  
- [Brogan M. Pratt – Omnidirectional Drive](https://www.youtube.com/watch?v=0k-Ey9bS9lE&ab_channel=BroganM.Pratt)  
- [RoboMaster – Mecanum Wheel Motion](https://www.youtube.com/watch?v=Xrc0l4TDnyw&ab_channel=RoboMaster)  
- [ResearchGate – Kinematic Model of a Four Mecanum Wheeled Mobile Robot](https://www.researchgate.net/publication/276344731_Kinematic_Model_of_a_Four_Mecanum_Wheeled_Mobile_Robot)  
- [IET Research – Mecanum Drive Kinematics Study](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/tje2.70006)

### 🧭 Field Control & Orientation
- [Code Red Robotics – Field-Oriented Drive Guide](https://controls.coderedrobotics.com/programminglessons/11.html)  
- [Kauai Labs – Field-Oriented Drive Examples](https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/)  
- [RoboteQ – Driving Mecanum Wheels](https://www.roboteq.com/applications/all-blogs/5-driving-mecanum-wheels-omnidirectional-robots)  
- [Game Manual 0 – FTC Mecanum Drive Tutorial](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)

### 📄 Research Papers
- [Holonomy Wheeled Robots Kinematics (PDF)](16311-4-Holonomy-WheeledRobots-Kinematics_compressed.pdf)  
- [IEEE Paper on Mecanum Drive Systems (PDF)](pxc3901586.pdf)

---

> *This document is a comprehensive resource created through multiple iterations and tests.
> Designed for both learners and engineers implementing reliable Mecanum drive systems.*

```

---
