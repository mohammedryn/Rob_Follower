# M S RAMAIAH INSTITUTE OF TECHNOLOGY
*(Affiliated to Visvesvaraya Technological University, Belagavi)*

**Department of Mechanical Engineering**

---

# PROJECT REPORT
## ON
# ROBOTIC PAYLOAD MOBILITY SYSTEM
### An Autonomous Human-Following Robot with Vision-Based Perception and Sensor Fusion

---

**Under the Guidance of**

**Dr. Sunith Babu L.**
Professor, Department of Mechanical Engineering
M S Ramaiah Institute of Technology, Bengaluru - 560054

**Academic Year 2024-25**

---

### Project Team

| Sl. No | Name | USN |
|--------|------|-----|
| 1 | Prathik R.N. | 1MS23ME088 |
| 2 | Rudra Vir Chauhan | 1MS23ME095 |
| 3 | Shauryaveer Singh Chauhan | 1MS23ME107 |
| 4 | Syed Azmaan Ali Madni | 1MS23ME119 |

---

## Abstract

The Robotic Payload Mobility System (RPMS) is an intelligent, autonomous mobile robot designed to autonomously follow a designated human target in real time while carrying lightweight payloads. The system addresses the practical challenge of manual load-carrying in dynamic environments such as educational institutions, industrial floors, and public spaces, by providing a low-cost, scalable robotic assistant.

The hardware platform consists of a four-wheel drive (4WD) chassis driven by four JGB37 DC gear motors controlled through two BTS7960 dual H-bridge motor drivers, with a Teensy 4.1 microcontroller serving as the real-time PWM execution layer. The perception stack is built on a Raspberry Pi 5 (8GB) single-board computer running a YOLOv8n object detection model compiled to the NCNN inference backend for maximum CPU throughput. Human detection and centroid extraction are performed on live video from a Raspberry Pi Camera Module 3, while absolute distance measurement and multi-zone obstacle detection are provided by a VL53L5CX 8x8 Time-of-Flight (ToF) imager over I2C.

A dual-channel PID control architecture translates detected human position (lateral pixel error) into angular steering velocity and ToF-derived distance error into linear speed commands. These commands are serialized over USB to the Teensy 4.1, which generates hardware-accurate PWM signals at 20 kHz to the BTS7960 drivers. The system achieves reliable human following at walking speeds with real-time obstacle avoidance, demonstrating that robust autonomous following behavior is achievable with commodity hardware at a total system cost under Rs. 15,000.

The report presents the complete system architecture, hardware and software design, implementation details, bill of materials, and a roadmap for future enhancements including encoder-based closed-loop motor control, re-identification across occlusions, and ROS2 integration.

**Keywords:** Human-Following Robot, YOLOv8n, NCNN, VL53L5CX, Raspberry Pi 5, Teensy 4.1, PID Control, BTS7960, Skid Steering, Sensor Fusion, Edge AI

---

## Table of Contents

1. [Chapter 1: Introduction](#chapter-1-introduction)
   - 1.1 [Background and Motivation](#11-background-and-motivation)
   - 1.2 [Project Overview](#12-project-overview)
   - 1.3 [Scope of the Report](#13-scope-of-the-report)
2. [Chapter 2: Literature Survey](#chapter-2-literature-survey)
   - 2.1 [Overview of Human-Following Robot Systems](#21-overview-of-human-following-robot-systems)
   - 2.2 [Vision-Based Detection and Tracking](#22-vision-based-detection-and-tracking)
   - 2.3 [Distance Sensing and Obstacle Avoidance](#23-distance-sensing-and-obstacle-avoidance)
   - 2.4 [Control Architectures](#24-control-architectures)
   - 2.5 [Identified Research Gaps](#25-identified-research-gaps)
3. [Chapter 3: Problem Statement and Objectives](#chapter-3-problem-statement-and-objectives)
   - 3.1 [Problem Statement](#31-problem-statement)
   - 3.2 [Objectives](#32-objectives)
   - 3.3 [Design Constraints and Specifications](#33-design-constraints-and-specifications)
4. [Chapter 4: System Architecture](#chapter-4-system-architecture)
   - 4.1 [High-Level Architecture](#41-high-level-architecture)
   - 4.2 [Hardware Topology](#42-hardware-topology)
   - 4.3 [Software Architecture](#43-software-architecture)
   - 4.4 [Control System Design](#44-control-system-design)
5. [Chapter 5: Hardware Components](#chapter-5-hardware-components)
   - 5.1 [Compute Unit: Raspberry Pi 5 (8GB)](#51-compute-unit-raspberry-pi-5-8gb)
   - 5.2 [Camera: Raspberry Pi Camera Module 3](#52-camera-raspberry-pi-camera-module-3)
   - 5.3 [Depth Sensor: VL53L5CX](#53-depth-sensor-vl53l5cx)
   - 5.4 [Real-Time Controller: Teensy 4.1](#54-real-time-controller-teensy-41)
   - 5.5 [Motor Drivers: BTS7960 Dual H-Bridge](#55-motor-drivers-bts7960-dual-h-bridge)
   - 5.6 [Drive Motors: JGB37 DC Gear Motors](#56-drive-motors-jgb37-dc-gear-motors)
   - 5.7 [Power Management: XY-3606 Buck Converter](#57-power-management-xy-3606-buck-converter)
   - 5.8 [Mechanical Chassis](#58-mechanical-chassis)
6. [Chapter 6: Software Implementation](#chapter-6-software-implementation)
   - 6.1 [Development Environment Setup](#61-development-environment-setup)
   - 6.2 [Teensy 4.1 Firmware](#62-teensy-41-firmware)
   - 6.3 [Vision Pipeline (Raspberry Pi 5)](#63-vision-pipeline-raspberry-pi-5)
   - 6.4 [VL53L5CX Integration](#64-vl53l5cx-integration)
   - 6.5 [PID Controller](#65-pid-controller)
   - 6.6 [Main Control Loop](#66-main-control-loop)
   - 6.7 [PID Gain Tuning Procedure](#67-pid-gain-tuning-procedure)
7. [Chapter 7: Results and Discussion](#chapter-7-results-and-discussion)
   - 7.1 [Detection Performance](#71-detection-performance)
   - 7.2 [ToF Sensor Performance](#72-tof-sensor-performance)
   - 7.3 [Following Performance](#73-following-performance)
   - 7.4 [Discussion of Limitations](#74-discussion-of-limitations)
8. [Chapter 8: Bill of Materials](#chapter-8-bill-of-materials)
   - 8.1 [Complete Component List](#81-complete-component-list)
9. [Chapter 9: Conclusion and Future Work](#chapter-9-conclusion-and-future-work)
   - 9.1 [Conclusion](#91-conclusion)
   - 9.2 [Future Work](#92-future-work)
10. [References](#references)

---

## Chapter 1: Introduction

### 1.1 Background and Motivation

The proliferation of robotics in everyday environments has created demand for systems that can interact naturally with humans — not merely executing preprogrammed tasks in structured settings, but adapting in real time to the movement and intent of people in dynamic, unstructured spaces. Human-following robots represent a fundamental class of human-robot interaction (HRI) that enables a robot to autonomously track and accompany a designated person, maintaining a safe following distance while avoiding obstacles.

The most direct application of this capability is payload assistance: relieving individuals of the physical burden of carrying loads in environments where trolleys are impractical and human assistants are unavailable. Specific use cases include students carrying materials across a campus, technicians transporting tools and components on a factory floor, shoppers in retail environments, and patients or elderly individuals in care facilities.

Despite the evident utility of such systems, commercially available solutions remain prohibitively expensive for most small-scale deployments. Research-grade systems typically rely on depth cameras, 3D LiDAR, and high-performance compute platforms that inflate costs far beyond what is viable for institutional or individual adoption. This project demonstrates that effective autonomous human-following is achievable with a carefully selected stack of commodity components, intelligent algorithm choices, and a well-structured software architecture.

### 1.2 Project Overview

The Robotic Payload Mobility System (RPMS) is a four-wheel skid-steered mobile robot that detects and follows a human target using a combination of monocular camera-based person detection and Time-of-Flight depth sensing. The robot is designed for indoor environments with flat flooring and targets following distances of 0.5 to 2.0 meters at walking speeds up to 1.5 m/s.

The system is structured as a two-tier compute architecture: a Raspberry Pi 5 running all perception and control logic, and a Teensy 4.1 microcontroller handling hard real-time PWM motor actuation. This separation isolates the latency-sensitive motor control loop from the compute-intensive inference pipeline, ensuring reliable drive behavior regardless of inference frame rate fluctuations.

### 1.3 Scope of the Report

This report covers the complete design, implementation, and evaluation of the RPMS. It is organized as follows:

- **Chapter 2** surveys existing literature on human-following robots, identifying relevant techniques and gaps.
- **Chapter 3** formalizes the problem statement and defines the project objectives.
- **Chapter 4** presents the complete system architecture — hardware topology, software pipeline, and control system design.
- **Chapter 5** provides detailed specifications and integration notes for each hardware component.
- **Chapter 6** documents the software implementation in full, including vision pipeline, ToF integration, PID controller, and Teensy firmware.
- **Chapter 7** presents results, performance analysis, and discussion.
- **Chapter 8** provides the complete bill of materials with cost analysis.
- **Chapter 9** concludes with a summary and roadmap for future work.

---

## Chapter 2: Literature Survey

### 2.1 Overview of Human-Following Robot Systems

The problem of autonomous person following has been studied extensively over the past two decades. Islam et al. (2018) provided a comprehensive categorical survey of person-following autonomous robots, classifying systems by sensing modality, detection algorithm, and control strategy. Their survey identifies three primary sensing approaches: appearance-based (using RGB cameras), geometry-based (using depth sensors), and combined fusion approaches. They conclude that fusion approaches consistently outperform single-modality systems in dynamic, cluttered environments — a finding that directly motivates the sensor fusion design of the present work.

### 2.2 Vision-Based Detection and Tracking

Computer vision remains the dominant approach for human detection in mobile robotics. Traditional approaches using Histogram of Oriented Gradients (HOG) combined with Support Vector Machine (SVM) classifiers were widely used in early systems but suffer from high false-positive rates and poor performance under occlusion. Modern deep learning-based detectors, particularly the YOLO (You Only Look Once) family of architectures, have substantially advanced the state of the art.

Gao et al. (2025) benchmarked multiple YOLO model variants specifically for human-robot interaction tasks, evaluating accuracy, inference speed, and robustness on edge compute platforms. Their findings indicate that YOLOv8n achieves the optimal accuracy-latency tradeoff for real-time HRI on single-board computers, achieving approximately 12-18 FPS on ARM Cortex-A72/A76 class processors when compiled to the NCNN inference backend — directly validating the model and backend choice in this project.

Sakri et al. (2024) demonstrated an autonomous person-following telepresence robot using a monocular camera and YOLO for detection, combined with bounding box centroid tracking. Their work establishes the viability of monocular centroid-based control for indoor following tasks, reporting stable following performance at distances of 0.5-3.0 meters — the same operating range targeted by the RPMS.

### 2.3 Distance Sensing and Obstacle Avoidance

Accurate distance measurement is essential for maintaining a safe following gap and preventing collisions. Ultrasonic sensors (HC-SR04) are the most commonly used low-cost option in hobbyist systems but suffer from narrow detection angles, susceptibility to ambient noise, and inability to detect objects outside a narrow cone.

LiDAR-based systems provide accurate 2D or 3D spatial mapping but at significantly higher cost. The Slamtec RPLIDAR series, priced at Rs. 8,000-20,000, is the minimum viable LiDAR option for indoor navigation and is considered high-end for student projects.

Time-of-Flight (ToF) sensors based on Single Photon Avalanche Diode (SPAD) arrays have emerged as a compelling middle ground. The STMicroelectronics VL53L5CX, used in this project, provides an 8x8 grid of distance measurements across a 65-degree diagonal field of view at up to 15 Hz over I2C — representing a 64-zone spatial depth map at a cost under Rs. 800. This capability significantly exceeds single-point ToF alternatives (VL53L1X) and ultrasonic sensors for obstacle detection, while remaining well within project budget constraints.

### 2.4 Control Architectures

PID (Proportional-Integral-Derivative) control is the most widely adopted control strategy for mobile robot trajectory following. Liu et al. (2023) applied PID control to a human-following robot in warehouse logistics contexts, demonstrating stable following with minimal overshoot when gains are tuned through iterative empirical methods. Acosta-Amaya et al. (2024) presented a lightweight two-layer control architecture for human-following robots, separating high-level trajectory planning from low-level motor control — an architectural principle reflected in the RPMS two-tier compute design.

Holkar et al. (2025) demonstrated voice-activated human following using computer vision, extending the basic following paradigm with command-based behavioral modes. While voice activation is not implemented in the current RPMS, the behavioral mode concept (follow, stop, seek) informs the state machine design discussed in Chapter 6.

### 2.5 Identified Research Gaps

The literature survey identifies the following gaps that the RPMS addresses:

- **High cost:** The majority of published systems rely on expensive depth cameras (Intel RealSense D435: Rs. 25,000+) or 3D LiDAR, limiting real-world deployment. The RPMS achieves comparable following functionality at approximately 15% of this cost.
- **Limited sensor fusion in low-cost models:** Most budget systems use either a camera or a single-point distance sensor, not both. The RPMS fuses 8x8 multizone ToF depth with camera-based detection, providing richer spatial awareness than either sensor alone.
- **Reliance on high-power compute:** Systems using GPU-based inference (Jetson Nano, Xavier) achieve high frame rates but consume 10-25W, requiring large battery systems. The RPMS runs YOLOv8n-NCNN on the Raspberry Pi 5's ARM Cortex-A76 at approximately 5-8W, extending battery life significantly.
- **Poor documentation of two-tier compute architectures:** Few student-grade projects clearly separate perception compute from real-time motor control, leading to timing issues. The RPMS explicitly separates these concerns across the RPi 5 and Teensy 4.1.

---

## Chapter 3: Problem Statement and Objectives

### 3.1 Problem Statement

Manual carrying of loads in dynamic environments — including educational campuses (textbooks, laboratory materials), industrial floors (tools, components, small assemblies), and retail or public spaces — imposes physical fatigue, increases the risk of musculoskeletal injury, and reduces the productivity of personnel who must divide attention between locomotion and task execution. Existing robotic solutions for payload assistance are either (a) too expensive for small-scale adoption, (b) require fixed infrastructure (tracks, markers, beacons), or (c) lack robust human-following capability in unstructured environments with multiple people and dynamic obstacles.

The problem addressed by this project is: **Design, build, and demonstrate a low-cost, infrastructure-free, autonomous mobile robot capable of reliably following a designated human operator in real-time indoor environments, maintaining a safe following distance, avoiding obstacles, and carrying lightweight payloads.**

### 3.2 Objectives

The specific technical objectives of the project are:

1. Design and fabricate a mechanically stable four-wheel drive robotic platform capable of supporting a payload of up to 2 kg while maintaining maneuverability in indoor corridors and open areas.
2. Implement a real-time human detection and tracking pipeline using YOLOv8n with NCNN backend on Raspberry Pi 5, achieving reliable person detection at distances of 0.5 to 3.0 meters at not less than 10 frames per second.
3. Integrate a VL53L5CX 8x8 Time-of-Flight sensor for accurate absolute distance measurement and multi-zone obstacle detection, fusing ToF data with camera-derived centroid information in the PID control loop.
4. Develop a dual-channel PID control system that independently regulates steering (based on lateral pixel error from frame center) and forward speed (based on distance error from set-point) to achieve smooth, stable human following.
5. Implement hard real-time PWM motor control on a Teensy 4.1 microcontroller with 20 kHz switching frequency, receiving velocity commands over USB serial from the Raspberry Pi 5 and driving four JGB37 motors through two BTS7960 H-bridge drivers.
6. Demonstrate end-to-end autonomous human following at walking speeds (up to 1.5 m/s) in a realistic indoor environment, with collision-free operation verified over a minimum 5-minute continuous test.
7. Document the complete system design, implementation, and performance evaluation in a form suitable for reproducibility and extension by future project teams.

### 3.3 Design Constraints and Specifications

The following constraints bound the design space:

**Table 3.1: Design Constraints and Target Specifications**

| Parameter | Target Value | Rationale |
|-----------|-------------|-----------|
| Total system cost | < Rs. 15,000 | Accessible to student teams |
| Following distance | 0.5 – 2.0 m | Safe, practical indoor range |
| Max following speed | 1.5 m/s | Normal human walking speed |
| Detection frame rate | >= 10 FPS | Adequate for walking speed |
| Payload capacity | Up to 2 kg | Textbooks, tools, small items |
| Operating environment | Indoor, flat floor | Scope limitation |
| Battery run time | > 30 minutes | Practical session length |

---

## Chapter 4: System Architecture

### 4.1 High-Level Architecture

The RPMS is organized as a three-layer architecture:

- **Perception Layer:** Camera Module 3 (IMX708) captures live video at 640x480 pixels; VL53L5CX samples 64-zone depth at 15 Hz. Both streams are processed on the Raspberry Pi 5.
- **Control Layer:** A dual-channel PID controller running on the Raspberry Pi 5 computes left and right motor velocity commands from the fused perception data.
- **Actuation Layer:** Commands are transmitted over USB serial to the Teensy 4.1, which executes hardware PWM to two BTS7960 motor drivers, each controlling two JGB37 motors via skid steering.

This two-tier compute separation is the defining architectural decision of the system. By offloading PWM generation to the Teensy 4.1, the Raspberry Pi 5 is freed from real-time scheduling constraints. Linux on the RPi 5 is not a real-time operating system; GPIO-based software PWM from a Linux process has millisecond-level jitter, causing audible motor noise and erratic speed control. The Teensy 4.1 generates PWM from dedicated hardware timers at nanosecond precision, entirely independent of Linux scheduling.

### 4.2 Hardware Topology

The physical interconnection of system components is as follows:

**Table 4.1: Hardware Interconnection Matrix**

| Source | Destination | Interface / Signal |
|--------|-------------|-------------------|
| Camera Module 3 | Raspberry Pi 5 | 22-pin MIPI CSI-2 (hardware ISP, zero CPU overhead) |
| VL53L5CX | Raspberry Pi 5 | I2C (GPIO 3/5, 400 kHz fast mode, addr 0x52) |
| Raspberry Pi 5 | Teensy 4.1 | USB Serial (115200 baud, ASCII command frames) |
| Teensy 4.1 | BTS7960 x2 | 6x PWM GPIO (20 kHz, 8-bit resolution) + Enable |
| BTS7960 x2 | JGB37 x4 | Bi-directional DC motor drive (up to 27V, 43A peak) |
| 3S LiPo (11.1V) | BTS7960 x2 | Direct power (motor rail) |
| XY-3606 Buck | Raspberry Pi 5 | Regulated 5.1V / 5A via USB-C PD |
| Teensy 4.1 | Self-powered | 5V from RPi5 USB-A port (max 600mA adequate for Teensy) |

### 4.3 Software Architecture

The Raspberry Pi 5 runs Raspberry Pi OS (Bookworm, 64-bit) with the following software stack:

- **picamera2:** Official Raspberry Pi camera library based on libcamera. Provides zero-copy DMA frame capture to NumPy arrays.
- **Ultralytics YOLOv8n (NCNN backend):** Person detection model converted to NCNN format for ARM NEON-optimized inference. Achieves 12-18 FPS at 640x480 on the Cortex-A76 cores of the RPi 5.
- **vl53l5cx-ctypes:** Python ctypes wrapper for the official STMicroelectronics VL53L5CX ULD C library. Provides 8x8 distance array at up to 15 Hz.
- **pyserial:** USB serial communication to Teensy 4.1. Sends ASCII command frames at 115200 baud.
- **PID controller:** Custom Python implementation with anti-windup integrator clamping, low-pass filtered derivative, and deadband zone.

### 4.4 Control System Design

#### 4.4.1 Steering PID Channel

The steering control error is defined as the horizontal pixel offset of the detected person's bounding box centroid from the image center:

```
error_steer = centroid_x - (FRAME_WIDTH / 2)
# Positive error = target is to the right of center
# Negative error = target is to the left of center

steer = Kp_s * error + Ki_s * integral + Kd_s * filtered_derivative
# steer > 0: turn right (increase right motor speed, decrease left)
# steer < 0: turn left (increase left motor speed, decrease right)
```

A deadband of 40 pixels around the frame center suppresses micro-corrections when the target is approximately aligned, preventing the robot from oscillating on a straight path. The integral term is clamped to prevent windup during target loss. The derivative term is low-pass filtered to attenuate pixel-level noise inherent in bounding box centroid estimates.

#### 4.4.2 Speed PD Channel

The speed control error is the difference between the measured following distance (from VL53L5CX center zone average) and the set-point following distance (800 mm):

```
error_speed = measured_distance_mm - SET_POINT_MM  # SET_POINT_MM = 800
# error_speed > 0: target is far, accelerate forward
# error_speed < 0: target too close, decelerate or reverse

speed = BASE_SPEED + Kp_d * error_speed + Kd_d * filtered_derivative
speed = clip(speed, -MAX_SPEED, MAX_SPEED)
```

> **Note:** The speed channel operates as a PD controller (Ki = 0.0) because steady-state distance offset is acceptably small without integral action, and integral windup during target loss would cause undesirable surge on re-acquisition.

An emergency stop is triggered when any ToF zone reads below 400 mm (STOP_DIST_MM), regardless of the PID output. This hard safety override prevents collisions with obstacles that enter the robot's path between detection cycles.

#### 4.4.3 Skid Steering Mix

Left and right motor velocities are computed by combining the speed and steering outputs:

```
left_pwm  = speed - steer   # clip to [-255, 255]
right_pwm = speed + steer   # clip to [-255, 255]
# Positive PWM = forward rotation
# Negative PWM = reverse rotation
```

This differential mix causes the robot to pivot when the steer term is large (target far off-center) and drive straight when steer is near zero (target centered). The skid steering mechanism — spinning same-side wheels at equal speed in opposite directions across sides — provides a mechanically simple turning mechanism well-suited to the 4WD chassis.

---

## Chapter 5: Hardware Components

### 5.1 Compute Unit: Raspberry Pi 5 (8GB)

The Raspberry Pi 5 is the central compute unit of the system, responsible for all perception processing, control logic, and system coordination. Key specifications relevant to this application:

**Table 5.1: Raspberry Pi 5 Key Specifications**

| Specification | Value |
|---------------|-------|
| Processor | Broadcom BCM2712, 4x ARM Cortex-A76 @ 2.4 GHz |
| RAM | 8 GB LPDDR4X @ 4267 MHz |
| Camera Interface | 2x 4-lane MIPI CSI-2 (22-pin connector) |
| I2C | Hardware I2C on GPIO 3 (SDA) / GPIO 5 (SCL), up to 400 kHz |
| USB | 2x USB 3.0, 2x USB 2.0 (serial to Teensy via USB) |
| Power Input | 5V via USB-C PD, 5A max (25W) |
| Idle power | ~3W |
| Peak power (inference) | ~8W |

The upgrade from Raspberry Pi 4 to Raspberry Pi 5 is significant for this application. The Cortex-A76 cores deliver approximately 2.5x the floating-point throughput of the Cortex-A72 in the Pi 4, enabling YOLOv8n-NCNN inference at 12-18 FPS versus approximately 5-7 FPS on Pi 4. The 8 GB RAM variant ensures zero memory pressure when running the full stack simultaneously (inference + ToF + serial + PID).

### 5.2 Camera: Raspberry Pi Camera Module 3

The Camera Module 3 uses the Sony IMX708 sensor and connects to the RPi 5 via MIPI CSI-2. It is natively supported by libcamera and the picamera2 Python library with zero driver configuration.

**Table 5.2: Raspberry Pi Camera Module 3 Specifications**

| Specification | Value |
|---------------|-------|
| Image Sensor | Sony IMX708, 1/2.43-inch optical format |
| Resolution | 12.3 MP (4608 x 2592 pixels) |
| Video | 1080p @ 50fps, 720p @ 100fps, 480p @ 120fps |
| Focus | Motorized autofocus (PDAF — Phase Detection AF) |
| Field of View | ~66 degrees diagonal (standard lens) |
| Interface | 22-pin MIPI CSI-2 (RPi 5 native) |
| HDR | Yes — handles mixed indoor lighting robustly |

The autofocus capability is particularly valuable for human following: as the target moves from 0.5m to 3m range, the PDAF system continuously adjusts focus, maintaining sharp images that improve YOLOv8n detection confidence. HDR mode handles the common indoor scenario of a window in the background that would otherwise overexpose the target. For the following application, autofocus is locked to approximately 1.5m at startup to eliminate focus hunting during the following session. The `LensPosition` parameter is in diopters (distance = 1 / LensPosition), so a value of 0.67 diopters corresponds to a focus distance of approximately 1.5m.

### 5.3 Depth Sensor: VL53L5CX

The VL53L5CX (STMicroelectronics) is a 64-zone direct Time-of-Flight ranging sensor based on a 940nm VCSEL laser and SPAD (Single-Photon Avalanche Diode) array. It provides an 8x8 matrix of independent distance measurements across a 65-degree diagonal field of view — effectively a low-resolution depth image.

**Table 5.3: VL53L5CX Specifications**

| Specification | Value |
|---------------|-------|
| Ranging zones | 8x8 = 64 independent zones |
| Field of View | 65 degrees diagonal |
| Range | 2 cm to 4 m (indoor, white target) |
| Range accuracy | +/- 5 mm (typical) |
| Interface | I2C (up to 1 MHz) |
| Default I2C address | 0x52 |
| Max frame rate | 15 Hz (all 64 zones) |
| Supply voltage | 3.3V (RPi 5 compatible) |

The 8x8 depth grid provides two critical capabilities beyond what a single-point ToF sensor offers: First, obstacle detection across the full frontal zone — a single-point sensor only detects obstacles directly ahead; the VL53L5CX detects a chair leg at 20 degrees off-center. Second, it differentiates the target person (center zone cluster at following distance) from nearer obstacles (peripheral zones at shorter range), enabling the obstacle avoidance behavior to avoid false stops.

### 5.4 Real-Time Controller: Teensy 4.1

The Teensy 4.1 serves as the PWM execution layer, receiving motor velocity commands from the Raspberry Pi 5 over USB serial and generating hardware-accurate PWM signals to the BTS7960 motor drivers.

**Table 5.4: Teensy 4.1 Specifications**

| Specification | Value |
|---------------|-------|
| Processor | ARM Cortex-M7 @ 600 MHz (NXP iMXRT1062) |
| PWM channels | 32 hardware PWM channels (FlexPWM + QuadTimer) |
| PWM frequency | Up to 66 MHz theoretical; set to 20 kHz for BTS7960 |
| PWM resolution | 8-bit (0-255) for this application |
| USB | USB 2.0 Full Speed (USB Serial CDC at 115200 baud) |
| GPIO voltage | 3.3V (level-shift if 5V BTS7960 logic is used) |
| Latency | < 1 ms command receive to PWM update |

The Cortex-M7 at 600 MHz is substantially overpowered for this application. Its inclusion is motivated by its outstanding PWM timer architecture (FlexPWM modules provide fully independent hardware control per channel), its familiarity to the team, and its reusability across multiple projects in the team's portfolio.

### 5.5 Motor Drivers: BTS7960 Dual H-Bridge

Two BTS7960 43A dual H-bridge motor driver modules provide the high-current motor drive. Each BTS7960 module contains two BTN7960 half-bridge ICs, enabling full bidirectional control of two DC motors with built-in current sensing and over-temperature protection.

**Table 5.5: BTS7960 Motor Driver Specifications**

| Specification | Value |
|---------------|-------|
| Peak current | 43A per channel |
| Continuous current | 15A per channel (with heatsinking) |
| Supply voltage | 6V – 27V |
| PWM frequency | Up to 25 kHz (20 kHz used here) |
| Logic voltage | 3.3V / 5V compatible |
| Protection | Over-current, over-temperature, under-voltage lockout |

**Configuration:** BTS7960 #1 drives the left side (front-left + rear-left JGB37 motors, both wired in parallel). BTS7960 #2 drives the right side (front-right + rear-right). This achieves true skid steering: both left-side wheels always turn at the same speed, and both right-side wheels always turn at the same speed, with differential speed between sides providing all turning.

### 5.6 Drive Motors: JGB37 DC Gear Motors

Four JGB37-520 DC gear motors provide the drive force. These are N20-format gearbox motors scaled for medium current applications.

**Table 5.6: JGB37 DC Gear Motor Specifications**

| Specification | Value |
|---------------|-------|
| Rated voltage | 12V (operated at 11.1V from 3S LiPo) |
| No-load speed | ~100-200 RPM (depending on gear ratio variant) |
| Stall torque | ~1.5 – 3.0 kg.cm |
| No-load current | ~150 mA |
| Stall current | ~1.5 A |
| Shaft | D-shaft, 4mm diameter |
| Encoders | None (standard variant) |

### 5.7 Power Management: XY-3606 Buck Converter

The XY-3606 DC-DC step-down converter module regulates the 11.1V 3S LiPo supply to 5.1V / 5A for the Raspberry Pi 5. Key specifications: input 4.5-30V, output adjustable 1.25-30V, rated output current 5A continuous with 6A peak, efficiency >93% at full load. Output is set to 5.1V (slightly above 5.0V to compensate for USB-C cable voltage drop under peak load). The 0.1V margin ensures the RPi 5 never experiences under-voltage throttling during inference peaks.

The motor rail (BTS7960 VCC) is powered directly from the LiPo without regulation — the BTS7960 accepts 6-27V and the JGB37 motors are rated for the 11.1V nominal voltage. Critically, the Raspberry Pi 5 is powered from a separate XY-3606 output with its own inductor and output capacitor, completely isolating it from motor switching noise. Motor PWM switching generates significant conducted EMI on the power supply rails; without this isolation, the RPi 5 would experience power supply noise-induced SD card corruption and unpredictable CPU throttling.

### 5.8 Mechanical Chassis

The chassis is constructed from acrylic plates for the horizontal deck layers and stainless steel L-brackets and standoffs for vertical support and motor mounting. This material combination provides adequate rigidity for the 2 kg payload target while keeping chassis weight below 0.8 kg.

- **Dimensions:** approximately 280mm (L) x 200mm (W) x 120mm (H)
- **Motor mounting:** four JGB37 motors mounted in a rectangular pattern, one at each corner, wheels outboard
- **Electronics deck:** RPi 5, Teensy 4.1, and VL53L5CX mounted on upper acrylic deck using M3 standoffs
- **Camera mounting:** Camera Module 3 on forward-facing acrylic bracket at approximately 150mm height
- **Battery compartment:** 3S LiPo and BTS7960 drivers mounted on lower deck

---

## Chapter 6: Software Implementation

### 6.1 Development Environment Setup

All Raspberry Pi 5 software runs on Raspberry Pi OS Bookworm (64-bit). The following packages are required:

```bash
# System packages
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-opencv -y

# Python packages
pip3 install picamera2 ultralytics pyserial vl53l5cx-ctypes

# Enable I2C for VL53L5CX
sudo raspi-config  # Interface Options -> I2C -> Enable

# Increase I2C speed to 400kHz
# Add to /boot/firmware/config.txt:
# dtparam=i2c_arm=on,i2c_arm_baudrate=400000

# Export YOLOv8n to NCNN (run once)
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='ncnn')"
```

### 6.2 Teensy 4.1 Firmware

The Teensy firmware implements a serial command parser, PWM motor control, and a **hardware watchdog** that stops all motors if no command frame is received within 500 ms. This ensures the robot cannot continue driving if the Raspberry Pi 5 process crashes, hangs, or the USB connection is interrupted.

Commands are ASCII frames of the form `'L<int>R<int>\n'` where integer values range from -255 (full reverse) to +255 (full forward).

```cpp
#include <Arduino.h>

// BTS7960 #1 (Left side motors)
#define L_RPWM  2    // Forward PWM
#define L_LPWM  3    // Reverse PWM
#define L_EN    4    // Enable (active HIGH)

// BTS7960 #2 (Right side motors)
#define R_RPWM  5    // Forward PWM
#define R_LPWM  6    // Reverse PWM
#define R_EN    7    // Enable (active HIGH)

// Watchdog: stop motors if no command received within this interval
#define WATCHDOG_MS 500

unsigned long lastCmdTime = 0;
bool motorsStopped = false;

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // Left side
  analogWrite(L_RPWM, left  >= 0 ? left  : 0);
  analogWrite(L_LPWM, left  <  0 ? -left : 0);

  // Right side
  analogWrite(R_RPWM, right >= 0 ? right : 0);
  analogWrite(R_LPWM, right <  0 ? -right: 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(L_EN, OUTPUT); digitalWrite(L_EN, HIGH);
  pinMode(R_EN, OUTPUT); digitalWrite(R_EN, HIGH);

  // Set all PWM channels to 20 kHz (above audible range)
  analogWriteFrequency(L_RPWM, 20000);
  analogWriteFrequency(L_LPWM, 20000);
  analogWriteFrequency(R_RPWM, 20000);
  analogWriteFrequency(R_LPWM, 20000);
  analogWriteResolution(8);  // 0-255 range

  lastCmdTime = millis();
}

void loop() {
  // Watchdog: halt motors if RPi5 stops sending commands
  if ((millis() - lastCmdTime) > WATCHDOG_MS) {
    if (!motorsStopped) {
      setMotors(0, 0);
      motorsStopped = true;
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int l_idx = cmd.indexOf('L');
    int r_idx = cmd.indexOf('R');
    if (l_idx >= 0 && r_idx > l_idx) {
      int left  = cmd.substring(l_idx + 1, r_idx).toInt();
      int right = cmd.substring(r_idx + 1).toInt();
      setMotors(left, right);
      lastCmdTime  = millis();
      motorsStopped = false;
    }
  }
}
```

> **Safety note:** The watchdog is critical. Without it, a crashed or hung Python process leaves the Teensy executing the last motor command indefinitely, causing the robot to drive uncontrolled.

### 6.3 Vision Pipeline (Raspberry Pi 5)

The vision pipeline uses picamera2 for frame capture and YOLOv8n-NCNN for person detection. The NCNN backend provides ARM NEON SIMD acceleration, achieving 2-3x faster inference than vanilla ONNX runtime on the Cortex-A76 cores.

```python
from picamera2 import Picamera2
from ultralytics import YOLO
import numpy as np

FRAME_W, FRAME_H = 640, 480

# Initialize camera
picam = Picamera2()
cfg   = picam.create_preview_configuration(
    main={'size': (FRAME_W, FRAME_H), 'format': 'RGB888'}
)
picam.configure(cfg)

# Lock autofocus at ~1.5m for stability during following.
# LensPosition is in diopters: focus_distance_m = 1 / LensPosition.
# LensPosition = 0.67 diopters -> 1 / 0.67 ≈ 1.5m focus distance.
picam.set_controls({'AfMode': 0, 'LensPosition': 0.67})
picam.start()

# Load NCNN model (exported once via YOLO.export(format='ncnn'))
model = YOLO('yolov8n_ncnn_model', task='detect')

def detect_person(frame):
    """Returns (centroid_x, bbox_area) for largest detected person,
    or (None, None) if no person detected."""
    results = model(frame, classes=[0], conf=0.50, verbose=False)
    boxes   = results[0].boxes.xyxy.cpu().numpy()  # [N, 4] xyxy
    if len(boxes) == 0:
        return None, None
    areas   = (boxes[:,2]-boxes[:,0]) * (boxes[:,3]-boxes[:,1])
    best    = boxes[np.argmax(areas)]
    cx      = int((best[0] + best[2]) / 2)
    area    = float(np.max(areas))
    return cx, area
```

> **Known limitation:** `detect_person()` selects the largest bounding box as the target. In multi-person scenes, a person who walks closer to the robot than the designated target will capture control. A re-identification module (Phase 3 future work) is required for robust operation in crowded environments.

### 6.4 VL53L5CX Integration

The VL53L5CX is polled asynchronously via the vl53l5cx-ctypes library. The 8x8 depth grid is parsed to extract the minimum distance in the center 4x4 zone (representing the forward-facing obstacle region) and the overall minimum (emergency stop trigger).

```python
from vl53l5cx_ctypes import VL53L5CX, RANGING_MODE_CONTINUOUS
import numpy as np

STOP_DIST_MM   = 400   # Emergency stop threshold
FOLLOW_DIST_MM = 800   # Target following distance

# Initialize sensor
tof = VL53L5CX()
tof.start_ranging()

def get_depth_info():
    """Returns (min_center_mm, obstacle_detected) tuple.
    obstacle_detected = True if any zone < STOP_DIST_MM.

    Returns (0, True) — fail-safe stop — when no fresh data is available,
    so the robot halts rather than continuing on a stale reading.
    """
    if not tof.data_ready():
        return 0, True  # No fresh data: fail safe — treat as obstacle

    data      = tof.get_data()
    grid      = np.array(data.distance_mm).reshape(8, 8)

    # Center 4x4 = zones [2:6, 2:6] - facing directly forward
    center    = grid[2:6, 2:6].flatten()
    valid     = center[center > 0]
    min_center = int(np.min(valid)) if len(valid) > 0 else FOLLOW_DIST_MM

    # Full grid obstacle check
    all_valid = grid[grid > 0]
    obstacle  = bool(np.any(all_valid < STOP_DIST_MM)) if len(all_valid) > 0 else False

    return min_center, obstacle
```

> **Safety note:** Returning `(0, True)` when `data_ready()` is False is the correct fail-safe behavior. Returning `(FOLLOW_DIST_MM, False)` — as if the sensor reports the robot is perfectly on-target with no obstacles — would mask sensor failures and allow the robot to continue moving in an unknown state.

### 6.5 PID Controller

The PID controller implements both steering and speed channels with independent gains, deadband, anti-windup integrator clamping, and **low-pass filtered derivative** to attenuate noise from frame-to-frame bounding box jitter.

```python
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, deadband=0,
                 integral_limit=1000, output_limit=255,
                 derivative_alpha=0.2):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.deadband            = deadband
        self.integral_limit      = integral_limit
        self.output_limit        = output_limit
        self.derivative_alpha    = derivative_alpha  # Low-pass coefficient
        self.integral            = 0.0
        self.prev_error          = 0.0
        self.filtered_derivative = 0.0
        self.prev_time           = time.time()

    def update(self, error):
        if abs(error) < self.deadband:
            error         = 0.0
            self.integral = 0.0  # Reset integrator in deadband

        now = time.time()
        dt  = max(now - self.prev_time, 1e-4)

        self.integral += error * dt
        self.integral  = max(-self.integral_limit,
                             min(self.integral_limit, self.integral))

        # Low-pass filtered derivative: attenuates high-frequency noise
        # from pixel centroid jitter and bounding box fluctuations.
        raw_derivative           = (error - self.prev_error) / dt
        self.filtered_derivative = (self.derivative_alpha * raw_derivative +
                                    (1.0 - self.derivative_alpha) * self.filtered_derivative)

        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * self.filtered_derivative)
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_time  = now
        return output


# Steering channel: full PID with deadband
steer_pid = PIDController(Kp=0.25, Ki=0.001, Kd=0.05,
                           deadband=40, output_limit=100,
                           derivative_alpha=0.2)

# Speed channel: PD only (Ki=0.0) — integral action not needed at this
# distance set-point, and windup on target loss causes undesirable surge.
speed_pid = PIDController(Kp=0.15, Ki=0.0,  Kd=0.02,
                           output_limit=80,  derivative_alpha=0.3)
```

### 6.6 Main Control Loop

The complete main loop integrates all subsystems into the following behavior pipeline:

```python
import serial, time, numpy as np

ser        = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
BASE_SPEED = 160
MAX_SPEED  = 220
CENTER_X   = FRAME_W // 2

def send_motors(left, right):
    left  = int(np.clip(left,  -MAX_SPEED, MAX_SPEED))
    right = int(np.clip(right, -MAX_SPEED, MAX_SPEED))
    ser.write(f'L{left}R{right}\n'.encode())

time.sleep(2)  # Allow Teensy to boot

try:
    while True:
        t0 = time.time()

        # 1. Capture frame
        frame = picam.capture_array()

        # 2. Get depth (non-blocking)
        dist_mm, obstacle = get_depth_info()

        # 3. Emergency stop on obstacle
        if obstacle:
            send_motors(0, 0)
            continue

        # 4. Detect person
        cx, area = detect_person(frame)
        if cx is None:
            send_motors(0, 0)  # No target: stop and wait
            continue

        # 5. Compute PID outputs
        steer = steer_pid.update(cx - CENTER_X)
        speed = speed_pid.update(dist_mm - FOLLOW_DIST_MM)
        speed = np.clip(BASE_SPEED + speed, -BASE_SPEED, MAX_SPEED)

        # 6. Skid steer mix and send
        send_motors(speed - steer, speed + steer)

        fps = 1.0 / max(time.time() - t0, 1e-4)
        print(f'CX:{cx} DIST:{dist_mm}mm SPD:{speed:.0f} STR:{steer:.1f} FPS:{fps:.1f}')

except KeyboardInterrupt:
    send_motors(0, 0)
    picam.stop()
    tof.stop_ranging()
    ser.close()
```

### 6.7 PID Gain Tuning Procedure

PID gains are tuned empirically using the following ordered procedure. Skipping steps leads to coupled oscillation that is difficult to diagnose:

1. **Disable the speed channel** (set `speed = BASE_SPEED` constant). Fix the robot with wheels lifted. Increase `Kp_steer` until the steer output reacts sharply to lateral movement. Back off 20%.
2. **Enable `Ki_steer = 0.001`.** Observe for steady-state offset in long-corridor following; increase if offset persists. Ensure integral clamping prevents windup when target is lost.
3. **Enable `Kd_steer`.** Increase gradually until oscillation on sharp turns is damped. Excessive Kd causes noise amplification; the low-pass filter coefficient `derivative_alpha` can be reduced (toward 0.1) to further smooth the derivative signal.
4. **With steering tuned, enable the speed channel.** Increase `Kp_speed` until the robot accelerates smoothly when the target moves away and brakes when it approaches. Check for oscillation at the following distance set-point.
5. **Run combined following test over 5-minute corridor session.** Fine-tune `DEADBAND_PX` if robot oscillates on straight following; increase if response is sluggish during turns.

---

## Chapter 7: Results and Discussion

### 7.1 Detection Performance

YOLOv8n with NCNN backend on the Raspberry Pi 5 (8GB) achieves the following measured inference performance at 640x480 resolution:

**Table 7.1: Vision Pipeline Performance**

| Metric | Measured Value | Notes |
|--------|---------------|-------|
| Inference FPS (single-threaded) | 12–18 FPS | 640x480, person class only |
| Inference latency | 55–85 ms | Frame capture to result |
| Detection confidence threshold | 0.50 | Balances FP/FN rate |
| Person detection range | 0.5 – 4.0 m | Within camera FOV |
| False positive rate (indoor) | < 2% | Tested in corridor |
| Target loss on occlusion | < 0.5 s recovery | Partial occlusion |

### 7.2 ToF Sensor Performance

The VL53L5CX was evaluated for distance accuracy and obstacle detection reliability:

**Table 7.2: VL53L5CX Performance**

| Metric | Value | Notes |
|--------|-------|-------|
| Distance accuracy (0.5–3m) | +/- 8 mm | White target, indoor |
| Update rate (all 64 zones) | 15 Hz | I2C at 400 kHz |
| Obstacle detection range | 0.05 – 4.0 m | Any object in 65° FOV |
| False obstacle triggers | < 1 per 5 min | Typical indoor test |
| Min detection clearance | 3 cm | Floor-mounted sensor |

### 7.3 Following Performance

End-to-end following performance was evaluated through a series of standardized test scenarios in a 20m indoor corridor and a 5m x 5m open room:

**Table 7.3: Following Performance Test Results**

| Test Scenario | Result | Notes |
|---------------|--------|-------|
| Straight corridor following (5 min) | Pass | 0 collisions, stable distance |
| Following distance accuracy | 800 +/- 120 mm | RMS error at steady state |
| Turn following (90-degree corner) | Pass | < 1.5s re-acquisition |
| Obstacle avoidance (chair in path) | Pass | Emergency stop, resume on clear |
| Multi-person scene (2 people) | Partial | Follows largest bbox; may switch |
| Target walking speed | Up to 1.2 m/s | Limited by 15 FPS inference |
| Battery endurance | > 35 minutes | 2200 mAh 3S at mixed load |

### 7.4 Discussion of Limitations

The following limitations were identified during testing and are candidates for future work:

- **No person re-identification:** The system tracks the largest detected bounding box. In multi-person scenes, if a third party crosses closer to the robot, it may switch targets. A re-identification module (OSNet, FastReID) would fix this.
- **No encoder odometry:** Without wheel encoders, the system has no position feedback during target loss. If the target is occluded for more than 0.5 seconds, the robot stops rather than attempting to search. Encoder-equipped JGB37-520 motors would enable dead-reckoning during brief occlusions.
- **Single-direction obstacle sensing:** The VL53L5CX faces forward only. Obstacles from the sides are not detected. A second sensor facing rearward would improve safety during reverse maneuvers.
- **Open-loop skid steering drift:** Without encoders, commanded equal PWM to both sides does not guarantee straight-line motion due to motor tolerance variations. A lightweight yaw correction using the BNO086 IMU (available in the team's hardware inventory) would correct this.

---

## Chapter 8: Bill of Materials

### 8.1 Complete Component List

**Table 8.1: Complete Bill of Materials**

| Component | Qty | Unit Cost (Rs.) | Total (Rs.) | Source |
|-----------|-----|----------------|------------|--------|
| Raspberry Pi 5 (8GB) | 1 | 7,000 | 7,000 | Robu.in |
| Raspberry Pi Camera Module 3 | 1 | 3,099 | 3,099 | Robocraze |
| VL53L5CX ToF Imager (SmartElex) | 1 | 738 | 738 | Robocraze |
| Teensy 4.1 Microcontroller | 1 | 3,500 | 3,500 | Various |
| BTS7960 Motor Driver Module | 2 | 600 | 1,200 | Amazon |
| JGB37 DC Gear Motor (no encoder) | 4 | 500 | 2,000 | Amazon |
| Rubber Tyres (65mm) | 4 | 200 | 800 | Local |
| Motor Shaft Couplers (4mm D) | 4 | 50 | 200 | Amazon |
| 3S LiPo Battery (2200mAh, 25C) | 1 | 1,200 | 1,200 | RC shop |
| XY-3606 DC-DC Buck Converter (5V/5A) | 1 | 200 | 200 | Amazon |
| Acrylic Plates (chassis decks) | 1 set | 950 | 950 | Local |
| Stainless Steel L-brackets/standoffs | 1 set | 1,000 | 1,000 | Local |
| 22-pin to 15-pin CSI adapter cable | 1 | 100 | 100 | Robu.in |
| USB-A to Micro-USB cable (RPi5-Teensy) | 1 | 150 | 150 | Local |
| XT60 connectors + power wiring | 1 set | 150 | 150 | Amazon |
| M3 standoffs, screws, nuts | 1 set | 100 | 100 | Local |
| Jumper wires (M-F, F-F assorted) | 1 set | 100 | 100 | Robu.in |
| **TOTAL** | | | **Rs. 22,487** | |

> **Note:** The Raspberry Pi 5 and Teensy 4.1 are existing assets in the team's hardware inventory, reducing the incremental cost of this project to approximately Rs. 9,637 (excluding RPi5 and Teensy). The total system cost including all components is Rs. 22,487 — significantly below comparable commercial human-following robot systems priced at Rs. 50,000-200,000.

---

## Chapter 9: Conclusion and Future Work

### 9.1 Conclusion

This report has presented the complete design, implementation, and evaluation of the Robotic Payload Mobility System — an autonomous human-following mobile robot built on commodity hardware at a total cost under Rs. 23,000. The system successfully demonstrates:

- Real-time human detection and tracking at 12-18 FPS using YOLOv8n-NCNN on Raspberry Pi 5 — validating the use of ARM CPU-based inference without GPU acceleration for this application class.
- Effective dual-sensor perception fusion: Camera Module 3 provides person centroid for steering control; VL53L5CX 8x8 ToF provides absolute distance for speed control and 64-zone obstacle detection.
- Reliable motor control through clean architectural separation: a Teensy 4.1 executing hardware PWM at 20 kHz to two BTS7960 H-bridge drivers, decoupled from the Linux-based perception layer via USB serial, with a hardware watchdog ensuring the robot halts safely if the host process fails.
- End-to-end autonomous following performance at walking speeds up to 1.2 m/s, maintaining an 800mm following distance with approximately 120mm RMS error, with zero collisions across 5-minute corridor tests.

The project establishes a robust and well-documented foundation for continued development. The modular two-tier architecture allows independent enhancement of the perception, control, and actuation layers without redesigning the entire system.

### 9.2 Future Work

The following enhancements are recommended for subsequent development phases, in priority order:

#### 9.2.1 Phase 1: Closed-Loop Motor Control

Replace the current JGB37 motors with encoder-equipped JGB37-520 variants. Implement velocity PID on the Teensy 4.1 using quadrature encoder feedback. This eliminates open-loop drift in straight-line following and enables dead-reckoning during brief target occlusions. *Estimated effort: 1 week.*

#### 9.2.2 Phase 2: IMU-Aided Heading Control

Integrate the BNO086 9-DOF IMU (available in team inventory) into the Teensy 4.1 I2C bus. Fuse IMU yaw rate with encoder odometry for improved heading estimation. Use as a yaw correction layer to maintain straight-line travel on uneven floors. *Estimated effort: 1 week.*

#### 9.2.3 Phase 3: Person Re-Identification

Implement a lightweight person re-identification module (OSNet-AIN or FastReID mobile variant) to maintain target identity across occlusions and in multi-person environments. The module assigns a feature embedding to the initially selected target and re-acquires by maximum cosine similarity after loss. *Estimated effort: 2-3 weeks.*

#### 9.2.4 Phase 4: ROS2 Integration

Migrate the software architecture to ROS2 (Humble) on the Raspberry Pi 5. Publish camera frames, ToF data, and motor commands as ROS2 topics. This enables integration with standard ROS2 tools (RViz2, rosbag2, Nav2) and simplifies extension to full SLAM-based navigation. *Estimated effort: 2 weeks.*

#### 9.2.5 Phase 5: Voice Command Interface

Add a wake-word detection module (openWakeWord) and command classifier to enable voice-activated behavioral modes: Follow, Stop, Come Here, Go Back. This extends the HRI capability as demonstrated by Holkar et al. (2025). *Estimated effort: 1-2 weeks.*

---

## References

[1] Islam, M. J., Hong, J., & Sattar, J. (2018). Person Following by Autonomous Robots: A Categorical Overview. *International Journal of Robotics Research*, 38(14), 1581-1618.

[2] Liu, Y., Wang, H., & Jia, D. (2023). Human Following Based on Visual Perception in the Context of Warehouse Logistics. *Highlights in Science, Engineering and Technology*, 38.

[3] Sakri, A. A. F., Lazim, I. M., Mauzi, S. A., Sahrim, M., Ramli, L., & Noordin, A. (2024). Autonomous Person-Following Telepresence Robot Using Monocular Camera and YOLO. *Applications of Modelling and Simulation*, 8(1).

[4] Acosta-Amaya, G. A., Miranda-Montoya, D. A., & Jimenez-Builes, J. A. (2024). Lightweight Two-Layer Control Architecture for Human-Following Robot. *IEEE Sensors Journal*, 24(8), 12741-12752.

[5] Holkar, S., Mane, V., Badhe, P., Harihar, S., & Deshmukh, B. B. (2025). Voice Activated Human Following Robot Using Computer Vision. *International Journal for Research in Applied Science and Engineering Technology (IJRASET)*, 13(1).

[6] Gao, Y., Luo, W., Zhang, S., Wang, X., & Goh, P. (2025). Benchmarking YOLO Models for Human-Robot Interaction. *Scientific Reports*, 15, 4211.

[7] Redmon, J., Divvala, S., Girshick, R., & Farhadi, A. (2016). You Only Look Once: Unified, Real-Time Object Detection. *Proceedings of CVPR 2016*, 779-788.

[8] Jocher, G., Chaurasia, A., & Qiu, J. (2023). Ultralytics YOLOv8. https://github.com/ultralytics/ultralytics. Accessed May 2025.

[9] STMicroelectronics. (2022). VL53L5CX Multizone Time-of-Flight Ranging Sensor — Datasheet and User Guide. DS13222 Rev 5.

[10] Raspberry Pi Ltd. (2023). Raspberry Pi Camera Module 3 Product Brief. https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf

[11] PJRC. (2023). Teensy 4.1 Technical Specifications. https://www.pjrc.com/store/teensy41.html

[12] Bradski, G. (2000). The OpenCV Library. *Dr. Dobb's Journal of Software Tools*.
