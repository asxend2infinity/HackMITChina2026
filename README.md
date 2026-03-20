# HackMITChina2026
AeroVis – Helmet Integrated Heads-Up Display for Time Trial Cycling
Overview

AeroVis is a helmet-integrated heads-up display (HUD) system designed for the Giro Aerohead II TT helmet, enabling cyclists to view real-time performance data without breaking their aerodynamic position.

In time trial cycling, even small posture changes can significantly impact performance. AeroVis eliminates the need to look down at a cycling computer by placing critical data directly within the rider’s field of view.

Key Features

Real-time display of cycling metrics (speed, power, cadence, etc.)

Maintains full aerodynamic riding posture

Minimal, non-intrusive UI designed for safety and clarity

Lightweight, helmet-integrated hardware design

Expandable architecture for future algorithm integration

System Architecture

The system consists of three main components:

Embedded System (ESP32)

Handles data processing and communication

Receives sensor or external cycling data

Drives the display output

Display Module (HUD)

Uses 2 0.91" OLED screens to reflect data onto the helmet visor

Optimized for visibility and minimal obstruction

Mechanical Integration

Custom-designed housing for Giro Aerohead II TT helmet

Designed to intergrate the AeroVis system without altering the structure of the helmet

Tech Stack

Microcontroller: ESP32 Dev Kit

Development Environment: VSCode + PlatformIO

CAD Design: SolidWorks

Programming Language: C/C++ (embedded)

Prototyping Support: ChatGPT-assisted rapid development

How It Works

The ESP32 receives cycling data (bluetooth).

Data is processed and formatted for display.

The HUD renders key metrics in real time.

The rider views information without changing head position.

Hardware Setup

ESP32 & custom PCB

HUD display module (OLED)

Power source (battery pack)

Custom 3D-printed mount (designed in SolidWorks)

Note: Exact wiring and pin configuration depend on the display module used.

Getting Started
1. Order PCB & components according to BOM, 3D print housing
2. Clone the Repository
3. Open in PlatformIO

Open the project folder in VSCode

Ensure PlatformIO extension is installed

4. Build & Upload

Connect ESP32 via USB

Select correct COM port

Upload firmware:

pio run --target upload

Challenges

Maintaining visibility without obstructing rider vision

Integrating electronics within structural constraints

Ensing stable real-time data transmission

What We’re Proud Of

Functional prototype combining hardware + embedded systems

Practical application in performance cycling

Clean integration into an existing aerodynamic helmet design

Strong foundation for future intelligent display algorithms

Future Work

Rider posture algorithm

Improved optical projection (transparency, brightness control)

weight balance

Safety alert features

Contributors

Entire project developed inhouse