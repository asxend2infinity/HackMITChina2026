# HackMITChina2026  
**AeroVis – Helmet Integrated Heads-Up Display for Time Trial Cycling**
**Rigonomics – Real-time Riding Posture Adjustment Algorithm**

---

**Overview**

AeroVis is a helmet-integrated heads-up display (HUD) system designed for the Giro Aerohead II TT helmet, enabling cyclists to view real-time performance data without breaking their aerodynamic position.

In time trial cycling, even small posture changes can significantly impact performance. AeroVis eliminates the need to look down at a cycling computer by placing critical data directly within the rider’s field of view.

As for Rigonomics: the majority of cyclists lack professional fitting advice, and ride in over-agressive or other non-optimal postures. Rigonomics provides real-time posture adjustment advices to riders through Aerovis. By calculating critical angles used by professional fitters during fitting and bike adjustments using body-mounted gyroscopes, it can notify cyclists when their knee joints are over straight, arms are over/under streached, or leaning too forward/backward. 

This helps amature cyclists prevent injuries due to incorrect cycling postures and professional cyclists maintain efficient riding postures (improve performance).

---

**Key Features**

- Real-time display of cycling metrics (speed, power, cadence, etc.)
- Maintains full aerodynamic riding posture
- Minimal, non-intrusive UI designed for safety and clarity
- Lightweight, helmet-integrated hardware design
- Expandable architecture for future algorithm integration
- Posture alert

---

**System Architecture**

The system consists of three main components:

**Embedded System (ESP32)**  
- Handles data processing and communication  
- Receives sensor or external cycling data  
- Drives the display output  

**Display Module (HUD)**  
- Uses 2 0.91" OLED screens to reflect data onto the helmet visor  
- Optimized for visibility and minimal obstruction  

**Mechanical Integration**  
- Custom-designed housing for Giro Aerohead II TT helmet  
- Designed to intergrate the AeroVis system without altering the structure of the helmet  

**WIT Bluetooth 5.0 Giroscope module**  
- Integrated gyroscope solution with Bluetooth broadcast and built-in battery.

---

**Tech Stack**

- Microcontroller: ESP32 Dev Kit  
- WIT Bluetooth 5.0 giroscope module
- Development Environment: VSCode + PlatformIO  
- CAD Design: SolidWorks  
- Programming Language: C/C++ (embedded)  
- Prototyping Support: ChatGPT-assisted rapid development  

---

**How It Works**

Aerovis:
1. The ESP32 receives cycling data (bluetooth).  
2. Data is processed and formatted for display.  
3. The HUD renders key metrics in real time.  
4. The rider views information without needing to look down.  

Rigonomics:
1. Giro data is sent to ESP32
2. Data is processed and determinant angles are calculated
3. Alerts are displayed when rider posture crosses critical thresholds

---

**Hardware Setup**

- ESP32 & custom PCB (file included)
- OLED screens (Listed in BOM)
- Power source (Listed in BOM)  
- Custom 3D-printed mount (SolidWorks file included)  
- Giro Aerohead II MIPS

**Note:** Exact pin configuration might depend on the display module used.

---

**Getting Started**

1. Order PCB & components according to BOM, 3D print housing  
2. Clone the Repository  
3. Open in PlatformIO  

Open the project folder in VSCode  

Ensure PlatformIO extension is installed  

For pairing: Run scan_util.py and open local webpage using a browser

4. Build & Upload  

Connect ESP32 via USB  

Select correct COM port  

Upload firmware:

```bash
pio run --target upload
```

---

**Challenges**

- Maintaining visibility without obstructing rider vision  
- Integrating electronics within structural constraints  
- Ensing stable real-time data transmission  
- Angle calculation and WIT sensor connection & data decode

---

**What We’re Proud Of**

- Functional AeroVis prototype, and partially functioning Rigonomics algorithm combining hardware + embedded systems  
- Practical application in both professional and amature cycling  
- Clean integration into an existing aerodynamic helmet design  
- Strong foundation for future algorithms and extended functions

---

**Contributors**

AeroVis/Rigonomics: Pinchen LIN
Pairing Utility: Shiting HUANG

**UNRESOLVED ISSUES**

New firmware has occasional issues connecting to cycling sensors, for stable pure info display functions use aerovis_1.0_firmware.cpp in Legacy code as main.cpp; WIT motion sensors also seem to incorrectly calibrated each time rebooting, leading to inconsistent alerts and potential false alarms without manual calibration. Both issues are under investigation

**NEW FUNCTIONS**

Developed during HackMIT China 2026: New firmware adds support for Rigonomics: an algorithm designed to give rider posture adjustment alerts based on gyroscope data. Sensors can now also be added through an HTML+python utility, so users do not need to change source code when pairing new sensors. NEW PARING ROUTINE: Use scan_util to update sensor_addresses.txt, and re-upload via PIO

**FUTURE WORK**
 - Might add different UI layouts to display specialized info
 - Companion firmware site under development: settings will be adjustable without having to install PIO, VSCode .etc
 - Optical/visual enhancements
