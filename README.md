# Autonomous Sensor Fusion Navigation System

An embedded autonomous navigation system that fuses multiple sensor inputs to perform real-time obstacle detection and decision-making on a microcontroller platform.

This project focuses on reliability, low-latency response, and embedded system constraints, with applications in autonomous navigation and assistive sensing.

---

## System Overview

The system continuously samples distance data from multiple ultrasonic sensors, filters noisy readings, and fuses the results into a single proximity estimate. A real-time decision layer evaluates confidence and distance thresholds to trigger navigation responses when unsafe conditions are detected.

Key features:
- Multi-sensor distance measurement  
- Median filtering for noise reduction  
- Confidence-weighted sensor fusion  
- Real-time embedded decision logic  

---

## Hardware & Software Stack

**Hardware**
- Arduino microcontroller  
- Ultrasonic distance sensors (HC-SR04 or equivalent)  
- Breadboard and jumper wires  
- Alert output (buzzer or LED)  

**Software**
- Embedded C++ (Arduino)
- Arduino IDE
- Serial debugging for real-time monitoring

---

## Core Logic

1. Each sensor samples multiple distance readings per update cycle  
2. Median filtering removes outliers and reduces measurement noise  
3. Filtered readings are assigned confidence scores based on proximity  
4. A weighted fusion algorithm computes a single distance estimate  
5. Decision logic triggers an alert when the fused distance falls below a defined safety threshold  

The implementation is designed to maintain consistent real-time performance under resource constraints.

---

## Testing & Evaluation

The system was evaluated through repeated trials across varying distances and sensor conditions.

- Conducted 40+ test trials across close- and mid-range distances  
- Evaluated behavior during partial sensor dropouts and noisy readings  
- Verified stable alert triggering within unsafe proximity ranges  
- Adjusted thresholds to reduce false positives while prioritizing safety  

Test results are provided in `test_results.csv`.

---

## Repository Contents

├── navigation_system.ino
├── test_results.csv
└── implementation_notes.md


- `navigation_system.ino` contains the complete embedded implementation  
- `test_results.csv` includes sample performance data from experimental trials  
- `implementation_notes.md` documents design decisions, limitations, and future improvements  

---

## Limitations

- Distance-based sensing only (no object classification)  
- Ultrasonic accuracy is affected by surface material and angle  
- Object velocity is not currently considered  

---

## Future Improvements

- Adaptive distance thresholds based on environmental conditions  
- Additional sensor modalities (IR, IMU)  
- Statistical confidence modeling  
- Integration with motor control for autonomous navigation  

---

## License

This project is released for educational and research purposes.
