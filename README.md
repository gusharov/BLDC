# BLDC Motor Design & Controller Firmware

**Tools:** SolidWorks, STM32F411 (Blackpill), C, 3D Printing  
**Duration:** 2025

## Overview

This project involved designing and fabricating a custom brushless DC (BLDC) motor, followed by implementing real-time control firmware on an STM32 microcontroller. It combined mechanical design, physical prototyping, and embedded programming into a complete system with adaptive speed control.

## Mechanical Design

- Designed rotor and stator components using SolidWorks.
- 3D printed parts and assembled the motor manually.
- Wrapped copper wire around the stator to form three-phase windings.
- Ensured proper alignment and tolerances for smooth motor operation.

## Embedded Firmware

- Developed bare-metal firmware in C for the STM32F411 (Blackpill).
- Configured GPIOs with EXTI interrupts to capture rising/falling edges from Hall-effect sensors.
- Implemented 6-step commutation logic based on rotor position from Hall sensors.
- Used PWM (via TIM1) to drive high-side MOSFETs of a 3-phase H-bridge.
- Controlled low-side switching through standard GPIOs.

## Adaptive PID Speed Control

- Implemented a PID controller to dynamically adjust PWM duty cycle based on speed error.
- Gains (Kp, Ki, Kd) can adapt based on error magnitude for improved response.
- Integral term is clamped to avoid wind-up.

**Example tuning values:**
- `Kp = 0.3`  
- `Ki = 0.01`  
- `Kd = 0.05`  
- Integral clamp: ±100

## Hardware Setup

- **PWM outputs (TIM1):** PA8, PA9, PA10  
- **Low-side H-Bridge control:** PB5, PB6, PB7  
- **Hall-effect sensor inputs (interrupts):** PC7, PC8, PC9

## Results

- Stable motor operation across low to moderate speeds.
- Real-time commutation based on sensor input.
- PID controller maintained consistent speed under load.

## Future Improvements

- Integrate speed feedback using a tachometer or encoder.
- Add overcurrent protection and implement soft start.
- Allow real-time PID tuning via UART or serial monitor.
- Migrate to a modular RTOS-based architecture.

---

This project showcases both physical system design and embedded control, making it a practical example of end-to-end electromechanical development.
