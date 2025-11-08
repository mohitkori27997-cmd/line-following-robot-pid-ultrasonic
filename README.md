# Line-Following Robot with PID Control & Ultrasonic Obstacle Avoidance

This repository contains the firmware and documentation for a **two-wheel autonomous robot**
developed using **Mbed OS** on the **STM32 Nucleo-F767ZI**.

The system combines:

- 3× IR reflective sensors for line detection
- A **PID controller** for smooth trajectory tracking
- An **HC-SR04 ultrasonic sensor** mounted on a **servo** for obstacle detection
- Differential drive motor control via PWM

It was originally implemented as part of the:

> **Embedded Control Systems (7ENT1042)**  
> University of Hertfordshire — Software Design and Evaluation

and has been refactored into a clean, production-style GitHub project suitable for
showcasing embedded systems and control engineering skills to employers and reviewers.

---

##  Repository Structure

```text
line-following-robot-pid-ultrasonic/
├─ src/
│  └─ main.cpp                # Mbed firmware (PID + ultrasonic + motors)
├─ docs/
│  ├─ system_overview.md      # High-level design + testing summary
│  └─ flowchart.png           # Control flow diagram
└─ README.md
```

---

##  Hardware Overview

| Module              | Connection (default) |
|---------------------|----------------------|
| IR Left Sensor      | A1                   |
| IR Centre Sensor    | A2                   |
| IR Right Sensor     | A3                   |
| Ultrasonic Trig     | D14                  |
| Ultrasonic Echo     | D15                  |
| Servo (US mount)    | PA_3                 |
| Left Motor PWM      | D10                  |
| Right Motor PWM     | D9                   |
| Left Motor Fwd/Rev  | D8 / D6              |
| Right Motor Fwd/Rev | D7 / D5              |

Adapt the pin mapping in `src/main.cpp` if your wiring differs.

---

##  Control Strategy

### Line Following (PID)

- IR sensors detect black line vs background.
- A virtual line position is computed:
  - Left = -1, Centre = 0, Right = +1 (averaged when multiple active)
- Error `e` = desired position (0) − measured position.
- PID controller computes steering correction:
  `u = Kp*e + Ki*∑e + Kd*Δe`
- Motor speeds:
  - `left_speed  = base_speed - u`
  - `right_speed = base_speed + u`

Initial gains:

```cpp
Kp = 0.6f;
Ki = 0.0f;
Kd = 0.12f;
```

Tune on track and commit changes as iterations.

### Ultrasonic Obstacle Avoidance

- HC-SR04 readings are averaged.
- If a valid obstacle is detected **closer than 10 cm**, the robot **stops**.
- This safety layer runs inside the main loop alongside PID.

---

##  Testing & Evaluation

See `docs/system_overview.md` for:

- Unit, integration, system, regression, and acceptance testing notes
- Link to the original **Software Design and Evaluation** work
- Conclusions on reliability and limitations

---

##  Why This Project Matters

This project demonstrates:

- Embedded C/C++ with Mbed OS
- Real-time sensor fusion and feedback control
- Clean code structure, documentation, and testing mindset
- Direct relevance to roles in **Embedded Systems**, **Robotics**, **Automotive**, and **Silicon/SoC validation**

**Author:** Mohit Kori  
**License:** You may add MIT or another license of your choice at the root.
