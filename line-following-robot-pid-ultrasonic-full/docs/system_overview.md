# System Overview — Line-Following Robot with PID & Ultrasonic Avoidance

## 1. Aim

Design and implement software for a **two-wheel robot car** using **Mbed OS**
that can:

- Follow a predefined black line using IR sensors.
- Maintain stability via PID-based control.
- Detect nearby obstacles with an ultrasonic sensor and react safely.

This work supports the **Software Design and Evaluation** requirements
for the Embedded Control Systems (7ENT1042) module.

## 2. Software Design Summary

- Modular C++ structure with clear separation of:
  - Sensor acquisition (IR + ultrasonic)
  - Control logic (PID)
  - Actuator control (motors, servo)
  - Safety checks (obstacle distance, line lost)
- Uses non-blocking timing via `ThisThread::sleep_for`.
- Serial logs for observability and debugging.

## 3. Ultrasonic Scan Logic

- Multiple HC-SR04 readings are taken and averaged.
- Invalid readings are discarded.
- Distance is used to:
  - Allow motion when clear.
  - Immediately stop when obstacle distance ≤ 10 cm.

## 4. Testing & Evaluation Approach

Testing activities (derived from the Software Design and Evaluation document):

1. **Unit Testing**
   - Validate functions like distance measurement and PID computation individually.

2. **Integration Testing**
   - Verify correct interaction between sensor reads, control logic, and motor outputs.

3. **System Testing**
   - Run the full robot on a real track with obstacles to confirm requirements.

4. **Regression Testing**
   - Re-test after code changes (e.g., tuning PID or scan logic) to avoid new bugs.

5. **Acceptance Testing**
   - Check behaviour against user / assignment expectations:
     - Smooth line following
     - Reliable stopping before obstacles
     - Safe response when the line is lost.

## 5. Conclusion

The implemented control scheme enables the robot to:

- Track a line autonomously.
- Avoid collisions using ultrasonic sensing.
- Provide a maintainable and extensible firmware base for future features
  (e.g., full servo scanning, higher-level navigation, ROS integration).

This repository version is structured for both academic traceability
and professional readability.
