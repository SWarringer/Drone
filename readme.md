Level 1 — Foundation (must-master first)

These are the non-negotiables. Without them, nothing higher-level will stick.

C programming for embedded systems

Pointers, memory management, structs.

How to work with registers and memory-mapped I/O.

Why: All microcontrollers are low-level; if you can’t manipulate hardware in C, you’ll struggle.

ESP-IDF + FreeRTOS

Tasks, queues, semaphores, timers, interrupts.

Peripheral APIs: I2C, SPI, UART, PWM, ADC/DAC, GPIO.

Logging and debugging with idf.py monitor.

Why: Your drone’s flight loop, sensors, and motor control are real-time tasks.

Embedded debugging skills

Serial debugging, logging.

GDB, JTAG debugging basics.

Stack analysis, memory leaks, RTOS task monitoring.

Why: You’ll spend most of your time debugging embedded code; tools matter more than coding tricks.

CMake & Build systems

How ESP-IDF’s CMake works.

compile_commands.json generation for LSP.

Why: You want your IDE and LSP to “just work” so you don’t waste hours on missing headers.

Level 2 — Core robotics & sensor fusion

Once you’re comfortable with Level 1, you can start building drone-specific systems:

Sensors & IMU integration

Reading accelerometers, gyros, barometers, magnetometers.

I2C/SPI handling, sensor calibration.

Understanding raw vs. filtered data.

Motor control

PWM, ESC protocols, brushless motors.

PID controllers for stabilization.

Closed-loop control loops.

RTOS-based task scheduling

Flight loop tasks, telemetry, logging, communication.

Task prioritization and timing analysis.

Why: Flight control loops need strict timing — FreeRTOS gives you the framework.

Basic linear algebra / sensor fusion

Complementary filter, Kalman filter basics.

Orientation (Euler angles, quaternions).

Why: Stabilizing a drone requires fusing sensor data correctly.

Level 3 — Intermediate frameworks & middleware

Once your drone is flying in a basic sense, you can explore:

ROS2

Publisher/subscriber architecture, nodes, topics.

Micro-ROS for microcontrollers (ESP32 + ROS2 bridge).

Why: ROS2 is standard in robotics; allows multi-robot coordination, higher-level algorithms, simulation.

ZephyrOS (optional)

Alternative RTOS for embedded systems.

Unified API for multiple boards.

Why: Good for multi-platform projects or industrial IoT learning, but less critical if you focus on ESP32/FreeRTOS.

Middleware & communication

UART, CAN bus, MAVLink (for drones).

Wireless protocols: WiFi, Bluetooth, LoRa.

Why: Any real drone needs telemetry and remote control.

Level 4 — Advanced robotics & system design

After you’re comfortable with the above, you can explore:

Autonomous control

Path planning, obstacle avoidance.

State estimation (EKF, SLAM basics).

Simulation & testing

Gazebo, PX4 SITL, or Ignition simulations.

Why: Testing in simulation saves crashes and hardware stress.

Low-level optimization

DMA usage, real-time scheduling tweaks.

Power optimization and low-latency ISR handling.

Cross-platform embedded development

Writing portable drivers across ESP32, STM32, nRF52.

Build system mastery (CMake, PlatformIO, Zephyr).

Suggested learning order for your case (drone + embedded)

C programming & ESP-IDF + FreeRTOS

Debugging & peripheral mastery (I2C, SPI, PWM, UART)

RTOS task management & motor control

Sensors & IMU integration + PID loops

Basic drone stabilization & flight loop

ROS2 / Micro-ROS integration for telemetry & control

Simulation & higher-level control algorithms

ZephyrOS or other RTOS for multi-platform experience (optional)
