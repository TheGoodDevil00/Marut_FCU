
<img width="2048" height="588" alt="new_log" src="https://github.com/user-attachments/assets/ab09742d-f94e-4978-b8e5-fd3087f4f949" />

# Marut  
### STM32 F4XX Blackpill Based Tri-Mode Flight Control Unit

**Marut** is an indigenous Flight Control Unit (FCU) currently under development at **AISSMS IOIT**. Built around the STM32 F4XX Blackpill platform, it is designed as a **tri-mode flight controller**, supporting **fixed-wing aircraft, multirotors, and VTOL configurations** within a single unified control framework.

Despite running on a compact microcontroller board costing approximately **₹300**, Marut is engineered to deliver reliable performance and precise control without unnecessary complexity. The FCU focuses on real-time sensor fusion, stable closed-loop control, telemetry support, and FPV OSD compatibility through **Mission Planner**, making it suitable for both experimental and practical airframes.

This project is a collaborative effort between **Team Aeroguardians** of the IoIT Drone Club and the **Catalyst Committee**, with the goal of developing a fully custom, locally designed flight controller from the ground up. Marut is intended to strengthen in-house UAV research and development by providing a transparent, low-cost FCU platform that supports multiple airframe types without reliance on expensive commercial solutions.

By prioritizing **tri-mode flexibility**, affordability, and openness, Marut lowers the barrier to advanced flight-control experimentation and enables a deeper understanding of control systems at both the firmware and hardware levels.

---

## Key Features

- **Tri-mode support**  
  - Fixed-wing  
  - Multirotor  
  - VTOL (hybrid configurations)

- Low-cost hardware using **STM32 F4XX Blackpill**
- Real-time IMU sensor fusion
- Stable closed-loop attitude control
- Telemetry support
- FPV OSD integration via Mission Planner
- Fully **indigenous** and **custom-built** FCU design
- Suitable for academic research, testing, and prototyping

---

## Project Background

Marut is developed as part of a student-led initiative at **AISSMS IOIT**, driven by the need for an affordable yet capable flight controller that can adapt to multiple airframe types while remaining open, understandable, and modifiable at every level.

## Team

* **Project Lead**: Aryan Basnet (TY ENTC)
* **Firmware Team**:

  * Siddesh Katvikar (SY ENTC)
  * Sharal Vishwakarma (SY ENTC)
* **Test Pilots & Flying Team**:

  * Karan Tikoo (TY Comp)
  * Shreyas Karade (FY IT)
* **Aircraft Testbed Builders**:

  * Yash Tawar (SY ENTC)

## Project blocks

> **Note:** The following phases are the ones currently planned. Autonomous navigation, takeoff, landing and vtol support will be added after completion of these two core phases.


<img width="1822" height="544" alt="Gemini_Generated_Image_g54lx0g54lx0g54l" src="https://github.com/user-attachments/assets/25d975a6-5c26-431e-b9ff-67f6fa19848b" />

### Basic control block

The Fixed Wing FCU development focuses on the core functionalities required for stable and controllable flight in fixed-wing aircraft.

* **ESC Driver Code**: Development of robust and efficient Electronic Speed Controller (ESC) driver code is paramount. This code will precisely control the speed and direction of the aircraft's motor(s), enabling thrust regulation for ascent, descent, and forward flight, including PWM and DShot protocols and motor synchronization.
* **SG90 & MG995 Servo Control Code**: Precise control code for hobby servos like SG90 and MG995 to manipulate control surfaces (ailerons, elevators, rudders) for pitch, roll, and yaw. Includes PWM generation, calibration, and feedback handling.
* **PPM & PWM RX Input Code**: Code to interpret signals from radio receivers (PPM & PWM), translating stick movements into FCU commands with error checking and fail-safe mechanisms.
* **BMP280/180 Barometer Interface**: Integration of barometric sensors for accurate altitude readings, including I2C communication, calibration, and conversion to altitude for telemetry and altitude hold.
* **9-Axis IMU Driver Code**: Development of driver code for a 9-axis IMU combining MPU6050 and QMC5883L for precise orientation sensing.
* **MAVLink Telemetry Integration**: Single-sided telemetry from the fixed-wing FCU to ground station software like QGC or Mission Planner for real-time monitoring.
  
<img width="1824" height="544" alt="Gemini_Generated_Image_fer6s9fer6s9fer6" src="https://github.com/user-attachments/assets/dd8dff25-a098-44ce-99b4-d04326f1ef4c" />

### Quadcopter control block

The Quad FCU development addresses multirotor-specific challenges.

* **Gyroscope Interface (MPU6050/MPU9250)**: Integration of IMU for angular rate data to detect and correct roll, pitch, and yaw using high-frequency readings and filtering.
* **3-input PID Stabilization Code**: Optimized PID system using gyroscope data to stabilize the quadcopter across all rotational axes with tuned gains.
* **Quad Control Surface Algorithms**: Algorithms converting desired maneuvers into differential thrust commands for each motor, coordinating with PID stabilization for stable and controllable flight. <- We are here right now
* **Real-Time PID tuning support**:Add optionalsupport for realtime P,I and D manipulation using the various control knobs present on RC transmitters with appropriate channel mapping.
* **Acro/Sport, Auto-level and Altitude hold support**:Develop complex control algorithms for the support of these 3 mdoes using various concepts like 3 input cascaded PID, Complimentary and Kalman filters. <- We are here right now
  


## License

This project is **open for collaboration within the institution**.

For external collaborations, please contact the Catalyst Committee.
