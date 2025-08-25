**ESP32 PID Motor Control System - Complete Guide**

Overview This project implements a closed-loop PID (Proportional-Integral-Derivative) speed control system for a brushed DC motor using an ESP32 microcontroller. The system maintains precise motor speed by comparing desired RPM with actual measured speed from a photoelectric encoder and adjusting PWM duty cycle accordingly.

**Components Required:**

**Main Components:**

ESP32 Development Board - Main controller

D4184 MOSFET Control Module - High-current motor driver

RS390 Brushed DC Motor - 12V DC motor

Photoelectric Speed Sensor Encoder - RPM feedback sensor

12V DC Power Supply - Motor power (via DC barrel jack)

SR5100 Schottky Diode - Flyback protection

0.01μF Capacitor - Noise suppression across motor terminals


**Tools & Supplies**

Breadboard or PCB for connections

Jumper wires

Multimeter for testing

Oscilloscope (optional, for debugging)


**Circuit Connections**

![PID_Controller_V1_Schematic Print_page-0001](https://github.com/user-attachments/assets/3fdec424-f9bb-42dd-b450-b6d216d16a04)


**Hardware Setup Details**

Motor Driver (D4184) The D4184 is a N-channel MOSFET module designed for controlling high-current DC loads. It switches the motor's negative terminal to ground based on the PWM signal from the ESP32.

**Key Features:**

Input voltage: 5-36V DC

Maximum current: 9A continuous

PWM frequency: Up to 20kHz

Logic level: 3.3V/5V compatible


Speed Sensing The photoelectric encoder generates pulses as the motor shaft rotates. Each pulse represents a fixed angular displacement, allowing calculation of RPM.

**Configuration:**

Sensor powered by 3.3V from ESP32

Digital output connected to GPIO 27 with internal pull-up

Interrupt-driven pulse counting for accuracy


**Protection Components**

SR5100 Schottky Diode:

Purpose: Protects against back-EMF when motor stops

Connection: Cathode to 12V+, Anode to motor negative

Specifications: Fast recovery, low forward voltage drop


Noise Suppression Capacitor:

Value: 0.01μF (10nF)

Purpose: Reduces electrical noise from motor brushes

Connection: Across motor terminals


**Software Architecture**

**Core Functions**

1. PWM Generation



// 12-bit resolution PWM at ~19.5kHz ledcSetup(CHANNEL, PWM_HZ, PWM_BITRES); ledcAttachPin(PWM_PIN, CHANNEL);

2. Speed Measurement



// Interrupt-driven pulse counting void IRAM_ATTR onTachRise() { pulseCount++; }

3. PID Controller



// Classical PID implementation double u = Kp * error + Ki * integ + Kd * deriv;

**Key Parameters**

**User Configurable Settings**

PULSES_PER_REV: Encoder pulses per motor revolution

setpointRPM: Target motor speed

Kp, Ki, Kd: PID tuning parameters

CONTROL_PERIOD_MS: Loop update frequency


**Hardware Settings**

PWM_PIN: GPIO 25 (motor control)

TACH_PIN: GPIO 27 (speed sensor)

PWM_BITRES: 12-bit (0-4095 range)

PWM_HZ: 19.531 kHz (above audio range)


**PID Tuning Guide**

Understanding PID Terms

Proportional (Kp):

Controls immediate response to error

Higher values = faster response, but may cause oscillation

Units: duty cycle per RPM error


Integral (Ki):

Eliminates steady-state error

Accumulates error over time

Too high causes overshoot and instability

Units: duty cycle per (RPM·second)


Derivative (Kd):

Predicts future error based on rate of change

Reduces overshoot and oscillation

Too high amplifies noise

Units: (duty cycle·second) per RPM


**Tuning Process**

1. Start with Kp only (Ki=0, Kd=0)

Increase until system responds quickly without excessive oscillation



2. Add Integral term (Ki)

Increase gradually to eliminate steady-state error

Watch for overshoot



3. Add Derivative term (Kd)

Small values help dampen oscillations

Start with Kd = Kp/8 and adjust




**Default Values Explained**

Kp = 0.8   // Moderate response speed Ki = 2.0   // Strong steady-state correction Kd = 0.004 // Light damping

**Safety Features**

Integral Windup Protection

const double INTEG_MIN = -3000.0; const double INTEG_MAX =  3000.0;

Prevents integral term from growing excessively during sustained errors.

PWM Output Limiting

if (u < 0) u = 0; if (u > DUTY_BITMAX) u = DUTY_BITMAX;

Ensures PWM output stays within valid range (0-4095).

Hardware Protection

Schottky diode prevents reverse voltage damage

Capacitor reduces electromagnetic interference

Pull-up resistor ensures clean digital signals


**Operation Guide**

Initial Setup

1. Connect hardware according to wiring diagram


2. Verify all connections with multimeter


3. Upload code to ESP32


4. Open Serial Monitor (115200 baud)



Testing Procedure

1. Static Test: Verify motor doesn't run at startup


2. Open Loop: Temporarily set large Kp (like 10) to test PWM response


3. Sensor Test: Manually spin motor, verify pulse counting


4. Closed Loop: Restore PID values, test speed control



**Troubleshooting**

Motor doesn't respond:

Check 12V power supply

Verify D4184 wiring

Test PWM signal with oscilloscope


Speed oscillates:

Reduce Kp gain

Increase control period (slower updates)

Check for mechanical resonance


Can't reach target speed:

Increase Ki gain

Check motor load

Verify power supply capacity


Noisy operation:

Add more capacitance across motor

Check ground connections

Shield sensor wires


**Technical Specifications**

System Performance

Control Frequency: 20 Hz (50ms updates)

Speed Resolution: ~1 RPM (depends on encoder)

Response Time: Typically 200-500ms to setpoint

Steady-State Error: <1% with proper tuning


**Electrical Ratings**

Motor Voltage: 12V DC

Maximum Current: 9A (D4184 limit)

PWM Frequency: 19.531 kHz

Logic Levels: 3.3V (ESP32 native)


**Advanced Modifications**

Encoder Resolution For higher precision, use encoders with more pulses per revolution:

const double PULSES_PER_REV = 20.0;  // 20 slots = ±3 RPM at 300 RPM

Adaptive Control Add feed-forward or model-based control for better performance:

// Simple feed-forward term double feedforward = setpointRPM * 0.1;  // Rough estimate double u = Kp * error + Ki * integ + Kd * deriv + feedforward;

Multi-Motor Control Expand to control multiple motors by duplicating channels and pins.

**Code Comments Explanation**

The code includes extensive comments explaining:

IRAM_ATTR: Stores interrupt function in fast RAM

Interrupt handling: Why we disable/enable interrupts during pulse reading

Time calculations: Converting milliseconds to seconds for proper PID math

Bit manipulation: Why we use (1 << PWM_BITRES) - 1 instead of hardcoding 4095


This design allows easy modification of PWM resolution without changing multiple constants throughout the code.

**Conclusion**

This PID motor control system provides a robust foundation for precise speed control applications. The modular design allows easy customization for different motors, encoders, and performance requirements. Proper tuning and understanding of the PID parameters will result in stable, accurate motor control suitable for robotics, automation, and educational projects.

