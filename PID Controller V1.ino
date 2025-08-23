// ======== DEFAULT CIRCUIT SET UP  ========
// Hardware: 12V brushed DC motor + D4184 MOSFET module + photoelectric tach + ESP32
// Wiring (quick):
//   ESP32 GPIO25 ---> D4184 signal pin (IN/SIG/PWM)
//   ESP32 GND -----> D4184 GND -----> 12V supply (-)
//   12V (+) ------> Motor (+)
//   Motor (-) ----> D4184 OUT-
//   Tach Vcc -----> 3V3 (if rated 3.3V)   Tach GND -> GND   Tach DO -> ESP32 GPIO27 (with INPUT_PULLUP)
//   (If your D4184 board has no flyback diode across the motor, add a Schottky: diode + to 12V, - to motor side.)

/*for custom setup, you can use the same program but tune the Kp, Ki, Kd.

for devs only:
update the resolution, PWM freq., max duty cycle, channel (don't modify unless you know how to reprogram the pin connections and ranges)*/

#include <Arduino.h>

// ---------------- USER KNOBS (change these) ----------------
const double PULSES_PER_REV = 1.0;   // sensor pulses per RPM 
double setpointRPM = 300.0;          // target speed in RPM

// PID gains (tune as per performance)
double Kp = 0.8;                     // proportional gain (unit: duty/RPM)
double Ki = 2.0;                     // integral gain (unit: duty/(RPM·s))
double Kd = 0.004;                   // derivative gain (unit: duty·s/RPM)

// Control loop period (how often we update control)
const unsigned long CONTROL_PERIOD_MS = 50;  // 50 ms = 20 updates per second

// ---------------- PINS & PWM SETTINGS ----------------
const int PWM_PIN = 25;              // PWM output pin to D4184 (pin 25 assigned)
const int TACH_PIN = 27;             // tach signal pin from photo sensor (pin 27 assigned)

// PWM Channel, Resolution, maximum duty cycle, Max freq. range 
const int    CHANNEL = 0;
const int    PWM_BITRES    = 12;      // 12-bit resolution -> duty 0-4095
const int    DUTY_BITMAX     = (1 << PWM_BITRES) - 1;  // 4095 (why not just write 4095? to allow change in bit resolution later on)
const double PWM_HZ       = 19531.0; // ~19.531 kHz (max feq. for 12-bit resolution but below audio range)

// ---------------- SPEED COUNTER (interrupt) ----------------
volatile unsigned long pulseCount = 0;  // increases by 1 on every photosensor pulse

void IRAM_ATTR onTachRise() { // IRAM_ATTR means Instruction RAM Attribute, this function asks the program to store instruction in IRAM instead of flash for faster access during interrupt
  // This function runs automatically on every rising edge from the photosensor
  pulseCount++;
}

// ---------------- PID STATE ----------------
//assumes initial state as zero for simplicity, if not it'll auto-correct during operation
double rpm      = 0.0;     // latest measured RPM
double errPrev  = 0.0;     // e[k-1] (last error)
double integ    = 0.0;     // integral term storage

// Limit the integral so it doesn't grow without bound ("anti-windup")
const double INTEG_MIN = -3000.0;     // these are conservative, modify as necessary
const double INTEG_MAX =  3000.0;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 based PID Speed Control");
  serial.println("-----------------------------");

  // PWM (LEDC)
  ledcSetup(CHANNEL, PWM_HZ, PWM_BITRES); // configure PWM timer (setup channel selected, PWM freq., bit resolution)
  ledcAttachPin(PWM_PIN, CHANNEL);       // connect PWM pin to that timer/channel
  ledcWrite(CHANNEL, 0);                 // start at 0% duty (motor off) 

  // photosensor input set to internal pull-up (active-low logic for sensor input)
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTachRise, RISING); 
 /*digitalPinToInterrupt is a ISR Function that pauses normal code when there is a change detected
 onTachRise is the function name given to the IRAM_ATTR function
 set to RISING since TACH_PIN is set to active-low logic, thus activates whenever change is detected i.e the photosensor when blocked go from LOW->HIGH*/  
}

// ---------------- MAIN LOOP ----------------
void loop() {
  static unsigned long lastMs = 0; // remember when we last updated
  unsigned long now = millis(); //millis runs kinda forever (49.71 days or 4,294,967,295ms i.e 32-bit) and resets 
  if (now - lastMs < CONTROL_PERIOD_MS) { 
    // Not yet time for the next control update; just idle.
    return;
  }
  unsigned long dt_ms = now - lastMs;   // actual elapsed time since last control tick (ms)
  lastMs = now;                         // mark this tick

  // ----- 1) Get how many pulses arrived since last tick -----
  unsigned long pulses;
  noInterrupts();            // briefly stop interrupts so the number can't change mid-read
  pulses = pulseCount;       // copy the shared variable
  pulseCount = 0;            // reset for the next 50 ms window
  interrupts();              // turn interrupts back on

  // ----- 2) Convert pulses to RPM -----
  // If we saw "pulses" in dt seconds, then revs = pulses / PULSES_PER_REV
  // rev/s = revs / dt_s; RPM = rev/s * 60
  double dt_s = dt_ms / 1000.0; // ms converted to s
  double revs = pulses / PULSES_PER_REV; // converts the number of pulses to number of revolutions during during control time window
  rpm = (revs / dt_s) * 60.0;

  // ----- 3) PID CONTROL -----
  // Error = (what I want) - (what I measured)
  double error = setpointRPM - rpm;

  // Integral: add error over time
  // (clamp the integrator to avoid windup if we hit duty limits later)
  integ += error * dt_s; // integ = integ + error*dt_s
  if (integ > INTEG_MAX) integ = INTEG_MAX;
  if (integ < INTEG_MIN) integ = INTEG_MIN;

  // Derivative: change in error over time
  double deriv = (error - errPrev) / dt_s;

  // PID output (unclamped)
  double u = Kp * error + Ki * integ + Kd * deriv;

  // Clamp output to valid PWM range
  if (u < 0) u = 0;
  if (u > DUTY_BITMAX) u = DUTY_BITMAX;

  // Remember for next time
  errPrev = error;

  // ----- 4) Apply PWM duty to D4184 -----
  ledcWrite(CHANNEL, (int)u);
}
