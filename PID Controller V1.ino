// ======== DEFAULT CIRCUIT SET UP  ========
// Hardware: 12V brushed DC motor + D4184 MOSFET module + photoelectric tach + ESP32
// Wiring (quick):
//   ESP32 GPIO25 ---> D4184 signal pin (IN/SIG/PWM)
//   ESP32 GND -----> D4184 GND -----> 12V supply (-)
//   12V (+) ------> Motor (+)
//   Motor (-) ----> D4184 OUT-
//   Tach Vcc -----> 3V3 (if rated 3.3V)   Tach GND -> GND   Tach DO -> ESP32 GPIO27 (with INPUT_PULLUP)

/*for custom setup, you can use the same program but tune the Kp, Ki, Kd.*/

#include <Arduino.h>

// ---------------- USER KNOBS ----------------
const double PULSES_PER_REV = 1.0;   // sensor pulses per RPM 
double setpointRPM = 300.0;          // target speed in RPM

// PID gains (tuned for realistic response)
double Kp = 2.5;    // proportional
double Ki = 8.0;    // integral
double Kd = 0.01;   // derivative

const double DERIV_ALPHA = 0.7;      // derivative filter smoothing

// Control loop period (update every 50 ms)
const unsigned long CONTROL_PERIOD_MS = 50;

// ---------------- PINS & PWM SETTINGS ----------------
const int PWM_PIN = 25;              
const int TACH_PIN = 27;             

const int CHANNEL = 0;
const int PWM_BITRES = 12;
const int DUTY_BITMAX = (1 << PWM_BITRES) - 1;
const double PWM_HZ = 19531.0;

// ---------------- SPEED COUNTER (interrupt) ----------------
volatile unsigned long pulseCount = 0;
void IRAM_ATTR onTachRise() { pulseCount++; }

// ---------------- PID STATE ----------------
double rpm = 0.0;          // latest measured RPM
double rpmPrev = 0.0;      // previous RPM for derivative
double integ = 0.0;        // integral term
double derivFiltered = 0.0;

const double INTEG_MIN = -5000.0;
const double INTEG_MAX = 5000.0;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 PID Motor Controller - Realistic Tuning");
  Serial.println("---------------------------------------------");

  // PWM setup
  ledcSetup(CHANNEL, PWM_HZ, PWM_BITRES);
  ledcAttachPin(PWM_PIN, CHANNEL);
  ledcWrite(CHANNEL, 0);

  // Tach input
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTachRise, RISING);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < CONTROL_PERIOD_MS) return;

  unsigned long dt_ms = now - lastMs;
  lastMs = now;
  double dt_s = dt_ms / 1000.0;

  // ----- 1) Read pulses safely -----
  unsigned long pulses;
  noInterrupts();
  pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  // ----- 2) Convert pulses to RPM -----
  double revs = pulses / PULSES_PER_REV;
  double rpmMeasured = (dt_s > 0.0) ? (revs / dt_s) * 60.0 : rpmPrev;

  // Simple filter to smooth RPM
  const double RPM_ALPHA = 0.8;
  rpm = RPM_ALPHA * rpm + (1.0 - RPM_ALPHA) * rpmMeasured;

  // ----- 3) PID CONTROL -----
  double error = setpointRPM - rpm;

  // Derivative on measurement
  double derivRaw = -(rpm - rpmPrev) / dt_s;
  derivFiltered = DERIV_ALPHA * derivFiltered + (1.0 - DERIV_ALPHA) * derivRaw;

  // Integral with anti-windup
  double integCandidate = integ + error * dt_s;
  if (integCandidate > INTEG_MAX) integCandidate = INTEG_MAX;
  if (integCandidate < INTEG_MIN) integCandidate = INTEG_MIN;

  double u_unclamped = Kp * error + Ki * integCandidate + Kd * derivFiltered;
  if (u_unclamped >= 0.0 && u_unclamped <= DUTY_BITMAX) {
    integ = integCandidate;  // commit only if output not saturated
  }

  // PID output
  double u = Kp * error + Ki * integ + Kd * derivFiltered;
  if (u < 0.0) u = 0.0;
  if (u > DUTY_BITMAX) u = DUTY_BITMAX;

  // Apply PWM
  ledcWrite(CHANNEL, (int)u);

  // Update previous RPM
  rpmPrev = rpm;

  // ----- 4) Debug print every 250ms -----
  static unsigned long dbgT = 0;
  if (millis() - dbgT > 250) {
    dbgT = millis();
    Serial.print("RPM: "); Serial.print(rpm, 1);
    Serial.print(" | Err: "); Serial.print(error, 1);
    Serial.print(" | Duty: "); Serial.print((int)u);
    Serial.print(" | I: "); Serial.print(integ, 1);
    Serial.print(" | dFilt: "); Serial.println(derivFiltered, 4);
  }
}