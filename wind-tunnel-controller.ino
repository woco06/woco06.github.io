#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BME280.h>

Adafruit_INA219 ina219;
Adafruit_BME280 bme;

// Pins
const int fanPin      = 25;
const int pressurePin = 34;

// ADC constants
const float adcMax        = 4095.0;
const float vRef          = 3.3;
const float Vs            = 5.0;    // sensor on 5V
const float DIVIDER_RATIO = 0.5;    // two equal 100kΩ resistors

// Smoothing
float V_smooth    = 0.0;
const float alpha = 0.20;  // faster response

// Dead-band (DISABLED)
const float V_DEADBAND = 1.0;

// Auto-calibration
bool          calibrated   = false;
unsigned long calibStart   = 0;
const unsigned long calibDuration = 15000;
float         q_offset     = 0.0;
int           calibSamples = 0;
float         q_sum        = 0.0;

// PWM (ESP32 core v3.x)
const int pwmFreq = 25000;
const int pwmBits = 8;
int       fanPWM  = 0;

// PID
float setpoint = 2.0;

float Kp = 30.0;
float Ki = 8.0;
float Kd = 1.0;

float pid_integral    = 0.0;
float pid_prevError   = 0.0;
unsigned long pid_lastTime  = 0;
unsigned long lastPIDUpdate = 0;

const float integralClamp = 100.0;
const int   FAN_MIN_PWM   = 40;
const int   FAN_MAX_PWM   = 255;

// Stability filter (DISABLED)
float history[5] = {0};
int   histIdx    = 0;

// Plot vs debug mode
bool plotMode = false;

// ── Median + trimmed mean ADC read ───────────────────
float readPressurePa() {
  const int N = 64;
  int samples[N];

  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(pressurePin);
    delayMicroseconds(50);
  }

  // Insertion sort
  for (int i = 1; i < N; i++) {
    int key = samples[i];
    int j   = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  // Average middle 32 — discard top/bottom 16 outliers
  long sum = 0;
  for (int i = 16; i < 48; i++) sum += samples[i];
  float rawADC = sum / 32.0;

  // Correct for voltage divider
  float voltage      = ((rawADC / adcMax) * vRef) / DIVIDER_RATIO;
  float pressure_kPa = (((voltage / Vs) - 0.5) / 0.2) * 2.0;
  return pressure_kPa * 1000.0;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(32, 27);
  ina219.begin();

  Wire1.begin(33, 14);
  if (!bme.begin(0x76, &Wire1)) {
    Serial.println("BME280 not found");
    while (1);
  }

  ledcAttach(fanPin, pwmFreq, pwmBits);
  ledcWrite(fanPin, 0);

  analogReadResolution(12);

  // Sensor warmup
  Serial.println("Sensor warming up for 10 seconds — do not disturb...");
  delay(10000);

  calibStart   = millis();
  pid_lastTime = millis();

  Serial.println("Calibrating for 15 seconds.");
  Serial.println("Pitot tube installed, fan OFF, no airflow.");
  Serial.println("Commands: number = setpoint | c = recalibrate | p = toggle plot");
}

void loop() {

  // ── Serial input ──────────────────────────────────
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "c") {
      calibrated    = false;
      calibStart    = millis();
      q_sum         = 0.0;
      calibSamples  = 0;
      q_offset      = 0.0;
      V_smooth      = 0.0;
      pid_integral  = 0.0;
      pid_prevError = 0.0;
      plotMode      = false;
      for (int i = 0; i < 5; i++) history[i] = 0.0;
      ledcWrite(fanPin, 0);
      Serial.println(">> Recalibrating — pitot installed, fan off, no airflow...");

    } else if (input == "p") {
      plotMode = !plotMode;
      Serial.println(plotMode ? ">> Plot mode ON" : ">> Plot mode OFF");

    } else {
      float val = input.toFloat();
      if (val > 0.0 && val <= 10.0) {
        setpoint     = val;
        pid_integral = 0.0;
        if (!plotMode) {
          Serial.print(">> Setpoint: ");
          Serial.print(setpoint);
          Serial.println(" m/s");
        }
      }
    }
  }

  // ── 1. Read pressure ──────────────────────────────
  float q_raw = readPressurePa();

  // ── 2. Calibration (fan OFF) ──────────────────────
  if (!calibrated) {
    if (millis() - calibStart <= calibDuration) {
      q_sum += q_raw;
      calibSamples++;
      int remaining = (int)((calibDuration - (millis() - calibStart)) / 1000) + 1;
      Serial.print("Calibrating... "); Serial.print(remaining); Serial.println("s");
      delay(300);
      return;
    } else {
      if (calibSamples > 0) q_offset = q_sum / calibSamples;
      calibrated = true;

      V_smooth      = 0.0;
      pid_integral  = 0.0;
      pid_prevError = 0.0;
      pid_lastTime  = millis();
      lastPIDUpdate = millis();

      Serial.print("Calibration done. q_offset = ");
      Serial.print(q_offset); Serial.println(" Pa");
      Serial.println("Check: q_used should be ~0 with no airflow.");
      Serial.println("PID active. Fan starting...");
      Serial.println("Send 'p' to switch to Serial Plotter mode.");
    }
  }

  // ── 3. Apply offset ───────────────────────────────
  float q = q_raw - q_offset;
  if (q < 0) q = 0;

  // ── 4. Air density ────────────────────────────────
  float T  = bme.readTemperature() + 273.15;
  float P  = bme.readPressure();
  float RH = bme.readHumidity();

  const float Rd = 287.05, Rv = 461.5;
  float es  = 610.94 * exp((17.625 * (T - 273.15)) / (T - 30.11));
  float e   = es * (RH / 100.0);
  float rho = ((P - e) / (Rd * T)) + (e / (Rv * T));

  // ── 5. Velocity ───────────────────────────────────
  float V  = (q > 0) ? sqrt((2.0 * q) / rho) : 0.0;
  V_smooth = alpha * V + (1.0 - alpha) * V_smooth;

  // DEAD-BAND REMOVED
  // if (V_smooth < V_DEADBAND) V_smooth = 0.0;

  // ── 6. Stability filter (DISABLED) ────────────────
  float V_stable = V_smooth;

  // ── 7. PID (update every 500ms) ───────────────────
  unsigned long now = millis();
  if (now - lastPIDUpdate >= 500) {
    lastPIDUpdate = now;

    float dt = (now - pid_lastTime) / 1000.0;
    pid_lastTime = now;

    if (dt > 0 && dt < 2.0) {
      float error = setpoint - V_stable;

      pid_integral += error * dt;
      pid_integral  = constrain(pid_integral,
                                -integralClamp / Ki,
                                 integralClamp / Ki);

      float dV      = (V_stable - pid_prevError) / dt;
      pid_prevError = V_stable;

      float output = (Kp * error) + (Ki * pid_integral) - (Kd * dV);

      if (output <= 0) {
        fanPWM = 0;
      } else {
        fanPWM = constrain((int)output + FAN_MIN_PWM, FAN_MIN_PWM, FAN_MAX_PWM);
      }

      ledcWrite(fanPin, fanPWM);
    }
  }

  // ── 8. Output ─────────────────────────────────────
  if (plotMode) {
    Serial.print("Setpoint:"); Serial.print(setpoint, 2);
    Serial.print(",Measured:"); Serial.print(V_stable, 2);
    Serial.print(",FanEffort:"); Serial.println(fanPWM / 25.5);
  } else {
    Serial.println("=== Wind Tunnel PID ===");
    Serial.print("Setpoint:  "); Serial.print(setpoint);            Serial.println(" m/s");
    Serial.print("Measured:  "); Serial.print(V_stable, 2);         Serial.println(" m/s");
    Serial.print("Error:     "); Serial.print(setpoint - V_stable); Serial.println(" m/s");
    Serial.print("q_raw:     "); Serial.print(q_raw);               Serial.println(" Pa");
    Serial.print("q_offset:  "); Serial.print(q_offset);            Serial.println(" Pa");
    Serial.print("q_used:    "); Serial.print(q);                   Serial.println(" Pa");
    Serial.print("rho:       "); Serial.print(rho);                 Serial.println(" kg/m^3");
    Serial.print("Fan PWM:   "); Serial.print(fanPWM);              Serial.println(" /255");
    Serial.println("-----------------------");
  }

  delay(100);
}
