#include "WiFiS3.h"

// Arduino UNO R4 – RC-controlled Power Wheel

// WiFi
const char ssid[] = "";
const char pass[] = "";

// Motor control (BTS7960)
const int M1_PWM_FWD = 5; // R
const int M1_PWM_REV = 3; // L
const int M1_L_EN    = 2;
const int M1_R_EN    = 4;

const int M2_PWM_FWD = 6; // R
const int M2_PWM_REV = 9; // L
const int M2_L_EN    = 7;
const int M2_R_EN    = 8;

const int M3_PWM_FWD = 10; // R
const int M3_PWM_REV = 11; // L
const int M3_L_EN    = 12;
const int M3_R_EN    = 13;

// RC channels (FS-i6X receiver)
const int RC_STEER = A1;       // Channel 1, steering left/right
const int RC_THROTTLE = A2;    // Channel 2, throttle forward/reverse
const int RC_EXTRA = A3;       // Channel 8, extra function, cut ramp up time in half.
const int RC_MAX_SPEED = A4;   // Channel 3, max speed control
const int RC_HARD_MAX_SPEED = NULL; // Channel 5, hard limit speed control (digital 0 does not seem to work)
const int RC_STEERING_SPEED = 1; // Channel 6, fine tune steering speed
const int RC_ENG1_SWITCH = A5; // Channel 7, engine 1 ON/OFF
const int RC_ENG2_SWITCH = A0; // Channel 10, engine 2 ON/OFF

const int FAILSAFE_PIN = 0; // Failsafe output pin (LED, relay, etc.)

const float RAMP_MAX_STEP_PER_MS = 255.0 / 1250.0; // Max throttle step per millisecond (255 på 1250 ms)
unsigned long lastRampUpdateTime = 0;
int currentThrottle = 0;

// General signal reader for mapped values
int readSignalValue(int pin, int minLimit, int maxLimit, int overrideLimit, int defaultValue) {
  const int PWM_MIN = 1000;
  const int PWM_MAX = 2000;

  int rawValue = pulseIn(pin, HIGH, 25000);
  //Serial.print("  RAW: "); Serial.print(rawValue);

  if (rawValue < 100) {
    return defaultValue;
  }

  int outSpan = maxLimit - minLimit;
  int buffer = abs(outSpan) * 0.02;

  // Apply override limit
  if(overrideLimit < maxLimit) maxLimit = overrideLimit;
  if(-overrideLimit > minLimit) minLimit = -overrideLimit;

  int mappedValue = constrain(
    map(rawValue, PWM_MIN, PWM_MAX, minLimit - buffer, maxLimit + buffer),
    minLimit, maxLimit
  );

  // Deadzone (use percentage to calculate instead?)
  if (abs(mappedValue) <= 12) mappedValue = 0;

  return mappedValue;
}

// General digital switch reader
bool readSwitchValue(int pin, bool defaultValue = false) {
  int value = readSignalValue(pin, 0, 100, 100, defaultValue ? 100 : 0);
  return value > 50;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (ssid[0] != '\0' && pass[0] != '\0') {
    if (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      Serial.println("Failed to connect to the WiFi network");
      while (true);
    }
    Serial.print("IP-address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi SSID or password not set, skipping WiFi connection.");
  }

  pinMode(M1_PWM_FWD, OUTPUT);
  pinMode(M1_PWM_REV, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);

  pinMode(M2_PWM_FWD, OUTPUT);
  pinMode(M2_PWM_REV, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);

  pinMode(M3_PWM_FWD, OUTPUT);
  pinMode(M3_PWM_REV, OUTPUT);
  pinMode(M3_L_EN, OUTPUT);
  pinMode(M3_R_EN, OUTPUT);

  pinMode(RC_STEER, INPUT);
  pinMode(RC_STEERING_SPEED, INPUT);
  pinMode(RC_THROTTLE, INPUT);
  pinMode(RC_ENG1_SWITCH, INPUT);
  pinMode(RC_ENG2_SWITCH, INPUT);
  pinMode(RC_EXTRA, INPUT);
  pinMode(RC_MAX_SPEED, INPUT);
  pinMode(RC_HARD_MAX_SPEED, INPUT);

  pinMode(FAILSAFE_PIN, OUTPUT);
  digitalWrite(FAILSAFE_PIN, LOW);

  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M3_L_EN, HIGH);
  digitalWrite(M3_R_EN, HIGH);
}

void loop() {
  // Read steering and set a slightly bigger dead zone than default
  int maxSteeringSpeed = readSignalValue(RC_STEERING_SPEED, 100, 140, 140, 120); // Default is 120
  int steer = readSignalValue(RC_STEER, -255, 255, maxSteeringSpeed, 9999); // 170 = ~12V
  if (abs(steer) <= 20) steer = 0;

  // Max speed via potentiometer or switch (channel 4)
  int maxSpeed = readSignalValue(RC_MAX_SPEED, 50, 255, 255, 150); // 170 = ~12V
  int hardMaxSpeed = readSignalValue(RC_HARD_MAX_SPEED, 50, 255, 255, 255); // Additional limit of max speed via knob (hard limit).
  maxSpeed = constrain(maxSpeed, -hardMaxSpeed, hardMaxSpeed);

  // Read throttle
  int throttleTarget = readSignalValue(RC_THROTTLE, -maxSpeed, maxSpeed, maxSpeed, 9999);
  if (throttleTarget < -120) throttleTarget = -120;

  //throttleTarget = constrain(throttleTarget, -maxSpeed, maxSpeed);

  // Engine switches (default true = on)
  bool engine1Enabled = readSwitchValue(RC_ENG1_SWITCH, true);
  bool engine2Enabled = readSwitchValue(RC_ENG2_SWITCH, true);

  // Extra function - for now, cut ramp up time in half
  bool extraFunction = readSwitchValue(RC_EXTRA, false);

  // Failsafe handling, if the remote is not sending signals (actually the receiver)
  bool failsafeActive = (steer == 9999 || throttleTarget == 9999);
  digitalWrite(FAILSAFE_PIN, failsafeActive ? HIGH : LOW);
  if (failsafeActive) {
    // Stop all motors, both FWD and REV
    analogWrite(M1_PWM_FWD, 0);
    analogWrite(M1_PWM_REV, 0);
    analogWrite(M2_PWM_FWD, 0);
    analogWrite(M2_PWM_REV, 0);
    analogWrite(M3_PWM_FWD, 0);
    analogWrite(M3_PWM_REV, 0);

    // Optionally also disable enable-pins for extra safety
    // digitalWrite(M1_L_EN, LOW);
    // digitalWrite(M1_R_EN, LOW);
    // digitalWrite(M2_L_EN, LOW);
    // digitalWrite(M2_R_EN, LOW);
    // digitalWrite(M3_L_EN, LOW);
    // digitalWrite(M3_R_EN, LOW);

    Serial.println("FAILSAFE ACTIVE!");
    delay(20);
    return;
  }

  // Calculate time elapsed since last loop iteration (in milliseconds)
  unsigned long now = millis();
  float elapsed = now - lastRampUpdateTime;
  lastRampUpdateTime = now;

  // Calculate the maximum allowed throttle change for this iteration
  float maxStepPerMs = extraFunction ? RAMP_MAX_STEP_PER_MS*2 : RAMP_MAX_STEP_PER_MS;
  float maxStep = elapsed * RAMP_MAX_STEP_PER_MS;

  // Compute difference between current throttle and target throttle
  int throttleDiff = throttleTarget - currentThrottle;

  // Limit the throttle change to the max step
  if (abs(throttleDiff) > maxStep) {
      // Move current throttle towards target throttle, limited by max step
      currentThrottle += (throttleDiff > 0 ? maxStep : -maxStep);
  } else {
      // Close enough to target, snap to target
      currentThrottle = throttleTarget;
  }

  // Motor 1 & 2 drive control with enable switches
  if (currentThrottle > 0) {
    // Engine 1
    if (engine1Enabled) {
        analogWrite(M1_PWM_REV, 0);
        analogWrite(M1_PWM_FWD, currentThrottle);
    } else {
        analogWrite(M1_PWM_FWD, 0);
        analogWrite(M1_PWM_REV, 0);
    }
    // Engine 2
    if (engine2Enabled) {
        analogWrite(M2_PWM_REV, 0);
        analogWrite(M2_PWM_FWD, currentThrottle);
    } else {
        analogWrite(M2_PWM_FWD, 0);
        analogWrite(M2_PWM_REV, 0);
    }
  } else if (currentThrottle < 0) {
    if (engine1Enabled) {
        analogWrite(M1_PWM_FWD, 0);
        analogWrite(M1_PWM_REV, -currentThrottle);
    } else {
        analogWrite(M1_PWM_FWD, 0);
        analogWrite(M1_PWM_REV, 0);
    }
    if (engine2Enabled) {
        analogWrite(M2_PWM_FWD, 0);
        analogWrite(M2_PWM_REV, -currentThrottle);
    } else {
        analogWrite(M2_PWM_FWD, 0);
        analogWrite(M2_PWM_REV, 0);
    }
  } else {
    analogWrite(M1_PWM_FWD, 0);
    analogWrite(M1_PWM_REV, 0);
    analogWrite(M2_PWM_FWD, 0);
    analogWrite(M2_PWM_REV, 0);
  }

  // Steering motor (Motor 3)
  if (steer > 0) {
    analogWrite(M3_PWM_REV, 0);
    analogWrite(M3_PWM_FWD, steer);
  } else if (steer < 0) {
    analogWrite(M3_PWM_FWD, 0);
    analogWrite(M3_PWM_REV, -steer);
  } else {
    analogWrite(M3_PWM_FWD, 0);
    analogWrite(M3_PWM_REV, 0);
  }

  // Use switch function for the relay to toggle the lights on/off for the extra channel
  //bool lightsOn = readSwitchValue(RC_EXTRA);

  // Debugging output
  Serial.print("Steering: "); Serial.print(steer);
  Serial.print("  MaxSteeringSpeed: "); Serial.print(maxSteeringSpeed);
  Serial.print("  Throttle: "); Serial.print(currentThrottle);
  Serial.print("  MaxSpeed: "); Serial.print(maxSpeed);
  Serial.print("  HardMaxSpeed: "); Serial.print(hardMaxSpeed);
  Serial.print("  ExtraFunction: "); Serial.print(extraFunction ? "ON" : "OFF");
  Serial.print("  Eng1: "); Serial.print(engine1Enabled ? "ON" : "OFF");
  Serial.print("  Eng2: "); Serial.print(engine2Enabled ? "ON" : "OFF");
  Serial.println("");

  delay(20); // Update frequency ~50Hz
}
