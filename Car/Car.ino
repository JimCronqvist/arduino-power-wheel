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
const int RC_STEER = A1;     // Steering left/right
const int RC_THROTTLE = A2;  // Throttle forward/reverse
const int RC_EXTRA = A3;     // Extra function (switch?)
const int RC_MAX_SPEED = A4; // Max speed control
const int RC_ENG1_SWITCH = A5; // Channel 7, engine 1 ON/OFF
const int RC_ENG2_SWITCH = A0; // Channel 10, engine 2 ON/OFF

const int FAILSAFE_PIN = 0; // Failsafe output pin (LED, relay, etc.)

const float RAMP_MIN_DURATION = 150.0;   // ms, minimum ramp duration
const float RAMP_MAX_DURATION = 1000.0;  // ms, max duration for full-range (255)

int currentThrottle = 0;
int rampStartValue = 0;              // Global for ramp interpolation
int rampStartTarget = 0;             // Global for ramp interpolation
unsigned long rampStartTime = 0;
bool ramping = false;

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

  int mappedValue = constrain(
    map(rawValue, PWM_MIN, PWM_MAX, minLimit - buffer, maxLimit + buffer),
    minLimit, maxLimit
  );

  // Deadzone (use percentage to calculate instead?)
  if (abs(mappedValue) <= 12) mappedValue = 0;

  // Final override limit
  if (mappedValue > overrideLimit) mappedValue = overrideLimit;
  if (mappedValue < -overrideLimit) mappedValue = -overrideLimit;

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
  pinMode(RC_THROTTLE, INPUT);
  pinMode(RC_ENG1_SWITCH, INPUT);
  pinMode(RC_ENG2_SWITCH, INPUT);
  pinMode(RC_EXTRA, INPUT);
  pinMode(RC_MAX_SPEED, INPUT);

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
  // Read steering and throttle using function
  int steer = readSignalValue(RC_STEER, -255, 255, 170, 9999); // 170 = ~12V
  int throttleTarget  = readSignalValue(RC_THROTTLE, -255, 255, 255, 9999);

  // Engine switches (default true = on)
  bool engine1Enabled = readSwitchValue(RC_ENG1_SWITCH, true);
  bool engine2Enabled = readSwitchValue(RC_ENG2_SWITCH, true);

  // Max speed via potentiometer or switch (channel 4)
  int maxSpeed = readSignalValue(RC_MAX_SPEED, 50, 255, 255, 100);
  throttleTarget = constrain(throttleTarget, -maxSpeed, maxSpeed);

  // --- Failsafe ---
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

  // Check if direction has changed explicitly
  bool directionChanged = (currentThrottle > 0 && throttleTarget < 0) || (currentThrottle < 0 && throttleTarget > 0);

  // Trigger ramp if there's a "significant" change (>5) or direction changed
  if ((throttleTarget != rampStartTarget && abs(currentThrottle - throttleTarget) > 5) || directionChanged) {
    ramping = true;
    rampStartTime = millis();
    rampStartValue = currentThrottle; // Store ramp start value globally
    rampStartTarget = throttleTarget; 
  }

  if (ramping) {
    float rampDuration = max(RAMP_MIN_DURATION, RAMP_MAX_DURATION * abs(rampStartTarget - rampStartValue) / 255.0);
    float rampProgress = (millis() - rampStartTime) / rampDuration;
    if (rampProgress >= 1.0) {
        ramping = false;
        currentThrottle = rampStartTarget;
    } else {
        currentThrottle = (int)(rampStartValue + (rampStartTarget - rampStartValue) * rampProgress);
    }
  } else {
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
  bool lightsOn = readSwitchValue(RC_EXTRA);

  // Debugging output
  Serial.print("Steering: "); Serial.print(steer);
  Serial.print("  Throttle: "); Serial.print(currentThrottle);
  Serial.print("  MaxSpeed: "); Serial.print(maxSpeed);
  Serial.print("  lightsOn: "); Serial.print(lightsOn ? "ON" : "OFF");
  Serial.print("  Eng1: "); Serial.print(engine1Enabled ? "ON" : "OFF");
  Serial.print("  Eng2: "); Serial.print(engine2Enabled ? "ON" : "OFF");
  Serial.println("");

  delay(20); // Update frequency ~50Hz
}
