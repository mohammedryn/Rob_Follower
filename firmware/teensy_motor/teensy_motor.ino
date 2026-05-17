// Receives L{int}R{int}\n commands from RPi5 over USB serial.
// Drives two BTS7960 H-bridge modules via 20kHz hardware PWM.
// Safety watchdog: stops motors if no command received in 500ms.

#include <Arduino.h>

// BTS7960 #1 — Left side motors (front-left + rear-left, wired in parallel)
#define L_RPWM 2   // Forward PWM
#define L_LPWM 3   // Reverse PWM
#define L_EN   4   // Enable (active HIGH)

// BTS7960 #2 — Right side motors (front-right + rear-right, wired in parallel)
#define R_RPWM 5   // Forward PWM
#define R_LPWM 6   // Reverse PWM
#define R_EN   7   // Enable (active HIGH)

#define WATCHDOG_MS 500   // Stop motors if silent for this long

unsigned long lastCmdMs  = 0;
bool          motorsSafe = false;

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  analogWrite(L_RPWM, left  >= 0 ? left  : 0);
  analogWrite(L_LPWM, left  <  0 ? -left : 0);
  analogWrite(R_RPWM, right >= 0 ? right : 0);
  analogWrite(R_LPWM, right <  0 ? -right : 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(L_EN, OUTPUT); digitalWrite(L_EN, HIGH);
  pinMode(R_EN, OUTPUT); digitalWrite(R_EN, HIGH);

  // 20kHz PWM — above audible range, within BTS7960 25kHz limit
  analogWriteFrequency(L_RPWM, 20000);
  analogWriteFrequency(L_LPWM, 20000);
  analogWriteFrequency(R_RPWM, 20000);
  analogWriteFrequency(R_LPWM, 20000);
  analogWriteResolution(8);  // 0–255 range

  lastCmdMs = millis();
}

void loop() {
  // Watchdog: halt if RPi5 goes silent (crash, disconnect, KeyboardInterrupt)
  if ((millis() - lastCmdMs) > WATCHDOG_MS) {
    if (!motorsSafe) {
      setMotors(0, 0);
      motorsSafe = true;
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int l_idx = cmd.indexOf('L');
    int r_idx = cmd.indexOf('R');
    if (l_idx >= 0 && r_idx > l_idx) {
      int left  = cmd.substring(l_idx + 1, r_idx).toInt();
      int right = cmd.substring(r_idx + 1).toInt();
      setMotors(left, right);
      lastCmdMs  = millis();
      motorsSafe = false;
    }
  }
}
