// ESP32-P4 motor controller — port of teensy_motor.ino
// Target    : Waveshare ESP32-P4-WIFI6 (ESP32 Arduino core 3.x)
// Protocol  : L{int}R{int}\n at 115200 baud over USB-C (USB CDC)
// PWM       : 20 kHz, 8-bit via LEDC (pin-based API)
// Watchdog  : stop motors 500 ms after last command
//
// Arduino IDE: Tools → USB CDC On Boot → Enabled  (makes Serial route to USB-C)

#include <Arduino.h>

// Right-side header pins — avoid GPIO24(DM)/GPIO25(DP) used by USB CDC serial,
// GPIO7(SDA)/GPIO8(SCL) reserved for I2C.
#define L_RPWM 26
#define L_LPWM 27
#define L_EN   32
#define R_RPWM 33
#define R_LPWM 46
#define R_EN   47

#define WATCHDOG_MS 500
#define PWM_FREQ    20000
#define PWM_RES     8      // bits → duty range 0–255

unsigned long lastCmdMs  = 0;
bool          motorsSafe = false;

void setMotors(int left, int right) {
    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);
    ledcWrite(L_RPWM, left  >= 0 ? left  : 0);
    ledcWrite(L_LPWM, left  <  0 ? -left : 0);
    ledcWrite(R_RPWM, right >= 0 ? right : 0);
    ledcWrite(R_LPWM, right <  0 ? -right : 0);
}

void setup() {
    Serial.begin(115200);

    ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
    ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

    pinMode(L_EN, OUTPUT); digitalWrite(L_EN, HIGH);
    pinMode(R_EN, OUTPUT); digitalWrite(R_EN, HIGH);

    setMotors(0, 0);
    lastCmdMs = millis();
}

void loop() {
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
