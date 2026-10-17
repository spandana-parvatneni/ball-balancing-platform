/**
 * ball_balancing.ino
 * Arduino Uno — actuation module
 *
 * Receives two servo angles from Python over serial.
 * Protocol: "90,85\n"  (X angle, Y angle, newline terminated)
 *
 * Wiring:
 *   ServoX signal → pin 9
 *   ServoY signal → pin 10
 *   Servo power (+) → external 5V supply (NOT Arduino 5V pin)
 *   Servo GND      → external supply GND → also Arduino GND
 *   Arduino        → USB to laptop
 *
 * Note: common ground between external supply and Arduino is mandatory.
 */

#include <Servo.h>

Servo servoX;
Servo servoY;

const int SERVO_X_PIN    = 10;
const int SERVO_Y_PIN    = 9;
const int SERVO_CENTER   = 91;
const int SERVO_MIN     = 71;
const int SERVO_MAX      = 111;
const int SERVO_CENTER_Y  = 91;
const int SERVO_MIN_Y     = 71;
const int SERVO_MAX_Y     = 111;

void setup() {
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // centre both servos on boot
  servoX.write(SERVO_CENTER);
  servoY.write(SERVO_CENTER_Y);
  delay(500);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    int comma = data.indexOf(',');
    if (comma > 0) {
      int ax = data.substring(0, comma).toInt();
      int ay = data.substring(comma + 1).toInt();

      // hardware safety clamp — backup to Python's saturation_filter
      ax = constrain(ax, SERVO_MIN, SERVO_MAX);
      ay = constrain(ay, SERVO_MIN_Y, SERVO_MAX_Y);

      servoX.write(ax);
      servoY.write(ay);
    }
  }
}
