#include "DCMotor.h"

const uint8_t DCMotor::motornum_to_pwm[] = {11,3,6,5};

// Bit positions in the 74HCT595 shift register output
const uint8_t DCMotor::motornum_to_motor1a[] = {2,1,5,0};
const uint8_t DCMotor::motornum_to_motor1b[] = {3,4,7,6};

uint8_t DCMotor::latch_state = 0;

void DCMotor::latch_tx() {
  digitalWrite(MOTORLATCH, LOW);

  digitalWrite(MOTORDATA, LOW);

  uint8_t i;
  for (i=0; i<8; i++) {
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & (1 << (7-i))) {
      digitalWrite(MOTORDATA, HIGH);
    } else {
      digitalWrite(MOTORDATA, LOW);
    }
    digitalWrite(MOTORCLK, HIGH);
  }

  digitalWrite(MOTORLATCH, HIGH);
}

void DCMotor::write(int speed) {
  // validate command
  if (speed > 255) speed = 255;
  else if (speed < -255) speed = -255;
  if (motornum < 0 || motornum > 4) return; // error

  int abs_speed = speed;
  if (speed < 0) abs_speed = -speed;
  if (abs_speed < deadband) abs_speed += deadband;
  analogWrite(motornum_to_pwm[motornum-1], abs_speed);

  int cmd;
  if (speed == 0) cmd = 0;
  else if (speed > 0) cmd = 1;
  else if (speed < 0) cmd = -1;

  // configure motor direction required
  if (cmd != l_cmd) {
    uint8_t a = motornum_to_motor1a[motornum-1];
    uint8_t b = motornum_to_motor1b[motornum-1];

    l_cmd = cmd;

    a = motornum_to_motor1a[motornum-1];
    b = motornum_to_motor1b[motornum-1];

    if (speed > 0) { // forward
      latch_state |=   (1 << a);
      latch_state &= ~ (1 << b);
    } else if (speed < 0) { // backward
      latch_state &= ~ (1 << a);
      latch_state |=   (1 << b);
    } else {
      latch_state &= ~ (1 << a);
      latch_state &= ~ (1 << b);
    }

    latch_tx();
  }

}
