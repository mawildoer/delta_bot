#ifndef DCSERVO_h
#define DCSERVO_h

#include <Arduino.h>
#include "PID.h"
#include "DCMotor.h"

class DCServo {
private:
  int feedback_pin;

public:
  DCMotor motor;
  PID<double> pid;
  double target = 0;
  double limit_max, limit_min;
  double feedback;
  double raw_feedback_sum = 0;
  int raw_feedback_samples = 0;
  double output;
  double zero_offset = 0; // In meaningful units
  double scaling = 1; // Multiply targets to get servo scale, divide feedback to get meaningful scale
  long last_motor_update = 0;
  long motor_update_interval;

  DCServo(int motornum, int _feedback_pin, double limit_min, double limit_max) : motor(motornum), limit_max(limit_max), limit_min(limit_min) {
    feedback_pin = _feedback_pin;
  }

  void enable(void) {
    motor.enable();
  }

  void disble(void) {
    motor.disable();
  }

  void update(long t) {
    raw_feedback_sum += analogRead(feedback_pin);
    raw_feedback_samples++;

    if (last_motor_update + motor_update_interval <= t) {
      if (target > limit_max) target = limit_max;
      else if (target < limit_min) target = limit_min;

      feedback = (raw_feedback_sum / raw_feedback_samples / scaling) - zero_offset;
      raw_feedback_sum = 0;
      raw_feedback_samples = 0;

      output = pid.update(target, feedback);
      motor.write(output); // Max PWM width

      last_motor_update = t;
    }
  }
};

#endif
