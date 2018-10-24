// Based on the depricated adafruit library for the same board. Much of the cose was reused of directly refactored

#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>

class DCMotor {
  public:
  //private:
    uint8_t motornum;
    uint8_t l_cmd;

    static uint8_t latch_state;
    static void latch_tx();

    static const uint8_t motornum_to_pwm[];

    // Bit positions in the 74HCT595 shift register output
    static const uint8_t motornum_to_motor1a[];
    static const uint8_t motornum_to_motor1b[];

    // Arduino pin names for interface to 74HCT595 latch
    static const uint8_t MOTORLATCH  = 12;
    static const uint8_t MOTORCLK    = 4;
    static const uint8_t MOTORENABLE = 7;
    static const uint8_t MOTORDATA   = 8;

  //public:
    DCMotor(uint8_t _motornum) {
      motornum = _motornum;
      pinMode(motornum_to_pwm[motornum], OUTPUT);
    }

    int deadband = 0;

    void write(int speed);

    inline static void enable(void) {
      pinMode(MOTORLATCH, OUTPUT);
      pinMode(MOTORENABLE, OUTPUT);
      pinMode(MOTORDATA, OUTPUT);
      pinMode(MOTORCLK, OUTPUT);

      latch_state = 0;
      latch_tx();  // Reset

      digitalWrite(MOTORENABLE, LOW);
    }

    inline static void disable(void) {
      latch_state = 0;
      latch_tx();  // Reset

      digitalWrite(MOTORENABLE, HIGH);
    }

    ~DCMotor() {
      disable();
    }
};

#endif
