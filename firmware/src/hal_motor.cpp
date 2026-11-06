#include "hal_motor.h"

#include <Arduino.h>

#include "config.h"

namespace hal {

MotorDriver::MotorDriver() : last_(dg::MotorCmd::STOP), guardTrips_(0) {}

void MotorDriver::begin() {
  pinMode(cfg::MOTOR_IN1, OUTPUT);
  pinMode(cfg::MOTOR_IN2, OUTPUT);
  digitalWrite(cfg::MOTOR_IN1, LOW);
  digitalWrite(cfg::MOTOR_IN2, LOW);
  last_ = dg::MotorCmd::STOP;
}

void MotorDriver::apply(dg::MotorCmd cmd) {
  switch (cmd) {
    case dg::MotorCmd::STOP:
      digitalWrite(cfg::MOTOR_IN1, LOW);
      digitalWrite(cfg::MOTOR_IN2, LOW);
      break;

    case dg::MotorCmd::LOWER:
      digitalWrite(cfg::MOTOR_IN2, LOW); // off pin first, always
      digitalWrite(cfg::MOTOR_IN1, HIGH);
      break;

    case dg::MotorCmd::RAISE:
      digitalWrite(cfg::MOTOR_IN1, LOW); // off pin first, always
      digitalWrite(cfg::MOTOR_IN2, HIGH);
      break;
  }
  last_ = cmd;

  // Last line of defence: read the pads back and kill the bridge if both are
  // somehow driven. This should never fire.
  if (digitalRead(cfg::MOTOR_IN1) == HIGH &&
      digitalRead(cfg::MOTOR_IN2) == HIGH) {
    digitalWrite(cfg::MOTOR_IN1, LOW);
    digitalWrite(cfg::MOTOR_IN2, LOW);
    last_ = dg::MotorCmd::STOP;
    ++guardTrips_;
  }
}

} // namespace hal
