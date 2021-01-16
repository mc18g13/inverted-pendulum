#include "L298N.h"

L298N::L298N(
  uint8_t coil_activation_pin_A,
  uint8_t coil_activation_pin_B,
  uint8_t motor_pin_1, uint8_t motor_pin_2,
  uint8_t motor_pin_3, uint8_t motor_pin_4) :
    m_coil_activation_pin_A(coil_activation_pin_A),
    m_coil_activation_pin_B(coil_activation_pin_B),
    m_stepper(new Stepper(
      steps_per_revolution, 
      motor_pin_1, motor_pin_2,
      motor_pin_3, motor_pin_4))
{

}

void L298N::initialise(uint32_t rpm_speed) {
  pinMode(m_coil_activation_pin_A, OUTPUT);
  pinMode(m_coil_activation_pin_B, OUTPUT);

  m_stepper->setSpeed(rpm_speed);
}

void L298N::enable() {
  digitalWrite(m_coil_activation_pin_A, HIGH);
  digitalWrite(m_coil_activation_pin_B, HIGH);
}

void L298N::disable() {
  digitalWrite(m_coil_activation_pin_A, LOW);
  digitalWrite(m_coil_activation_pin_B, LOW);
}

void L298N::move(float percentage_of_circle) {
  m_stepper->step((int32_t)(percentage_of_circle * steps_per_revolution));
}

