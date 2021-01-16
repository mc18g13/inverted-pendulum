#include <Arduino.h>
#include <Stepper.h>

class L298N {
private:
  static const int steps_per_revolution = 200; 
  uint8_t m_coil_activation_pin_A;
  uint8_t m_coil_activation_pin_B;
  Stepper* m_stepper;
public:
  L298N(
    uint8_t coil_activation_pin_A,
    uint8_t coil_activation_pin_B,
    uint8_t motor_pin_1, uint8_t motor_pin_2,
    uint8_t motor_pin_3, uint8_t motor_pin_4);

  void initialise(uint32_t rpm_speed);
  void enable();
  void disable();
  void move(float percentage_of_circle);
};
