#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "pico/stdlib.h"

enum motor_direction {
  CLOCKWISE,
  ANTI_CLOCKWISE
};

void setup_motor_driver(void);
void run_motor(enum motor_direction direction, uint speed);
void stop_motor(void);

#endif