#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "pico/stdlib.h"

void print_gpio_order(void);

void setup_motor_encoder(void);
int get_trigger_count(void);

#endif
