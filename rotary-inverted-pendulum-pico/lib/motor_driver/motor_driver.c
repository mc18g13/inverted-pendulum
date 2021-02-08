#include "motor_driver.h"
#include "hardware/gpio.h"

#define CLOCKWISE 6
#define ANTI_CLOCKWISE 7
#define SPEED 8

void setup_motor_driver(void) {
    gpio_init(CLOCKWISE);
    gpio_init(ANTI_CLOCKWISE);
    gpio_init(SPEED);
    gpio_set_dir(CLOCKWISE, GPIO_OUT);
    gpio_set_dir(ANTI_CLOCKWISE, GPIO_OUT);
    gpio_set_dir(SPEED, GPIO_OUT);
}

void run_motors_clockwise(void) {
  gpio_put(SPEED, 1);
  gpio_put(ANTI_CLOCKWISE, 0);
  gpio_put(CLOCKWISE, 1);
}