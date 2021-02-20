#include "motor_driver.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "math.h"

#define CLOCKWISE_PIN 6
#define ANTI_CLOCKWISE_PIN 7
#define SPEED_CONTROL_PIN 8
#define MAX_MOTOR_SPEED 255
#define MOTOR_STOP_SPEED 0

void set_motor_speed(uint speed);
uint slice_num = 0;
void setup_motor_driver(void) {
    gpio_init(CLOCKWISE_PIN);
    gpio_init(ANTI_CLOCKWISE_PIN);
    gpio_init(SPEED_CONTROL_PIN);

    gpio_set_function(SPEED_CONTROL_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    slice_num = pwm_gpio_to_slice_num(SPEED_CONTROL_PIN);
    pwm_set_wrap(slice_num, MAX_MOTOR_SPEED);

    gpio_set_dir(CLOCKWISE_PIN, GPIO_OUT);
    gpio_set_dir(ANTI_CLOCKWISE_PIN, GPIO_OUT);
}

void run_motor(enum motor_direction direction, uint speed) {
  // printf("direction %d\n", direction);
  if (CLOCKWISE == direction) {
    gpio_put(ANTI_CLOCKWISE_PIN, 0);
    gpio_put(CLOCKWISE_PIN, 1);
    // printf("clockwise run cw %d acw %d\n", gpio_get(CLOCKWISE_PIN), gpio_get(ANTI_CLOCKWISE_PIN));
  } else {
    gpio_put(ANTI_CLOCKWISE_PIN, 1);
    gpio_put(CLOCKWISE_PIN, 0);
    // printf("anti clockwise run cw %d acw %d\n", gpio_get(CLOCKWISE_PIN), gpio_get(ANTI_CLOCKWISE_PIN));
  }

  set_motor_speed(speed);
}

void stop_motor(void) {
  set_motor_speed(MOTOR_STOP_SPEED);
  gpio_put(ANTI_CLOCKWISE_PIN, 0);
  gpio_put(CLOCKWISE_PIN, 0);
}

void set_motor_speed(uint speed) {

  if (speed > MAX_MOTOR_SPEED) {
    // printf("motor speed exceeding max\n");
    speed = MAX_MOTOR_SPEED;
  }

  if (speed == MOTOR_STOP_SPEED) {
    pwm_set_enabled(slice_num, false);
  } else {
    pwm_set_enabled(slice_num, true);
  }

  pwm_set_gpio_level(SPEED_CONTROL_PIN, speed);
}