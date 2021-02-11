#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor_driver.h"
#include "motor_encoder.h"
#include "arm_encoder.h"

#define LED_PIN 25
void setup_on_board_led() {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
}

void toggle_on_board_led() {
  gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

int main() {
    stdio_init_all();
    setup_on_board_led();
    setup_motor_driver();
    setup_motor_encoder();
    setup_arm_encoder();
    toggle_on_board_led();

    calibrate_arm_encoder();

    float angle;
    while (1) {
      // printf("triggered %d \n", get_trigger_count());
      angle = get_arm_encoder_angle();
      // printf("angle = %f\n", angle);
      // run_motors_clockwise();
      sleep_ms(100);
      // print_gpio_order();
    }

    return 0;
}