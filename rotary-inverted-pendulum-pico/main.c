#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor_driver.h"
#include "motor_encoder.h"
#include "arm_encoder.h"
#include "pid_controller.h"
#include "math.h"
#include "arm_math.h"

const float32_t K_f32[4] = 
{
-31.6228, 300, -11.5907, 30.583
};

float32_t input_U_f32[4];

float32_t output_X_f32[4];

#define SNR_THRESHOLD   90

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

    const float fifteen_degrees_as_rads = 30.0f * M_PI / 180.0f;
    const float one_degrees_as_rads = 1.0f * M_PI / 180.0f;

    float previous_pendulum_angle;
    float current_pendulum_angle;
    float current_pendulum_speed;
    float previous_motor_angle;
    float current_motor_angle;
    float current_motor_speed;
    bool first_iteration_done = false;
    while (1) {

      if (!first_iteration_done) {
        previous_pendulum_angle = get_arm_encoder_angle();
        previous_motor_angle = get_motor_displacement_radians();
        first_iteration_done = true;
      }

      current_pendulum_angle = get_arm_encoder_angle();
      current_motor_angle = get_motor_displacement_radians();
      // printf("current_pendulum_angle %f\n", current_pendulum_angle);
      // printf("current_motor_angle %f\n", current_motor_angle);
      // Possible add deadband around balance point
      if (fabs(current_pendulum_angle) < fifteen_degrees_as_rads) {
        current_pendulum_speed = previous_pendulum_angle - current_pendulum_angle;
        current_motor_speed = current_motor_angle - previous_motor_angle;

        // printf("current_motor_speed %f\n", current_pendulum_angle);
        // printf("current_pendulum_speed %f\n", current_pendulum_speed);
        float32_t output = K_f32[0] * current_motor_angle + K_f32[1] * current_pendulum_angle + K_f32[2] * current_motor_speed + K_f32[3] * current_pendulum_speed; 
        uint output_absolute = (uint)fabs(output);
        // output_absolute = (output_absolute * output_absolute);
        // printf("output_absolute %d \n", output_absolute);
        if (output > 0) {
          run_motor(CLOCKWISE, output_absolute);
        } else {
          run_motor(ANTI_CLOCKWISE, output_absolute);
        }
        
      } else {
        stop_motor();
      }

      previous_pendulum_angle = current_pendulum_angle;
      previous_motor_angle = current_motor_angle;


      // sleep_ms(10);
    }

    return 0;
}