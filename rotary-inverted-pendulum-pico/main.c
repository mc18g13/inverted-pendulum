#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor_driver.h"
#include "motor_encoder.h"
#include "arm_encoder.h"
#include "pid_controller.h"
#include "math.h"
#include "arm_math.h"


const float32_t B_f32[4] =
{
  782.0, 7577.0, 470.0, 4505.0
};

const float32_t A_f32[16] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
  1.0,     32.0,      4.0,     128.0,
  1.0,     32.0,     64.0,    2048.0,
  1.0,     16.0,      4.0,      64.0,
  1.0,     16.0,     64.0,    1024.0,
};
/* ----------------------------------------------------------------------
* Temporary buffers  for storing intermediate values
* ------------------------------------------------------------------- */
/* Transpose of A Buffer */
float32_t AT_f32[16];
/* (Transpose of A * A) Buffer */
float32_t ATMA_f32[16];
/* Inverse(Transpose of A * A)  Buffer */
float32_t ATMAI_f32[16];
/* Test Output Buffer */
float32_t X_f32[4];
/* ----------------------------------------------------------------------
* Reference ouput buffer C1, C2, C3 and C4 taken from MATLAB
* ------------------------------------------------------------------- */
const float32_t xRef_f32[4] = {73.0, 8.0, 21.25, 2.875};
float32_t snr;
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

    struct PIDData arm_pid;
    struct PIDData motor_position_pid;

    const float outputLimit = 255;
    const float integralLimit = 120;

    const float Kp = 100;
    const float Ki = 0;
    const float Kd = 0;

    float Kd_motor = 0.f;
    setup_pid_controller(&arm_pid, Kp, Ki, Kd, outputLimit, integralLimit);
    setup_pid_controller(&motor_position_pid, 0, 0, Kd_motor, outputLimit, integralLimit);

    const float fifteen_degrees_as_rads = 30.0f * M_PI / 180.0f;
    const float one_degrees_as_rads = 1.0f * M_PI / 180.0f;

    float current_angle;
    float dead_band_angle = one_degrees_as_rads;
    float output_arm = 0;
    float output_motor = 0;
    while (1) {
      current_angle = get_arm_encoder_angle();

      arm_matrix_instance_f32 A;      /* Matrix A Instance */
      arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
      arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
      arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
      arm_matrix_instance_f32 B;      /* Matrix B instance */ 
      arm_matrix_instance_f32 X;      /* Matrix X(Unknown Matrix) instance */
      uint32_t srcRows, srcColumns;  /* Temporary variables */
      arm_status status;
      /* Initialise A Matrix Instance with numRows, numCols and data array(A_f32) */
      srcRows = 4;
      srcColumns = 4;
      arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)A_f32);
      /* Initialise Matrix Instance AT with numRows, numCols and data array(AT_f32) */
      srcRows = 4;
      srcColumns = 4;
      arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
      /* calculation of A transpose */
      status = arm_mat_trans_f32(&A, &AT);
      /* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
      srcRows = 4;
      srcColumns = 4;
      arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
      /* calculation of AT Multiply with A */
      status = arm_mat_mult_f32(&AT, &A, &ATMA);
      /* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
      srcRows = 4;
      srcColumns = 4;
      arm_mat_init_f32(&ATMAI, srcRows, srcColumns, ATMAI_f32);
      /* calculation of Inverse((Transpose(A) * A) */
      status = arm_mat_inverse_f32(&ATMA, &ATMAI);
      /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
      status = arm_mat_mult_f32(&ATMAI, &AT, &ATMA);
      /* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
      srcRows = 4;
      srcColumns = 1;
      arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);
      /* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
      srcRows = 4;
      srcColumns = 1;
      arm_mat_init_f32(&X, srcRows, srcColumns, X_f32);
      /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
      status = arm_mat_mult_f32(&ATMA, &B, &X);
      /* Comparison of reference with test output */
      snr = arm_snr_f32((float32_t *)xRef_f32, X_f32, 4);
      /*------------------------------------------------------------------------------
      *            Initialise status depending on SNR calculations
      *------------------------------------------------------------------------------*/
      status = (snr < SNR_THRESHOLD) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;
      
      if (status != ARM_MATH_SUCCESS)
      {
        printf("FAILURE\n");
      }
      else
      {
        printf("SUCCESS\n");
      }

      // Possible add deadband around balance point
      if (fabs(current_angle) < fifteen_degrees_as_rads) {
        // printf("angle = %f\n", current_angle);
        update_pid(&arm_pid, 0, current_angle);
        float motor_displacement = -(float)get_motor_displacement_count_from_start();
        // printf("motor displacement %f\n", motor_displacement);

        update_pid(&motor_position_pid, 0, -(float)get_motor_displacement_count_from_start());

        output_arm = get_pid_output(&arm_pid);
        output_motor = get_pid_output(&motor_position_pid);


        uint output_absolute = (uint)fabs(output_arm + output_motor);
        output_absolute = (output_absolute * output_absolute);
        // printf("output_absolute %d \n", output_absolute);

        if ((output_arm + output_motor) > 0) {
          run_motor(CLOCKWISE, output_absolute);
        } else {
          run_motor(ANTI_CLOCKWISE, output_absolute);
        }
        
      } else {
        reset_pid(&arm_pid);
        reset_pid(&motor_position_pid);
        stop_motor();
      }


      sleep_ms(10);
    }

    return 0;
}