#include "pid_controller.h"

void check_and_limit_value(float *value, float limit);

void setup_pid_controller(
  struct PIDData* pid_data,
  const float proportional_constant, 
  const float integral_constant,
  const float derivative_constant,
  const float outputLimit,
  const float integralLimit) {
    pid_data->proportional_constant = proportional_constant;
    pid_data->integral_constant = integral_constant;
    pid_data->derivative_constant = derivative_constant;
    pid_data->current_integral = 0;
    pid_data->previous_error = 0;
    pid_data->outputLimit = outputLimit;
    pid_data->integralLimit = integralLimit;
    pid_data->output = 0;
}

void check_and_limit_value(float *value, float limit) {
  if (*value > limit) {
    *value = limit;
  }

  if (*value < -limit) {
    *value = -limit;
  }
};


void update_pid(struct PIDData* pid_data, float setPoint, float actual) {
  float error = setPoint - actual;
  pid_data->proportional = error * pid_data->proportional_constant;
  pid_data->current_integral += pid_data->integral_constant * error;
  pid_data->derivative = pid_data->derivative_constant * (error - pid_data->previous_error);
  pid_data->previous_error = error;
  

  check_and_limit_value(&(pid_data->current_integral), pid_data->integralLimit);
  
  pid_data->output = pid_data->proportional + pid_data->current_integral + pid_data->derivative;
  check_and_limit_value(&pid_data->output, pid_data->outputLimit);

}

void reset_pid(struct PIDData* pid_data)
{
  pid_data->previous_error   = 0;
  pid_data->current_integral = 0;
  pid_data->proportional = 0;
  pid_data->derivative = 0;
}

float get_pid_output(struct PIDData* pid_data) {
  return pid_data->output;
}
