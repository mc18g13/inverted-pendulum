#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PIDData {
  float proportional_constant;
  float integral_constant;
  float derivative_constant;

  float current_integral;
  float previous_error;

  float derivative;
  float proportional;

  float outputLimit;
  float integralLimit;

  float output;
};


void setup_pid_controller(
  struct PIDData* pid_data,
  const float proportional_constant, 
  const float integral_constant,
  const float derivative_constant,
  const float outputLimit,
  const float integralLimit);
void update_pid(struct PIDData* pid_data, float setPoint, float actual);
void reset_pid(struct PIDData* pid_data);
float get_pid_output(struct PIDData* pid_data);
#endif
