#ifndef ARM_ENCODER_H
#define ARM_ENCODER_H
#include "stdint.h"
#include "stdbool.h"
bool arm_is_at_rest(void);
void calibrate_arm_encoder(void);
float get_arm_encoder_angle(void);
void setup_arm_encoder(void);
#endif