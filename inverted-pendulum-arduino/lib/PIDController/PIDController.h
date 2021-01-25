#ifndef PID_CONTROLLER
#define PID_CONTROLLER
#include "arm_math.h"
#define PRINT_WITH_NAME(NAME, X) Serial.print(NAME); Serial.print(" "); Serial.println(X);

class PIDController {
public:
  PIDController(
    const float32_t proportionalConstant, 
    const float32_t integralConstant,
    const float32_t derivativeConstant,
    const float32_t outputLimit,
    const float32_t integralLimit);

  void update(float32_t setPoint, float32_t actual);
  
  void reset();
  
  float32_t getOutput();

  float32_t getDerivative() { return m_derivative; }
  float32_t getProportional() { return m_proportional; }

private:
  const float32_t m_proportionalConstant;
  const float32_t m_integralConstant;
  const float32_t m_derivativeConstant;

  float32_t m_currentIntegral;
  float32_t m_previousError;

  float32_t m_derivative;
  float32_t m_proportional;

  const float32_t m_outputLimit;
  const float32_t m_integralLimit;

  float32_t m_output;
};

#endif