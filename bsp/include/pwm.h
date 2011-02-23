#include <stdint.h>
#include <bsp.h>

typedef enum pwmIdentifier {PWM1 = 1, PWM2, PWM3, PWM4, PWM5, PWM6} pwmIdentifier_t;


void pwmInit(uint32_t majorCycleHz);
void pwmChannelInit(pwmIdentifier_t pwm, uint8_t dutyCyclePercent);
void pwmChangeDutyCycle(pwmIdentifier_t pwm, uint8_t dutyCyclePercent);
