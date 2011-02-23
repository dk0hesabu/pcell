#include <assert.h>
#include <iolpc2378.h>
#include <board.h>
#include <bsp.h>
#include <pwm.h>

static uint32_t pwmMatchMajor = 0;

void pwmInit(uint32_t majorCycleHz) {
  PWM1TCR = 0x02;  // reset PWM
  PWM1PR = 0x00;   // set prescaler to 0
  PWM1CTCR = 0x00; // set mode: every rising PCLK edge
  pwmMatchMajor = getFpclk(PWM1_PCLK_OFFSET) / majorCycleHz;
  PWM1MR0 = pwmMatchMajor; // set the major cycle
  PWM1MCR = 0x02;  // enable reset on match
  PWM1TCR = 0x09;  // enable PWM function and start PWM1
}

void pwmChannelInit(pwmIdentifier_t channel, uint8_t dutyCyclePercent) {
  uint32_t pwmMatchDuty = pwmMatchMajor * dutyCyclePercent / 100;

  if (channel == PWM3) {
    PWM1MR3 = pwmMatchDuty;  // set the duty cycle on channel 3
    PWM1LER |= (1 << 4);         // latch the new duty cycle value on channel 3
    PWM1PCR &= ~(1 << 3);     // select single-edge PWM on channel 3
    PWM1PCR |= (1 << 11);      // enable PWM output on channel 3
 }
}

void pwmChangeDutyCycle(pwmIdentifier_t channel, uint8_t dutyCyclePercent) {
  uint32_t pwmMatchDuty = pwmMatchMajor * dutyCyclePercent / 100;
	
  if (channel == PWM3) {
    PWM1MR3 = pwmMatchDuty;  // set the duty cycle on channel 3
    PWM1LER |= (1 << 4);         // latch the new duty cycle value on channel 3
  }
}


