#include <assert.h>
#include <stdint.h>
#include <pwm.h>
#include <interface.h>
#include <robot.h>

/* Major cycle of 50Hz (20ms) 
 * Duty cycle ranges from 1ms to 2ms
 * i.e. 5% to 10% of major cycle 
 * 
 */
#define PWM_MAJOR_CYCLE_HZ 50

static uint32_t jointPos[5] = {JOINT_MIDPOS, JOINT_MIDPOS, JOINT_MIDPOS, JOINT_MIDPOS, JOINT_MIDPOS};

void robotInit(void) {
  interfaceInit(ROBOT);
  pwmInit(PWM_MAJOR_CYCLE_HZ);               // set major cycle (should be 20ms period)        
  pwmChannelInit((pwmIdentifier_t)ROBOT_HAND, JOINT_MIDPOS);  // set duty cycle to 7.5%  (1.5ms)
  pwmChannelInit((pwmIdentifier_t)ROBOT_WRIST, JOINT_MIDPOS); // set duty cycle to 7.5%  (1.5ms)
  pwmChannelInit((pwmIdentifier_t)ROBOT_ELBOW, JOINT_MIDPOS); // set duty cycle to 7.5%  (1.5ms)
  pwmChannelInit((pwmIdentifier_t)ROBOT_WAIST, JOINT_MIDPOS); // set duty cycle to 7.5%  (1.5ms)
}

uint32_t robotJointStepUp(robotJoint_t joint) {
  uint32_t newJointPos;
  
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  newJointPos = jointPos[joint] + JOINT_STEP;
  if (newJointPos > JOINT_MAXPOS) {
    newJointPos = JOINT_MAXPOS;
  } 
  pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  return (newJointPos);
}

uint32_t robotJointStepDown(robotJoint_t joint) {
  uint32_t newJointPos;
  
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  newJointPos = jointPos[joint] - JOINT_STEP;  
  if (newJointPos < JOINT_MINPOS) {
    newJointPos = JOINT_MINPOS;
  } 
  pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  return (newJointPos);
}

