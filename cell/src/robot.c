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
#define JOINT_MIDPOS 68750

#define HAND_NEUTRAL 68750
#define WRIST_NEUTRAL 82250
#define ELBOW_NEUTRAL 87500
#define WAIST_NEUTRAL 67250

static uint32_t jointPos[5] = {JOINT_MIDPOS, HAND_NEUTRAL, WRIST_NEUTRAL, ELBOW_NEUTRAL, WAIST_NEUTRAL};

void robotInit(void) {
  interfaceInit(ROBOT);
  pwmInit(PWM_MAJOR_CYCLE_HZ);               // set major cycle (should be 20ms period)        
  pwmChannelInit((pwmIdentifier_t)ROBOT_HAND, HAND_NEUTRAL);  
  pwmChannelInit((pwmIdentifier_t)ROBOT_WRIST, WRIST_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_ELBOW, ELBOW_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_WAIST, WAIST_NEUTRAL); 
}

void robotJointSetState(robotJoint_t joint, robotJointStep_t step) {
  uint32_t newJointPos;
  
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  if (step == ROBOT_JOINT_POS_INC) {
    newJointPos = jointPos[joint] + JOINT_STEP;
    if (newJointPos > JOINT_MAXPOS) {
      newJointPos = JOINT_MAXPOS;
    }
  } else {
    newJointPos = jointPos[joint] - JOINT_STEP;  
    if (newJointPos < JOINT_MINPOS) {
      newJointPos = JOINT_MINPOS;
    } 
  }
  pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
}

uint32_t robotJointGetState(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointPos[joint];
}


