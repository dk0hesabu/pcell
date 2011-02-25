#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

#define JOINT_MINPOS 45000
#define JOINT_MIDPOS 70000
#define JOINT_MAXPOS 95000
#define JOINT_STEP 250


typedef enum robotJoint {ROBOT_HAND=1, ROBOT_WRIST=2, ROBOT_ELBOW=3, ROBOT_WAIST=4} robotJoint_t;

void robotInit(void);
uint32_t robotJointStepUp(robotJoint_t joint);
uint32_t robotJointStepDown(robotJoint_t joint);

#endif
