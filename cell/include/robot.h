#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

#define JOINT_MINPOS 42500
#define JOINT_MAXPOS 95000
#define JOINT_STEP     250


typedef enum robotJoint {ROBOT_HAND=1, ROBOT_WRIST=2, ROBOT_ELBOW=3, ROBOT_WAIST=4} robotJoint_t;
typedef enum robotJointStep_t {ROBOT_JOINT_POS_INC, ROBOT_JOINT_POS_DEC} robotJointStep_t;

void robotInit(void);
void robotJointSetState(robotJoint_t, robotJointStep_t);
uint32_t robotJointGetState(robotJoint_t);

#endif
