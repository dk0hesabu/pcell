#include <stdint.h>
#include <iolpc2378.h>
#include <control.h>

#define ALARM_BUTTON  0x00000008
#define ALARM_SOUNDER 0x00000004

static controlAlarmState_t state = CONTROL_ALARM_OFF;

void controlInit(void) {
  interfaceInit(ALARM);
  controlAlarmSetState(CONTROL_ALARM_OFF);
}


void controlAlarmSetState(controlAlarmState_t newState) {
  state = newState;
}


controlAlarmState_t controlAlarmGetState(void) {
  return state;
}


bool controlAlarmButtonPressed(void) {
  return (FIO2PIN & ALARM_BUTTON ? false : true); 
}


bool controlItemPresent(uint32_t sensor) {
  return (FIO2PIN & sensor ? false : true);
}
