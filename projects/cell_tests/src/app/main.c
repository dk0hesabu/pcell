#include <stdbool.h>
#include <ucos_ii.h>
#include <board.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <lcd.h>
#include <potentiometer.h>
#include <accelerometer.h>
#include <interface.h>
#include <control.h>
#include <conveyor.h>
#include <robot.h>
#include <buffer.h>

/* Define at most one of CONTROL_TEST, CONVEYOR_TEST or ROBOT_TEST */
#define ROBOT_TEST 1

#define BUTTONS_TASK_ID 0
#define POT_TASK_ID     1
#define ACCEL_TASK_ID   2

/*
*********************************************************************************************************
*                                            PRIORITIES
*********************************************************************************************************
*/

#define  BUF_MUTEX_PRIO                            4
#define  APP_TASK_BUTTONS_PRIO                     5
#define  APP_TASK_POT_PRIO                        10
#define  APP_TASK_ACCEL_PRIO                      15
#define  APP_TASK_LINK_PRIO                       17
#define  APP_TASK_DISPLAY_PRIO                    20

/*
*********************************************************************************************************
*                                            APPLICATION TASK STACKS
*********************************************************************************************************
*/

#define  APP_TASK_BUTTONS_STK_SIZE              256
#define  APP_TASK_POT_STK_SIZE                  256
#define  APP_TASK_ACCEL_STK_SIZE                256
#define  APP_TASK_LINK_STK_SIZE                 256
#define  APP_TASK_DISPLAY_STK_SIZE              256

static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];
static OS_STK appTaskPotStk[APP_TASK_POT_STK_SIZE];
static OS_STK appTaskAccelStk[APP_TASK_ACCEL_STK_SIZE];
static OS_STK appTaskLinkStk[APP_TASK_LINK_STK_SIZE];
static OS_STK appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE];

/*
*********************************************************************************************************
*                                            APPLICATION FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void appTaskButtons(void *pdata);
static void appTaskPot(void *pdata);
static void appTaskAccel(void *pdata);
static void appTaskLink(void *pdata);
static void appTaskDisplay(void *pdata);

/*
*********************************************************************************************************
*                                          OTHER LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void safePutBuffer(message_t *);
void buttonsMsgSrc(message_t *);
void potMsgSrc(message_t *);
void accelMsgSrc(message_t *);
void unknownMsgSrc(message_t *);
void ledAlarmTimerHandler(void *, void *);

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLE DEFINITIONS
*********************************************************************************************************
*/


OS_EVENT *fullSlot;
OS_EVENT *freeSlot;
OS_EVENT *bufMutex;

OS_TMR   *ledAlarmTimer;

INT8U osEventErr;

bool flashing = false;
bool ledAlarmTimerActive = false;

/*
*********************************************************************************************************
*                                            GLOBAL FUNCTION DEFINITIONS
*********************************************************************************************************
*/

int main() {

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);
   
  OSTaskCreate(appTaskPot,                               
               (void *)0,
               (OS_STK *)&appTaskPotStk[APP_TASK_POT_STK_SIZE - 1],
               APP_TASK_POT_PRIO);
   
  OSTaskCreate(appTaskAccel,                               
               (void *)0,
               (OS_STK *)&appTaskAccelStk[APP_TASK_ACCEL_STK_SIZE - 1],
               APP_TASK_ACCEL_PRIO);
  
  OSTaskCreate(appTaskLink,                               
               (void *)0,
               (OS_STK *)&appTaskLinkStk[APP_TASK_LINK_STK_SIZE - 1],
               APP_TASK_LINK_PRIO);

  OSTaskCreate(appTaskDisplay,                               
               (void *)0,
               (OS_STK *)&appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE - 1],
               APP_TASK_DISPLAY_PRIO);
  
  fullSlot = OSSemCreate(0);
  freeSlot = OSSemCreate(BUF_SIZE);
  bufMutex = OSMutexCreate(BUF_MUTEX_PRIO, &osEventErr);
  
  ledAlarmTimer = OSTmrCreate(50, 0, OS_TMR_OPT_ONE_SHOT, 
                              ledAlarmTimerHandler, (void *)0, 
                             (void *)0, &osEventErr);



  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*
*********************************************************************************************************
*                                            APPLICATION TASK DEFINITIONS
*********************************************************************************************************
*/


static void appTaskButtons(void *pdata) {
  
  message_t msg;
  uint32_t oldState;
  
  /* Initialise the hardware and start the OS ticker 
   *
   * Note this must be done in the highest priority task
   */
  bspInit();
#ifdef CONTROL_TEST
  controlInit();
#endif
#ifdef CONVEYOR_TEST
  conveyorInit();
#endif
#ifdef ROBOT_TEST
  robotInit();
#endif
  osStartTick();
  
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    msg.dataA = buttonsRead();
    if (msg.dataA != oldState) {
      msg.id = BUTTONS_TASK_ID;
      oldState = msg.dataA;
      safePutBuffer(&msg);
    }
#ifdef ROBOT_TEST
    {
      static uint8_t joint = ROBOT_HAND;
      static uint32_t leds[5] = {D1_LED, D1_LED, D2_LED, D3_LED, D4_LED};
      static bool jsRightPressed = false;
      
      if (isButtonPressedInState(msg.dataA, JS_RIGHT)) {
        jsRightPressed = true;
      }
      if (jsRightPressed && (!isButtonPressedInState(msg.dataA, JS_RIGHT))) {
        jsRightPressed = false;
        interfaceLedSetState(leds[joint], LED_OFF);
        joint = (joint % 4) + 1;
      }
      interfaceLedSetState(leds[joint], LED_ON);

      if (isButtonPressedInState(msg.dataA, JS_UP)) {
        robotJointStepUp((robotJoint_t)joint);
      } else if (isButtonPressedInState(msg.dataA, JS_DOWN)) {
        robotJointStepDown((robotJoint_t)joint);
      }
    }                       
#endif
    OSTimeDly(20);
  }
}

static void appTaskPot(void *pdata) {
  
  message_t msg;
  uint32_t oldState;
  
  while (true) {
    msg.dataA = potentiometerRead();
    if ((msg.dataA >= oldState + 5) || (msg.dataA <= oldState - 5)) {
      msg.id = POT_TASK_ID;
      oldState = msg.dataA;
      safePutBuffer(&msg);
      if (msg.dataA > 511) {
        flashing = true;
        if (ledAlarmTimerActive) {
          OSTmrStop(ledAlarmTimer, OS_TMR_OPT_NONE, (void *)0, &osEventErr);
          ledAlarmTimerActive = false;
        }
      } else if ((msg.dataA <= 511) && flashing && ! ledAlarmTimerActive) {
          OSTmrStart(ledAlarmTimer, &osEventErr);
          ledAlarmTimerActive = true;
      }
    }
    OSTimeDly(100);
  } 
}


static void appTaskAccel(void *pdata) {
  
  message_t msg;
  uint32_t oldX;
  uint32_t oldY;
  uint32_t oldZ;

  /* Task main loop */
  while (true) {
    msg.dataA = accelerometerRead(X_CHANNEL);
    msg.dataB = accelerometerRead(Y_CHANNEL);
    msg.dataC = accelerometerRead(Z_CHANNEL);
    if ((msg.dataA >= oldX + 15) || (msg.dataB >= oldY + 15) || (msg.dataC >= oldZ + 15) ||
        (msg.dataA <= oldX - 15) || (msg.dataB <= oldY - 15) || (msg.dataC <= oldZ - 15)) {
      msg.id = ACCEL_TASK_ID;
      oldX = msg.dataA;
      oldY = msg.dataB;
      oldZ = msg.dataC;
      safePutBuffer(&msg);
    }
    OSTimeDly(100);
  }
}

static void appTaskLink(void *pdata) {
  while (true) {
    if (flashing) {
      ledToggle(USB_LINK_LED);
    }
    OSTimeDly(250);
  }
}

static void appTaskDisplay(void *pdata) {
  message_t msg;
  lcdSetTextPos(2, 2);
  lcdWrite("BUT 1 : ");
  lcdSetTextPos(2, 3);
  lcdWrite("BUT 2 : ");
  lcdSetTextPos(2, 4);
  lcdWrite("JS    : ");
  lcdSetTextPos(2, 6);
  lcdWrite("POT   : ");
  lcdSetTextPos(2, 8);
  lcdWrite("AC.X  : ");
  lcdSetTextPos(2, 9);
  lcdWrite("AC.Y  : ");
  lcdSetTextPos(2, 10);
  lcdWrite("AC.Z  : ");
  
  /* Task main loop */
  while (true) {
    OSSemPend(fullSlot, 0, &osEventErr);
    OSMutexPend(bufMutex, 0, &osEventErr);
    getBuffer(&msg);
    OSMutexPost(bufMutex);
    OSSemPost(freeSlot);
    switch (msg.id) {
    case BUTTONS_TASK_ID : {
      buttonsMsgSrc(&msg);
      break;
    }
    case POT_TASK_ID : {
      potMsgSrc(&msg);
      break;
    }
    case ACCEL_TASK_ID : {
      accelMsgSrc(&msg);
      break;
    }
    default: {
      unknownMsgSrc(&msg);
    }
    } 
  }
}



/*
*********************************************************************************************************
*                                            LOCAL FUNCTION DEFINITIONS
*********************************************************************************************************
*/

void safePutBuffer(message_t *msg) {
  OSSemPend(freeSlot, 0, &osEventErr);
  OSMutexPend(bufMutex, 0, &osEventErr);
  putBuffer(msg);
  OSMutexPost(bufMutex);
  OSSemPost(fullSlot);
}

void buttonsMsgSrc(message_t *msg) {
  uint32_t s = msg->dataA;
  
  lcdSetTextPos(10, 2);
  if (isButtonPressedInState(s, BUT_1)) {
    lcdWrite("P");
  } else {
    lcdWrite("N");
  }
  lcdSetTextPos(10,3);
  if (isButtonPressedInState(s, BUT_2)) {
    lcdWrite("P");  
  } else {
    lcdWrite("N"); 
  }
  lcdSetTextPos(10, 4);
  if (isButtonPressedInState(s, JS_LEFT)) {
    lcdWrite("L | ");
  } else if (isButtonPressedInState(s, JS_RIGHT)) {
    lcdWrite("R | ");
  } else if (isButtonPressedInState(s, JS_UP)) {
    lcdWrite("U | ");
  } else if (isButtonPressedInState(s, JS_DOWN)) {
    lcdWrite("D | ");
  } else {
    lcdWrite("C | ");
  }
  lcdSetTextPos(14, 4); 
  if (isButtonPressedInState(s, JS_CENTRE)) {
    lcdWrite("P");
  } else {
    lcdWrite("N");
  }
}

void potMsgSrc(message_t *msg) {
  lcdSetTextPos(10, 6);
  lcdWrite("%4d", msg->dataA);
}

void accelMsgSrc(message_t *msg) {
  lcdSetTextPos(10, 8);
  lcdWrite("%4d", msg->dataA);
  lcdSetTextPos(10, 9);
  lcdWrite("%4d", msg->dataB);
  lcdSetTextPos(10, 10);
  lcdWrite("%4d", msg->dataC);
}

void unknownMsgSrc(message_t *msg) {
  lcdSetTextPos(10, 10);
  lcdWrite("BAD MSG %d", msg->id);
}

void ledAlarmTimerHandler(void *timer, void *pdata) {  
  flashing = false;
  ledAlarmTimerActive = false;
  ledSetState(USB_LINK_LED, LED_OFF);
#ifdef CONVEYOR_TEST
  {
  static uint32_t counter;
  switch (conveyorGetState()) {
  case CONVEYOR_OFF :
    if ((counter % 2) == 0) {
      conveyorSetState(CONVEYOR_FORWARD);
    } else {
      conveyorSetState(CONVEYOR_REVERSE);
    }
    counter += 1;
    break;
  case CONVEYOR_FORWARD :
  case CONVEYOR_REVERSE :
    conveyorSetState(CONVEYOR_OFF);
    break;
  default:
    break;
  }
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
  if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {
        interfaceLedSetState(D1_LED, LED_ON);
  } 
  if (conveyorItemPresent(CONVEYOR_SENSOR_2)) {
        interfaceLedSetState(D2_LED, LED_ON);
  } 
  }
#endif
#ifdef CONTROL_TEST
  if (controlEmergencyStopButtonPressed()) {
    controlAlarmToggleState();
  } 
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
  if (controlItemPresent(CONTROL_SENSOR_1)) {
        interfaceLedSetState(D1_LED, LED_ON);
  } 
  if (controlItemPresent(CONTROL_SENSOR_2)) {
        interfaceLedSetState(D2_LED, LED_ON);
  } 
#endif
}

