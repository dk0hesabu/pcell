/* Test Robot
 * Joystick UP    -> inc current joint coordinate
 * Joystick DOWN  -> dec current joint coordinate
 * Joystick RIGHT -> cycle joint selection HAND -> WRIST -> ELBOW -> WAIST
 * Joystick left  -> cycle joint selection HAND <- WRIST <- ELBOW <- WAIST
 */
#include <stdbool.h>
#include <ucos_ii.h>
#include <board.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <lcd.h>
#include <interface.h>
#include <robot.h>

#define BUTTONS_TASK_ID 0

/***************************************************************************
*                       PRIORITIES
***************************************************************************/

#define  APP_TASK_BUTTONS_PRIO                     5

/****************************************************************************
*                  APPLICATION TASK STACKS
****************************************************************************/

#define  APP_TASK_BUTTONS_STK_SIZE              256

static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];

/*****************************************************************************
*                APPLICATION FUNCTION PROTOTYPES
*****************************************************************************/

static void appTaskButtons(void *pdata);

/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/

int main() {

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);

  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/**************************************************************************
*                             APPLICATION TASK DEFINITIONS
****************************************************************************/


static void appTaskButtons(void *pdata) {
  
  uint32_t btnState;
  
  /* Initialise hardware and start the OS ticker (highest priority task) */
  bspInit();
  robotInit();
  osStartTick();
  
  /* the main task loop for this task  */
  while (true) {
    btnState = buttonsRead();

    static uint8_t joint = ROBOT_HAND;
    static uint32_t leds[5] = {D1_LED, D1_LED, D2_LED, D3_LED, D4_LED};
    static bool jsRightPressed = false, jsLeftPressed = false;
    static bool robotTestStart = true;
    
    if (robotTestStart) {
      lcdSetTextPos(2, 7);
      lcdWrite("HAND :%08u", robotJointGetState(ROBOT_HAND));
      lcdSetTextPos(2, 8);
      lcdWrite("WRIST:%08u", robotJointGetState(ROBOT_WRIST));
      lcdSetTextPos(2, 9);
      lcdWrite("ELBOW:%08u", robotJointGetState(ROBOT_ELBOW));
      lcdSetTextPos(2, 10);
      lcdWrite("WAIST:%08u", robotJointGetState(ROBOT_WAIST));
      robotTestStart = false;
    }
    if (isButtonPressedInState(btnState, JS_RIGHT)) {
      jsRightPressed = true;
    }
    if (jsRightPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      jsRightPressed = false;
      interfaceLedSetState(leds[joint], LED_OFF);
      joint = (joint % 4) + 1;
    }
    if (isButtonPressedInState(btnState, JS_LEFT)) {
      jsLeftPressed = true;
    }
    if (jsLeftPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
      jsLeftPressed = false;
      interfaceLedSetState(leds[joint], LED_OFF);
      joint--;
      if(joint < 1) joint += 4;
    }
    interfaceLedSetState(leds[joint], LED_ON);
    
    if (isButtonPressedInState(btnState, JS_UP)) {
      robotJointSetState((robotJoint_t)joint, ROBOT_JOINT_POS_INC);
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    } else if (isButtonPressedInState(btnState, JS_DOWN)) {
      robotJointSetState((robotJoint_t)joint, ROBOT_JOINT_POS_DEC);
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    }
    OSTimeDly(20);
  }                       
}