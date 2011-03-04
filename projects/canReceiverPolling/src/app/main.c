/* CAN Receiver
 *
 * Poll CAN 1 every 20ms and toggle interface LED D1 on reception
 *
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <board.h>
#include <bsp.h>
#include <osutils.h>
#include <can.h>
#include <leds.h>
#include <lcd.h>
#include <interface.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

#define  APP_TASK_CAN_RECEIVE_PRIO                    5

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

#define  APP_TASK_CAN_RECEIVE_STK_SIZE             256

static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskCanReceive(void *pdata);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskCanReceive,                               
               (void *)0,
               (OS_STK *)&appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE - 1],
               APP_TASK_CAN_RECEIVE_PRIO);
      
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskCanReceive(void *pdata) {
  canMessage_t rxMsg;
  
  /* Initialise the hardware and start the OS ticker
   * (must be done in the highest priority task)
   */
  bspInit();
  interfaceInit(NO_DEVICE);
  osStartTick();

  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    // Try to receive message on CAN 1
    if (canReady(1)) {  
      interfaceLedToggle(D1_LED);
      canRead(1, &rxMsg);
      lcdSetTextPos(2,1);
      lcdWrite("ID    : %x", rxMsg.id); 
      lcdSetTextPos(2,2);
      lcdWrite("LEN   : %x", rxMsg.len); 
      lcdSetTextPos(2,3);
      lcdWrite("DATA_A: %x", rxMsg.dataA); 
      lcdSetTextPos(2,4);
      lcdWrite("DATA_B: %x", rxMsg.dataB); 
    }
    interfaceLedToggle(D4_LED);
    OSTimeDly(20);
  }
}

