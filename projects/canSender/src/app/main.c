/* CAN Sender
 *
 * Send a message on CAN 1 every second and toggle interface LED D1
 *
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <board.h>
#include <bsp.h>
#include <osutils.h>
#include <can.h>
#include <leds.h>
#include <interface.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

#define  APP_TASK_CAN_SEND_PRIO                    5

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

#define  APP_TASK_CAN_SEND_STK_SIZE             256

static OS_STK appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskCanSend(void *pdata);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskCanSend,                               
               (void *)0,
               (OS_STK *)&appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE - 1],
               APP_TASK_CAN_SEND_PRIO);
      
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskCanSend(void *pdata) {
  canMessage_t msg;
  uint32_t txCount;
  bool txOk;
    
  /* Initialise the hardware and start the OS ticker
   * (must be done in the highest priority task)
   */
  bspInit();
  interfaceInit(NO_DEVICE);
  osStartTick();

  /* Initialise the CAN message structure */
  msg.id = 0x07;  // arbitrary CAN message id
  msg.len = 4;    // data length 4
  msg.dataA = 0;
  msg.dataB = 0;
  
  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    // Transmit message on CAN 1
    txOk = canWrite(1, &msg);
    if (txOk) {
      interfaceLedToggle(D1_LED);
      txCount += 1;
      msg.dataA = txCount;
    }
    OSTimeDly(1000);
  }
}

