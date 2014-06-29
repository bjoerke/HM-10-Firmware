/**************************************************************************************************
  Filename:       uartManager.c
  Description:    handles command send over the serial interface
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OSAL_ibeacon.h"
#include "ibeacon.h"

#include "hal_led.h"
#include "hal_uart.h"
#include "hal_flash.h"
#include "uartManager.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BAUD_RATE                  HAL_UART_BR_57600
#define POLL_INTERVAL               50
#define PAYLOAD_READ_TRIES_MAX      10
#define FIRMWARE_INFO_STRING_LENGTH 64

// command opcodes and payloads
#define OPCODE_NULL                     0  //invalid opcode
#define OPCODE_TEST                     1  //just for testing - gets ACK back
#define OPCODE_SET_IBEACON_PARAMETERS   2  //changes ibeacon parameters (uuid, minor, major)
#define OPCODE_SET_IBEACON_NAME         3  //changes ibeacon name
#define OPCODE_GET_FIRMWARE_INFO_STRING 4  //gets the firmware info string
#define OPCODE_START                    5  //starts scanning and advertising
#define OPCODE_STOP                     6  //stops
#define OPCODE_MAX                      7  //smallest invalid opcode number
   
static uint8 payloadLengths[] = {
  0,                            //NULL
  0,                            //TEST
  sizeof(ibeaconParameters_t),  //SET_IBEACON_PARAMETERS
  NAME_LENGTH_MAX,              //SET_IBEACON_NAME
  0,                            //GET_FIRMWARE_INFO_STRING
  2,                            //START
  0,                            //STOP
};
#define MAX_PAYLOAD_LENGTH    NAME_LENGTH_MAX

// responses
#define RESPONSE_DEVICE_FOUND          0x02  //used for sending results of device discovery
#define RESPONSE_ACK                   0x40
#define RESPONSE_NCK                   0x23

// events
#define UART_START_DEVICE_EVT 0x01
#define UART_PERIODIC_EVT     0x02

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 uartManagerTaskID;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uartManager_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void HalUARTCback (uint8 port, uint8 event);
static void execute_command();
static void send_ack();
static void send_nck();

/*********************************************************************
 * LOCAL VARIABLES
 */
static halUARTCfg_t config =
{
  .configured = TRUE,
  .baudRate = BAUD_RATE,
  .flowControl = FALSE,
  .idleTimeout = 100,
  .rx = { .maxBufSize = 3*(MAX_PAYLOAD_LENGTH+1) },
  .tx = { .maxBufSize = 3*(MAX_PAYLOAD_LENGTH+1) },
  .intEnable = TRUE,
  .callBackFunc = HalUARTCback
};

//pahses
enum
{
  PHASE_OPCODE,
  PHASE_PAYLOAD
};
static uint8 phase = PHASE_OPCODE;
static uint8 remainingPayloadLength, currentPayloadLength;
static uint8 opcode = OPCODE_NULL;
static uint8 remainingTries;
static uint8 payloadData[MAX_PAYLOAD_LENGTH];

static uint8 firmwareInfoString[FIRMWARE_INFO_STRING_LENGTH] = "Firmware v1.0";

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      uartManager_Init
 *
 * @brief   TODO
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void uartManager_Init( uint8 task_id )
{
  uartManagerTaskID = task_id;
  osal_set_event(uartManagerTaskID, UART_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      uartManager_ProcessEvent
 *
 * @brief   This function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 uartManager_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive(uartManagerTaskID )) != NULL )
    {
      uartManager_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      VOID osal_msg_deallocate( pMsg ); //Release the OSAL message
    }
    return (events ^ SYS_EVENT_MSG);  //return unprocessed events
  }
  
  if ( events & UART_START_DEVICE_EVT )
  {
    HalUARTOpen(HAL_UART_PORT_1, &config);
    osal_start_reload_timer(uartManagerTaskID, UART_PERIODIC_EVT, POLL_INTERVAL);
    return ( events ^ UART_START_DEVICE_EVT );
  }
  
  if ( events & UART_PERIODIC_EVT)
  {
    if(Hal_UART_RxBufLen(HAL_UART_PORT_1) > 0)
    {
      if(phase == PHASE_OPCODE)
      {
         HalUARTRead(HAL_UART_PORT_1, &opcode, 1);
         if(opcode == OPCODE_NULL || opcode >= OPCODE_MAX)
         {
           send_nck();
         } else {
           remainingPayloadLength = payloadLengths[opcode];
           remainingTries = PAYLOAD_READ_TRIES_MAX;
           if(remainingPayloadLength == 0)
           {
             execute_command();
           } else {
             currentPayloadLength = 0;
             phase = PHASE_PAYLOAD;
           }
         }
      }
      if(phase == PHASE_PAYLOAD)
      {
        uint8 bytesRead = HalUARTRead(HAL_UART_PORT_1, &payloadData[currentPayloadLength], remainingPayloadLength);
        currentPayloadLength += bytesRead;
        remainingPayloadLength -= bytesRead;
        if(--remainingTries == 0)
        {
          send_nck();
        } else {
          if(remainingPayloadLength == 0)
          {
            execute_command();
            phase = PHASE_OPCODE;
          }
        }
      }

    }
    return ( events ^ UART_PERIODIC_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      uartManager_ProcessOSALMsg
 * @brief   Process an incoming task message.
 * @param   pMsg - message to process
 * @return  none
 */
static void uartManager_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case IB_MSG_DEVICE_FOUND:
    {
      ibeaconDeviceFoundMsg_t* msg = (ibeaconDeviceFoundMsg_t*) pMsg;
      msg->event = RESPONSE_DEVICE_FOUND;
      HalUARTWrite(HAL_UART_PORT_1, (uint8*) msg, sizeof(ibeaconDeviceFoundMsg_t) );
      break;
    }
  }
}

/***************************
 * @fn       execute_command()
 * @breif    executes the command send over serial
 */
static void execute_command()
{
  switch(opcode)
  {
  case OPCODE_TEST:
    send_ack();
    break;
  case OPCODE_SET_IBEACON_PARAMETERS:
    {
      ibeaconSetParametersMsg_t* msg = (ibeaconSetParametersMsg_t*) osal_msg_allocate(sizeof(ibeaconSetParametersMsg_t));
      msg->event = IB_MSG_SET_PARAMETERS;
      osal_memcpy(&msg->parameters, &payloadData, sizeof(ibeaconParameters_t));
      osal_msg_send(simpleBLETaskId, (uint8*) msg);
      send_ack();
      break;
    }
  case OPCODE_SET_IBEACON_NAME:
    {
      ibeaconSetNameMsg_t* msg = (ibeaconSetNameMsg_t*) osal_msg_allocate(sizeof(ibeaconSetNameMsg_t));
      msg->event = IB_MSG_SET_NAME;
      osal_memcpy(&msg->name, &payloadData, NAME_LENGTH_MAX);
      osal_msg_send(simpleBLETaskId, (uint8*) msg);
      send_ack();
      break;
    }      
  case OPCODE_GET_FIRMWARE_INFO_STRING:
     HalUARTWrite(HAL_UART_PORT_1, firmwareInfoString, FIRMWARE_INFO_STRING_LENGTH);
     break;
  case OPCODE_START:
    osal_set_event( simpleBLETaskId, IB_START_DEVICE_EVT );
    send_ack();
    break;
  case OPCODE_STOP:
    osal_set_event( simpleBLETaskId, IB_STOP_DEVICE_EVT );
    send_ack();
    break;
  }
}

/**
 * @brief      UART event callback
 */
static void HalUARTCback(uint8 port, uint8 event)
{
  switch(event)
  {
    case HAL_UART_RX_FULL:
    case HAL_UART_RX_ABOUT_FULL:
    case HAL_UART_RX_TIMEOUT:
    case HAL_UART_TX_FULL:
    case HAL_UART_TX_EMPTY:
      break;
  }
}

static void send_ack()
{
  uint8 ack = RESPONSE_ACK;
  HalUARTWrite(HAL_UART_PORT_1, &ack, 1);
}

static void send_nck()
{
  uint8 nck = RESPONSE_NCK;
  HalUARTWrite(HAL_UART_PORT_1, &nck, 1);
}

/** 
#define FLASH_PAGE_SIZE             2048
//This could be added in future versions
static void command_flash()
{
  while(Hal_UART_RxBufLen(HAL_UART_PORT_1) == 0); //wait for first data
  HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
  HalFlashErase(0);
  uint8 current_page = 0;
  uint32 current_byte = 0;
  uint8 buf[4];
  
  //start writing to flash
  while(TRUE)
  {
    while(Hal_UART_RxBufLen(HAL_UART_PORT_1) < 4) ;  //wait for at least 4 bytes
    HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);
    if(current_byte == FLASH_PAGE_SIZE)
    {
      current_page++;
      HalFlashErase(current_page);
      current_byte=0;
    }
    HalUARTRead(HAL_UART_PORT_1, buf, 4);
    HalFlashWrite(current_byte / 4, buf, 1); 
    current_byte += 4;
  }
}
**/

/*********************************************************************
*********************************************************************/
