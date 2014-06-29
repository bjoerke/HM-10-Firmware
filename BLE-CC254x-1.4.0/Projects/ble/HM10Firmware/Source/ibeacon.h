/**************************************************************************************************
  Filename:       ibeacon.h
  Description:    
**************************************************************************************************/

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */

//the name is part of the scan response which cannot exceed 31 bytes
#define NAME_LENGTH_MAX 20 
#define B_ADDR_LEN  6    //48 bit bluetooth address
#define IBEACON_UUID_LENGTH 16
  
//Events
#define IB_INIT_DEVICE_EVT             0x0001
#define IB_START_DEVICE_EVT            0x0002
#define IB_STOP_DEVICE_EVT             0x0004

//Messages
#define IB_MSG_SET_PARAMETERS 0xA1
#define IB_MSG_SET_NAME       0xA2
#define IB_MSG_DEVICE_FOUND   0xA3

/*********************************************************************
 * TYPES
 */
typedef struct ibeaconParameters
{
  uint8 uuid[IBEACON_UUID_LENGTH];
  uint16 major;
  uint16 minor;
}ibeaconParameters_t;

typedef struct ibeaconSetParametersMsg
{
  uint8 event;  //will be IB_MSG_SET_PARAMETERS
  ibeaconParameters_t parameters;
}ibeaconSetParametersMsg_t;

typedef struct ibeaconSetNameMsg
{
  uint8 event;  //will be IB_MSG_SET_NAME
  uint8 name[NAME_LENGTH_MAX];
}ibeaconSetNameMsg_t;

typedef struct ibeaconDeviceFoundMsg
{
  uint8 event;  //will be IB_MSG_DEVICE_FOUND
  uint8 uuid[16];
  uint16 major;
  uint16 minor;
  uint8 txPower;
  uint8 address[B_ADDR_LEN];
  uint8 rssi;
}ibeaconDeviceFoundMsg_t;

/*********************************************************************
 * MACROS
 */
 
/*********************************************************************
 * FUNCTIONS
 */
extern void SimpleBLECentral_Init( uint8 task_id );
extern uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events );

/*************************************************************
 * VARIABLES
 */
extern uint8 simpleBLETaskId;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
