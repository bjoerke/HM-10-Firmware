#ifndef _IBEACON_H_
#define _IBEACON_H_

#include <Arduino.h>

#define B_ADDR_STR_LEN 15   //0x112233445566
#define B_ADDR_LEN 6

typedef struct iBeaconAdvertData 
{
  uint8_t len1;
  uint8_t type1;
  uint8_t flags;
  
  uint8_t len2;
  uint8_t type2;
  uint8_t manufacturerID_lo;
  uint8_t manufacturerID_hi;
  uint16_t advertisement;
  uint8_t uuid[16];
  uint8_t major_hi;
  uint8_t major_lo;
  uint8_t minor_hi;
  uint8_t minor_lo;
  uint8_t txPower;
}__attribute__((packed)) iBeaconAdvertData_t;


/**
 * \brief   Checks if givven advertising data fits iBeacon spec
 * \return  true if valid
 */
bool isIBeaconAdvData(uint8_t* data)
{
  iBeaconAdvertData_t* iad = (iBeaconAdvertData_t*) data;
  if( iad->len1 != 2 &&
     iad->type1 != 0x01 &&
     iad->flags != 0x06 )
  return false;

  if(iad->len2              != 26 &&
     iad->type2             != 0xFF &&
     iad->manufacturerID_lo != 0x4C &&
     iad->manufacturerID_hi != 0x00 &&
     iad->advertisement     != 0x1502)
  return false;
  
  return true;
}

/**
 * \brief  convers an 6 byte value to a string containing the correspoinding BT address
 * \return string representation
 */
char* bdAddr2Str( uint8_t* pAddr )
{
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for (int i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  *pStr = 0;
  return str;
}



#endif
