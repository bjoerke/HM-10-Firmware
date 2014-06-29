/**
 Just a testing application / quick prototype
 Setup:
 * Arduino Uno
 * BleShield (http://imall.iteadstudio.com/im130704001.htm)
 * Mount BleShield ontop of the Arduino which looks as 
   follows (from top)
 
______________________________________________________________
 
  0°   |       |       _   _                        °
   °   |       |      | |_| |_|         |      |    °
   °   |    ###|       Antenna          |      |    °
   °   |###    |                        |      |    °
   °   |       |        H M - 1 0       |      |    °
   °   |       |        C H I P         |      |    °
   °   |       |                        |      |
  7°   |       |                                    °
        Jumpers                                     °
  8°                                                °
   °                                                °
   °                                        _       °
   °                 Status *         #    |*|5V    °
   °                   Pwr  *         #    | |
 13°                                Reset  |_|3.3V
   °
   °
       #####
       #####
       #####
         #
         Usb Cable
         
___________________________________________________________________

 * Setup a the jumpers on the left so that TX is connected to
   D2 and RX is connected to D3 (if you want to choose other
   pins update the RX_PIN and TX_PIN constants)
 * select 5V at the switch on the right
 * Upload this sketch onto the Arduino
 *
**/

#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 3

#define B_ADDR_STR_LEN 15   //0x112233445566
#define B_ADDR_LEN 6
#define FIRMWARE_INFO_STRING_LEN 64

SoftwareSerial ble(RX_PIN, TX_PIN);

typedef struct ibeaconParameters
{
  uint8_t uuid[16];
  uint16_t major;
  uint16_t minor;
}__attribute__((packed)) ibeaconParameters_t;

typedef struct ibeaconDeviceFoundMsg
{
  uint8_t uuid[16];
  uint16_t major;
  uint16_t minor;
  uint8_t txPower;
  uint8_t address[B_ADDR_LEN];
  uint8_t rssi;
}__attribute__((packed)) ibeaconDeviceFound_t;


ibeaconParameters_t myBeacon;
uint8_t myBeaconName[20] = "littleBeacon";
ibeaconDeviceFound_t devInfo;

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


void setup()
{
  for(int i=0; i<16; i++)  myBeacon.uuid[i] = i;
  myBeacon.major = 0x1122;
  myBeacon.minor = 0x3344;
  Serial.begin(9600);
  ble.begin(57600);
  Serial.println("\nSend commands to shield via Serial\nExample: type G and then B\n");
}
 
void loop()
{
  while(Serial.available())
  {
    switch(Serial.read())
    {
      case 'T':  //TEST
        ble.write(1);
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'P':  //SET PARAMETERS
        ble.write(2);
        for(int i=0; i<sizeof(ibeaconParameters_t); i++)
        {
          ble.write( ( (uint8_t*) &myBeacon)[i] );
        }
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'N':  //SET NAME
        ble.write(3);
        for(int i=0; i<20; i++)
        {
          ble.write(myBeaconName[i]);
        }
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'G':  //GET FIRMWARE INFO STRING
      {
        ble.write(4); 
        boolean string_finished = false;
        for(int i=0; i<FIRMWARE_INFO_STRING_LEN; i++)
        {
          while(!ble.available());
          char c = ble.read();
          if(!string_finished && c=='\0') string_finished = true;
          if(!string_finished) Serial.write(c);
        }
        Serial.println("\\\\\n");
        break;
      }
      case 'B':  //START DEVICE
        ble.write(5);
        ble.write((uint8_t) 0);
        ble.write((uint8_t) 0);
        while(!ble.available());
        ble.read();
        break;
      case 'S':  //STOP DEVICE
        ble.write(6);
        while(!ble.available());
        ble.read();
        break;
    }
  }
  while(ble.available())
  {
    uint8_t opcode = ble.read();
    switch(opcode)
    {
      case 0x02:  //DEVICE FOUND
        for(int i=0; i<sizeof(ibeaconDeviceFound_t); i++)
        {
          while(! ble.available());
          ( (uint8_t*) &devInfo)[i] = ble.read();
        }
        Serial.print(bdAddr2Str(devInfo.address)); Serial.print(" @ "); 
        Serial.print(devInfo.uuid[0], HEX); Serial.print(devInfo.uuid[1], HEX); Serial.print(devInfo.uuid[2], HEX); Serial.print(devInfo.uuid[3], HEX);  Serial.print("-"); 
        Serial.print(devInfo.uuid[4], HEX); Serial.print(devInfo.uuid[5], HEX); Serial.print(devInfo.uuid[6], HEX); Serial.print(devInfo.uuid[7], HEX);  Serial.print("-"); 
        Serial.print(devInfo.uuid[8], HEX); Serial.print(devInfo.uuid[9], HEX); Serial.print(devInfo.uuid[10], HEX); Serial.print(devInfo.uuid[11], HEX);  Serial.print("-"); 
        Serial.print(devInfo.uuid[12], HEX); Serial.print(devInfo.uuid[13], HEX); Serial.print(devInfo.uuid[14], HEX); Serial.print(devInfo.uuid[15], HEX);  Serial.println("@"); 
        Serial.print(devInfo.rssi, DEC); Serial.print(" @ ");
        Serial.print(devInfo.major, DEC); Serial.print(", "); Serial.println(devInfo.minor, DEC); 
        break;
      default:
        Serial.write('!'); Serial.print(opcode, HEX); Serial.print(" ");
    }
  }
}


