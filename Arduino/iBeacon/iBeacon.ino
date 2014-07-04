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
#include "ibeacon.h"

#define RX_PIN 2
#define TX_PIN 3

SoftwareSerial ble(RX_PIN, TX_PIN);

iBeaconAdvertData_t advertData;
uint8_t name[] = "littleBeacon";

struct deviceInfo
{
  uint8_t address[B_ADDR_LEN];
  uint8_t rssi;
  uint8_t advData[31];
}__attribute__((packed));
struct deviceInfo devInfo;

void setup()
{
  advertData.len1 = 2;
  advertData.type1 = 0x01; //GAP_ADTYPE_FLAGS
  advertData.flags = 0x06; //GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL
  advertData.len2 = 26;
  advertData.type2 = 0xFF; //GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  advertData.manufacturerID_lo = 0x4C;  //Apple
  advertData.manufacturerID_hi = 0x00;
  advertData.advertisement = 0x1502;
  advertData.major_lo = 0xBB;
  advertData.major_hi = 0xBB;
  advertData.minor_lo = 0xAA;
  advertData.minor_hi = 0xAA;
  advertData.txPower = 0x90;
  for(int i=0; i<16; i++) advertData.uuid[i] = i;
  
  Serial.begin(9600);
  ble.begin(9600);
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
      case 'A':  //SET ADVERTISING DATA
        ble.write(2);
        ble.write(sizeof(struct iBeaconAdvertData));
        for(int i=0; i<sizeof(struct iBeaconAdvertData); i++)
        {
          ble.write( ( (uint8_t*) &advertData)[i] );
        }
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'N':  //SET NAME
        ble.write(3);
        ble.write(strlen((const char*) name));
        for(int i=0; i<strlen((const char*) name); i++)
        {
          ble.write(name[i]);
        }
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'G':  //GET FIRMWARE INFO STRING
      {
        uint8_t length;
        ble.write(4);
        while(! ble.available()) ;
        length = ble.read();
        for(int i=0; i<length; i++)
        {
          while(!ble.available());
          Serial.write(ble.read());
        }
        Serial.println("\n");
        break;
      }
      case 'B':  //START DEVICE
        ble.write(5);
        ble.write((uint8_t) 0);
        ble.write((uint8_t) 0);
        while(!ble.available());
        ble.read(); //ACK
        break;
      case 'S':  //STOP DEVICE
        ble.write(6);
        while(!ble.available());
        ble.read(); //ACK
        break;
    }
  }
  while(ble.available())
  {
    uint8_t opcode = ble.read();
    switch(opcode)
    {
      case 0x02:  //DEVICE FOUND
      {
          uint8_t length;
          while(! ble.available())  ;
          length = ble.read();
          for(int i=0; i<length; i++)
          {
            while(! ble.available());
            ( (uint8_t*) &devInfo)[i] = ble.read();
          }
          Serial.print(bdAddr2Str(devInfo.address));
          if(isIBeaconAdvData(devInfo.advData)) Serial.println(" [IB]");
          else                                 Serial.println();
          break;
      }
      default:
        Serial.write('!'); Serial.print(opcode, HEX); Serial.print(" ");
    }
  }
}


