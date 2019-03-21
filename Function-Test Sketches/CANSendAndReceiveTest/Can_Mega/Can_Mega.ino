#include <SPI.h>
#include "mcp_can.h"

MCP_CAN CAN(49); // 53 for mega, 49 for our new board

void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Shield init fail");
    delay(1000);
  }
  Serial.println("CAN BUS init ok!");
}

//convert 2 32 bits integer to 8 bytes array
typedef union {
  struct {
    uint32_t sspeed;
    uint32_t angle;
  };
  uint8_t bytes[8];
} speedAngleMessage;

//convert 4 byte array to 32 bits integer
typedef union {
  byte arr[4];
  uint32_t integer;
} bufToInt;

//send actual speed and angle to high level through can bus
void sendMSG(int actual_speed, int actual_angle) {
  
  if (actual_speed < 0) { // check if the speed valid
    Serial.println("Speed can not be negative!!! Ignored");
  }
  else {
    speedAngleMessage MSG;
    MSG.sspeed = actual_speed;
    MSG.angle = actual_angle;

    speedAngleMessage MSG2;
    MSG2.sspeed = actual_speed * 2;
    MSG2.angle = actual_angle * 2;
    CAN.sendMsgBuf(0x0A, 0, 8, (uint8_t*)&MSG);
    delay(1000);
    CAN.sendMsgBuf(0x0B, 0, 8, (uint8_t*)&MSG2); 
    delay(1000);
    Serial.println("MEGA SEND CAN MESSAGE!!!");
  }
}

void receiveMSG() {
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
    
    //if (canId == 0X05) { // the drive ID receive from high level
      Serial.print("RECEIVE DRIVE CAN MESSAGE FROM HIGH LEVEL WITH ID: ");
      Serial.println(canId, HEX);
      
      int low_result = (unsigned int)(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
      Serial.print("Speed: ");
      Serial.print(low_result, DEC);

      int high_result = (unsigned int)(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|(buf[4]);
      Serial.print("  Angle: ");
      Serial.println(high_result, DEC);

      /*
      bufToInt low_converter;
      low_converter.arr[0] = buf[0];
      low_converter.arr[1] = buf[1];
      low_converter.arr[2] = buf[2];
      low_converter.arr[3] = buf[3];
      Serial.print("Speed: ");
      Serial.print(low_converter.integer, DEC);

      bufToInt high_converter;
      high_converter.arr[0] = buf[4];
      high_converter.arr[1] = buf[5];
      high_converter.arr[2] = buf[6];
      high_converter.arr[3] = buf[7];
      Serial.print("  Angle: ");
      Serial.println(high_converter.integer, DEC);
      */
    //}
  } 
}

void loop()
{
  receiveMSG();
  //sendMSG(12345, -9);
}
