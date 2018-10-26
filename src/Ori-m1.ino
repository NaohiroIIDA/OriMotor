
#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9;
const int LED        = 8;
boolean ledON        = 1;

int hall_count_a;
int hall_count_b;

int cmd_loop;

int speed_loop;

int speed; 
int speed_flug; 

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin




void motor_1_cmd(boolean ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (ccw) {
    stmp[0] = 0x20;
  } else {
    stmp[0] = 0x10;
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1CE, 0, 8, stmp);
}


void motor_2_cmd(boolean ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (ccw) {
    stmp[0] = 0x20;
  } else {
    stmp[0] = 0x10;
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1D0, 0, 8, stmp);
}


void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");

  cmd_loop = 0;

  speed_loop = 0;

  hall_count_a = 0;
  hall_count_b = 0;

  speed = 0;
  speed_flug = 1;

}


void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  


  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    if (canId == 0x1FA) {
      hall_count_a = int(buf[1]);
    }

    if (canId == 0x1FB) {
      hall_count_b = int(buf[1]);
    }

    Serial.print(  hall_count_a );

    Serial.print( "\t" );

    Serial.print(  hall_count_b );
    Serial.print( "\t" );

    Serial.print(  speed );


    Serial.println();
  }

  cmd_loop++;

  

  if (cmd_loop > 5)
  {
    motor_1_cmd( 0, speed);
    motor_2_cmd( 0, speed);

    cmd_loop = 0;

  }

  speed_loop++;
  if(speed_loop > 200)
  {
    speed_loop = 0;

    if(speed_flug){ 
      speed++;
      if(speed > 250){
        speed_flug = 0;
      }
    
    }else{
      speed--;
      if(speed == 0 ){
        speed_flug = 1;
      }
    }
  }

}

//END FILE
