
#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9;

int hall_count_a;
int hall_count_b;

int cmd_loop;

int speed_loop;

int speed[2] = {0, 0};
int dir[2] = {0, 0};

unsigned int adc[6];
int ave_sesor[6];

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

void sensor_read()
{
  adc[0] = analogRead(A0);
  adc[1] = analogRead(A1);
  adc[2] = analogRead(A2);
  adc[3] = analogRead(A3);
  adc[4] = analogRead(A4);
  adc[5] = analogRead(A5);
}

void avarage_sensor()
{

  unsigned int i;
  for (i = 0; i < 6; i++)
  {
    if (adc[i] > 300)
    {
      ave_sesor[i] += 10;
      if (ave_sesor[i] > 500)
      {
        ave_sesor[i] = 500;
      }
    }
    ave_sesor[i]--;
    if (ave_sesor[i] < 0)
    {
      ave_sesor[i] = 0;
    }
  }
}

void lineSensor_debug()
{
  int i;
  for (i = 0; i < 6; i++)
  {
    if (ave_sesor[i] > 400)
    {
      Serial.print("#");
    }
    else
    {
      Serial.print("_");
    }
    //Serial.print(ave_sesor[i]);
    //Serial.print("\t");
  }
}

void dir_set_by_sensor()
{


    if (ave_sesor[0] > 400)
   {
     dir[0] = 1;
     dir[1] = 0;

     speed[0] = 10;
     speed[1] = 10;
   }

   if (ave_sesor[5] > 400)
   {
     dir[0] = 0;
     dir[1] = 1;

     speed[0] = 10;
     speed[1] = 10;
   }

    if (ave_sesor[1] > 400)
   {
     dir[0] = 1;
     dir[1] = 0;

     speed[0] = 100;
     speed[1] = 50;
   }

   if (ave_sesor[4] > 400)
   {
     dir[0] = 0;
     dir[1] = 1;

     speed[0] = 50;
     speed[1] = 100;
   }

   if (ave_sesor[2] > 400)
   {
     dir[0] = 1;
     dir[1] = 1;

     speed[0] = 100;
     speed[1] = 100;
   }

   if (ave_sesor[3] > 400)
   {
     dir[0] = 1;
     dir[1] = 1;

     speed[0] = 100;
     speed[1] = 100;
   }


}


void motor_1_cmd(boolean ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (ccw)
  {
    stmp[0] = 0x10;
  }
  else
  {
    stmp[0] = 0x20;
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1CE, 0, 8, stmp);
}

void motor_2_cmd(boolean ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (ccw)
  {
    stmp[0] = 0x10;
  }
  else
  {
    stmp[0] = 0x20;
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1D0, 0, 8, stmp);
}

void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
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


  motor_1_cmd(dir[0], speed[0]);
  motor_2_cmd(dir[0], speed[0]);

  delay(3000);
  Serial.println(" START ! ");

}

void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  sensor_read();
  avarage_sensor();

  if (CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    if (canId == 0x1FA)
    {
      hall_count_a = int(buf[1]);
    }

    if (canId == 0x1FB)
    {
      hall_count_b = int(buf[1]);
    }

    Serial.print(hall_count_a);

    Serial.print("\t");

    Serial.print(hall_count_b);

    // Serial.print("\t");
    // Serial.print(speed);

    Serial.print("\t");
    lineSensor_debug();

    Serial.println();
  }

  cmd_loop++;

  //  dir[0] = 0;
  //  dir[1] = 1;
   
    //speed[0] = 0;
    //speed[1] = 0;

  
   

  if (cmd_loop > 50)
  {

    dir_set_by_sensor();
    
    motor_1_cmd(dir[0], speed[0]);
    motor_2_cmd(dir[1], speed[1]);

    cmd_loop = 0;
  }

  // speed_loop++;
  // if (speed_loop > 200)
  // {
  //   speed_loop = 0;

  //   if (speed_flug)
  //   {
  //     speed++;
  //     if (speed > 250)
  //     {
  //       speed_flug = 0;
  //     }
  //   }
  //   else
  //   {
  //     speed--;
  //     if (speed == 0)
  //     {
  //       speed_flug = 1;
  //     }
  //   }
  // }
}

//END FILE
