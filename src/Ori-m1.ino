
#include <SPI.h>
#include "mcp_can.h"

#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); // RX, TX

int TD[16];
int t_count;

#define SIZE 8

const int SPI_CS_PIN = 9;

int hall_count_a;
int hall_count_b;

int old_hall_count_a;
int old_hall_count_b;

long sum_hall_count_a;
long sum_hall_count_b;

int cmd_loop;

int speed_loop;

int speed[2] = {0, 0};
int dir[2] = {0, 0};

unsigned int adc[6];
int ave_sesor[6];

bool lrFlug;

char RD[16];
int r_count;
char *argv[SIZE];

long aim_distance;
long aim_roll;

bool LineTraceMode;
bool AimTarget;

int top_speed;

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

void tag_buff_clear() {
  for (int i = 0; i < 16; i++) {
    TD[i] = 0x00;
  }
  t_count = 0;
}

void tag_buff_print() {
  char md;
  
  for (int i = 0; i < 16; i++) {
    md = TD[i];
    Serial.print(TD[i]);
    Serial.print("\t");
    Serial.println(md);
    
    
  }

}
void tag_buff_numprint() {
  char md;
  for (int i = 2; i < 11; i++) {
    md = TD[i];
    Serial.print(md);
  }
  Serial.println("");

}

void clear_buff(void)
{
  r_count = 0;
  for (int i = 0; i < 15; i++)
  {
    RD[i] = 0x00;
  }
}

void clear_encorder(void)
{

  sum_hall_count_a = 0;
  sum_hall_count_b = 0;
  aim_distance = 0;
  aim_roll = 0;
}

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

    

    // Serial.print(ave_sesor[i]);
    // Serial.print("\t");
  }
  Serial.print("    ");

    if(lrFlug){
      Serial.print("L");
  }else{
      Serial.print("R");
  }
  Serial.println("");
}

void dir_set_by_sensor()
{
  int sum_r,sum_l;

  sum_r = ave_sesor[5] + ave_sesor[4];
  sum_l = ave_sesor[0] + ave_sesor[1];

  if((ave_sesor[5] > 100) || (ave_sesor[0] > 100) ){
  if(sum_l > sum_r){
   lrFlug = false;
  }else{
    lrFlug = true;
  }
  }

  if(lrFlug){
    dir[0] = 0;
  dir[1] = 1;
  }else{
    dir[0] = 1;
  dir[1] = 0;
  }
      speed[0] = 50;
      speed[1] = 50;

  if (ave_sesor[0] > 200)
  {
    dir[0] = 1;
    dir[1] = 0;

    speed[0] = 50;
    speed[1] = 50;
  }

  if (ave_sesor[5] > 200)
  {
    dir[0] = 0;
    dir[1] = 1;

    speed[0] = 50;
    speed[1] = 50;
  }

  if (ave_sesor[1] > 200)
  {
    dir[0] = 1;
    dir[1] = 0;

    speed[0] = 100;
    speed[1] = 0;
  }

  if (ave_sesor[4] > 200)
  {
    dir[0] = 0;
    dir[1] = 1;

    speed[0] = 0;
    speed[1] = 100;
  }

  if (ave_sesor[2] > 200)
  {
    dir[0] = 1;
    dir[1] = 1;

    speed[0] = top_speed;
    speed[1] = top_speed;
  }

  if (ave_sesor[3] > 200)
  {
    dir[0] = 1;
    dir[1] = 1;

    speed[0] = top_speed;
    speed[1] = top_speed;
  }
}

void motor_1_cmd(int ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  stmp[0] = 0xFF;
  stmp[1] = 0x00;
  stmp[2] = 100;

  if (spd != 0)
  {
    if (ccw == 1)
    {
      stmp[0] = 0x10;
      stmp[2] = 0;
    }
    if (ccw == 0)
    {
      stmp[0] = 0x20;
      stmp[2] = 0;
    }
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1CE, 0, 8, stmp);
}

void motor_2_cmd(int ccw, unsigned int spd)
{

  unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  stmp[0] = 0xFF;
  stmp[1] = 0x00;
  stmp[2] = 100;

  if (spd != 0)
  {
    if (ccw == 1)
    {
      stmp[0] = 0x10;
      stmp[2] = 0;
    }
    if (ccw == 0)
    {
      stmp[0] = 0x20;
      stmp[2] = 0;
    }
  }

  stmp[1] = char(spd);

  CAN.sendMsgBuf(0x1D0, 0, 8, stmp);
}

int diff_old_hall_count(int hall_count, int old_hall_count)
{
  int sum;

  sum = hall_count - old_hall_count;

  if (hall_count == 0 && old_hall_count == 255)
  {
    sum = 1;
  }
  if (hall_count == 255 && old_hall_count == 0)
  {
    sum = -1;
  }

  // if (sum < -200)
  // {
  //   sum = 255 - sum;
  // }
  // if (sum > 200)
  // {
  //   sum = (sum - 255);
  // }

  return sum;
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

  lrFlug = false;

  LineTraceMode = false;  //ライントレースモード
  AimTarget = false;      //距離モード

  delay(3000);
  clear_buff();
  tag_buff_clear();
  top_speed = 120;

  Serial.println(" START ! ");
}

void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];
  char cn;
  int sum_cnt;
  int d;

  sensor_read();
  avarage_sensor();

  if (CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    if (canId == 0x1FA)
    {
      old_hall_count_a = hall_count_a;
      hall_count_a = int(buf[1]);
      sum_hall_count_a = sum_hall_count_a + diff_old_hall_count(hall_count_a, old_hall_count_a);
    }

    if (canId == 0x1FB)
    {
      old_hall_count_b = hall_count_b;
      hall_count_b = int(buf[1]);
      sum_hall_count_b = sum_hall_count_b - diff_old_hall_count(hall_count_b, old_hall_count_b);
    }

    if (AimTarget)
    {
      if (aim_distance < abs(sum_hall_count_a))
      {
        clear_encorder();

        dir[0] = 9;
        dir[1] = 9;

        speed[0] = 0;
        speed[1] = 0;
        AimTarget = false;
      }
    }

    // Serial.print(hall_count_a);
    // Serial.print("\t");
    // Serial.print(hall_count_b);
    // Serial.print("\t");

    // Serial.print(sum_hall_count_a);
    // Serial.print("\t");
    // Serial.print(sum_hall_count_b);
    // Serial.print("\t");

    // Serial.print(speed);

    // Serial.print("\t");

    lineSensor_debug();

    //Serial.println();
  }

  cmd_loop++;

  if (cmd_loop > 10)

  {
    if (LineTraceMode)
    {
      dir_set_by_sensor();
    }

    motor_1_cmd(dir[0], speed[0]);
    motor_2_cmd(dir[1], speed[1]);

    // Serial.print(speed[0]);
    // Serial.print("\t");
    // Serial.print(speed[1]);

    // Serial.println("");

    cmd_loop = 0;
  }
  
  while (mySerial.available()) {
    d = mySerial.read();

    if (TD[0] == 0x02 || d == 0x02) {
      TD[t_count] = d;
      t_count++;


      if (t_count > 13) {
        //buff_print();
        tag_buff_numprint();
        tag_buff_clear();
      }
    }

    //    Serial.write(mySerial.read());
  }

  while (Serial.available())
  {

    cn = Serial.read();

    if ((cn == 'S') || (cn == 'D') || (cn == 'F') || (cn == 'R') || (cn == 'L') || (cn == 'B') || (cn == 'f') || (cn == 'r') || (cn == 'l') || (cn == 'b') || (cn == 'T') || (cn == 't') ||
        (RD[0] == 'S') || (RD[0] == 'D') || (RD[0] == 'F') || (RD[0] == 'R') || (RD[0] == 'L') || (RD[0] == 'B') || (RD[0] == 'f') || (RD[0] == 'r') || (RD[0] == 'l') || (RD[0] == 'b') || (RD[0] == 'T') || (RD[0] == 't'))
    {

      if (cn == 0x0d)
      {
        Serial.println(RD);
        int i = 0;
        argv[i] = strtok(RD, ",");

        do
        {
          argv[++i] = strtok(NULL, ",");
        } while ((i < SIZE) && (argv[i] != NULL));

        if (RD[0] == 'S')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 9;
          dir[1] = 9;

          speed[0] = 0;
          speed[1] = 0;
        }

        if (RD[0] == 'D')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = atoi(argv[1]);
          dir[1] = atoi(argv[2]);

          speed[0] = atoi(argv[3]);
          speed[1] = atoi(argv[4]);
        }
        if (RD[0] == 'F')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 1;
          dir[1] = 1;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);
        }

        if (RD[0] == 'B')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 0;
          dir[1] = 0;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);
        }

        if (RD[0] == 'L')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 1;
          dir[1] = 0;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);
        }

        if (RD[0] == 'R')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 0;
          dir[1] = 1;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);
        }

        if (RD[0] == 'f')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 1;
          dir[1] = 1;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);

          AimTarget = true;
          aim_distance = atoi(argv[2]) * 5;
        }

        if (RD[0] == 'b')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 0;
          dir[1] = 0;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);

          AimTarget = true;
          aim_distance = atoi(argv[2]) * 5;
        }

        if (RD[0] == 'l')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 1;
          dir[1] = 0;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);

          AimTarget = true;
          aim_distance = atoi(argv[2]) / 1.2;
        }

        if (RD[0] == 'r')
        {
          LineTraceMode = false;
          clear_encorder();

          dir[0] = 0;
          dir[1] = 1;

          speed[0] = atoi(argv[1]);
          speed[1] = atoi(argv[1]);

          AimTarget = true;
          aim_distance = atoi(argv[2]) / 1.2;
        }

        if (RD[0] == 'T')
        {
          LineTraceMode = true;
          clear_encorder();
          top_speed = 120;
        }
        if (RD[0] == 't')
        {
          LineTraceMode = true;
          clear_encorder();
          top_speed = atoi(argv[1]);
        }

        motor_1_cmd(dir[0], speed[0]);
        motor_2_cmd(dir[1], speed[1]);

        clear_buff();
      }
      else
      {
        RD[r_count] = cn;
        r_count++;
      }
    }
  }
}

//END FILE
