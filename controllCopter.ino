#include <Debug.h>
#include <WiFly.h>
#include <WiFlyClient.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

#include <SPI.h>


//char ssid[] = "CX-10WD-6DA5F0";
//char pass[] = "abc12345";
#define droneSSID "CX-10WD-6DA5F0"
#define AUTH  WIFLY_AUTH_OPEN 

#define UDP_HOST_IP "172.16.10.1"
#define UDP_REMOTE_PORT 8895
#define UDP_LOCAL_PORT 55555
#define LeftRight 1
#define ForBackward 2
#define Matchup 6
/*
byte turnLeft[] = {0xcc, 0x82, 0x75, 0x83, 0x30, 0x00, 0x44, 0x33};
byte turnRight[] = {0xcc, 0x82, 0x75, 0x84, 0xd0, 0x00, 0xa3, 0x33};
byte shiftLeft[] = {0xcc, 0x5a, 0x75, 0x80, 0x80, 0x00, 0x2f, 0x33};
byte shiftRight[] = {0xcc, 0xaa, 0x75, 0x80, 0x80, 0x00, 0xdf, 0x33};

byte forward[] = {0xcc, 0x82, 0x9d, 0x80, 0x80, 0x00, 0x1f, 0x33};
byte backward[] = {0xcc, 0x82, 0x4d, 0x80, 0x80, 0x00, 0xcf, 0x33};
byte decend[] = {0xcc, 0x98, 0x78, 0x00, 0x80, 0x00, 0x60, 0x33};
byte acend[] = {0xcc, 0x98, 0x78, 0xff, 0x80, 0x00, 0x9f, 0x33};

byte takeOff[] = {0xcc, 0x80, 0x69, 0x80, 0x80, 0x01, 0xe8, 0x33};
byte landing[] = {0xcc, 0x80, 0x69, 0x80, 0x80, 0x02, 0xeb, 0x33};*/
byte balance[] = {0xcc, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x33};
byte drone[] = {0xe0, 0xb9, 0x4d, 0x6d, 0xa5, 0xf0};

//cc9873808200e933
SoftwareSerial uart(2,3);
WiFly wifly(&uart);
//String str = (char*)buff;

const int SW_pin = 2;
const int X_pin = 0;
const int Y_pin = 1;

bool isBalanced = false;


void setupUDP(const char *host_ip, uint16_t remote_port, uint16_t local_port)
{
  char cmd[32];
  
  wifly.sendCommand("set w j 1\r", "AOK");   // enable auto join
  
  wifly.sendCommand("set i p 1\r", "AOK");
  snprintf(cmd, sizeof(cmd), "set i h %s\r", host_ip);
  wifly.sendCommand(cmd, "AOK");
  snprintf(cmd, sizeof(cmd), "set i r %d\r", remote_port);
  wifly.sendCommand(cmd, "AOK");
  snprintf(cmd, sizeof(cmd), "set i l %d\r", local_port);
  wifly.sendCommand(cmd, "AOK");
  wifly.sendCommand("save\r");
  wifly.sendCommand("reboot\r");
}

void setup() {
  // put your setup code here, to run once:
    uart.begin(9600);
    Serial.begin(9600);
    wifly.reset(); // reset the shield
    delay(3000);
    Serial.println("start");
    while(wifly.join(droneSSID, AUTH) == false)
    {
      Serial.println("Failed to connect to accesspoint. Will try again.");
    }
    Serial.println("Connected Success!");

    setupUDP(UDP_HOST_IP, UDP_REMOTE_PORT, UDP_LOCAL_PORT);
    
    pinMode(SW_pin, INPUT);
    digitalWrite(SW_pin, HIGH);

    delay(1000);
    wifly.clear();
}

int countL = 0;

int LRforMatch = 256; //用來計算第七位的左右平移變數
int FBforMatch = 256; //前後平移變數 因為圓點是零 設定256在之後紀錄變化
                      //只要-256就可以得到相較於零的正負值 方便計算

bool isAdjusted = false; //檢查調整用

void loop() {

    delay(30);
    for(int i = 0; i < 8; i++) //沒動作就傳送封包
      {
        wifly.write(balance[i]);
        Serial.print(balance[i], HEX);
      }
    Serial.println("");
    if(isAdjusted == false)//檢查是否平衡完成 以便後續動作
    {
      
      while (analogRead(X_pin)<300)
      {
        if((FBforMatch-256)>24) //超過原本平衡設定的最大值24 就僅傳送封包
        {
          for(int i = 0; i < 8; i++)
          {
          wifly.write(balance[i]);
          }
          delay(30);
        }
        else
        {
          balance[ForBackward]++; //調整動作對應位置封包
          FBforMatch++; //記錄相較於原點的變化
          balance[Matchup] = findMatchup(LRforMatch, FBforMatch); //計算第七位
          for(int i = 0; i < 8; i++) //並傳送封包
          {
            wifly.write(balance[i]);
          }
          delay(100);
        }
      }
      while (analogRead(X_pin)>750)
      {
        if((FBforMatch-256)<-24)
        {
          for(int i = 0; i < 8; i++)
          {
          wifly.write(balance[i]);
          }
          delay(30);
        }
        else
        {
          balance[ForBackward]--;
          FBforMatch--;
          balance[Matchup] = findMatchup(LRforMatch, FBforMatch);
          for(int i = 0; i < 8; i++)
          {
            wifly.write(balance[i]);
          }
          delay(100);
        }
      }
      while (analogRead(Y_pin)<300)
      {
        if((LRforMatch-256)>24)
        {
          for(int i = 0; i < 8; i++)
          {
          wifly.write(balance[i]);
          }
          delay(30);
        }
        else
        {
          balance[LeftRight]++;
          LRforMatch++;
          balance[Matchup] = findMatchup(LRforMatch, FBforMatch);
          for(int i = 0; i < 8; i++)
          {
            wifly.write(balance[i]);
          }
          delay(100);
        }
      }
      while (analogRead(Y_pin)>750)
      {
        if((LRforMatch-256)<-24)
        {
          for(int i = 0; i < 8; i++)
          {
          wifly.write(balance[i]);
          }
          delay(30);
        }
        else
        {
          balance[LeftRight]--;
          LRforMatch--;
          balance[Matchup] = findMatchup(LRforMatch, FBforMatch);
          for(int i = 0; i < 8; i++)
          {
            wifly.write(balance[i]);
          }
          delay(100);
        }
      }
    }
  
  
   
/*    while( analogRead(X_pin)<300 )  // 這裡是間歇轉動飛行
    {
       for(int i = 0 ; i < 8 ; i++)
       {
         wifly.write(forward[i]);
       }
       delay(30);
    }

    while( analogRead(X_pin)>750)
    {
        for(int i = 0 ; i < 8 ; i++)
       {
         wifly.write(backward[i]);
       }
       delay(30);
    }
    
    while(analogRead(Y_pin)<300)
    {
    //  right = true;
        if(countL > 500000)
       {
        countL = 0;
       }
//       if(left == true)
//       {
//        for(int y = 0; y < 16; y++)
//        {
//          for(int i = 0 ; i < 8 ; i++)
//          {
//          wifly.write(shiftRight[i]);
//          //wifly.write(turnLeft[i]);
//          countL += 1;
//          }
//        }
//          left = false;
//       }
        for(int i = 0 ; i < 8 ; i++)
       {
         wifly.write(shiftRight[i]);
     //    wifly.write(turnLeft[i]);
         countL += 1;
       }
      
       if(countL%110==0)
       {
         for(int y = 0; y < 15; y++)
        {
          for(int i = 0 ; i < 8 ; i++)
         {
           wifly.write(turnLeft[i]);
         }
         delay(30);
        }
       }
       delay(30);
    }

    while(analogRead(Y_pin)>750)
    {
    //    left = true;
       if(countL > 500000)
        {
        countL = 0;
        }
//        if(right == true)
//       {
//        for(int y = 0; y < 16; y++)
//        {
//          for(int i = 0 ; i < 8 ; i++)
//          {
//          wifly.write(shiftLeft[i]);
//          //wifly.write(turnLeft[i]);
//          countL += 1;
//          }
//          right = false;
//        }
//       }
        for(int i = 0 ; i < 8 ; i++)
       {
         wifly.write(shiftLeft[i]);
   //      wifly.write(turnRight[i]);
         countL+=1;
       }
       if(countL%150==0)
       {
        for(int y = 0; y < 15; y++)
        {
          for(int i = 0 ; i < 8 ; i++)
          {
            wifly.write(turnRight[i]);
          }
          delay(30);
        }
       }
       delay(30);
    }*/
}

byte findMatchup(int LR, int FB) // 此函式計算封包第七位數值
{
  int temp1 = 0;
  int temp2 = 0;
  temp1 = LR-256;  //前面變數的設定 讓這裡計算較方便
  temp2 = FB-256;
  if (temp1*temp2 == 0) //其中一個為0 
  {
      if(temp1 < 0)
        return char(256-abs(temp1)); //256 當作從零減
      else if(temp1 > 0)
        return char(abs(temp1));
      else
      {
        if(temp2 < 0)
          return char(256-abs(temp2));
        else if(temp2 > 0)
          return char(abs(temp2));
        else
          return 0x00;
      }     
  }
  else //兩者皆不為零 需要計算
  {
      if(abs(temp1) == abs(temp2)) //兩者數值相等
      {
         if(temp1*temp2 > 0)
            return char(abs(temp1)+abs(temp2));
         else
            return 0x00;
      }
      else //數值不等，用較大數值去減較小數值，然後判斷相乘正負來決定從零加還是從256減 
      {
         int temp3 = 0;
         temp3 = max(abs(temp1),abs(temp2)) - min(abs(temp1),abs(temp2));
         if(temp1*temp2 > 0)
            return char(temp3);
         else
            return char(256-temp3);
      }
  }
  
}

