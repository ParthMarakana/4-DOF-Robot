#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include<stdio.h>
#include<stdbool.h>
#include <Servo.h> 
/****引脚定义*****/   
#define PWMD 3 //D电机转速
#define DIRD 2 //D电机转向
#define PWMC 5 //C电机转速
#define DIRC 4 //C电机转向
#define PWMB 6 //B电机转速
#define DIRB 7 //B电机转向
#define PWMA 9 //A电机转速
#define DIRA 8 //A电机转向
//控制电机运动    宏定义
#define MOTOR_GO_FORWARD  {digitalWrite(DIRA,HIGH);analogWrite(PWMA,200);digitalWrite(DIRB,LOW);analogWrite(PWMB,200);digitalWrite(DIRC,HIGH);analogWrite(PWMC,200);digitalWrite(DIRD,LOW);analogWrite(PWMD,200);} //车体前进                              
#define MOTOR_GO_BACK   {digitalWrite(DIRA,LOW);analogWrite(PWMA,200);digitalWrite(DIRB,HIGH);analogWrite(PWMB,200);digitalWrite(DIRC,LOW);analogWrite(PWMC,200);digitalWrite(DIRD,HIGH);analogWrite(PWMD,200);}   //车体后退
#define MOTOR_GO_LEFT   {digitalWrite(DIRD,LOW);analogWrite(PWMD,255);digitalWrite(DIRB,LOW);analogWrite(PWMB,255);digitalWrite(DIRA,LOW);analogWrite(PWMA,255);digitalWrite(DIRC,LOW);analogWrite(PWMC,255);}  //车体左转
#define MOTOR_GO_RIGHT    {digitalWrite(DIRA,HIGH);analogWrite(PWMA,255);digitalWrite(DIRC,HIGH);analogWrite(PWMC,255);digitalWrite(DIRD,HIGH);analogWrite(PWMD,255);digitalWrite(DIRB,HIGH);analogWrite(PWMB,255);}  //车体右转
#define MOTOR_GO_STOP   {digitalWrite(DIRA,LOW);analogWrite(PWMA,0);digitalWrite(DIRB,LOW);analogWrite(PWMB,0);digitalWrite(DIRC,HIGH);analogWrite(PWMC,0);digitalWrite(DIRD,LOW);analogWrite(PWMD,0);}       //车体静止
#define MAX_PACKETSIZE 32  //串口接收缓冲区
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//脉宽范围定义
#define SERVOMIN 125//最小脉宽
#define SERVOMAX 625//最大脉宽
#define SERVOMID 375//中间脉宽
//角度范围小于180的单独定义
#define SERVOMIN_3 125
#define SERVOMAX_3 625
#define SERVOMIN_12 125
#define SERVOMAX_12 625
#define SERVOMIN_8 125
#define SERVOMAX_8 625

//数组声明
int Flag[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//每个舵机变量声明
uint16_t pulselen_0=SERVOMID;
float pulselen_1=SERVOMID;
float pulselen_2=SERVOMID;
uint16_t pulselen_3=SERVOMID;
uint16_t pulselen_4=SERVOMID;
uint16_t pulselen_5=SERVOMID;
uint16_t pulselen_6=SERVOMID;
uint16_t pulselen_7=SERVOMID;
uint16_t pulselen_8=SERVOMID;
//角度增量声明
uint16_t angle=0;
float angle1=0,n=4;


enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  DEF
}Drive_Num=DEF;

//电机控制标志量
bool flag1=false;
bool stopFlag = true;
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;

//小车电机控制
void CAR_Control()
{
	switch (Drive_Num) 
    {
      case GO_ADVANCE:MOTOR_GO_FORWARD;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_LEFT: MOTOR_GO_LEFT;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_RIGHT:MOTOR_GO_RIGHT;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_BACK:MOTOR_GO_BACK;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case STOP_STOP: MOTOR_GO_STOP;JogTime = 0;JogFlag=false;stopFlag=true;break;
      default:break;
    }
    Drive_Num=DEF;
    //小车保持姿态210ms
    if(millis()-JogTime>=210)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        MOTOR_GO_STOP;
      }
    }
}

//舵机驱动板调用函数
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000; // 1,000,000 us per second
  pulselength /= 60; // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096; // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void Servo1(uint16_t pulselen,int i)//舵机正转
{
  angle=pulselen-1;
    for(;pulselen>angle;pulselen=pulselen-1){;
    pwm.setPWM(i,0,pulselen);
    }
}
void Servo0(uint16_t pulselen,int i)//舵机反转
{
  angle=pulselen+1;
    for(;pulselen<angle;pulselen=pulselen+1){
    pwm.setPWM(i,0,pulselen);
    }
}
void Servo1_12(float pulselen,int i)//舵机1和舵机2
{
  angle1=pulselen-0.5;
    for(;pulselen>angle1;pulselen=pulselen-0.5){
    pwm.setPWM(i,0,pulselen);
 
    }

}
void Servo0_12(float pulselen,int i)
{
  angle1=pulselen+0.5;
 
    for(;pulselen<angle1;pulselen=pulselen+0.5){
    pwm.setPWM(i,0,pulselen);
    }

}

//标志位初始化
void Init()
{
  Flag[0]=0;  
  Flag[1]=0; 
  Flag[2]=0; 
  Flag[3]=0; 
  Flag[4]=0;
  Flag[5]=0;
  Flag[6]=0;  
  Flag[7]=0; 
  Flag[8]=0; 
  Flag[9]=0; 
  Flag[10]=0;
  Flag[11]=0;
  Flag[12]=0;
  Flag[13]=0;
  Flag[14]=0;
  Flag[15]=0;
  Flag[16]=0;
  Flag[17]=0;
}
//串口接收
void UART_Control(){
if(Serial.available()){ 
  char Uart_Date = Serial.read();
  
	if(buffUARTIndex > 0 && (millis() - preUARTTick >= 100))//超过100ms没接到数据，则认为已经接收到完整指令
	{ //data ready
		buffUART[buffUARTIndex] = 0x00;
		/*if((buffUART[0]=='C') && (buffUART[1]=='M') && (buffUART[2]=='D')) //若发送指令非法，则忽略
	    {
	    	;
	    }
		else Uart_Date=buffUART[0];
    	buffUARTIndex = 0;*/
    }
    switch (Uart_Date)    //串口控制指令
	{//蓝牙控制，方法：向蓝牙发送这些字母就可以就行控制舵机转动
      case'a':Flag[0]=true;Serial.print(Uart_Date);break;   //1舵机正转 1Steering gear
      case'A':Flag[1]=true;Serial.print(Uart_Date);break;   //1舵机反转 1Steering gear reversal
      case'b':Flag[2]=true;Serial.print(Uart_Date);break;   //2舵机正转 2Steering gear
      case'B':Flag[3]=true;Serial.print(Uart_Date);break;   //2舵机反转 2Steering gear reversal
      case'c':Flag[4]=true;Serial.print(Uart_Date);break;   //3舵机正转 3Steering gear
      case'C':Flag[5]=true;Serial.print(Uart_Date);break;   //3舵机反转 3Steering gear reversal
      case'd':Flag[6]=true;Serial.print(Uart_Date);break;   //4舵机正转 4Steering gear
      case'D':Flag[7]=true;Serial.print(Uart_Date);break;   //4舵机反转 4Steering gear reversal
      case'e':Flag[8]=true;Serial.print(Uart_Date);break;   //5舵机正转 5Steering gear
      case'E':Flag[9]=true;Serial.print(Uart_Date);break;   //5舵机反转 5Steering gear reversal
      case'f':Flag[10]=true;Serial.print(Uart_Date);break;  //6舵机正转 6Steering gear
      case'F':Flag[11]=true;Serial.print(Uart_Date);break;  //6舵机反转 6Steering gear reversal
      case'g':Flag[12]=true;Serial.print(Uart_Date);break;  //7舵机正转 7Steering gear
      case'G':Flag[13]=true;Serial.print(Uart_Date);break;  //7舵机反转 7Steering gear reversal
      case'h':Flag[14]=true;Serial.print(Uart_Date);break;  //8舵机正转 8Steering gear
      case'H':Flag[15]=true;Serial.print(Uart_Date);break;  //8舵机反转 8Steering gear reversal
      case'k':Flag[16]=true;Serial.print(Uart_Date);break;  //9舵机正转 9Steering gear
      case'K':Flag[17]=true;Serial.print(Uart_Date);break;  //9舵机反转 9Steering gear reversal
      case '2': Drive_Num=GO_ADVANCE;break;
      case '4': Drive_Num=GO_LEFT; break;
      case '6': Drive_Num=GO_RIGHT; break;
      case '8': Drive_Num=GO_BACK; break;
      case '5': Drive_Num=STOP_STOP;break;
      //case '1': Drive_Num=GO_BACK; Serial.print(Uart_Date);break;
      default:Serial.print(Uart_Date);break;
	}
     }
}

//初始化

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pwm.setPWM(0, 0, SERVOMID);
  pwm.setPWM(1, 0, SERVOMID);
  pwm.setPWM(2, 0, SERVOMID);
  pwm.setPWM(3, 0, SERVOMID);
  pwm.setPWM(4, 0, SERVOMID);
  pwm.setPWM(5, 0, SERVOMID);
  pwm.setPWM(6, 0, SERVOMID);
  pwm.setPWM(7, 0, SERVOMID);
  pwm.setPWM(8, 0, SERVOMID);//SERVOMIN_9
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);  
  pinMode(DIRB,OUTPUT);
  pinMode(PWMB,OUTPUT); 
  pinMode(DIRC,OUTPUT);
  pinMode(PWMC,OUTPUT);
  pinMode(DIRD,OUTPUT);
  pinMode(PWMD,OUTPUT);
  MOTOR_GO_STOP;
  delay(10);
}

void loop() {
  Init();
  UART_Control();
  CAR_Control();
  
  if(Flag[0]==true&&pulselen_0<=SERVOMAX){
    
    Servo0(pulselen_0,0);
    pulselen_0=pulselen_0+n;
  }
  if(Flag[1]==true&&pulselen_0>=SERVOMIN){
     
    Servo1(pulselen_0,0);
    pulselen_0=pulselen_0-n;
  }
  if(Flag[2]==true&&pulselen_1<=SERVOMAX_12){
    
    Servo0_12(pulselen_1,1);
    pulselen_1=pulselen_1+n;
  }
  if(Flag[3]==true&&pulselen_1>=SERVOMIN_12){
     
    Servo1_12(pulselen_1,1);
    pulselen_1=pulselen_1-n;
  }
  if(Flag[4]==true&&pulselen_2<=SERVOMAX_12){
    
    Servo1_12(pulselen_2,2);
    pulselen_2=pulselen_2+n;
  }
  if(Flag[5]==true&&pulselen_2>=SERVOMIN_12){
     
    Servo0_12(pulselen_2,2);
    pulselen_2=pulselen_2-n;
  }
  if(Flag[6]==true&&pulselen_3<=SERVOMAX_3){
    
    Servo0(pulselen_3,3);
    pulselen_3=pulselen_3+n;
  }
  if(Flag[7]==true&&pulselen_3>=SERVOMIN_3){
     
    Servo1(pulselen_3,3);
    pulselen_3=pulselen_3-n;
  }
  if(Flag[8]==true&&pulselen_4<=SERVOMAX){

    Servo0(pulselen_4,4);
    pulselen_4=pulselen_4+n;
  }
  if(Flag[9]==true&&pulselen_4>=SERVOMIN){

    Servo1(pulselen_4,4);
    pulselen_4=pulselen_4-n;
  }
  if(Flag[10]==true&&pulselen_5<=SERVOMAX){
    
    Servo0(pulselen_5,5);
    pulselen_5=pulselen_5+n;
  }
  if(Flag[11]==true&&pulselen_5>=SERVOMIN){
     
    Servo1(pulselen_5,5);
    pulselen_5=pulselen_5-n;
  }
  if(Flag[12]==true&&pulselen_6<=SERVOMAX){

    Servo0(pulselen_6,6);
    pulselen_6=pulselen_6+n;
  }
  if(Flag[13]==true&&pulselen_6>=SERVOMIN){

    Servo1(pulselen_6,6);
    pulselen_6=pulselen_6-n;
  }
  if(Flag[14]==true&&pulselen_7<=SERVOMAX){

    Servo0(pulselen_7,7);
    pulselen_7=pulselen_7+n;
  }
  if(Flag[15]==true&&pulselen_7>=SERVOMIN){

    Servo1(pulselen_7,7);
    pulselen_7=pulselen_7-n;
  }

   if(Flag[16]==true&&pulselen_8<=SERVOMAX_8){

    Servo0(pulselen_8,8);
    pulselen_7=pulselen_8+n;
  }
  if(Flag[17]==true&&pulselen_8>=SERVOMIN_8){

    Servo1(pulselen_8,8);
    pulselen_8=pulselen_8-n;
  }

  
  
}
