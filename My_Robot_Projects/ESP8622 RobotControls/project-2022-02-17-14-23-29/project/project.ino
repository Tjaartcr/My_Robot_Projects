/*
   -- RobotControl17022022 --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.7 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.10.2 or later version;
     - for iOS 1.8.2 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_SERIAL_RX 2
#define REMOTEXY_SERIAL_TX 3
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,11,0,0,0,81,0,15,31,1,
  1,0,3,47,12,12,2,31,0,1,
  0,48,46,12,12,2,31,0,10,48,
  3,4,12,12,4,26,31,79,78,0,
  31,79,70,70,0,5,0,4,26,17,
  17,2,26,31,5,0,42,26,17,17,
  2,26,31,5,0,22,74,18,18,2,
  26,31,4,160,7,63,48,7,2,26,
  4,176,17,5,29,7,2,26 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t lefthand_1; // =1 if button pressed, else =0 
  uint8_t righthand_1; // =1 if button pressed, else =0 
  uint8_t pushSwitch_1; // =1 if state is ON, else =0 
  int8_t leftshoulderelbow_1_x; // =-100..100 x-coordinate joystick position 
  int8_t leftshoulderelbow_1_y; // =-100..100 y-coordinate joystick position 
  int8_t righthoulderelbow_1_x; // =-100..100 x-coordinate joystick position 
  int8_t righthoulderelbow_1_y; // =-100..100 y-coordinate joystick position 
  int8_t direction_1_x; // =-100..100 x-coordinate joystick position 
  int8_t direction_1_y; // =-100..100 y-coordinate joystick position 
  int8_t torso_1; // =-100..100 slider position 
  int8_t head_1; // =-100..100 slider position 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_LEFTHAND_1 10
#define PIN_RIGHTHAND_1 11


void setup() 
{
  RemoteXY_Init (); 
  
  pinMode (PIN_LEFTHAND_1, OUTPUT);
  pinMode (PIN_RIGHTHAND_1, OUTPUT);
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  
  digitalWrite(PIN_LEFTHAND_1, (RemoteXY.lefthand_1==0)?LOW:HIGH);
  digitalWrite(PIN_RIGHTHAND_1, (RemoteXY.righthand_1==0)?LOW:HIGH);
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay() 


}