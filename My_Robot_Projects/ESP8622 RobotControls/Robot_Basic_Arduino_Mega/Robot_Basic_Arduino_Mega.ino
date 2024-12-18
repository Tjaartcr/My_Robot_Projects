/*
   -- New project --
   
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
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,8,0,0,0,47,0,15,31,1,
  5,32,2,45,18,18,2,26,31,5,
  32,41,45,18,18,2,26,31,5,32,
  24,67,18,18,2,26,31,4,176,3,
  22,55,14,2,26,1,0,46,4,12,
  12,2,31,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t left_1_x; // =-100..100 x-coordinate joystick position 
  int8_t left_1_y; // =-100..100 y-coordinate joystick position 
  int8_t right_1_x; // =-100..100 x-coordinate joystick position 
  int8_t right_1_y; // =-100..100 y-coordinate joystick position 
  int8_t direction_1_x; // =-100..100 x-coordinate joystick position 
  int8_t direction_1_y; // =-100..100 y-coordinate joystick position 
  int8_t torso_1; // =-100..100 slider position 
  uint8_t start_1; // =1 if button pressed, else =0 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_START_1 A14


void setup() 
{
  RemoteXY_Init (); 
  
  pinMode (PIN_START_1, OUTPUT);
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  
  digitalWrite(PIN_START_1, (RemoteXY.start_1==0)?LOW:HIGH);
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay() 


}

