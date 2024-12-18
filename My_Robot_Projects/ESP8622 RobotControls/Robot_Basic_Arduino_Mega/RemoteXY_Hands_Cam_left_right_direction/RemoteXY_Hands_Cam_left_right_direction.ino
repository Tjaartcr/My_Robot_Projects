/*
   -- Robo_Hands_cam_directions --
   
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
  { 255,10,0,0,0,48,0,15,31,1,
  5,0,4,45,20,20,15,26,31,5,
  0,39,45,20,20,15,26,31,5,0,
  22,73,20,20,1,26,31,5,32,4,
  14,20,20,6,26,31,5,0,39,14,
  20,20,3,26,31 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t left_1_x; // =-100..100 x-coordinate joystick position 
  int8_t left_1_y; // =-100..100 y-coordinate joystick position 
  int8_t right_1_x; // =-100..100 x-coordinate joystick position 
  int8_t right_1_y; // =-100..100 y-coordinate joystick position 
  int8_t direction_1_x; // =-100..100 x-coordinate joystick position 
  int8_t direction_1_y; // =-100..100 y-coordinate joystick position 
  int8_t Shoulder_CAM_x; // =-100..100 x-coordinate joystick position 
  int8_t Shoulder_CAM_y; // =-100..100 y-coordinate joystick position 
  int8_t Left_Right_Hands_x; // =-100..100 x-coordinate joystick position 
  int8_t Left_Right_Hands_y; // =-100..100 y-coordinate joystick position 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////



void setup() 
{
  RemoteXY_Init (); 
  
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay() 


}
