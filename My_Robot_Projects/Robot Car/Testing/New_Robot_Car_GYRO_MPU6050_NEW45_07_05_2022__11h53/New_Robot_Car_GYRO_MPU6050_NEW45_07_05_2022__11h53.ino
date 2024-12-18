//**************************************************************  
  ////                 GYRO MPU-6050
  
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

//**************************************************************

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#include<UltraDistSensor.h>

UltraDistSensor mysensor4;   // BOTTOM CENTER SCAN SERVO DISTANCE SENSOR (CAR)
UltraDistSensor mysensor5;   // BOTTOM LEFT DISTANCE SENSOR (CAR)
UltraDistSensor mysensor6;   // BOTTOM RIGHT DISTANCE SENSOR (CAR)



//**************************************************************
//                             DISTANCE CONTROL

int Distreading4;    // Bottom Middel SCANNER SENSOR
int Distreading4L;    // Bottom LEFT Middel SCANNER SENSOR
int Distreading4R;    // Bottom RIGHT Middel SCANNER SENSOR (CAR)
int Distreading5;    // Bottom LEFT SENSOR (CAR)
int Distreading6;    // Bottom RIGHT SENSOR (CAR)

int MotionDetectcted = 0;

int CloseSetPoint3 = 150;         //  Centre Dist Set point Closeby
int FarSetPoint3 = 300;         //  Centre Dist Set point Far

int CloseSetPoint4 = 50;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4 = 400;         //  Bottom Centre Dist Set point Far

int CloseSetPoint5L = 100;         // Bottom LEFT Dist Set point Closeby
int FarSetPoint5L = 400;         //  Bottom LEFT Dist Set point Far

int CloseSetPoint5R = 100;         // Bottom RIGHT Dist Set point Closeby
int FarSetPoint5R = 400;         //  Bottom RIGHT Dist Set point Far

// ************************************************************************
//       ROBOT CAR SPEED CONTROL

unsigned long rampUpEndTime = 10000;
unsigned long rampDnEndTime = 20000;
unsigned long rampStartTime;
unsigned long rampRunner ;
unsigned long rampProgres;

int startSpeedForward = 0;
int stopSpeedForward = 250;

#define speedForwardAI 100
#define speedForwardLeftAI 100
#define speedForwardRightAI 100

int speedRampForwardAI = 0;

#define speedBackwardAI 100
#define speedBackwardLeftAI 100
#define speedBackwardRightAI 100

#define speedRampBackwardAI 100

#define speedLeftAI 100
#define speedLeftLeftAI 100    // Turn Left Left Wheel Speed
#define speedLeftRightAI 100    // Turn Left Right Wheel Speed

#define speedRampLeftAI 100

#define speedRightAI 100
#define speedRightLeftAI 100    // Turn Right Left Wheel Speed
#define speedRightRightAI 130    // Turn Right Right Wheel Speed

#define speedRampRightLeftAI 80


//*************************************************************
//     SCANNER ANGLE CONTROL

#include <Servo.h>

int ScanAngleLeft = 100;
int ScanAngleRight = 80;

int ScanAngleLC = 100;
int ScanAngleRC = 80;

int AngleBotLeft = 0;
int AngleBotRight = 0;

Servo myservoScan;  // create servo object to control a servo
Servo myservoScanB;  // create servo object to control a servo

//***********************************************
//      STANDBY TIMER CONTROL

unsigned long StandbySetTime = 30000;
unsigned long StandbyStartTime;
unsigned long StandbyProgress;
unsigned long StandbyResetTime = 2000;

int StandbyActive = 0;


//***********************************************
//      SCANNER LEFT / RIGHT CONTROL

unsigned long ServoMoveTime = 1000;
unsigned long ServoMoveStartTime;
unsigned long Progress;
unsigned long ResetTime = 2000;

int StartAngleM = 60;
int StopAngleM =  120;

int StartAngle = 0;
int StopAngle =  180;

int StartAngleR = 40;
int StopAngleR =  140;

unsigned long Angle;
unsigned long AngleM;
unsigned long AngleL;
unsigned long AngleR;

//***********************************************
//            SERIAL PRINT TIMER

unsigned long ServoMoveTime2 = 50;
unsigned long ServoMoveStartTime2;
unsigned long Progress2;
unsigned long ResetTime2 = 1000;
int StartAngle2 = 0;
int StopAngle2 =  180;

unsigned long Angle2;

int ledPinState = 0;

//*****************************************************************
//                 ENCODER 1 (LEFT)

volatile boolean TurnDetected1;  // need volatile for Interrupts
volatile boolean rotationdirection1; // CW or CCW rotation

const int PinCLK1 = 2; // 2 Generating interrupts using CLK signal

int WheelPosition1 = 0; // To store Stepper Motor Position
int PrevPosition1;   // Previous Rotary Position Value to check accuracy
int RPMavg1 = 0;
int RPM1 = 0;   // Revolutions Per Minute
int WheelCounter1 = 20; //Wheel has 20 gaps
int TimeStart1 = 0;
int TimeStart1a = 1000;
int Reset1 = 2000;
int TimeRunner1 = 0;
int RPMCounter1 = 0;
int RPMCounter1a = 0;
int LeftTurn45 = 0;
int LeftTurn90 = 0;
int LeftTurnCNT = 0;

int LeftCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
int LeftOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6


//Interrupt routine runs if CLK goes from HIGH to LOW

void isr1 () {

  //  delay(2); // delay for Debouncing  (4)
  if (digitalRead(PinCLK1))

    rotationdirection1 = digitalRead(PinCLK1);
  TurnDetected1 = true;

}

//*****************************************************************
//                 ENCODER 2 (RIGHT)

volatile boolean TurnDetected2;  // need volatile for Interrupts
volatile boolean rotationdirection2; // CW or CCW rotation

const int PinCLK2 = 3; // 3 Generating interrupts using CLK signal

int WheelPosition2 = 0; // To store Stepper Motor Position
int PrevPosition2;   // Previous Rotary Position Value to check accuracy
int RPMavg2 = 0;   // Revolutions Per Minute
int RPM2 = 0;   // Revolutions Per Minute
int WheelCounter2 = 20; //Wheel has 20 gaps
int TimeStart2 = 0;
int TimeStart2a = 1000;
int Reset2 = 2000;
int TimeRunner2 = 0;
int RPMCounter2 = 0;
int RPMCounter2a = 0;
int RightTurn45 = 0;
int RightTurn90 = 0;
int RightTurnCNT = 0;

int RightCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
int RightOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6


//Interrupt routine runs if CLK goes from HIGH to LOW

void isr2 () {

  //  delay(2); // delay for Debouncing  (4)
  if (digitalRead(PinCLK2))

    rotationdirection2 = digitalRead(PinCLK2);
  TurnDetected2 = true;

}
#define ledPin 13

#define LeftSpeedPin 7
#define RightSpeedPin 6

#define leftPin 10
#define rightPin 11

#define runPin 14
int RunState = 0;
#define run2Pin 15

int Run2State = 0;

float tempPin = A8;
int Temp = 0;

//
//////************************************************************************************
////                                      MOVE FORWARD CONTROL REMOTE XY
//
//
//void MoveForward() {
//
//
//
//  analogWrite (LeftSpeedPin, speedForward);
//  analogWrite (RightSpeedPin, speedForward);
//
//  digitalWrite(leftPin, HIGH);
//  digitalWrite(rightPin, HIGH);
//
//}
//
//////************************************************************************************
//
//
//////************************************************************************************
////                               MOVE BACKWARD CONTROL REMOTE XY
//
//
//
//void MoveBackward() {
//
//  analogWrite (LeftSpeedPin, speedBackward);
//  analogWrite (RightSpeedPin, speedBackward);
//
//  digitalWrite(leftPin, LOW);
//  digitalWrite(rightPin, LOW);
//
//}
////
//
//
//////************************************************************************************
////                               MOVE LEFT CONTROL REMOTE XY
//
//void MoveLeft() {
//
//  analogWrite (LeftSpeedPin, speedLeft);
//  analogWrite (RightSpeedPin, speedLeft);
//
//  digitalWrite(leftPin, HIGH);
//  digitalWrite(rightPin, LOW);
//
//}
//
//
//////************************************************************************************
//
//
//////************************************************************************************
////                               MOVE RIGHT CONTROL REMOTE XY
//
//void MoveRight() {
//
//  analogWrite (LeftSpeedPin, speedRight);
//  analogWrite (RightSpeedPin, speedRight);
//
//  digitalWrite(leftPin, LOW);
//  digitalWrite(rightPin, HIGH);
//
//}
//
//////************************************************************************************
//
//////************************************************************************************
////                               MOVE STOP CONTROL REMOTE XY
//
//void MoveStop() {
//
//
//  analogWrite (LeftSpeedPin, 0);
//  analogWrite (RightSpeedPin, 0);
//
//  //  digitalWrite(leftPin, LOW);
//  //  digitalWrite(rightPin, LOW);
//
//}
//
//////************************************************************************************
////************************************************************************************
//                        MOVE FORWARD RAMP TEST CONTROL AI


void MoveForwardRampTestAI() {

  //  analogWrite (LeftSpeedPin, speedForwardAI);
  //  analogWrite (RightSpeedPin, speedForwardAI);
  //
  //  digitalWrite(leftPin, HIGH);
  //  digitalWrite(rightPin, HIGH);


  //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
  //
  //      if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
  //        speedRampForwardAI = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);
  //
  ////      MoveForwardRampTestAI();
  //
  //  analogWrite (LeftSpeedPin, speedRampForwardAI);
  //  analogWrite (RightSpeedPin, speedRampForwardAI);
  //
  //  digitalWrite(leftPin, HIGH);
  //  digitalWrite(rightPin, HIGH);
  //      }
  //
  ////rampStartTime = millis();


}

////************************************************************************************

////************************************************************************************
//                               MOVE STOP LEFT CONTROL AI

void MoveStopLeftAI() {

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}

////************************************************************************************

////************************************************************************************
//                               MOVE STOP RIGHT CONTROL AI

void MoveStopRightAI() {

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}

////************************************************************************************

////************************************************************************************
//                                      MOVE FORWARD CONTROL AI


void MoveForwardAI() {

  analogWrite (LeftSpeedPin, speedForwardAI);
  analogWrite (RightSpeedPin, speedForwardAI);

  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, HIGH);

}

////************************************************************************************
////************************************************************************************
//                               MOVE STOP CONTROL AI

void MoveStopAI() {

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}

////************************************************************************************

////************************************************************************************
//                               MOVE BACKWARD CONTROL AI

void MoveBackwardAI() {

  analogWrite (LeftSpeedPin, speedBackwardAI);
  analogWrite (RightSpeedPin, speedBackwardAI);

  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);

}
//


////************************************************************************************
//                               MOVE LEFT 90 CONTROL AI

void MoveLeft90AI() {


  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  //  LeftTurnCNT = 0;
  //  RightTurnCNT = 0;

  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, LOW);
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);


  while ((LeftTurnCNT <= LeftTurn90) && (RightTurnCNT <= RightTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

    if (LeftTurnCNT <= LeftTurn90) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn90) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= LeftTurn90) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    else if (RightTurnCNT >= LeftTurn90) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }


  }

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);
  LeftTurnCNT = 0;
  RightTurnCNT = 0;
}


////************************************************************************************
////************************************************************************************
//                               MOVE LEFT 45 CONTROL AI


void MoveLeft45AI() {

  RightTurn45 = 8;  // 150 mm = 13.5 counts
  LeftTurn45 = 8;  // 150 mm = 13.5 counts

  //LeftTurnCNT = 0;
  //RightTurnCNT = 0;
  
  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, LOW);
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);
  
  while ((LeftTurnCNT <= LeftTurn90) && (RightTurnCNT <= RightTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

  if (LeftTurnCNT <= LeftTurn45) {
    digitalWrite(leftPin, HIGH);
    analogWrite (LeftSpeedPin, speedLeftLeftAI);
  }

  if (LeftTurnCNT >= LeftTurn45) {
    analogWrite (LeftSpeedPin, 0);
  }

  if (RightTurnCNT <= RightTurn45) {
    digitalWrite(rightPin, LOW);
    analogWrite (RightSpeedPin, speedLeftRightAI);
  }

  if (RightTurnCNT >= RightTurn45) {
    analogWrite (RightSpeedPin, 0);
  }

//if (Distreading6 <= 150){
//  
//  if (LeftTurnCNT <= LeftTurn45) {
//    digitalWrite(leftPin, LOW);
//    analogWrite (LeftSpeedPin, speedRightLeftAI);
//  }
//
//  if (LeftTurnCNT >= LeftTurn45) {
//    analogWrite (LeftSpeedPin, 0);
//  }
//
//  if (RightTurnCNT <= RightTurn45) {
//    digitalWrite(rightPin, HIGH);
//    analogWrite (RightSpeedPin, speedRightRightAI);
//  }
//
//  if (RightTurnCNT >= RightTurn45) {
//    analogWrite (RightSpeedPin, 0);
//  }
//}
}
}

////************************************************************************************


////************************************************************************************
//                               MOVE RIGHT 90 CONTROL AI

void MoveRight90AI() {

  //  LeftTurnCNT = 0;
  //  RightTurnCNT = 0;

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, HIGH);
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);

  while ((LeftTurnCNT <= LeftTurn90) && (RightTurnCNT <= RightTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }


    if (LeftTurnCNT <= RightTurn90) {
      analogWrite (LeftSpeedPin, speedRightLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn90) {
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= RightTurn90) {
      analogWrite (RightSpeedPin, speedRightRightAI);
    }

    else if (RightTurnCNT >= RightTurn90) {                                   //if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }


  }

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;
}


////************************************************************************************
////************************************************************************************
//                               MOVE RIGHT 45 CONTROL AI

void MoveRight45AI() {

  RightTurn45 = 7;  // 150 mm = 13.5 counts
  LeftTurn45 = 7;  // 150 mm = 13.5 counts

  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, HIGH);
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);

  while ((LeftTurnCNT <= LeftTurn90) && (RightTurnCNT <= RightTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

  if (LeftTurnCNT <= LeftTurn45) {
    digitalWrite(leftPin, LOW);
    analogWrite (LeftSpeedPin, speedRightLeftAI);
  }

  if (LeftTurnCNT >= LeftTurn45) {
    analogWrite (LeftSpeedPin, 0);
  }

  if (RightTurnCNT <= RightTurn45) {
    digitalWrite(rightPin, HIGH);
    analogWrite (RightSpeedPin, speedRightRightAI);
  }

  if (RightTurnCNT >= RightTurn45) {
    analogWrite (RightSpeedPin, 0);
  }
  }
}

////************************************************************************************
////************************************************************************************
//                               MOVE 500mm CONTROL AI


void Move500AI() {

//  RightTurn45 = 8;  // 150 mm = 13.5 counts
//  LeftTurn45 = 8;  // 150 mm = 13.5 counts

//LeftCntFor = 46
//RightCntFor = 46

  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, HIGH);
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);

  while ((LeftTurnCNT <= LeftCntFor) && (RightTurnCNT <= RightCntFor)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

    if (LeftTurnCNT <= LeftCntFor) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= RightCntFor) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }
  }
}
 
////************************************************************************************
//                               MOVE 500mm + Offset CONTROL AI
  
void MoveOffset() {

//  RightTurn45 = 8;  // 150 mm = 13.5 counts
//  LeftTurn45 = 8;  // 150 mm = 13.5 counts
//int LeftCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
//int LeftOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6
//int RightCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
//int RightOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6

  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, HIGH);
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  delay(1000);
  
  while ((LeftTurnCNT <= (LeftCntFor + LeftOffset)) && (RightTurnCNT <= (RightCntFor + RightOffset))) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }


    if (LeftTurnCNT <= (LeftCntFor + LeftOffset)) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    else if (LeftTurnCNT >= (LeftCntFor + LeftOffset)) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= (RightCntFor + RightOffset)) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    else if (RightTurnCNT >= (RightCntFor + RightOffset)) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }


  }

  }

  
void setup() {
  // put your setup code here, to run once:

  lcd.init();
  lcd.backlight();

  //  lcd1.init();
  //  lcd1.backlight();


  Serial.begin(115200);
  Wire.begin();

//***************************************************************
  ////                 GYRO MPU-6050
    
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //**************************************************************

  //  RemoteXY_Init ();

  //  pinMode (PIN_START_1, OUTPUT);

  pinMode (ledPin, OUTPUT);

  pinMode (leftPin, OUTPUT);
  pinMode (rightPin, OUTPUT);


  pinMode (runPin, INPUT_PULLUP);
  pinMode (run2Pin, INPUT_PULLUP);

  //****************************************************
  //                   ENCODERS

  pinMode (PinCLK1, INPUT);
  attachInterrupt (0, isr1, RISING);  //interrupt 0 always connected to pin 2 on Arduino Uno

  pinMode (PinCLK2, INPUT);
  attachInterrupt (1, isr2, RISING);  //interrupt 0 always connected to pin 2 on Arduino Uno

  TimeRunner1 = millis();
  TimeRunner2 = millis();

  //****************************************************

  myservoScanB.attach(5);  // create servo object to control a servo

  mysensor4.attach(A4, A5); //Trigger pin , Echo pin Front
  mysensor5.attach(A0, A1); //Trigger pin , Echo pin Left
  mysensor6.attach(A2, A3); //Trigger pin , Echo pin Right

  rampStartTime = millis();
  ServoMoveStartTime2 = millis();
  ServoMoveStartTime = millis();
  StandbyStartTime = millis();

//  digitalWrite(leftPin, HIGH);
//  digitalWrite(rightPin, HIGH);


}



//************************************************************************************************************************
//************************************************************************************************************************


void loop() {
  // put your main code here, to run repeatedly:
  
  //******************************************************************************************************
  //           SERIAL PRINT DELAY TIMER


  ledPinState = digitalRead(ledPin);

  Progress2 = millis() - ServoMoveStartTime2;     // Servo Head Progress

  if (Progress2 <= ServoMoveTime2) {
    digitalWrite(ledPin, HIGH);
    ledPinState = HIGH;
  }

  if (Progress2 >= ServoMoveTime2) {
    digitalWrite(ledPin, LOW);
    ledPinState = LOW;
  }

  if (Progress2 >= ResetTime2) {
    ServoMoveStartTime2 = millis();
  }



  //******************************************************************************************************



  //  //****************************************************
  ////                 GYRO MPU-6050
  
  mpu6050.update();
//  Serial.print("angleX : ");
//  Serial.print(mpu6050.getAngleX());
//  Serial.print("\tangleY : ");
//  Serial.print(mpu6050.getAngleY());
//  Serial.print("\tangleZ : ");
//  Serial.println(mpu6050.getAngleZ());
//
//  //**********************************************************
  //                   RUN STATE

  RunState = digitalRead(runPin);
  Run2State = digitalRead(run2Pin);

  //************************************************************************************************************
  //************************************************************************************************************
  //                SCANNER                SCANNER                SCANNER                SCANNER
  //************************************************************************************************************

  //************************** DISTANCE MEASURING IN MM ********************************************************

  Distreading4 = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
  Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
  Distreading4R = 0;  // BOTTOM MIDDEL SCANNER
  Distreading5 = (mysensor5.distanceInCm() * 10);  // BOTTOM LEFT SENSOR
  Distreading6 = (mysensor6.distanceInCm() * 10);  // BOTTOM RIGHT SENSOR

  //************************************************************
  //                     DISTANCE DETECTION CONTROL

  //      FRONT SENSOR

  //  if ((Distreading4 >= 0) || (Distreading5 >= 0) || (Distreading6 >= 0)){
  //    MotionDetectcted = HIGH;
  //
  //      StandbyStartTime = millis();
  //  }else{
  //    MotionDetectcted = LOW;
  //  }


  //************************************************************
  //                 ENCODER 1

  // Runs if rotation was detected

  if (TurnDetected1) {
    PrevPosition1 = WheelPosition1;  // Save previous position in variable
    WheelPosition1 = WheelPosition1 + 1;  // Increase Position by 1
    RPMCounter1 = RPMCounter1 + 1;
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();

  }

  TimeStart1 = millis() - TimeRunner1;

  if (TimeStart1 <= TimeStart1a) {
    RPM1 = 0;
    RPMCounter1 = 0;
  }

  if (TimeStart1 >= TimeStart1a) {
    RPMCounter1a = RPMCounter1;
  }

  if (TimeStart1 >= Reset1) {
    RPM1 = (RPMCounter1a / 20) * 60;
    RPMavg1 = (RPM1 + RPM1) / 2;
    TimeRunner1 = millis();
    //    Serial.print("RPM1: ");
    //    Serial.println(RPM1);
    //
    //    Serial.print("RPMavg1: ");
    //    Serial.println(RPMavg1);
    //
    //    Serial.print("RPMCounter1a: ");
    //    Serial.println(RPMCounter1a);

  }

  //************************************************************
  //                 ENCODER 2

  // Runs if rotation was detected

  if (TurnDetected2) {
    PrevPosition2 = WheelPosition2;  // Save previous position in variable
    WheelPosition2 = WheelPosition2 + 1;  // Increase Position by 1
    RPMCounter2 = RPMCounter2 + 1;
    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();
  }

  TimeStart2 = millis() - TimeRunner2;

  if (TimeStart2 <= TimeStart2a) {
    RPM2 = 0;
    RPMCounter2 = 0;
  }

  if (TimeStart2 >= TimeStart2a) {
    RPMCounter2a = RPMCounter2;

  }

  if (TimeStart2 >= Reset2) {
    RPM2 = (RPMCounter2a / 20) * 60;
    RPMavg2 = (RPM2 + RPM2) / 2;
    TimeRunner2 = millis();
    //    Serial.print("RPM2: ");
    //    Serial.println(RPM2);
    //
    //    Serial.print("RPMavg2: ");
    //    Serial.println(RPMavg2);
    //
    //    Serial.print("RPMCounter2a: ");
    //    Serial.println(RPMCounter2a);

  }

  //***************************************************************
  //if (Run2State == LOW){
  ////    //************************************************************************************************************
  //////                           TEST RAMP UP MOTOR CONTROL
  //
  //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
  //
  //      if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
  //        speedRampForwardAI = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);
  //
  ////      MoveForwardRampTestAI();
  //
  //  analogWrite (LeftSpeedPin, speedRampForwardAI);
  //  analogWrite (RightSpeedPin, speedRampForwardAI);
  //
  //  digitalWrite(leftPin, HIGH);
  //  digitalWrite(rightPin, HIGH);
  //      }
  //
  ////rampStartTime = millis();
  //
  ////  delay(2000);
  //
  ////int startSpeedForward = 0;
  ////int stopSpeedForward = 250;
  ////speedRampForwardAI
  ////unsigned long rampUpEndTime = 10000;
  ////unsigned long rampDnEndTime = 20000;
  ////unsigned long rampStartTime;
  ////unsigned long rampRunner ;
  ////unsigned long rampProgres;
  //    //************************************************************************************************************
  ////                           TEST RAMP DOWN MOTOR CONTROL
  //
  //
  ////      rampProgres = millis() - rampStartTime;     // Servo Head Progress
  //
  //      if ((rampProgres >= rampUpEndTime) && (rampProgres <= rampDnEndTime)) {
  //        speedRampForwardAI = map(rampProgres, rampUpEndTime, rampDnEndTime, stopSpeedForward, startSpeedForward);
  //
  ////      MoveForwardRampTestAI();
  //
  //  analogWrite (LeftSpeedPin, speedRampForwardAI);
  //  analogWrite (RightSpeedPin, speedRampForwardAI);
  //
  //  digitalWrite(leftPin, HIGH);
  //  digitalWrite(rightPin, HIGH);
  //      }
  //
  //      if ((rampProgres >= rampDnEndTime)) {
  //
  //rampStartTime = millis();
  //      }
  //
  //}
  //
  //
  //  //******************************************************************
  //                       STANDBY TIMER CONTROL


  //unsigned long StandbySetTime = 30000;
  //unsigned long StandbyStartTime;
  //unsigned long StandbyProgress;
  //unsigned long StandbyResetTime = 2000;
  //int StandbyActive = 0;

  StandbyProgress = millis() - StandbyStartTime;     // Servo Head Progress

  if ((StandbyProgress >= StandbySetTime) && (TurnDetected1 = false) && (TurnDetected2 == false)) {         // &&(MotionDetectcted == LOW)) {

    StandbyActive = HIGH;
    digitalWrite(leftPin, HIGH);
    digitalWrite(rightPin, HIGH);
    
  }


if ((TurnDetected1 = true) && (TurnDetected2 == true)) { 
  
    StandbyActive = LOW;
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, LOW);

      StandbyStartTime = millis();
      
}


  //    if (StandbyProgress >= ServoMoveTime) {
  //      AngleM = map(StandbyProgress, StandbySetTime, StandbyResetTime, StartAngleM, StopAngleM);
  ////      myservoScan.write(AngleM);
  //      myservoScanB.write(AngleM);
  //
  //    }
  //
  //    if (StandbyProgress >= StandbyResetTime) {
  //      StandbyStartTime = millis();
  //
  //    }
  
    //***************************************************************************
    //                          TEMPERATURE CHECKKING & FAN CONTROL

    Temp = analogRead(tempPin);
    Temp = Temp  * 0.13;


    Serial.println(Temp);
    Serial.print((char)223);

    lcd.setCursor (12, 0);
    lcd.print (Temp);
    lcd.print((char)223);
    lcd.print("C");

    
  //**************************************************************************************************************
  //                                   RUNNING AI CONTROL

  if ((RunState == LOW) && (StandbyActive == LOW)) {
    //************************************************************************************************************
    //                          DISTANCE (SCANNER AND HEAD SERVO) CONTROL

    //************************************************************************************************************
    //                                         SCANNER SERVO CONTROL

    //    LookLeftRightAState = digitalRead(LookLeftRightAPin);
    //    LookUpDownState = digitalRead(LookUpDownPin);

    //    AngleM = Angle3A;
    AngleM = Angle;

    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

    if (Progress <= ServoMoveTime) {
      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
      //      myservoScan.write(AngleM);
      myservoScanB.write(AngleM);
    }

    if (Progress >= ServoMoveTime) {
      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
      //      myservoScan.write(AngleM);
      myservoScanB.write(AngleM);

    }

    if (Progress >= ResetTime) {
      ServoMoveStartTime = millis();

    }



    //    //************************************************************************************************************
    ////                             RAMP UP MOTOR CONTROL

    //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
    //
    //      if ((rampProgres >= 0) && (rampProgres <= rampEndTime)) {
    //        speedRampForwardAI = map(rampProgres, 0, rampEndTime, startSpeedForward, stopSpeedForward);
    //
    ////      MoveForwardRampTestAI();
    //
    //  analogWrite (LeftSpeedPin, speedRampForwardAI);
    //  analogWrite (RightSpeedPin, speedRampForwardAI);
    //
    //  digitalWrite(leftPin, HIGH);
    //  digitalWrite(rightPin, HIGH);
    //      }



    ////int startSpeedForward = 60;
    ////int stopSpeedForward = 120;
    ////speedRampForwardAI
    ////unsigned long rampEndTime = 2000;
    ////unsigned long rampStartTime;
    ////unsigned long rampRunner ;
    ////unsigned long rampProgres;

    //************************************************************************************************************
    //                                 CAR CONTROL

    //          if ((Distreading4 >= FarSetPoint4)){
    //
    //              MoveForwardAI();
    //
    //          }

    //**************************************************************************************************************
    //                         BOTTOM MIDDEL SENSOR SWEEP


    if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
        && (AngleM <= 100) && (AngleM >= 80)) {

      MoveStopAI();

      AngleBotLeft = AngleM;
      AngleBotRight = AngleM;

      myservoScanB.write(160);

      delay(1000);



      Distreading4L = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER

      lcd.setCursor(11, 1);
      lcd.print(Distreading4L  );
      lcd.print(" ");

      delay(1000);

      myservoScanB.write(30);

      delay(1000);

      Distreading4R = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER

      lcd.setCursor(3, 1);
      lcd.print(Distreading4R  );
      lcd.print(" ");

      delay(1000);

      myservoScanB.write(90);
      delay(1000);


      if (Distreading4L >= Distreading4R) {


        LeftTurnCNT = 0;
        RightTurnCNT = 0;
        digitalWrite(leftPin, HIGH);
        digitalWrite(rightPin, LOW);
        analogWrite (LeftSpeedPin, 0);
        analogWrite (RightSpeedPin, 0);

        delay(1000);

        MoveLeft90AI();
      }

      if (Distreading4R >= Distreading4L) {


        LeftTurnCNT = 0;
        RightTurnCNT = 0;
        digitalWrite(leftPin, LOW);
        digitalWrite(rightPin, HIGH);
        analogWrite (LeftSpeedPin, 0);
        analogWrite (RightSpeedPin, 0);

        delay(1000);

        MoveRight90AI();
      }


      //      if ((Distreading4L == Distreading4R)) {
      //        MoveBackwardAI();
      //        delay(2000);
      //        MoveStopAI();
      //      }

      ////rampStartTime = millis();


    }

    //**********************************************************************************************************
    //                          45 DEGREE MANUVER LEFT
    
        if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
           && (AngleM <= 140) && (AngleM >= 95)) {
    
            LeftTurnCNT = 0;
            RightTurnCNT = 0;

            MoveStopAI();            
            MoveLeft45AI();
      MoveStopAI();    
Move500AI();
      MoveStopAI();    
            MoveRight45AI();
      MoveStopAI();    
MoveOffset();     
      MoveStopAI();    
MoveOffset();     
      MoveStopAI();    
            MoveRight45AI();
      MoveStopAI();    
Move500AI();
      MoveStopAI();    
            MoveLeft45AI();
      MoveStopAI();    
//              MoveForwardAI();
   
            }
    
//Move500AI();
//MoveOffset();     
    //**********************************************************************************************************
    //                          45 DEGREE MANUVER RIGHT
    
        if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
           && (AngleM <= 60) && (AngleM >= 85)) {
    
            LeftTurnCNT = 0;
            RightTurnCNT = 0;
            
            MoveStopAI();            
            MoveRight45AI();
      MoveStopAI();    
Move500AI();
      MoveStopAI();    
            MoveLeft45AI();
      MoveStopAI();    
MoveOffset();     
      MoveStopAI();    
MoveOffset();     
      MoveStopAI();    
            MoveLeft45AI();
      MoveStopAI();    
Move500AI();
      MoveStopAI();    
            MoveRight45AI();
      MoveStopAI();    
//              MoveForwardAI();
    
            }
    

    //**********************************************************************************************************

  }

  //********************************************************************************************************
  //                LCD2 OUTPUT CONTROL
  //
  //  lcd.setCursor(0, 0);
  //  lcd.print("L:");
  //  lcd.print(Distreading5);
  //  lcd.print(" ");
  //  lcd.setCursor(10, 0);
  //  lcd.print("R:");
  //  lcd.print(Distreading6);
  //  lcd.print(" ");

  lcd.setCursor(0, 1);
  lcd.print("R: ");
  lcd.print(Distreading6 );
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print("L: ");
  lcd.print(Distreading5 );
  lcd.print(" ");


  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.print(RightTurnCNT);
  lcd.print(" ");
  lcd.print(TurnDetected1);
  lcd.print(" ");
//  lcd.setCursor(10, 0);
//  lcd.print("L:");
//  lcd.print(LeftTurnCNT);
//  lcd.print(" ");
//  lcd.print(TurnDetected2);
//  lcd.print(" ");



  //  lcd.setCursor(8, 0);
  //  lcd.print("PROG:");
  //  lcd.print(rampProgres);
  //  lcd.print(" ");
  //  lcd.setCursor(8, 1);
  //  lcd.print("R/SPD:");
  //  lcd.print(speedRampForwardAI);
  //  lcd.print(" ");

  //**************************************************************************************************************
  //                   FOR DIAGNOSTICS (DECOMMENT OR COMMENT SECTIONS TO USE)
  //****************************************************************************************************************
  //****************************************************************************************************************

  //****************************************************************************************************************
  //**************************** SCANNER DISTANCE SENSING **********************************************************


    if (ledPinState == HIGH){

  Serial.println("");
  Serial.print("Progress2 :");
  Serial.print(Progress2);
  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("SDistreading1 :");
  //  Serial.print(Distreading1);
  //  Serial.print(" MM");
  //  Serial.println("");
  //
  //  Serial.println("");
  //
  //    Serial.println("");
  //    Serial.print("Distreading4 :");
  //    Serial.print(Distreading4);
  //    Serial.print(" MM");
  //    Serial.println("");
  //
  //    Serial.println("");
  //    Serial.print("Distreading5 :");
  //    Serial.print(Distreading5);
  //    Serial.print(" MM");
  //    Serial.println("");
  //
  //    Serial.println("");
  //    Serial.print("Distreading6 :");
  //    Serial.print(Distreading6);
  //    Serial.print(" MM");
  //    Serial.println("");

  //****************************************************************************************************************
  //                            ENCODERS

  //Serial.print("WheelPosition: ");
  //Serial.println(WheelPosition);
  //
  //Serial.print("WheelPosition2: ");
  //Serial.println(WheelPosition2);

  //*****************************************************************
  //**************************** SCANNER SERVO CONTROL AND TIMING **************************************************

  //  Serial.println("");
  //  Serial.print("Progress:");
  //  Serial.print(Progress);
  //  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("AngleL:");
  //  Serial.print(AngleL);
  //  Serial.println("");

  //*****************************************************************
  //                       RAMP UP TEST

  //    Serial.println("");
  //    Serial.print("rampProgres:");
  //    Serial.print(rampProgres);
  //    Serial.println("");
  //
  //    Serial.println("");
  //    Serial.print("speedRampForwardAI:");
  //    Serial.print(speedRampForwardAI);
  //    Serial.println("");


  //****************************************************************************************************************
  //                            GYRO MPU-6050
  
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());

  //**********************************************************

  //
  //  Serial.println("********************************************");
    }
}
