

//**************************************************************
////                 GYRO MPU-6050

//#include <MPU6050_tockn.h>
//#include <Wire.h>
//
//MPU6050 mpu6050(Wire);

//**************************************************************

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#include<UltraDistSensor.h>

UltraDistSensor mysensor1;   // BOTTOM CENTER SCAN SERVO DISTANCE SENSOR (CAR)
UltraDistSensor mysensor2;   // BOTTOM LEFT DISTANCE SENSOR (CAR)

//**************************************************************
//                             DISTANCE CONTROL

int Distreading4;    // Bottom Middel SCANNER SENSOR
int Distreading4L;    // Bottom LEFT Middel SCANNER SENSOR
int Distreading4R;   // Bottom RIGHT Middel SCANNER SENSOR (CAR)
int Distreading5;    // Bottom LEFT SENSOR (CAR)
int Distreading5L;    // Bottom RIGHT SENSOR (CAR)
int Distreading5R;    // Bottom RIGHT SENSOR (CAR)
int Distreading5LF;    // Bottom RIGHT SENSOR (CAR)
int Distreading5RF;    // Bottom RIGHT SENSOR (CAR)

int DistLeftCheckCNT = 0;
int LastDistLeftCheckCNT = 0;
int LeftCntOn = 0;

int DistRightCheckCNT = 0;
int LastDistRightCheckCNT = 0;
int RightCntOn = 0;

int FirstLeftState = 0;
int SecondLeftState = 0;

int FirstRightState = 0;
int SecondRightState = 0;

int MotionDetectcted = 0;

int CloseSetPoint3 = 150;         //  Centre Dist Set point Closeby
int FarSetPoint3 = 300;         //  Centre Dist Set point Far

int CloseSetPoint4 = 50;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4 = 300;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4L = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4L = 700;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4R = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4R = 700;         //  Bottom Centre Dist Set point Far

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
int stopSpeedForward = 200;

#define speedForwardAI 100
#define speedForwardLeftAI 150
#define speedForwardRightAI 100

int speedRampForwardAI = 0;

#define speedBackwardAI 100
#define speedBackwardLeftAI 150
#define speedBackwardRightAI 100

#define speedRampBackwardAI 100

#define speedLeftAI 100
#define speedLeftLeftAI 150    // Turn Left Left Wheel Speed
#define speedLeftRightAI 100    // Turn Left Right Wheel Speed

#define speedRampLeftAI 100

#define speedRightAI 100
#define speedRightLeftAI 150    // Turn Right Left Wheel Speed
#define speedRightRightAI 100    // Turn Right Right Wheel Speed

#define speedRampRightLeftAI 80

//*************************************************************
//           MANUVER DELAY CONTROL

#define ManuverDelay 1000

//*************************************************************
//     SCANNER ANGLE CONTROL

#include <Servo.h>

int ScanAngleLeft = 100;
int ScanAngleRight = 80;

int ScanAngleLC = 100;
int ScanAngleRC = 80;

int AngleBotLeft = 0;
int AngleBotRight = 0;

Servo myservoScanBack;  // create servo object to control a servo
Servo myservoScanFront;  // create servo object to control a servo

//***********************************************
//      STANDBY TIMER CONTROL

unsigned long StandbySetTime = 30000;
unsigned long StandbyStartTime;
unsigned long StandbyProgress;
unsigned long StandbyResetTime = 2000;

int StandbyActive = 0;


//***********************************************
//      SCANNER FRONT LEFT / RIGHT CONTROL

unsigned long ServoMoveTime = 1000;
unsigned long ServoMoveStartTime;
unsigned long Progress;
unsigned long ResetTime = 2000;

int StartAngleM = 60;
int StopAngleM =  120;

int StartAngle = 20;
int StopAngle =  160;

int StartAngleR = 40;
int StopAngleR =  140;

unsigned long Angle;
unsigned long AngleM;
unsigned long AngleL;
unsigned long AngleR;

//***********************************************
//      SCANNER REAR LEFT / RIGHT CONTROL

unsigned long ServoMoveTime1 = 1500;
unsigned long ServoMoveStartTime1;
unsigned long Progress1;
unsigned long ResetTime1 = 3000;

int StartAngleM1 = 10;
int StopAngleM1 =  165;

int StartAngle1 = 5;
int StopAngle1 =  175;

int StartAngleR1 = 40;
int StopAngleR1 =  140;

unsigned long Angle1;
unsigned long AngleM1;
unsigned long AngleL1;
unsigned long AngleR1;

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

const int PinCLK1 = 19; // 5 Generating interrupts using CLK signal

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

int LeftCnt200For = 23;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
int Left200Offset = 2;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6
int RightCnt200For = 23;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
int Right200Offset = 2;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6


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

const int PinCLK2 = 18; // 4 Generating interrupts using CLK signal

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

#define leftPin1A 5
#define leftPinEN 6
#define leftPin1B 7

#define rightPin2A 8
#define rightPinEN 9
#define rightPin2B 10

#define runPin 14
int RunState = 0;
#define run2Pin 15

int Run2State = 0;

float tempPin = A8;
int Temp = 0;

////***********************************************************************************************************

void serialprint() {

      if (ledPinState == HIGH) {

//  Serial.println("");
//  Serial.print("RunState :");
//  Serial.print(RunState);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("Run2State :");
//  Serial.print(Run2State);
//  Serial.println("");
//  
//  Serial.println("");
//  Serial.print("Progress2 :");
//  Serial.print(Progress2);
//  Serial.println("");

  //
  //  Serial.println("");
  //  Serial.print("SDistreading1 :");
  //  Serial.print(Distreading1);
  //  Serial.print(" MM");
  //  Serial.println("");
  //
  //  Serial.println("");
  //
//      Serial.println("");
//      Serial.print("Front :");
//      Serial.print(Distreading4);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("Distreading5 :");
//      Serial.print(Distreading5);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("FirstLeftState :");
//      Serial.print(FirstLeftState);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("SecondLeftState :");
//      Serial.print(SecondLeftState);
//      Serial.print(" MM");
//      Serial.println("");
//
//      Serial.println("");
//      Serial.print("DistLeftCheckCNT :");
//      Serial.print(DistLeftCheckCNT);
//      Serial.println("");
////
//
//      Serial.println("");
//      Serial.print("FirstRightState :");
//      Serial.print(FirstRightState);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("SecondRightState :");
//      Serial.print(SecondRightState);
//      Serial.print(" MM");
//      Serial.println("");
//
//      Serial.println("");
//      Serial.print("DistRightCheckCNT :");
//      Serial.print(DistRightCheckCNT);
//      Serial.println("");
//            

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                            ENCODERS

  //Serial.print("WheelPosition: ");
  //Serial.println(WheelPosition);
  //
  //Serial.print("WheelPosition2: ");
  //Serial.println(WheelPosition2);

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   SCANNER SERVO CONTROL AND TIMING

  //  Serial.println("");
  //  Serial.print("Progress:");
  //  Serial.print(Progress);
  //  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("AngleL:");
  //  Serial.print(AngleL);
  //  Serial.println("");

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                       RAMP UP TEST
//
//  Serial.println("");
//  Serial.print("rampProgres:");
//  Serial.print(rampProgres);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("Run2State :");
//  Serial.print(Run2State);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("speedRampForwardAI:");
//  Serial.print(speedRampForwardAI);
//  Serial.println("");


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                            GYRO MPU-6050

  //  Serial.print("angleX : ");
  //  Serial.print(mpu6050.getAngleX());
  //  Serial.print("\tangleY : ");
  //  Serial.print(mpu6050.getAngleY());
  //  Serial.print("\tangleZ : ");
  //  Serial.println(mpu6050.getAngleZ());

  ////***********************************************************************************************************
  ////***********************************************************************************************************

  //
  //  Serial.println("********************************************");

      }
}



////***********************************************************************************************************
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
//
////***********************************************************************************************************
////***********************************************************************************************************
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
////***********************************************************************************************************
////***********************************************************************************************************
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
////***********************************************************************************************************
////***********************************************************************************************************
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
////***********************************************************************************************************
////***********************************************************************************************************
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
////***********************************************************************************************************
//***********************************************************************************************************
//                        MOVE FORWARD RAMP TEST CONTROL AI
//
//
//void MoveForwardRampTestAI() {
//
//    rampProgres = millis() - rampStartTime;     // Servo Head Progress
//
//    if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
//      speedRampForwardAI = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);
//
//      //      MoveForwardRampTestAI();
//
//
//      analogWrite (leftPinEN, speedRampForwardAI);
//      analogWrite (rightPinEN, speedRampForwardAI);
//
//      digitalWrite(leftPin1A, HIGH);
//      digitalWrite(leftPin1B, LOW);
//      digitalWrite(rightPin2A, HIGH);
//      digitalWrite(rightPin2B, LOW);
//
//    }
//  //
//  ////rampStartTime = millis();
//  
//if (rampProgres >= rampUpEndTime){
//rampStartTime = millis();  
//}
//
//}
//
////***********************************************************************************************************
////***********************************************************************************************************
////                               MOVE STOP LEFT CONTROL AI
//
//void MoveStopLeftAI() {
//
//  analogWrite (leftPinEN, 0);
//  analogWrite (rightPinEN, 0);
//
//}
//
////***********************************************************************************************************
////***********************************************************************************************************
////                               MOVE STOP RIGHT CONTROL AI
//
//void MoveStopRightAI() {
//
//  analogWrite (leftPinEN, 0);
//  analogWrite (rightPinEN, 0);
//
//}
//
////***********************************************************************************************************
//***********************************************************************************************************
//                                      MOVE FORWARD CONTROL AI


void MoveForwardAI() {

//  LeftTurnCNT = 0;
//  RightTurnCNT = 0;

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

ScanForward();

  //    delay(1000);

  //    while (Distreading4 >= FarSetPoint4) {

  if (Distreading4 >= FarSetPoint4)  {
    analogWrite (leftPinEN, speedForwardLeftAI);
    analogWrite (rightPinEN, speedForwardRightAI);
  }

  else if (Distreading4 <= FarSetPoint4) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
    analogWrite (rightPinEN, 0);
  }
  //    }

//  delay(ManuverDelay);

//  analogWrite (leftPinEN, 0);
//  analogWrite (rightPinEN, 0);

  //    LeftTurnCNT = 0;
  //    RightTurnCNT = 0;

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE STOP CONTROL AI

void MoveStopAI() {

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  //    digitalWrite(leftPin1A, HIGH);
  //    digitalWrite(leftPin1B, LOW);
  //    digitalWrite(rightPin2A, HIGH);
  //    digitalWrite(rightPin2B, LOW);

  delay(ManuverDelay);
  
//rampStartTime = millis();

      LeftTurnCNT = 0;
      RightTurnCNT = 0;
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE BACKWARD CONTROL AI

void MoveBackwardAI() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, LOW);
  digitalWrite(leftPin1B, HIGH);
  digitalWrite(rightPin2A, LOW);
  digitalWrite(rightPin2B, HIGH);


  //    delay(1000);

  //    while (Distreading4L == Distreading4R) {

  if (Distreading4L == Distreading4R) {
    analogWrite (leftPinEN, speedLeftLeftAI);
    analogWrite (rightPinEN, speedLeftRightAI);
  }

  else if (Distreading4L != Distreading4R) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
    analogWrite (rightPinEN, 0);
  }

  //    }

  delay(ManuverDelay);

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  //    LeftTurnCNT = 0;
  //    RightTurnCNT = 0;

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT 90 CONTROL AI

void MoveLeft90AI() {

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, LOW);
  digitalWrite(leftPin1B, HIGH);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

  while ((LeftTurnCNT <= LeftTurn90) || (RightTurnCNT <= LeftTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

    if (LeftTurnCNT <= LeftTurn90) {
      analogWrite (leftPinEN, speedLeftLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn90) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (leftPinEN, 0);
    }

    if (RightTurnCNT <= LeftTurn90) {
      analogWrite (rightPinEN, speedLeftRightAI);
    }

    else if (RightTurnCNT >= LeftTurn90) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (rightPinEN, 0);
    }
  }

  delay(ManuverDelay);

rampStartTime = millis();
}


//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT 45 CONTROL AI


void MoveLeft45AI() {

  RightTurn45 = 8;  // 150 mm = 13.5 counts
  LeftTurn45 = 8;  // 150 mm = 13.5 counts

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, LOW);
  digitalWrite(leftPin1B, HIGH);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

  while ((LeftTurnCNT <= LeftTurn45) || (RightTurnCNT <= LeftTurn45)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

    if (LeftTurnCNT <= LeftTurn45) {
      analogWrite (leftPinEN, speedLeftLeftAI);
    }

    if (LeftTurnCNT >= LeftTurn45) {
      analogWrite (leftPinEN, 0);
    }

    if (RightTurnCNT <= LeftTurn45) {
      analogWrite (rightPinEN, speedLeftRightAI);
    }

    if (RightTurnCNT >= LeftTurn45) {
      analogWrite (rightPinEN, 0);
    }
  }

  delay(ManuverDelay);

rampStartTime = millis();
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT 90 CONTROL AI

void MoveRight90AI() {

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, LOW);
  digitalWrite(rightPin2B, HIGH);

  while ((LeftTurnCNT <= RightTurn90) || (RightTurnCNT <= RightTurn90)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }


    if (LeftTurnCNT <= RightTurn90) {
      analogWrite (leftPinEN, speedRightLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn90) {
      analogWrite (leftPinEN, 0);
    }

    if (RightTurnCNT <= RightTurn90) {
      analogWrite (rightPinEN, speedRightRightAI);
    }

    else if (RightTurnCNT >= RightTurn90) {                                   //if (RightTurnCNT >= RightTurn90){
      analogWrite (rightPinEN, 0);
    }
  }

  delay(ManuverDelay);

rampStartTime = millis();
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT 45 CONTROL AI

void MoveRight45AI() {

  RightTurn45 = 7;  // 150 mm = 13.5 counts
  LeftTurn45 = 7;  // 150 mm = 13.5 counts

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, LOW);
  digitalWrite(rightPin2B, HIGH);

//ScanRight();

  while ((LeftTurnCNT <= RightTurn45) || (RightTurnCNT <= RightTurn45)) {

    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
    }

    if (LeftTurnCNT <= RightTurn45) {
      analogWrite (leftPinEN, speedRightLeftAI);
    }

    if (LeftTurnCNT >= RightTurn45) {
      analogWrite (leftPinEN, 0);
    }

    if (RightTurnCNT <= RightTurn45) {
      analogWrite (rightPinEN, speedRightRightAI);
    }

    if (RightTurnCNT >= RightTurn45) {
      analogWrite (rightPinEN, 0);
    }
  }

  delay(ManuverDelay);

rampStartTime = millis();
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE 500mm CONTROL AI


void Move500AI() {

  //  ***

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

//ScanForwardMan();

  //    delay(1000);

  if (TurnDetected1) {
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
  }
  if (TurnDetected2) {
    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
  }

  if (LeftTurnCNT <= LeftCntFor) {
    analogWrite (leftPinEN, speedForwardLeftAI);
  }

  else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
  }

  if (RightTurnCNT <= RightCntFor) {
    analogWrite (rightPinEN, speedForwardRightAI);
  }

  else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
    analogWrite (rightPinEN, 0);
  }

  delay(ManuverDelay);

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;
  //***
rampStartTime = millis();
}

//***********************************************************************************************************
//                               MOVE 200mm LEFT CONTROL AI

void Move200LeftAI() {

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

  if (TurnDetected1) {
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
  }
  if (TurnDetected2) {
    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
  }

//ScanLeft();

while (Distreading5L <= 150){

ScanLeft();

if (Distreading5L <= 150){

      ScanLeft();

    analogWrite (leftPinEN, speedForwardLeftAI);
    analogWrite (rightPinEN, speedForwardRightAI);

}
//else 
if (Distreading5L >= 250)
{
  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);
  
}
}

//  delay(ManuverDelay);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

rampStartTime = millis();
}

//***********************************************************************************************************
//                               MOVE 200mm RIGHT CONTROL AI


void Move200RightAI() {

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

//ScanForwardMan();

//      delay(2000);

  if (TurnDetected1) {
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
  }
  if (TurnDetected2) {
    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
  }
  
//ScanRight();

while (Distreading5R <= 150){

ScanRight();

if (Distreading5R <= 150){
  
ScanRight();
      
    analogWrite (leftPinEN, speedForwardLeftAI);
    analogWrite (rightPinEN, speedForwardRightAI);

}
//else
if (Distreading5R >= 250)
{
  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);
  
}
}

}

//***********************************************************************************************************


//***********************************************************************************************************
//                               MOVE 500mm + Offset CONTROL AI

void MoveOffset() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;


  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  digitalWrite(leftPin1A, HIGH);
  digitalWrite(leftPin1B, LOW);
  digitalWrite(rightPin2A, HIGH);
  digitalWrite(rightPin2B, LOW);

//ScanForwardMan();

  //    delay(1000);

  if (TurnDetected1) {
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
  }
  if (TurnDetected2) {
    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
  }


  if (LeftTurnCNT <= (LeftCntFor + LeftOffset)) {
    analogWrite (leftPinEN, speedForwardLeftAI);
  }

  else if (LeftTurnCNT >= (LeftCntFor + LeftOffset)) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
  }

  if (RightTurnCNT <= (RightCntFor + RightOffset)) {
    analogWrite (rightPinEN, speedForwardRightAI);
  }

  else if (RightTurnCNT >= (RightCntFor + RightOffset)) {                           // if (RightTurnCNT >= RightTurn90){
    analogWrite (rightPinEN, 0);
  }


  delay(ManuverDelay);

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  //***
rampStartTime = millis();
}

//***********************************************************************************************************
////***********************************************************************************************************
////                                      SCAN FORWARD (LEFT / RIGHT)
    ////***********************************************************************************************************
    //                                         SCANNER FRONT SERVO CONTROL

void ScanForward() {

    AngleM = Angle;

    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

    if (Progress <= ServoMoveTime) {
      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM -10);
      
    }

    if (Progress >= ServoMoveTime) {
      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM + 10);

    }

    if (Progress >= ResetTime) {
      ServoMoveStartTime = millis();

    }

}

//***********************************************************************************************************
////***********************************************************************************************************
////                                      SCAN FORWARD (LEFT / RIGHT)
    ////***********************************************************************************************************
    //                                         SCANNER REAR SERVO CONTROL

void ScanRear() {

    AngleM1 = Angle1;

    Progress1 = millis() - ServoMoveStartTime1;     // Servo Head Progress

    if (Progress1 <= ServoMoveTime1) {
      AngleM1 = map(Progress1, 0, ServoMoveTime1, StopAngleM1, StartAngleM1);
//      myservoScanFront.write(AngleM);
      myservoScanBack.write(AngleM1);
      
    }

    if (Progress1 >= ServoMoveTime1) {
      AngleM1 = map(Progress1, ServoMoveTime1, ResetTime1, StartAngleM1, StopAngleM1);
//      myservoScanFront.write(AngleM);
      myservoScanBack.write(AngleM1);

    }

    if (Progress1 >= ResetTime1) {
      ServoMoveStartTime1 = millis();

    }

}

//***********************************************************************************************************

    ////***********************************************************************************************************
    //                                         SCANNER SERVO CONTROL

void ScanForwardMan() {

    AngleM = Angle;

    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

    if (Progress <= ServoMoveTime) {
      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM);
      
    }

    if (Progress >= ServoMoveTime) {
      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM);

    }

    if (Progress >= ResetTime) {
      ServoMoveStartTime = millis();

    }
delay(1000);

}

//***********************************************************************************************************


//***********************************************************************************************************
//                                      SCAN LEFT


void ScanLeft() {

          myservoScanBack.write(20);

delay(1000);

Distreading5L  = (mysensor2.distanceInCm() * 10);

  lcd.setCursor(8, 0);
  lcd.print("L:");
  lcd.print(Distreading5L);
  lcd.print(" ");
  
//delay(1000);
}
          

////***********************************************************************************************************
////***********************************************************************************************************
////                                      SCAN RIGHT


void ScanRight() {

      myservoScanBack.write(170);

delay(1000);
  
Distreading5R  = (mysensor2.distanceInCm() * 10);
  
  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.print(Distreading5R);
  lcd.print(" ");
  
delay(1000);
}

    ////***********************************************************************************************************
    //                        LEFT / RIGHT REAR FORWARD CHECK TO TURN

void LRcheckForward(){
  
 ScanRear();

//    SCAN LEFT

if (AngleM1 <= 20)
{
//Distreading5LF  = (mysensor2.distanceInCm() * 10);
//DistLeftCheckCNT = DistLeftCheckCNT ++;

//delay(100);
LeftCntOn = HIGH;

      Serial.println("");
      Serial.print("DistLeftCheckCNT :");
      Serial.print(DistLeftCheckCNT);
      Serial.println("");

}else{
LeftCntOn = LOW;  
}
//if (AngleM1 <= 20){
if (LeftCntOn != LastDistLeftCheckCNT){
if (LeftCntOn == HIGH){
DistLeftCheckCNT ++;
} else {

  
}
    delay(10);

}
LastDistLeftCheckCNT = LeftCntOn;

if (DistLeftCheckCNT == 2)
{
DistLeftCheckCNT = 0;
}  

if (DistLeftCheckCNT == 0)
{
  if (AngleM1 <= 20)
{
FirstLeftState  = (mysensor2.distanceInCm() * 10);
//FirstLeftState = ((FirstLeftState <= FirstLeftState + 200) && (FirstLeftState >= FirstLeftState - 200));      

      Serial.println("");
      Serial.print("FirstLeftState :");
      Serial.print(FirstLeftState);
      Serial.print(" MM");
      Serial.println("");

  lcd.setCursor(8, 0);
  lcd.print("0:");
  lcd.print(FirstLeftState);
  lcd.print(" "); 
}
}


if (DistLeftCheckCNT == 1)
{
  if (AngleM1 <= 20)
{
SecondLeftState  = (mysensor2.distanceInCm() * 10);
//SecondLeftState = ((SecondLeftState <= SecondLeftState + 200) && (SecondLeftState >= SecondLeftState - 200));
      Serial.println("");
      Serial.print("SecondLeftState :");
      Serial.print(SecondLeftState);
      Serial.print(" MM");
      Serial.println("");

  lcd.setCursor(8, 0);
  lcd.print("1:");
  lcd.print(SecondLeftState);
  lcd.print(" "); 
}
}
//



//    SCAN RIGHT

if (AngleM1 >= 160)
{
//Distreading5RF  = (mysensor2.distanceInCm() * 10);
//DistRightCheckCNT = DistRightCheckCNT ++;

RightCntOn = HIGH;

//delay(100);

      Serial.println("");
      Serial.print("DistRightCheckCNT :");
      Serial.print(DistRightCheckCNT);
      Serial.println("");
}else{
RightCntOn = LOW; 
}


//if (AngleM1 <= 20){
if (RightCntOn != LastDistRightCheckCNT){
if (RightCntOn == HIGH){
DistRightCheckCNT ++;
} else {

  
}
    delay(10);

}
LastDistRightCheckCNT = RightCntOn;



//if (RightCntOn == HIGH){
//DistRightCheckCNT = DistRightCheckCNT +1;
// 
//}

if (DistRightCheckCNT == 2)
{
DistRightCheckCNT = 0;  

}

if (DistRightCheckCNT == 0)
{
  if (AngleM1 >= 160)
{
FirstRightState  = (mysensor2.distanceInCm() * 10);
//FirstRightState = ((FirstRightState <= FirstRightState + 200) && (FirstRightState >= FirstRightState - 200));
      
      Serial.println("");
      Serial.print("FirstRightState :");
      Serial.print(FirstRightState);
      Serial.print(" MM");
      Serial.println("");

  lcd.setCursor(0, 0);
  lcd.print("0:");
  lcd.print(FirstRightState);
  lcd.print(" ");      
}
}

if (DistRightCheckCNT == 1)
{
if (AngleM1 >= 160)
{
SecondRightState  = (mysensor2.distanceInCm() * 10);
//SecondRightState = ((SecondRightState <= SecondRightState + 200) && (SecondRightState >= SecondRightState - 200));

      Serial.println("");
      Serial.print("SecondRightState :");
      Serial.print(SecondRightState);
      Serial.print(" MM");
      Serial.println("");

  lcd.setCursor(0, 0);
  lcd.print("1:");
  lcd.print(SecondRightState);
  lcd.print(" "); 
}
}

}


//int FirstLeftState = 0;
//int SecondLeftState = 0;
//
//int FirstRightState = 0;
//int SecondRightState = 0;

////***********************************************************************************************************

//***********************************************************************************************************

void setup() {
  // put your setup code here, to run once:

  lcd.init();
  lcd.backlight();

  Serial.begin(115200);
  //  Wire.begin();

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //  ////                 GYRO MPU-6050
  //
  //  mpu6050.begin();
  //  mpu6050.calcGyroOffsets(true);

  //**************************************************************

  //  RemoteXY_Init ();

  //  pinMode (PIN_START_1, OUTPUT);

  pinMode (ledPin, OUTPUT);

  pinMode (leftPin1A, OUTPUT);      // LEFT MOTOR
  pinMode (leftPin1B, OUTPUT);      // LEFT MOTOR

  pinMode (rightPin2A, OUTPUT);      // RIGHT MOTOR
  pinMode (rightPin2B, OUTPUT);      // RIGHT MOTOR

  pinMode (runPin, INPUT_PULLUP);
  pinMode (run2Pin, INPUT_PULLUP);

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   ENCODERS

  pinMode (PinCLK1, INPUT);
  attachInterrupt (4, isr1, FALLING);

  pinMode (PinCLK2, INPUT);
  attachInterrupt (5, isr2, FALLING);

  TimeRunner1 = millis();
  TimeRunner2 = millis();

  ////***********************************************************************************************************
  ////***********************************************************************************************************

  myservoScanBack.attach(12);  // create servo object to control a servo
  myservoScanFront.attach(4);  // create servo object to control a servo

  mysensor1.attach(A0, A1); //Trigger pin , Echo pin Front
  mysensor2.attach(A4, A5); //Trigger pin , Echo pin Right

  rampStartTime = millis();
  ServoMoveStartTime2 = millis();
  ServoMoveStartTime = millis();
  ServoMoveStartTime1 = millis();
  StandbyStartTime = millis();

//LeftCntOn = LOW; 
//RightCntOn = LOW;
}

////***********************************************************************************************************
////***********************************************************************************************************


void loop() {
  // put your main code here, to run repeatedly:

  ////***********************************************************************************************************
  ////***********************************************************************************************************
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

  serialprint();

  ////***********************************************************************************************************

 ScanForward();
 ScanRear();
  
  ////***********************************************************************************************************
  ////                 GYRO MPU-6050

  //  mpu6050.update();
  //  Serial.print("angleX : ");
  //  Serial.print(mpu6050.getAngleX());
  //  Serial.print("\tangleY : ");
  //  Serial.print(mpu6050.getAngleY());
  //  Serial.print("\tangleZ : ");
  //  Serial.println(mpu6050.getAngleZ());
  //
  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   RUN STATE

  RunState = digitalRead(runPin);
  Run2State = digitalRead(run2Pin);

  //************************************************************************************************************
  //************************************************************************************************************
  //                SCANNER                SCANNER                SCANNER                SCANNER
  //************************************************************************************************************

  //************************** DISTANCE MEASURING IN MM ********************************************************

  Distreading4 = (mysensor1.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
  Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
  Distreading4R = 0;  // BOTTOM MIDDEL SCANNER
  
  Distreading5 = (mysensor2.distanceInCm() * 10);  // BOTTOM LEFT SENSOR
  Distreading5L = 0;  // BOTTOM MIDDEL SCANNER
  Distreading5R = 0;  // BOTTOM MIDDEL SCANNER

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                     DISTANCE DETECTION CONTROL

  //      FRONT SENSOR

  //  if ((Distreading4 >= 0) || (Distreading5 >= 0) || (Distreading6 >= 0)){
  //    MotionDetectcted = HIGH;
  //
  //      StandbyStartTime = millis();
  //  }else{
  //    MotionDetectcted = LOW;
  //  }

  ////***********************************************************************************************************
  ////***********************************************************************************************************
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

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                 ENCODER 2*

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

  ////***********************************************************************************************************

    ////***********************************************************************************************************
    //                        LEFT / RIGHT REAR FORWARD CHECK TO TURN

LRcheckForward();

//int FirstLeftState = 0;
//int SecondLeftState = 0;
//
//int FirstRightState = 0;
//int SecondRightState = 0;


if ((SecondLeftState >= FirstLeftState + 100) && (SecondRightState == FirstRightState)) {
LRcheckForward();
 MoveLeft90AI(); 
}
else if ((SecondLeftState == FirstLeftState) && (SecondRightState >= FirstRightState + 100)) {
 LRcheckForward();
 MoveLeft90AI(); 
}
else{

// MoveForwardAI();
}

    ////***********************************************************************************************************

  
  ////***********************************************************************************************************
  //                                   RUNNING AI CONTROL

  if ((RunState == LOW)) {


    //                                 CAR CONTROL

    //          if ((Distreading4 >= FarSetPoint4)){
    //
    //              MoveForwardAI();
    //
    //          }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                         FRONT SENSOR SWEEP


//    if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
//        && (AngleM <= 100) && (AngleM >= 80)) {
//
//      MoveStopAI();
//
//      AngleBotLeft = AngleM;
//      AngleBotRight = AngleM;
//
//      myservoScanFront.write(160);
//
//      delay(1000);
//
//
//
//      Distreading4L = (mysensor1.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
//
//      lcd.setCursor(11, 1);
//      lcd.print(Distreading4L  );
//      lcd.print(" ");
//
//      myservoScanFront.write(20);
//
//      delay(2000);
//
//      Distreading4R = (mysensor1.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
//
//      lcd.setCursor(3, 1);
//      lcd.print(Distreading4R  );
//      lcd.print(" ");
//
//
//      myservoScanFront.write(90);
//      delay(1000);
//
//
//      if (Distreading4L >= Distreading4R) {
//
//ScanRight();
//
//        delay(1000);
//
//        MoveLeft90AI();
//      }
//
//      if (Distreading4R >= Distreading4L) {
//
//ScanLeft();
//
//        delay(1000);
//
//        MoveRight90AI();
//
//      }
//
//    }


//    ////***********************************************************************************************************
//    ////***********************************************************************************************************
//    //                          45 DEGREE MANUVER LEFT
//
//    if (((Distreading4 <= FarSetPoint4L) && (Distreading4 >= CloseSetPoint4L))
//        && (AngleM <= 140) && (AngleM >= 100)) {
//
//      LeftTurnCNT = 0;
//      RightTurnCNT = 0;
//
//
//      MoveStopAI();
//      MoveRight45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//      MoveLeft45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//            
//ScanLeft();
// if (Distreading5L <= 150){
//      Move200LeftAI();
// }
//
// MoveStopAI();
//
//      MoveLeft45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//      MoveRight45AI();
//      MoveStopAI();
//
//      LeftTurnCNT = 0;
//      RightTurnCNT = 0;
//    }
//
//    ////***********************************************************************************************************
//    ////***********************************************************************************************************
//    //                          45 DEGREE MANUVER RIGHT
//
//    if (((Distreading4 <= FarSetPoint4R) && (Distreading4 >= CloseSetPoint4R))
//        && (AngleM >= 40) && (AngleM <= 80)) {
//
//      LeftTurnCNT = 0;
//      RightTurnCNT = 0;
//
//
//      MoveStopAI();
//      MoveLeft45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//      MoveRight45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//
//ScanRight();
// 
// if (Distreading5L <= 150){
//
//Move200RightAI();
//
// }
// 
//      MoveStopAI();
//        
//      MoveRight45AI();
//      MoveStopAI();
//      Move500AI();
//      MoveStopAI();
//      MoveLeft45AI();
//      MoveStopAI();
//
//      //              MoveForwardAI();
//      LeftTurnCNT = 0;
//      RightTurnCNT = 0;
//    }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                       Testing Of Motors

//      rampStartTime = millis();

//                  MoveForwardAI();
//                  MoveStopAI();
//                  MoveForwardRampTestAI();
//                  MoveStopAI();
                  
    //          Move500AI();
    //          MoveStopAI();
    //          MoveOffset();
    //          MoveStopAI();
    //          MoveBackwardAI();
    //          MoveStopAI();
    //          MoveLeft90AI();
    //          MoveStopAI();
    //          MoveLeft45AI();
    //          MoveStopAI();
    //          MoveRight90AI();
    //          MoveStopAI();
    //          MoveRight45AI();
    //          MoveStopAI();

    ////***********************************************************************************************************
    ////***********************************************************************************************************

  }

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                LCD2 OUTPUT CONTROL
  
  lcd.setCursor(0, 1);
    lcd.print("F: ");
  lcd.print(  Distreading4);
    lcd.print("  ");

  lcd.setCursor(8, 1);
  lcd.print("B: ");
  lcd.print(  Distreading5 );
  lcd.print("   ");

//  lcd.setCursor(0, 0);
//  lcd.print("R:");
//  lcd.print(Distreading5R);
//  lcd.print(" ");
//
//  lcd.setCursor(8, 0);
//  lcd.print("L:");
//  lcd.print(Distreading5L);
//  lcd.print(" ");

//  lcd.setCursor(8, 1);
//  lcd.print("L: ");
//  lcd.print(Distreading5 );
//  lcd.print(" ");


//  lcd.setCursor(0, 0);
//  lcd.print("R:");
//  lcd.print(RightTurnCNT);
//  lcd.print(" ");
//  //  lcd.print(TurnDetected2);
//  //  lcd.print(" ");
//  lcd.setCursor(11, 0);
//  lcd.print("L:");
//  lcd.print(LeftTurnCNT);
//  lcd.print(" ");
//    lcd.print(TurnDetected1);
//    lcd.print(" ");



  //  lcd.setCursor(8, 0);
  //  lcd.print("PROG:");
  //  lcd.print(rampProgres);
  //  lcd.print(" ");
  //  lcd.setCursor(8, 1);
  //  lcd.print("R/SPD:");
  //  lcd.print(speedRampForwardAI);
  //  lcd.print(" ");

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   FOR DIAGNOSTICS (DECOMMENT OR COMMENT SECTIONS TO USE)
  ////***********************************************************************************************************
  ////***********************************************************************************************************

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                              SCANNER DISTANCE SENSING
//  Serial.println("");
//  Serial.print("RunState :");
//  Serial.print(RunState);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("Run2State :");
//  Serial.print(Run2State);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("Progress2 :");
//  Serial.print(Progress2);
//  Serial.println("");

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

//      Serial.println("");
//      Serial.print("FirstLeftState :");
//      Serial.print(FirstLeftState);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("SecondLeftState :");
//      Serial.print(SecondLeftState);
//      Serial.print(" MM");
//      Serial.println("");
//
//      Serial.println("");
//      Serial.print("DistLeftCheckCNT :");
//      Serial.print(DistLeftCheckCNT);
//      Serial.println("");
////
//
//      Serial.println("");
//      Serial.print("FirstRightState :");
//      Serial.print(FirstRightState);
//      Serial.print(" MM");
//      Serial.println("");
//  
//      Serial.println("");
//      Serial.print("SecondRightState :");
//      Serial.print(SecondRightState);
//      Serial.print(" MM");
//      Serial.println("");
//
//      Serial.println("");
//      Serial.print("DistRightCheckCNT :");
//      Serial.print(DistRightCheckCNT);
//      Serial.println("");
            

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                            ENCODERS

  //Serial.print("WheelPosition: ");
  //Serial.println(WheelPosition);
  //
  //Serial.print("WheelPosition2: ");
  //Serial.println(WheelPosition2);

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   SCANNER SERVO CONTROL AND TIMING

  //  Serial.println("");
  //  Serial.print("Progress:");
  //  Serial.print(Progress);
  //  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("AngleL:");
  //  Serial.print(AngleL);
  //  Serial.println("");

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                       RAMP UP TEST
//
//  Serial.println("");
//  Serial.print("rampProgres:");
//  Serial.print(rampProgres);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("Run2State :");
//  Serial.print(Run2State);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("speedRampForwardAI:");
//  Serial.print(speedRampForwardAI);
//  Serial.println("");


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                            GYRO MPU-6050

  //  Serial.print("angleX : ");
  //  Serial.print(mpu6050.getAngleX());
  //  Serial.print("\tangleY : ");
  //  Serial.print(mpu6050.getAngleY());
  //  Serial.print("\tangleZ : ");
  //  Serial.println(mpu6050.getAngleZ());

  ////***********************************************************************************************************
  ////***********************************************************************************************************

  //
  //  Serial.println("********************************************");

  ////***********************************************************************************************************

  if (Run2State == LOW) {
    //    //************************************************************************************************************
    ////                           TEST RAMP UP MOTOR CONTROL

    rampProgres = millis() - rampStartTime;     // Servo Head Progress

    if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
      speedRampForwardAI = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);

      //      MoveForwardRampTestAI();


      analogWrite (leftPinEN, speedRampForwardAI);
      analogWrite (rightPinEN, speedRampForwardAI);

      digitalWrite(leftPin1A, HIGH);
      digitalWrite(leftPin1B, LOW);
      digitalWrite(rightPin2A, HIGH);
      digitalWrite(rightPin2B, LOW);

    }

    //************************************************************************************************************
    //                           TEST RAMP DOWN MOTOR CONTROL


    //      rampProgres = millis() - rampStartTime;     // Servo Head Progress

    if ((rampProgres >= rampUpEndTime) && (rampProgres <= rampDnEndTime)) {
      speedRampForwardAI = map(rampProgres, rampUpEndTime, rampDnEndTime, stopSpeedForward, startSpeedForward);

      //      MoveForwardRampTestAI();

      analogWrite (leftPinEN, speedRampForwardAI);
      analogWrite (rightPinEN, speedRampForwardAI);

      digitalWrite(leftPin1A, HIGH);
      digitalWrite(leftPin1B, LOW);
      digitalWrite(rightPin2A, HIGH);
      digitalWrite(rightPin2B, LOW);
    }

    if ((rampProgres >= rampDnEndTime)) {

      LeftTurnCNT = 0;
      RightTurnCNT = 0;

      rampStartTime = millis();
    }

  }


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                       STANDBY TIMER CONTROL

  //
  //  //unsigned long StandbySetTime = 30000;
  //  //unsigned long StandbyStartTime;
  //  //unsigned long StandbyProgress;
  //  //unsigned long StandbyResetTime = 2000;
  //  //int StandbyActive = 0;
  //
  //  StandbyProgress = millis() - StandbyStartTime;     // Servo Head Progress
  //
  //  if ((StandbyProgress >= StandbySetTime) && (TurnDetected1 = false) && (TurnDetected2 == false)) {         // &&(MotionDetectcted == LOW)) {
  //
  //    StandbyActive = HIGH;
  //
  //  //
  //  //  digitalWrite(rightPin2B, LOW);
  //  //  digitalWrite(lefttPin1B, LOW);
  //
  //  //  digitalWrite(rightPin2A, LOW);
  //  //  digitalWrite(leftPin1A, LOW);
  //  }


  //if ((TurnDetected1 = true) && (TurnDetected2 == true)) {
  //
  //    StandbyActive = LOW;
  //
  //    digitalWrite(rightPin2B, LOW);
  //    digitalWrite(leftPin1B, LOW);
  //
  //    digitalWrite(rightPin2A, LOW);
  //    digitalWrite(leftPin1A, LOW);
  //
  //      StandbyStartTime = millis();
  //
  //}
  //
  //
  //  //    if (StandbyProgress >= ServoMoveTime) {
  //  //      AngleM = map(StandbyProgress, StandbySetTime, StandbyResetTime, StartAngleM, StopAngleM);
  //  ////      myservoScan.write(AngleM);
  //  //      myservoScanFront.write(AngleM);
  //  //
  //  //    }
  //  //
  //  //    if (StandbyProgress >= StandbyResetTime) {
  //  //      StandbyStartTime = millis();
  //  //
  //  //    }

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                          TEMPERATURE CHECKKING & FAN CONTROL
  //
  //  Temp = analogRead(tempPin);
  //  Temp = Temp  * 0.13;
  //
  //
  //  Serial.println(Temp);
  //  Serial.print((char)223);
  //
  //  //    lcd.setCursor (12, 0);
  //  //    lcd.print (Temp);
  //  //    lcd.print((char)223);
  //  //    lcd.print("C");

  ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                          DISTANCE (SCANNER AND HEAD SERVO) CONTROL

    ////***********************************************************************************************************
//    ////***********************************************************************************************************
//    //                                         SCANNER SERVO CONTROL
//
//    AngleM = Angle;
//
//    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress
//
//    if (Progress <= ServoMoveTime) {
//      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
//      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM);
//      
//    }
//
//    if (Progress >= ServoMoveTime) {
//      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
//      myservoScanFront.write(AngleM);
//      myservoScanBack.write(AngleM);
//
//    }
//
//    if (Progress >= ResetTime) {
//      ServoMoveStartTime = millis();
//
//    }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    ////                             RAMP UP MOTOR CONTROL

    //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
    //
    //      if ((rampProgres >= 0) && (rampProgres <= rampEndTime)) {
    //        speedRampForwardAI = map(rampProgres, 0, rampEndTime, startSpeedForward, stopSpeedForward);
    //
    ////      MoveForwardRampTestAI();
    //
    //  analogWrite (rightPin2B, speedRampForwardAI);
    //  analogWrite (leftPin1B, speedRampForwardAI);
    //
    //  digitalWrite(rightPin2A, HIGH);
    //  digitalWrite(lefttPin1A, HIGH);
    //      }
    //
    ////int startSpeedForward = 60;
    ////int stopSpeedForward = 120;
    ////speedRampForwardAI
    ////unsigned long rampEndTime = 2000;
    ////unsigned long rampStartTime;
    ////unsigned long rampRunner ;
    ////unsigned long rampProgres;

    ////***********************************************************************************************************
   

}

