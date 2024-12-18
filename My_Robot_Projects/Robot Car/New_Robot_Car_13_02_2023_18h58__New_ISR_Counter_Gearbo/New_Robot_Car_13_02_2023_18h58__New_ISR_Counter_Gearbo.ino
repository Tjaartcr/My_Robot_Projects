//**************************************************************

#include <Wire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
LiquidCrystal_I2C lcd1(0x23, 16, 2);
//**************************************************************


#include "TimerOne.h"

const byte MotorRightCountPin = 18;
const byte MotorLeftCountPin = 19;

// Integers for pulse counters

unsigned int MotorRightCounter = 0;
unsigned int MotorLeftCounter = 0;


unsigned int MotorRightCounter1 = 0;
unsigned int MotorLeftCounter1 = 0;


float diskslots = 16;
//**************************************************************

//
//#include <TimerFour.h>

//const byte MotorRightCountPin = 18;
//const byte MotorLeftCountPin = 19;

// Integers for pulse counters

unsigned int MotorRightCountera = 0;
unsigned int MotorLeftCountera = 0;


unsigned int MotorRightCounter1a = 0;
unsigned int MotorLeftCounter1a = 0;


float diskslotsa = 16;

//**************************************************************

byte batteryFull[20] = {

0b01110,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
};

//**************************************************************

byte battery3Quater[20] = {

0b01110,
0b11111,
0b10001,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
};

//**************************************************************

byte batteryHalf[20] = {

0b01110,
0b11111,
0b10001,
0b10001,
0b11111,
0b11111,
0b11111,
0b11111,
};

//**************************************************************

byte batteryQuat[20] = {

0b01110,
0b11111,
0b10001,
0b10001,
0b10001,
0b10001,
0b11111,
0b11111,
};

//**************************************************************

byte batteryEmpty[20] = {

0b01110,
0b11111,
0b10001,
0b10001,
0b10001,
0b10001,
0b10001,
0b11111,
};

//**************************************************************

#include<UltraDistSensor.h>

UltraDistSensor FrontSensor;   // BOTTOM CENTER SCAN SERVO DISTANCE SENSOR (CAR)
UltraDistSensor BackSensor;   // BOTTOM LEFT DISTANCE SENSOR (CAR)

//**************************************************************
//                             DISTANCE CONTROL

int FrontDistReading;    // Bottom Middel SCANNER SENSOR
int FrontDistReadingL;    // Bottom LEFT Middel SCANNER SENSOR
int FrontDistReadingR;   // Bottom RIGHT Middel SCANNER SENSOR (CAR)
int DistreadingFrontWatch;

int RearDistReading;    // Bottom LEFT SENSOR (CAR)
int RearDistReadingL;    // Bottom RIGHT SENSOR (CAR)
int RearDistReadingR;    // Bottom RIGHT SENSOR (CAR)
int RearDistReadingLF;    // Bottom RIGHT SENSOR (CAR)
int RearDistReadingRF;    // Bottom RIGHT SENSOR (CAR)

int RearDistReadingTurnL = 0;  // BOTTOM MIDDEL SCANNER
int RearDistReadingTurnR = 0;  // BOTTOM MIDDEL SCANNER

int DifferanceLeftTurnU180 = 0;
int DifferanceRightTurnU180 = 0;

int DistLeftCheckCNT = 0;
int LastDistLeftCheckCNT = 0;
int LeftCntOn = 0;

int DistRightCheckCNT = 0;
int LastDistRightCheckCNT = 0;
int RightCntOn = 0;

int FirstLeftState = 0;
int PercentFirstLeftState = 0;
int SecondLeftState = 0;
int PercentSecondLeftState = 0;

int FirstRightState = 0;
int PercentFirstRightState = 0;
int SecondRightState = 0;
int PercentSecondRightState = 0;

int LeftDifference = 0;
int RightDifference = 0;

int ForwardScanDist = 0;
int ForwardScanSetFar = 150;
int ForwardScanSetClose = 50;

int LeftDifferenceOn = LOW;
int RightDifferenceOn = LOW;

int MotionDetectcted = 0;

int CloseSetPoint3 = 150;         //  Centre Dist Set point Closeby
int FarSetPoint3 = 300;         //  Centre Dist Set point Far

int CloseSetPoint4 = 100;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4 = 200;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4L = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4L = 700;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4R = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4R = 700;         //  Bottom Centre Dist Set point Far

int CloseSetPoint5L = 100;         // Bottom LEFT Dist Set point Closeby
int FarSetPoint5L = 400;         //  Bottom LEFT Dist Set point Far

int CloseSetPoint5R = 100;         // Bottom RIGHT Dist Set point Closeby
int FarSetPoint5R = 400;         //  Bottom RIGHT Dist Set point Far

// ************************************************************************
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

// ************************************************************************
//       ROBOT CAR SPEED CONTROL

unsigned long rampUpEndTime = 10000;
unsigned long rampDnEndTime = 20000;
unsigned long rampStartTime;
unsigned long rampRunner ;
unsigned long rampProgres;

int startSpeedForward = 0;
int stopSpeedForward = 200;

#define speedForwardAI 40
#define speedForwardLeftAI 60 * 1.5
#define speedForwardRightAI 60

//int speedForwardLeftAI;
//int speedForwardRightAI;

int speedRampForwardAI = 0;

#define speedBackwardAI 40
#define speedBackwardLeftAI 60 * 1.5
#define speedBackwardRightAI 60

#define speedRampBackwardAI 40

#define speedLeftAI 40
#define speedLeftLeftAI 60 * 1.5    // Turn Left Left Wheel Speed
#define speedLeftRightAI 60    // Turn Left Right Wheel Speed

#define speedRampLeftAI 60

#define speedRightAI 40
#define speedRightLeftAI 60 * 1.5    // Turn Right Left Wheel Speed
#define speedRightRightAI 60    // Turn Right Right Wheel Speed

#define speedRampRightLeftAI 255

//*************************************************************
//           MANUVER DELAY CONTROL

#define ManuverDelay 1000

//*************************************************************

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

int StartAngleM = 10;
int StopAngleM =  170;

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

int StartAngleM1 = 5;
int StopAngleM1 =  170;

int StartAngle1 = 5;
int StopAngle1 =  175;

int StartAngleR1 = 40;
int StopAngleR1 =  140;

unsigned long Angle1;
unsigned long AngleM1;
unsigned long AngleL1;
unsigned long AngleR1;

int BigLeftDist = 0;
int BigRightDist = 0;

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


//***********************************************
//            Battery Buzzer TIMER

unsigned long BatBuzzMoveTime = 50;
unsigned long BatBuzzStartTime;
unsigned long BatBuzzProgress;
unsigned long BatBuzzResetTime = 1000;

int BatBuzzState;

//***********************************************
//***********************************************
//            Battery Blink TIMER

unsigned long BatBlinkMoveTime = 1000;
unsigned long BatBlinkStartTime;
unsigned long BatBlinkProgress;
unsigned long BatBlinkResetTime = 2000;

int BatBlinkState;

//***********************************************
//***********************************************
//       REAR LEFT / RIGHT SCAN TIMER CONTROL

unsigned long ServoMoveTime3 = 3000;
unsigned long ServoMoveStartTime3;
unsigned long Progress3;
unsigned long ResetTime3 = 30000;

int StartAngleM3 = 10;
int StopAngleM3 =  165;

int StartAngle3 = 5;
int StopAngle3 =  175;

int StartAngleR3 = 40;
int StopAngleR3 =  140;

unsigned long Angle3;
unsigned long AngleM3;
unsigned long AngleL3;
unsigned long AngleR3;

int RearScanOn = 0;
int ScanRearRunning = 0;

//***********************************************
//   REAR LEFT PULSE SCAN TIMER CONTROL

unsigned long ServoMoveTime4 = 10;
unsigned long ServoMoveStartTime4;
unsigned long Progress4;
unsigned long ResetTime4 = 1010;

int PulseOnL = 0;

int StartAngleM4 = 10;
int StopAngleM4 =  165;

int StartAngle4 = 10;
int StopAngle4 =  165;

int StartAngleR4 = 40;
int StopAngleR4 =  140;

unsigned long Angle4;
unsigned long AngleM4;
unsigned long AngleL4;
unsigned long AngleR4;

//***********************************************
//   REAR RIGHT PULSE SCAN TIMER CONTROL

unsigned long ServoMoveTime5 = 10;
unsigned long ServoMoveStartTime5;
unsigned long Progress5;
unsigned long ResetTime5 = 1010;

int PulseOnR = 0;

int StartAngleM5 = 10;
int StopAngleM5 =  165;

int StartAngle5 = 5;
int StopAngle5 =  175;

int StartAngleR5 = 40;
int StopAngleR5 =  140;

unsigned long Angle5;
unsigned long AngleM5;
unsigned long AngleL5;
unsigned long AngleR5;

//*****************************************************************
//***********************************************
//   REAR  LEFT / RIGHT 4 STEP SCAN TIMER CONTROL

unsigned long ServoMoveTime6 = 1000;
unsigned long ServoMoveStartTime6;
unsigned long Progress6;
unsigned long ResetTime6 = 2000;
unsigned long ResetTime6a = 3000;
unsigned long ResetTime6b = 4000;
unsigned long ResetTime6c = 5000;
unsigned long ResetTime6d = 6000;

int Step1On = 0;
int Step2On = 0;
int Step3On = 0;
int Step4On = 0;

//***********************************************
//   REAR  LEFT  2 STEP SCAN TIMER CONTROL

unsigned long ServoMoveTime7 = 7000;
unsigned long ServoMoveStartTime7;
unsigned long Progress7;
unsigned long ResetTime7 = 8000;

int StepScan1OnLeft = 0;
int StepScan2OnLeft = 0;

//*****************************************************************
//***********************************************
//   REAR RIGHT 2 STEP SCAN TIMER CONTROL

unsigned long ServoMoveTime8 = 7000;
unsigned long ServoMoveStartTime8;
unsigned long Progress8;
unsigned long ResetTime8 = 8000;

int StepScan1OnRight = 0;
int StepScan2OnRight = 0;

//*****************************************************************


//*****************************************************************

//*****************************************************************
//                 ENCODER 1 (LEFT)

volatile boolean TurnDetected1;  // need volatile for Interrupts
volatile boolean rotationdirection1; // CW or CCW rotation

const int PinCLK1 = 19; // 5 Generating interrupts using CLK signal

int WheelPosition1; // To store Stepper Motor Position
int PrevPosition1;   // Previous Rotary Position Value to check accuracy
int RPMavg1;
int RPM1;   // Revolutions Per Minute
int WheelCounter1 = 16; //Wheel has 20 gaps
int TimeStart1 = 0;
int TimeStart1a = 2000;
int Reset1 = 2100;
int TimeRunner1;
int RPMCounter1;
int RPMCounter1a;
int LeftTurn45;
int LeftTurn90;
int LeftUTurn180;
int LeftTurn180;
int LeftUTurn180On;
int LeftTurn180On;
int LeftUTurn180CNT;
int LeftTurn180CNT;
int LeftUTurn180CNTOn;
int LeftTurn180CNTOn;
int LastLeftUTurn180CNT;
int LastLeftTurn180CNT;

float LeftTurnCNT;
float LeftTurnRpmCNT;

float LeftCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
float LeftOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6

float LeftCnt200For = 23;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
float Left200Offset = 2;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6
float RightCnt200For = 23;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
float Right200Offset = 2;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6

float DiscSlotsLeft = 16;



//*****************************************************************
//                 ENCODER 2 (RIGHT)

volatile boolean TurnDetected2;  // need volatile for Interrupts
volatile boolean rotationdirection2; // CW or CCW rotation

const int PinCLK2 = 18; // 4 Generating interrupts using CLK signal

int WheelPosition2; // To store Stepper Motor Position
int PrevPosition2;   // Previous Rotary Position Value to check accuracy
int RPMavg2;   // Revolutions Per Minute
int RPM2;   // Revolutions Per Minute
int WheelCounter2 = 16; //Wheel has 20 gaps
int TimeStart2 = 0;
int TimeStart2a = 2000;
int Reset2 = 2100;
int TimeRunner2;
int RPMCounter2;
int RPMCounter2a;
int RightTurn45;
int RightTurn90;
int RightUTurn180;
int RightTurn180;
int RightUTurn180On;
int RightTurn180On;
int RightUTurn180CNT;
int RightTurn180CNT;
int RightUTurn180CNTOn;
int RightTurn180CNTOn;
int LastRightUTurn180CNT;
int LastRightTurn180CNT;

float RightTurnCNT;
float RightTurnRpmCNT;


float RightCntFor = 46;        // 1 Rotation = 220mm. Need to go 500mm. 500/220 = 2,27 * 20 = 45,45
float RightOffset = 14;       // 1 Rotation = 220mm. Need to go 150mm. 150/220 = 0,68 * 20 = 13,6

float DiscSlotsRight = 16;


#define SpeedPotPin A2
int SpeedValue;

#define MotorTestPin A3
int MotorTestPinState;


#define LeftSelPin A14
int LeftSelState = 0;

#define RightSelPin A15
int RightSelState = 0;

#define LeftForRevPin 5
#define LeftSpeedPin 6
#define LeftMotorPowerOnPin 4

#define RightForRevPin 8
#define RightSpeedPin 9
#define RightMotorPowerOnPin 10
//
//#define LeftForRevPin 5
//#define RightSpeedPin 6
//#define RightMotorPowerOnPin 7
//
//#define RightForRevPin 8
//#define RightSpeedPin 9
//#define RightMotorPowerOnPin 10

#define grnledPin 22
#define bluledPin 23
#define redledPin 24
#define whtledPin 25

int whtledState = 0;
int redledState = 0;
int grnledState = 0;
int bluledState = 0;

#define ledPin 27

#define buzzerPin 26
#define batbuzzerPin 13

#define runPin 14
int RunState = 0;
#define run2Pin 15

int Run2State = 0;

float tempPin = A12;
int Temp = 0;

#define BatteryVoltagePin  A6
float BatteryVoltage = 0;

int Entry = 0;

////***********************************************************************************************************

 float AdjustedLeftRpm;
 float AdjustedRightRpm;
 
  float RightRpm;
  float LeftRpm;

  float RightRpma;
  float LeftRpma;
  
float SpeedSyncAdjust;
float SpeedSyncAdjustLeft;
float SpeedSyncAdjustRight;
float PreviousSpeedSyncAdjust;
  
////***********************************************************************************************************

void SerialPrint(){

  if (ledPinState == HIGH) {

    //  Serial.println("");
    //  Serial.print("RunState :");
    //  Serial.print(RunState);
    //
    //  Serial.println("");
    //  Serial.print("Run2State :");
    //  Serial.print(Run2State);
    //
    //  Serial.println("");
    //  Serial.print("Progress2 :");
    //  Serial.print(Progress2);
    //
    //      Serial.println("");
    //      Serial.print("Progress5 :");
    //      Serial.print(Progress5);

    //  Serial.println("");
    //  Serial.print("SDistreading1 :");
    //  Serial.print(Distreading1);
    //  Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("Front :");
    //      Serial.print(FrontDistReading);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("RearDistReading :");
    //      Serial.print(RearDistReading);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("FirstLeftState :");
    //      Serial.print(FirstLeftState);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("SecondLeftState :");
    //      Serial.print(SecondLeftState);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("DistLeftCheckCNT :");
    //      Serial.print(DistLeftCheckCNT);
    ////
    //
    //      Serial.println("");
    //      Serial.print("FirstRightState :");
    //      Serial.print(FirstRightState);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("SecondRightState :");
    //      Serial.print(SecondRightState);
    //      Serial.print(" MM");
    //
    //      Serial.println("");
    //      Serial.print("DistRightCheckCNT :");
    //      Serial.print(DistRightCheckCNT);
    //

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                            ENCODERS
  Serial.println("");
    Serial.print("WheelPosition1 = Left: ");
    Serial.println(WheelPosition1);
    
  Serial.println("");    
    Serial.print("WheelPosition2  = Right: ");
    Serial.println(WheelPosition2);

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                   SCANNER SERVO CONTROL AND TIMING

    //  Serial.println("");
    //  Serial.print("Progress:");
    //  Serial.print(Progress);
    //
    //  Serial.println("");
    //  Serial.print("AngleL:");
    //  Serial.print(AngleL);

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                       RAMP UP TEST
    //
    //  Serial.println("");
    //  Serial.print("rampProgres:");
    //  Serial.print(rampProgres);
    //
    //  Serial.println("");
    //  Serial.print("Run2State :");
    //  Serial.print(Run2State);
    //
    //  Serial.println("");
    //  Serial.print("speedRampForwardAI:");
    //  Serial.print(speedRampForwardAI);


    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                            GYRO MPU-6050
//  Serial.println("");
    //  Serial.print("angleX : ");
    //  Serial.print(mpu6050.getAngleX());
    //  Serial.print("\tangleY : ");
    //  Serial.print(mpu6050.getAngleY());
    //  Serial.print("\tangleZ : ");
    //  Serial.print(mpu6050.getAngleZ());

    ////***********************************************************************************************************



  Serial.println("");   
  Serial.print("  RPM1 : ");
  Serial.print(RPM1);

  Serial.println("");  
  Serial.print("  RPMCounter1 : ");
  Serial.print(RPMCounter1);

  Serial.println("");  

  Serial.print("  TimeStart1 : ");
  Serial.print(TimeStart1);

  Serial.println("");      
  Serial.print("  RPM2 : ");
  Serial.print(RPM2);

  Serial.println("");  
  Serial.print("  RPMCounter2 : ");
  Serial.print(RPMCounter2);

  Serial.println("");  
  Serial.print("  TimeStart2 : ");
  Serial.print(TimeStart2);
    
  Serial.println("");      
  Serial.print("Motor Speed Left: ");
  Serial.print(LeftRpm);
  Serial.print(" RPM - ");
  Serial.print("Motor Speed Right: ");
  Serial.print(RightRpm);
  Serial.print(" RPM - ");    

//      Serial.println(""); 
//  Serial.print("LeftSelState : ");
//  Serial.print(LeftSelState);

//  Serial.println("");
//  Serial.print("RightSelState : ");
//  Serial.print(RightSelState);

//  Serial.println("");
//  Serial.print("MotorTestPinState : ");
//  Serial.print(MotorTestPinState);

  Serial.println("");
  Serial.print("SpeedValue : ");
  Serial.print(SpeedValue);

  Serial.println("");
  Serial.print("  MotorLeftCounter1 : ");
  Serial.print(MotorLeftCounter1);

  Serial.println("");
  Serial.print("  MotorRightCounter1 : ");
  Serial.print(MotorRightCounter1);

  Serial.println("");
  Serial.print("  LeftTurnCNT : ");
  Serial.print(LeftTurnCNT);

  Serial.println("");
  Serial.print("  RightTurnCNT : ");
  Serial.print(RightTurnCNT);

  Serial.println("");
  Serial.print("SpeedSyncAdjustLeft : ");
  Serial.print(SpeedSyncAdjustLeft);

  Serial.println("");
  Serial.print("AdjustedLeftRpm : ");
  Serial.print(AdjustedLeftRpm);
  
//  Serial.println("");
//  Serial.print("SpeedSyncAdjustRight : ");
//  Serial.print(SpeedSyncAdjustRight);

      
    
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
//      analogWrite (RightSpeedPin, speedRampForwardAI);
//      analogWrite (RightSpeedPin, speedRampForwardAI);
//
//    analogWrite(LeftForRevPin, 255);
//    analogWrite(LeftMotorPowerOnPin, 0);
//    analogWrite(RightForRevPin, 255);
//    analogWrite(RightMotorPowerOnPin, 0);
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
//  analogWrite (LeftSpeedPin, 0);
//  analogWrite (RightSpeedPin, 0);
//
//}
//
////***********************************************************************************************************
////***********************************************************************************************************
////                               MOVE STOP RIGHT CONTROL AI
//
//void MoveStopRightAI() {
//
//  analogWrite (LeftSpeedPin, 0);
//  analogWrite (RightSpeedPin, 0);
//
//}
//
////***********************************************************************************************************
//***********************************************************************************************************
//                                      MOVE FORWARD CONTROL AI


void MoveForwardAI() {

  //  LeftTurnCNT = 0;
  //  RightTurnCNT = 0;

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  ScanForward();

  if (FrontDistReading >= FarSetPoint4)  {
    analogWrite (LeftSpeedPin, speedForwardLeftAI);
    analogWrite (RightSpeedPin, speedForwardRightAI);
  }

  else if (FrontDistReading <= FarSetPoint4) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (LeftSpeedPin, 0);
    analogWrite (RightSpeedPin, 0);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                                      MOVE FORWARD ETRY CONTROL AI
//int ForwardScanSetFar = 150;
//int ForwardScanSetClose = 50;

void MoveForwardEnterAI() {

  //  analogWrite (leftPinEN, 0);
  //  analogWrite (rightPinEN, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  CheckForwardEntry();


  while (ForwardScanDist >= ForwardScanSetFar) {

    CheckForwardEntry();

    if (ForwardScanDist >= ForwardScanSetFar)  {
      analogWrite (LeftSpeedPin, speedForwardLeftAI);
      analogWrite (RightSpeedPin, speedForwardRightAI);
    }

    if (ForwardScanDist <= ForwardScanSetFar) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
      analogWrite (LeftSpeedPin, 0);
    }
  }

  // FrontScanLeft180();
  // FrontScanRight180();

  //if (RearDistReadingTurnL <= RearDistReadingTurnR){
  //MoveStopAI();
  ////MoveLeft90AI();
  ////MoveStopAI();
  ////MoveForwardEnterTurnAI();
  //}
  //
  //
  //if (RearDistReadingTurnL >= RearDistReadingTurnR){
  //MoveStopAI();
  ////MoveRight90AI();
  ////MoveStopAI();
  ////MoveForwardEnterTurnAI();
  //}

//  Serial.println("");
//  Serial.print("ForwardScanDist :");
//  Serial.print(ForwardScanDist);
//  Serial.println("");
//

}


//***********************************************************************************************************
//***********************************************************************************************************
//                                      MOVE FORWARD ETRY SCAN FIRST 180 TURN CONTROL AI


void MoveForwardEnterTurnAI() {

  //  LeftTurnCNT = 0;
  //  RightTurnCNT = 0;

  //  analogWrite (leftPinEN, 0);
  //  analogWrite (rightPinEN, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);


  CheckForwardEntry();

  while (ForwardScanDist >= ForwardScanSetFar) {

    CheckForwardEntry();

    if (ForwardScanDist >= ForwardScanSetFar)  {

      //  ScanForward();

      analogWrite (LeftSpeedPin, speedForwardLeftAI);
      analogWrite (RightSpeedPin, speedForwardRightAI);
    }

    if (ForwardScanDist <= ForwardScanSetFar) {                            //if (LeftTurnCNT >= LeftTurn90){
      //  ScanForward();

      analogWrite (LeftSpeedPin, 0);
      analogWrite (LeftSpeedPin, 0);

      // FrontScanLeft180();
      // FrontScanRight180();

    }
  }

  //if (RearDistReadingTurnL <= RearDistReadingTurnR){
  //  ScanForward();
  //MoveStopAI();
  ////MoveRight180AI();
  ////MoveStopAI();
  ////MoveForwardAI();
  //}
  //
  //
  //if (RearDistReadingTurnL >= RearDistReadingTurnR){
  //  ScanForward();
  //
  //MoveStopAI();
  ////MoveLeft180AI();
  ////MoveStopAI();
  ////MoveForwardAI();
  //}
  //MoveStopAI();


//  Serial.println("");
//  Serial.print("ForwardScanDist :");
//  Serial.print(ForwardScanDist);
//  Serial.println("");

}

//***********************************************************************************************************

//***********************************************************************************************************
//                               MOVE STOP CONTROL AI

void MoveStopAI() {

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  delay(2000);

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE BACKWARD CONTROL AI

void MoveBackwardAI() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 0);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 0);
  analogWrite(RightMotorPowerOnPin, 255);

  if (FrontDistReadingL == FrontDistReadingR) {
    analogWrite (LeftSpeedPin, speedLeftLeftAI);
    analogWrite (RightSpeedPin, speedLeftRightAI);
  }

  else if (FrontDistReadingL != FrontDistReadingR) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (LeftSpeedPin, 0);
    analogWrite (RightSpeedPin, 0);
  }

  delay(ManuverDelay);

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT 90 CONTROL AI

void MoveLeft90AI() {

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

//ISR_count1();
//ISR_count2();


  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 0);
  analogWrite(RightMotorPowerOnPin, 255);




  while ((LeftTurnCNT <= LeftTurn90) || (RightTurnCNT <= LeftTurn90)) {
    //LRcheckForward();
//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

ISR_count1();
ISR_count2();

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
  LeftDifference =  0;
  RightDifference = 0;
}


//***********************************************************************************************************
//***********************************************************************************************************
//                              TURN MOVE LEFT 90 CONTROL AI

void TurnMoveLeft90AI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

  LeftCntFor = 46;
  RightCntFor = 46;

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  while ((LeftTurnCNT <= LeftTurn90) || (RightTurnCNT <= LeftTurn90)) {

ISR_count1();
ISR_count2();

    analogWrite(LeftForRevPin, 255);
    analogWrite(LeftMotorPowerOnPin, 255);
    analogWrite(RightForRevPin, 0);
    analogWrite(RightMotorPowerOnPin, 255);

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= LeftTurn90) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    if (RightTurnCNT <= LeftTurn90) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

  }

  while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)) {

    ScanPulseTimerL();
    ScanPulseTimerR();
    ScanRearTimer();

ISR_count1();
ISR_count2();

    analogWrite(LeftForRevPin, 255);
    analogWrite(LeftMotorPowerOnPin, 255);
    analogWrite(RightForRevPin, 255);
    analogWrite(RightMotorPowerOnPin, 255);

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= LeftCntFor) {
      analogWrite (LeftSpeedPin, speedForwardLeftAI);
    }

    else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
      digitalWrite(grnledPin, LOW);
    }

    if (RightTurnCNT <= RightCntFor) {
      analogWrite (RightSpeedPin, speedForwardRightAI);
    }

    else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
      digitalWrite(bluledPin, LOW);
    }

    LeftDifference =  0;
    RightDifference = 0;

  }

}

//***********************************************************************************************************

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT U_TURN 180 CONTROL AI

void MoveLeftU180AI() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  RightUTurn180 = 0;  // 300 mm = 27 counts
  LeftUTurn180 = 64;  // 300 mm = 27 counts



  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);


  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 0);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((RightTurnCNT <= LeftUTurn180)) {

ISR_count1();
ISR_count2();

    LeftUTurn180CNTOn = HIGH;

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT >= LeftUTurn180) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= LeftUTurn180) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    else if (RightTurnCNT >= LeftUTurn180) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }

  }

  LeftDifference =  0;
  RightDifference = 0;

//  Serial.println("");
//  Serial.print("LeftUTurn180On :");
//  Serial.print(LeftUTurn180On);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("LeftUTurn180CNT :");
//  Serial.print(LeftUTurn180CNT);
//  Serial.println("");

}


//***********************************************************************************************************

void MoveLeftU180CNTAI() {

  if (LeftUTurn180CNTOn == HIGH)
  {

    LeftUTurn180On = HIGH;


  } else if (LeftUTurn180CNTOn == LOW) {
    LeftUTurn180On = LOW;
  }


  if (LeftUTurn180On != LastLeftUTurn180CNT) {
    if (LeftUTurn180On == HIGH) {
      LeftUTurn180CNT ++;
    } else {


    }
    //    delay(10);
  }

  LastLeftUTurn180CNT = LeftUTurn180On;


  if (LeftUTurn180CNT == 2)
  {
    LeftUTurn180CNT = 0;
  }
  //
  LeftUTurn180CNTOn = LOW;
  //        RightUTurn180CNTOn = LOW;

  //  Serial.println("");
  //  Serial.print("LeftUTurn180On :");
  //  Serial.print(LeftUTurn180On);
  //  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("LeftUTurn180CNT :");
  //  Serial.print(LeftUTurn180CNT);
  //  Serial.println("");

  lcd.setCursor(14, 0);
  lcd.print(LeftUTurn180CNT);
  lcd.print(" ");

}


//***********************************************************************************************************
//                               MOVE LEFT 180 CONTROL AI

void MoveLeft180AI() {

  RightTurn180 = 32;  // 300 mm = 27 counts
  LeftTurn180 = 32;  // 300 mm = 27 counts

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);


  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 0);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((LeftTurnCNT <= LeftTurn180) || (RightTurnCNT <= LeftTurn180)) {
    //LRcheckForward();

ISR_count1();
ISR_count2();

    LeftTurn180CNTOn = HIGH;

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= LeftTurn180) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn180) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= LeftTurn180) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    else if (RightTurnCNT >= LeftTurn180) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }


  }

  LeftTurn180CNTOn = LOW;

}


//***********************************************************************************************************

void MoveLeft180CNTAI() {

  if (LeftTurn180CNTOn == HIGH)
  {

    LeftTurn180On = HIGH;

    //        Serial.println("");
    //        Serial.print("DistLeftCheckCNT :");
    //        Serial.print(DistLeftCheckCNT);
    //        Serial.println("");

  } else if (LeftTurn180CNTOn == LOW) {
    LeftTurn180On = LOW;
  }


  if (LeftTurn180On != LastLeftTurn180CNT) {
    if (LeftTurn180On == HIGH) {
      LeftTurn180CNT ++;
    } else {


    }
    delay(10);
  }

  LastLeftTurn180CNT = LeftTurn180On;


  if (LeftTurn180CNT == 2)
  {
    LeftTurn180CNT = 0;
  }
  //
  //            Serial.println("");
  //            Serial.print("LeftTurn180On :");
  //            Serial.print(LeftTurn180On);
  //            Serial.println("");
}


//***********************************************************************************************************
//                               MOVE LEFT 45 CONTROL AI


void MoveLeft45AI() {

  RightTurn45 = 8;  // 150 mm = 13.5 counts
  LeftTurn45 = 8;  // 150 mm = 13.5 counts

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 0);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((LeftTurnCNT <= LeftTurn45) || (RightTurnCNT <= LeftTurn45)) {

ISR_count1();
ISR_count2();

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= LeftTurn45) {
      analogWrite (LeftSpeedPin, speedLeftLeftAI);
    }

    if (LeftTurnCNT >= LeftTurn45) {
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= LeftTurn45) {
      analogWrite (RightSpeedPin, speedLeftRightAI);
    }

    if (RightTurnCNT >= LeftTurn45) {
      analogWrite (RightSpeedPin, 0);
    }
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT 90 CONTROL AI

void MoveRight90AI() {

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 0);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((LeftTurnCNT <= RightTurn90) || (RightTurnCNT <= RightTurn90)) {
    //LRcheckForward();

ISR_count1();
ISR_count2();

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }


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
  LeftDifference =  0;
  RightDifference = 0;
}

//***********************************************************************************************************
//***********************************************************************************************************
//                          TURN MOVE RIGHT 90 CONTROL AI

void TurnMoveRight90AI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

  LeftCntFor = 46;
  RightCntFor = 46;

  RightTurn90 = 16;  // 300 mm = 27 counts
  LeftTurn90 = 16;  // 300 mm = 27 counts

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  while ((LeftTurnCNT <= RightTurn90) || (RightTurnCNT <= RightTurn90)) {
    
ISR_count1();
ISR_count2();
    
    analogWrite(LeftForRevPin, 0);
    analogWrite(LeftMotorPowerOnPin, 255);
    analogWrite(RightForRevPin, 255);
    analogWrite(RightMotorPowerOnPin, 255);

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }


    if (LeftTurnCNT <= RightTurn90) {
      analogWrite (LeftSpeedPin, speedRightLeftAI);
    }

    if (RightTurnCNT <= RightTurn90) {
      analogWrite (RightSpeedPin, speedRightRightAI);
    }

  }

  while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)) {

ISR_count1();
ISR_count2();

    ScanPulseTimerL();
    ScanPulseTimerR();
    ScanRearTimer();

    analogWrite(LeftForRevPin, 255);
    analogWrite(LeftMotorPowerOnPin, 255);
    analogWrite(RightForRevPin, 255);
    analogWrite(RightMotorPowerOnPin, 255);

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= LeftCntFor) {
      analogWrite (LeftSpeedPin, speedForwardLeftAI);
    }

    else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (LeftSpeedPin, 0);
      digitalWrite(grnledPin, LOW);
    }

    if (RightTurnCNT <= RightCntFor) {
      analogWrite (RightSpeedPin, speedForwardRightAI);
    }

    else if (RightTurnCNT >= RightCntFor) {
      analogWrite (RightSpeedPin, 0);
      digitalWrite(bluledPin, LOW);
    }


    LeftDifference =  0;
    RightDifference = 0;
  }


}

//***********************************************************************************************************


//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT U-TURN 180 CONTROL AI

void MoveRightU180AI() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  RightUTurn180 = 64;  // 300 mm = 27 counts
  LeftUTurn180 = 0;  // 300 mm = 27 counts

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 0);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((LeftTurnCNT <= RightUTurn180)) { // || (RightTurnCNT <= RightUTurn180)) {

ISR_count1();
ISR_count2();

    RightUTurn180CNTOn = HIGH;

    //LRcheckForward();
//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }


    if (LeftTurnCNT <= RightUTurn180) {
      analogWrite (LeftSpeedPin, speedRightLeftAI);
    }

    else if (LeftTurnCNT >= LeftUTurn180) {
      analogWrite (LeftSpeedPin, 0);
    }

    //        if (RightTurnCNT <= RightUTurn180) {
    //          analogWrite (RightSpeedPin, speedRightRightAI);
    //        }
    //
    if (RightTurnCNT >= RightUTurn180) {                                   //if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }

  }

  LeftDifference =  0;
  RightDifference = 0;

//  Serial.println("");
//  Serial.print("RightUTurn180On :");
//  Serial.print(RightUTurn180On);
//  Serial.println("");
//
//  Serial.println("");
//  Serial.print("RightUTurn180CNT :");
//  Serial.print(RightUTurn180CNT);
//  Serial.println("");


}


//***********************************************************************************************************

void MoveRightU180CNTAI() {

  if (RightUTurn180CNTOn == HIGH)
  {

    RightUTurn180On = HIGH;

    //        Serial.println("");
    //        Serial.print("DistLeftCheckCNT :");
    //        Serial.print(DistLeftCheckCNT);
    //        Serial.println("");

  } else if (RightUTurn180CNTOn == LOW) {
    RightUTurn180On = LOW;
  }


  if (RightUTurn180On != LastRightUTurn180CNT) {
    if (RightUTurn180On == HIGH) {
      RightUTurn180CNT ++;
    } else {


    }
    //    delay(10);
  }

  LastRightUTurn180CNT = RightUTurn180On;


  if (RightUTurn180CNT == 2)
  {
    RightUTurn180CNT = 0;
  }
  //         LeftUTurn180CNTOn = LOW;
  RightUTurn180CNTOn = LOW;
  //
  //  Serial.println("");
  //  Serial.print("RightUTurn180On :");
  //  Serial.print(RightUTurn180On);
  //  Serial.println("");
  //
  //  Serial.println("");
  //  Serial.print("RightUTurn180CNT :");
  //  Serial.print(RightUTurn180CNT);
  //  Serial.println("");

  lcd.setCursor(6, 0);
  lcd.print(RightUTurn180CNT);
  lcd.print(" ");

}

//***********************************************************************************************************
//                               MOVE RIGHT 180 CONTROL AI

void MoveRight180AI() {

  RightTurn180 = 32;  // 300 mm = 27 counts
  LeftTurn180 = 32;  // 300 mm = 27 counts


  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 0);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  while ((LeftTurnCNT <= RightTurn180) || (RightTurnCNT <= RightTurn180)) {

ISR_count1();
ISR_count2();

    RightTurn180CNTOn = HIGH;

    //LRcheckForward();
//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }


    if (LeftTurnCNT <= RightTurn180) {
      analogWrite (LeftSpeedPin, speedRightLeftAI);
    }

    else if (LeftTurnCNT >= LeftTurn180) {
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= RightTurn180) {
      analogWrite (RightSpeedPin, speedRightRightAI);
    }

    else if (RightTurnCNT >= RightTurn180) {                                   //if (RightTurnCNT >= RightTurn90){
      analogWrite (RightSpeedPin, 0);
    }


  }
  LeftDifference =  0;
  RightDifference = 0;

  //            Serial.println("");
  //            Serial.print("RightTurn180On :");
  //            Serial.print(RightTurn180On);
  //            Serial.println("");

  //            Serial.println("");
  //            Serial.print("RightTurn180CNT :");
  //            Serial.print(RightTurn180CNT);
  //            Serial.println("");

  RightTurn180CNTOn = LOW;

}

//***********************************************************************************************************

void MoveRight180CNTAI() {


  if (RightTurn180CNTOn == HIGH)
  {

    RightTurn180On = HIGH;


  } else if (RightTurn180CNTOn == LOW) {
    RightTurn180On = LOW;
  }


  if (RightTurn180On != LastRightTurn180CNT) {
    if (RightTurn180On == HIGH) {
      RightTurn180CNT ++;
    } else {


    }
    delay(10);
  }

  LastRightTurn180CNT = RightTurn180On;


  if (RightTurn180CNT == 2)
  {
    RightTurn180CNT = 0;
  }
  //

  //            Serial.println("");
  //            Serial.print("RightTurn180On :");
  //            Serial.print(RightTurn180On);
  //            Serial.println("");

  //            Serial.println("");
  //            Serial.print("RightTurn180CNT :");
  //            Serial.print(RightTurn180CNT);
  //            Serial.println("");


}



//***********************************************************************************************************
//                               MOVE RIGHT 45 CONTROL AI

void MoveRight45AI() {

  RightTurn45 = 7;  // 150 mm = 13.5 counts
  LeftTurn45 = 7;  // 150 mm = 13.5 counts

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 0);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  //ScanRight();

  while ((LeftTurnCNT <= RightTurn45) || (RightTurnCNT <= RightTurn45)) {


ISR_count1();
ISR_count2();

//    if (TurnDetected1) {
//      LeftTurnCNT = LeftTurnCNT + 1;
//      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//    }
//    if (TurnDetected2) {
//      RightTurnCNT = RightTurnCNT + 1;
//      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//    }

    if (LeftTurnCNT <= RightTurn45) {
      analogWrite (LeftSpeedPin, speedRightLeftAI);
    }

    if (LeftTurnCNT >= RightTurn45) {
      analogWrite (LeftSpeedPin, 0);
    }

    if (RightTurnCNT <= RightTurn45) {
      analogWrite (RightSpeedPin, speedRightRightAI);
    }

    if (RightTurnCNT >= RightTurn45) {
      analogWrite (RightSpeedPin, 0);
    }
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE 500mm CONTROL AI


void Move500AI() {

  LeftCntFor = 46;
  RightCntFor = 46;

  LeftTurnCNT = 0;
  RightTurnCNT = 0;


ISR_count1();
ISR_count2();


  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  //ScanForwardMan();

  //    delay(1000);

//  if (TurnDetected1) {
//    LeftTurnCNT = LeftTurnCNT + 1;
//    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//  }
//  if (TurnDetected2) {
//    RightTurnCNT = RightTurnCNT + 1;
//    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//  }

  //while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)){

  if (LeftTurnCNT <= LeftCntFor) {
    analogWrite (LeftSpeedPin, speedForwardLeftAI);
  }

  else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (LeftSpeedPin, 0);
  }

  if (RightTurnCNT <= RightCntFor) {
    analogWrite (RightSpeedPin, speedForwardRightAI);
  }

  else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
    analogWrite (RightSpeedPin, 0);
  }

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}
//}

//***********************************************************************************************************
void MoveScanAI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

  LeftCntFor = 46;
  RightCntFor = 46;

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

ISR_count1();
ISR_count2();
  
  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  //ScanForwardMan();

  //    delay(1000);

//  if (TurnDetected1) {
//    LeftTurnCNT = LeftTurnCNT + 1;
//    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//  }
//  if (TurnDetected2) {
//    RightTurnCNT = RightTurnCNT + 1;
//    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//  }

  //while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)){

  if (LeftTurnCNT <= LeftCntFor) {
    analogWrite (LeftSpeedPin, speedForwardLeftAI);
  }

  else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (LeftSpeedPin, 0);
  }

  if (RightTurnCNT <= RightCntFor) {
    analogWrite (RightSpeedPin, speedForwardRightAI);
  }

  else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
    analogWrite (RightSpeedPin, 0);
  }

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

}
//}

//***********************************************************************************************************
//                               MOVE 200mm LEFT CONTROL AI

void Move200LeftAI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

ISR_count1();
ISR_count2();

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

//  if (TurnDetected1) {
//    LeftTurnCNT = LeftTurnCNT + 1;
//    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//  }
//  if (TurnDetected2) {
//    RightTurnCNT = RightTurnCNT + 1;
//    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//  }

  //ScanLeft();

  while (RearDistReadingL <= 150) {

    //    ScanLeft();
    BackScanLeft();

    if (RearDistReadingL <= 150) {

      //      ScanLeft();
      BackScanLeft();

      analogWrite (LeftSpeedPin, speedForwardLeftAI);
      analogWrite (RightSpeedPin, speedForwardRightAI);

    }
    //else
    if (RearDistReadingL >= 250)
    {
      analogWrite (LeftSpeedPin, 0);
      analogWrite (RightSpeedPin, 0);

    }
  }

}

//***********************************************************************************************************
//                               MOVE 200mm RIGHT CONTROL AI


void Move200RightAI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

ISR_count1();
ISR_count2();

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  //ScanForwardMan();

  //      delay(2000);

//  if (TurnDetected1) {
//    LeftTurnCNT = LeftTurnCNT + 1;
//    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//  }
//  if (TurnDetected2) {
//    RightTurnCNT = RightTurnCNT + 1;
//    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//  }

  //ScanRight();

  while (RearDistReadingR <= 150) {

    //    ScanRight();
    BackScanRight();


    if (RearDistReadingR <= 150) {

      //      ScanRight();
      BackScanRight();

      analogWrite (LeftSpeedPin, speedForwardLeftAI);
      analogWrite (RightSpeedPin, speedForwardRightAI);

    }
    //else
    if (RearDistReadingR >= 250)
    {
      analogWrite (LeftSpeedPin, 0);
      analogWrite (RightSpeedPin, 0);

    }
  }

}

//***********************************************************************************************************


//***********************************************************************************************************
//                               MOVE 500mm + Offset CONTROL AI

void MoveOffset() {

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

ISR_count1();
ISR_count2();

  analogWrite (LeftSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  analogWrite(LeftForRevPin, 255);
  analogWrite(LeftMotorPowerOnPin, 255);
  analogWrite(RightForRevPin, 255);
  analogWrite(RightMotorPowerOnPin, 255);

  //ScanForwardMan();

  //    delay(1000);

//  if (TurnDetected1) {
//    LeftTurnCNT = LeftTurnCNT + 1;
//    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
//  }
//  if (TurnDetected2) {
//    RightTurnCNT = RightTurnCNT + 1;
//    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
//  }


  if (LeftTurnCNT <= (LeftCntFor + LeftOffset)) {
    analogWrite (LeftSpeedPin, speedForwardLeftAI);
  }

  else if (LeftTurnCNT >= (LeftCntFor + LeftOffset)) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (LeftSpeedPin, 0);
  }

  if (RightTurnCNT <= (RightCntFor + RightOffset)) {
    analogWrite (RightSpeedPin, speedForwardRightAI);
  }

  else if (RightTurnCNT >= (RightCntFor + RightOffset)) {                           // if (RightTurnCNT >= RightTurn90){
    analogWrite (RightSpeedPin, 0);
  }


  //  delay(ManuverDelay);

  analogWrite (RightSpeedPin, 0);
  analogWrite (RightSpeedPin, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  //***
  rampStartTime = millis();
}

//***********************************************************************************************************

//***********************************************************************************************************
////***********************************************************************************************************
////                                      SCAN FORWARD (LEFT / RIGHT)
////***********************************************************************************************************
//                                         SCANNER FRONT SERVO CONTROL

void ScanForward() {

  AngleM = Angle;
//
//    DistreadingFrontWatch = (FrontSensor.distanceInCm() * 10);
//    FrontDistReading = (FrontSensor.distanceInCm() * 10);

  Progress = millis() - ServoMoveStartTime;

  if (Progress <= ServoMoveTime) {
    AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
    myservoScanFront.write(AngleM);

  }

  if (Progress >= ServoMoveTime) {
    AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
    myservoScanFront.write(AngleM);

  }

  if (Progress >= ResetTime) {
    ServoMoveStartTime = millis();

  }

  if ((AngleM <= 170) || (AngleM >= 10)) {
    DistreadingFrontWatch = (FrontSensor.distanceInCm() * 10);
    FrontDistReading = (FrontSensor.distanceInCm() * 10);
  }

}

//***********************************************************************************************************
////***********************************************************************************************************
////                                      FORWARD CHECK ENTRY SCAN

void CheckForwardEntry() {


  //ForwardScanDist = 0;

  myservoScanFront.write(90);

  delay(20);

  ForwardScanDist  = (FrontSensor.distanceInCm() * 10);

  delay(10);
//
//
//  Serial.println("");
//  Serial.print("ForwardScanDist :");
//  Serial.print(ForwardScanDist);
//  Serial.println("");

}

////***********************************************************************************************************

////***********************************************************************************************************
//                                         SCANNER SERVO CONTROL

void ScanForwardMan() {

  AngleM = Angle;

  Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

  if (Progress <= ServoMoveTime) {
    AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
    myservoScanFront.write(AngleM);

  }

  if (Progress >= ServoMoveTime) {
    AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
    myservoScanFront.write(AngleM);

  }

  if (Progress >= ResetTime) {
    ServoMoveStartTime = millis();

  }
  //  delay(1000);

}

//***********************************************************************************************************

////***********************************************************************************************************
////                                      SCAN REAR (LEFT / RIGHT)

////***********************************************************************************************************
//                                         SCANNER REAR SERVO CONTROL

//void ScanRear() {
//
//  ScanForward();
//  Scan4StepControlTimer();
//  LeftScan2StepControlTimer();
//  RightScan2StepControlTimer();
//
//  myservoScanBack.write(0);
//
//  delay(1000);
//
//  BigLeftDist  = (BackSensor.distanceInCm() * 10);
//
//  lcd1.setCursor(8, 0);
//  lcd1.print("BL:");
//  lcd1.print(BigLeftDist);
//  lcd1.print(" ");
//  //}
//  delay(1000);
//
//
//  //if (StepScan1OnLeft == HIGH){
//
//  myservoScanBack.write(170);
//
//  delay(1000);
//
//  BigRightDist  = (BackSensor.distanceInCm() * 10);
//
//  delay(1000);
//
//  lcd1.setCursor(0, 0);
//  lcd1.print("BR:");
//  lcd1.print(BigRightDist);
//  lcd1.print(" ");
//  //}
//  // ScanRearRunning = LOW;
//
//
//  //**
//
//  if ((LeftSelState == LOW)) {  // && (ScanRearRunning = LOW))) {
//
//    BackCheckLeft();
//
//  }
//
//
//  if ((RightSelState == LOW)) {  // && (ScanRearRunning = LOW))){
//
//    BackCheckRight();
//
//  }
//
//}
//***********************************************************************************************************

////***********************************************************************************************************
//                                         SCANNER REAR SERVO CONTROL

void ScanRear() {

  ScanForward();
  Scan4StepControlTimer();
  LeftScan2StepControlTimer();
  RightScan2StepControlTimer();

  myservoScanBack.write(0);

  delay(1000);

  BigLeftDist  = (BackSensor.distanceInCm() * 10);

  lcd.setCursor(8, 2);
  lcd.print("BL:");
  lcd.print(BigLeftDist);
  lcd.print(" ");
  //}
  delay(1000);


  //if (StepScan1OnLeft == HIGH){

  myservoScanBack.write(170);

  delay(1000);

  BigRightDist  = (BackSensor.distanceInCm() * 10);

  delay(1000);

  lcd.setCursor(0, 2);
  lcd.print("BR:");
  lcd.print(BigRightDist);
  lcd.print(" ");
  //}
  // ScanRearRunning = LOW;


  //**

  if ((BigLeftDist <= BigRightDist)) {  // && (ScanRearRunning = LOW))) {
    LeftUTurn180CNT = 1;
    RightUTurn180CNT = 0;
    BackCheckLeft();
  }


  if ((BigRightDist <= BigLeftDist)) {  // && (ScanRearRunning = LOW))){
    LeftUTurn180CNT = 0;
    RightUTurn180CNT = 1;
    BackCheckRight();

  }
}
//***********************************************************************************************************
//                      SWITCH LEFT / RIGHT SELECTOR STATE

void SelLeftRight() {

  if (LeftSelState == LOW) {  // && (ScanRearRunning = LOW))) {
    //LeftUTurn180CNT = 1;
    //RightUTurn180CNT = 0;

    BackCheckLeft();

  }


  if (RightSelState == LOW) {  // && (ScanRearRunning = LOW))){
    //LeftUTurn180CNT = 0;
    //RightUTurn180CNT = 1;

    BackCheckRight();

  }

}



//***********************************************************************************************************
//                                      CHECK LEFT


void BackCheckLeft() {

  ScanForward();
  Scan4StepControlTimer();
  LeftScan2StepControlTimer();
  RightScan2StepControlTimer();
  LeftScan2StepControlTimer();

  //  if (StepScan1OnLeft == HIGH) {

  LeftDifference = 0;
  RightDifference = 0;
  digitalWrite(grnledPin, LOW);
  digitalWrite(bluledPin, LOW);
  myservoScanBack.write(0);
  //  }

  //    if (StepScan2OnLeft == HIGH) {

  delay(1000);
  ScanRearRunning = LOW;
  BackScanLeftTurn();
  //    }
}


////***********************************************************************************************************
//***********************************************************************************************************
//                                      CHECK RIGHT


void BackCheckRight() {

  ScanForward();
  Scan4StepControlTimer();
  LeftScan2StepControlTimer();
  RightScan2StepControlTimer();

  //  if (StepScan1OnRight == HIGH) {

  LeftDifference = 0;
  RightDifference = 0;
  digitalWrite(grnledPin, LOW);
  digitalWrite(bluledPin, LOW);
  myservoScanBack.write(170);
  //  }
  delay(1000);

  //  if (StepScan2OnRight == HIGH) {

  ScanRearRunning = LOW;
  BackScanRightTurn();
  //  }
}


////***********************************************************************************************************

//***********************************************************************************************************
//                                      SCAN LEFT


void BackScanLeft() {


  myservoScanBack.write(0);

  delay(1000);

  RearDistReadingL  = (BackSensor.distanceInCm() * 10);

  BigLeftDist = RearDistReadingL;

  lcd.setCursor(8, 2);
  lcd.print("BL:");
  lcd.print(RearDistReadingL);
  lcd.print(" ");

  delay(1000);
}

////***********************************************************************************************************
//***********************************************************************************************************
//                                      SCAN LEFT 180


void FrontScanLeft180() {

  RearDistReadingTurnL  = 0;

  myservoScanFront.write(170);

  delay(1000);

  RearDistReadingTurnL  = (FrontSensor.distanceInCm() * 10);
  digitalWrite(buzzerPin, HIGH);

  delay(100);

  //  DifferanceLeftTurnU180  = ((RearDistReadingTurnL >= RearDistReadingTurnL + 100) || (RearDistReadingTurnL <= RearDistReadingTurnL - 100));


  lcd.setCursor(8, 0);
  lcd.print(RearDistReadingTurnL);
  lcd.print(" ");


  digitalWrite(buzzerPin, LOW);

  delay(1000);

  //int DifferanceLeftTurnU180 = 0;
  //int DifferanceRightTurnU180 = 0;

}

////***********************************************************************************************************

////***********************************************************************************************************
////                                      SCAN RIGHT

void BackScanRight() {


  RearDistReadingR = 0;

  myservoScanBack.write(170);

  delay(1500);

  RearDistReadingR  = (BackSensor.distanceInCm() * 10);
  //  RearDistReadingTurnR  = (BackSensor.distanceInCm() * 10);

  BigRightDist = RearDistReadingR;

  lcd.setCursor(8, 2);
  lcd.print("BL:");
  lcd.print(RearDistReadingL);
  lcd.print(" ");

  delay(1000);
}

////***********************************************************************************************************
////***********************************************************************************************************
////                                      SCAN RIGHT 180


void FrontScanRight180() {


  myservoScanFront.write(10);

  delay(1500);

  RearDistReadingTurnR = (FrontSensor.distanceInCm() * 10);

  //DifferanceRightTurnU180  = ((RearDistReadingTurnR >= RearDistReadingTurnR + 100) && (RearDistReadingTurnR <= RearDistReadingTurnR - 100));

  digitalWrite(buzzerPin, HIGH);

  delay(100);

  lcd.setCursor(1, 0);
  lcd.print(RearDistReadingTurnR);
  lcd.print(" ");


  digitalWrite(buzzerPin, LOW);

  delay(1000);
}

////***********************************************************************************************************

////***********************************************************************************************************
//                                         REAR SCANNER SCAN TIMER CONTROL

void ScanRearTimer() {


  if (Entry == LOW) {

    AngleM3 = Angle3;

    Progress3 = millis() - ServoMoveStartTime3;     // Servo Head Progress

    if (Progress3 <= ServoMoveTime3) {
      AngleM3 = map(Progress3, 0, ServoMoveTime3, StopAngleM3, StartAngleM3);

      RearScanOn = HIGH;
      ScanRear();
    }

    if (Progress3 >= ServoMoveTime3) {
      AngleM3 = map(Progress1, ServoMoveTime1, ResetTime1, StartAngleM1, StopAngleM1);
      RearScanOn = LOW;

    }

    if (Progress3 >= ResetTime3) {
      ServoMoveStartTime3 = millis();

    }
  }
}

//***********************************************************************************************************

////***********************************************************************************************************
//                                    LEFT SCANNER SCAN PULSE TIMER CONTROL

void ScanPulseTimerL() {

  AngleM4 = Angle4;

  Progress4 = millis() - ServoMoveStartTime4;     // Servo Head Progress

  if (Progress4 <= ServoMoveTime4) {
    AngleM4 = map(Progress4, 0, ServoMoveTime4, StopAngleM4, StartAngleM4);
    PulseOnL = HIGH;
    digitalWrite(redledPin, HIGH);
    PulseOnR = HIGH;
    digitalWrite(whtledPin, HIGH);
  }

  if (Progress4 >= ServoMoveTime4) {
    AngleM4 = map(Progress4, ServoMoveTime4, ResetTime4, StartAngleM4, StopAngleM4);
    PulseOnL = LOW;
    digitalWrite(redledPin, LOW);
    PulseOnR = LOW;
    digitalWrite(whtledPin, LOW);

  }

  if (Progress4 >= ResetTime4) {
    ServoMoveStartTime4 = millis();

  }

}

//***********************************************************************************************************
////***********************************************************************************************************
//                                    RIGHT SCANNER SCAN PULSE TIMER CONTROL

void ScanPulseTimerR() {

  //  AngleM5 = Angle5;
  //
  //  Progress5 = millis() - ServoMoveStartTime5;     // Servo Head Progress
  //
  //  if (Progress5 <= ServoMoveTime5) {
  //    AngleM5 = map(Progress5, 0, ServoMoveTime5, StopAngleM5, StartAngleM5);
  //    PulseOnR = HIGH;
  //    digitalWrite(whtledPin, HIGH);
  //  }
  //
  //  if (Progress5 >= ServoMoveTime5) {
  //    AngleM5 = map(Progress5, ServoMoveTime5, ResetTime5, StartAngleM5, StopAngleM5);
  //    PulseOnR = LOW;
  //    digitalWrite(whtledPin, LOW);
  //
  //  }
  //
  //  if (Progress5 >= ResetTime5) {
  //    ServoMoveStartTime5 = millis();
  //
  //  }

}

//***********************************************************************************************************
////***********************************************************************************************************
//                                    RIGHT SCANNER SCAN PULSE TIMER CONTROL

void Scan4StepControlTimer() {

  Progress6 = millis() - ServoMoveStartTime6;     // Servo Head Progress

  if (Progress6 <= ServoMoveTime6) {
    Step1On = HIGH;
  }

  if (Progress6 >= ResetTime6) {
    Step1On = LOW;
    Step2On = HIGH;

  }

  if (Progress6 >= ResetTime6a) {
    Step2On = LOW;
    Step3On = HIGH;

  }

  if (Progress6 >= ResetTime6b) {
    Step3On = LOW;
    Step4On = HIGH;

  }

  if (Progress6 >= ResetTime6c) {
    Step4On = LOW;
    //      Step2On = HIGH;
    //      ServoMoveStartTime6 = millis();

  }

  if (Progress6 >= ResetTime6d) {

    Step1On = LOW;
    Step2On = LOW;
    Step3On = LOW;
    Step4On = LOW;
    ServoMoveStartTime6 = millis();

  }

}


//***********************************************************************************************************

void LeftScan2StepControlTimer() {

  Progress7 = millis() - ServoMoveStartTime7;     // Servo Head Progress

  if (Progress7 <= ServoMoveTime7) {
    StepScan1OnLeft = HIGH;
  }

  if (Progress7 >= ServoMoveTime7) {
    StepScan1OnLeft = LOW;
    StepScan2OnLeft = HIGH;

  }

  if (Progress7 >= ResetTime7) {

    StepScan1OnLeft = LOW;
    StepScan2OnLeft = LOW;

    ServoMoveStartTime7 = millis();
  }

}

//***********************************************************************************************************
//***********************************************************************************************************

void RightScan2StepControlTimer() {

  Progress8 = millis() - ServoMoveStartTime8;     // Servo Head Progress

  if (Progress8 <= ServoMoveTime8) {
    StepScan1OnRight = HIGH;
  }

  if (Progress8 >= ServoMoveTime8) {
    StepScan1OnRight = LOW;
    StepScan2OnRight = HIGH;

  }

  if (Progress8 >= ResetTime8) {

    StepScan1OnRight = LOW;
    StepScan2OnRight = LOW;

    ServoMoveStartTime8 = millis();
  }

}

//***********************************************************************************************************

//                        SCAN LEFT / RIGHT FORWARD CHECK TO TURN

void LRcheckForward() {

  //          SCAN LEFT / RIGHT FORWARD CHECK TO TURN
  //

  //  ScanRearTimer();
  ScanPulseTimerL();
  //  ScanPulseTimerR();


}

////***********************************************************************************************************

void BackScanLeftTurn() {

  digitalWrite(grnledPin, LOW);


  if (LeftSelState == LOW) {
    //  if (BigLeftDist <= BigRightDist) {

    //    ScanRearTimer();
    ScanPulseTimerL();
    //  ScanPulseTimerR();

    if ((ScanRearRunning == LOW) && (RearScanOn == LOW)) {
      myservoScanBack.write(0);
      AngleM1 = 3;
      //delay(250);
    }

    if ((AngleM1 <= 10) && (AngleM1 >= 0)) {
      //      ScanRearTimer();
      ScanPulseTimerL();
      //  ScanPulseTimerR();
      if (PulseOnL == HIGH)
      {
        digitalWrite(buzzerPin, HIGH);
        LeftCntOn = HIGH;

        //        Serial.println("");
        //        Serial.print("DistLeftCheckCNT :");
        //        Serial.print(DistLeftCheckCNT);
        //        Serial.println("");

      } else if (PulseOnL == LOW) {
        digitalWrite(buzzerPin, LOW);
        LeftCntOn = LOW;
      }


      if (LeftCntOn != LastDistLeftCheckCNT) {
        if (LeftCntOn == HIGH) {
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

    }

    if (((DistLeftCheckCNT == 0) && (PulseOnL == HIGH)) && ((AngleM1 <= 5) && (AngleM1 >= 0)))
      //      if ((DistLeftCheckCNT == 0) && (PulseOnL == HIGH))
    {

      //        LeftDifferenceOn = LOW;
      //        digitalWrite(grnledPin, LOW);
      //        grnledState = LOW;
      //        LeftDifference = 0;

      FirstLeftState  = (BackSensor.distanceInCm() * 10);
      PercentFirstLeftState = FirstLeftState;

      //          Serial.println("");
      //          Serial.print("FirstLeftState :");
      //          Serial.print(FirstLeftState);
      //          Serial.print(" MM");
      //          Serial.println("");

      lcd.setCursor(8, 2);
      lcd.print("0:");
      lcd.print(FirstLeftState);
      lcd.print(" ");
      lcd.print(LeftDifferenceOn);
      lcd.print(" ");

    }

    if (AngleM1 >= 5) {
      digitalWrite(grnledPin, LOW);
    }

    if (((DistLeftCheckCNT == 1) && (PulseOnL == HIGH)) && ((AngleM1 <= 5) && (AngleM1 >= 0)))
      //      if ((DistLeftCheckCNT == 1) && (PulseOnL == HIGH))
    {

      SecondLeftState  = (BackSensor.distanceInCm() * 10);
      PercentSecondLeftState = SecondLeftState;

      //          Serial.println("");
      //          Serial.print("SecondLeftState :");
      //          Serial.print(SecondLeftState);
      //          Serial.print(" MM");
      //          Serial.println("");

      lcd.setCursor(8, 3);
      lcd.print("1:");
      lcd.print(SecondLeftState);
      lcd.print(" ");
    }
    //    }



    //    if (grnledState == HIGH) {
    //      LeftDifference = 0;
    //    } else {
    LeftDifference = (PercentFirstLeftState - PercentSecondLeftState);
    //    }


    //    if ((LeftDifference >= 100) || (LeftDifference <= -100) && (grnledState == LOW)) {
    if ((LeftDifference >= 200) || (LeftDifference <= -200)) {
      LeftDifferenceOn = HIGH;
      digitalWrite(grnledPin, HIGH);
      grnledState = HIGH;
    }
    else {
      LeftDifferenceOn = LOW;
      digitalWrite(grnledPin, LOW);
      grnledState = LOW;
      LeftDifference = 0;
      PercentFirstLeftState = 0;
      PercentSecondLeftState = 0;

      lcd.setCursor(14, 3);
      lcd.print(grnledState);
      lcd.print(" ");

    }


    //  }
    //  }
    //if ((grnledState == HIGH) && (PulseOnL == HIGH)){
    //     digitalWrite(grnledPin, LOW);
    //}
  }
}
//***********************************************************************************************************
////***********************************************************************************************************

void BackScanRightTurn() {

  if (RightSelState == LOW) {

    digitalWrite(bluledPin, LOW);
    //  ScanRearTimer();
    ScanPulseTimerL();
    //  ScanPulseTimerR();

    //    SCAN RIGHT

    //  if (BigRightDist <= BigLeftDist) {

    if ((ScanRearRunning == LOW)) { // && (RearScanOn == LOW)){
      myservoScanBack.write(170);
      AngleM1 = 175;
      //delay(250);
    }

    //        if (AngleM1 >= 160){

    if ((AngleM1 >= 170) && (AngleM1 <= 180)) {
      //      ScanRearTimer();
      ScanPulseTimerL();
      //  ScanPulseTimerR();
      if (PulseOnR == HIGH)
      {
        digitalWrite(buzzerPin, HIGH);
        RightCntOn = HIGH;

        //        Serial.println("");
        //        Serial.print("DistRightCheckCNT :");
        //        Serial.print(DistRightCheckCNT);
        //        Serial.println("");
      } else if (PulseOnR == LOW) {

        digitalWrite(buzzerPin, LOW);
        RightCntOn = LOW;
      }


      //        if (myservoScanBack >= 160) {
      // LeftDifference =  0;
      if (RightCntOn != LastDistRightCheckCNT) {
        if (RightCntOn == HIGH) {
          DistRightCheckCNT ++;
        } else {

        }
        delay(10);
      }
      //        }

      LastDistRightCheckCNT = RightCntOn;

      if (DistRightCheckCNT == 2)
      {
        DistRightCheckCNT = 0;
      }

      if (((DistRightCheckCNT == 0) && (PulseOnR == HIGH)) && ((AngleM1 >= 170) && (AngleM1 <= 180)))
        //      if ((DistRightCheckCNT == 0) && (PulseOnR == HIGH))
      {

        RightDifferenceOn = LOW;
        digitalWrite(bluledPin, LOW);
        bluledState = LOW;
        RightDifference = 0;

        FirstRightState  = (BackSensor.distanceInCm() * 10);
        PercentFirstRightState = FirstRightState;

        //          Serial.println("");
        //          Serial.print("PercentFirstRightState :");
        //          Serial.print(PercentFirstRightState);
        //          Serial.print(" MM");
        //          Serial.println("");

        lcd.setCursor(0, 3);
        lcd.print("0:");
        lcd.print(PercentFirstRightState);
        lcd.print(" ");

      }

      if (((DistRightCheckCNT == 1) && (PulseOnR == HIGH))  && ((AngleM1 >= 170) && (AngleM1 <= 180)))
        //      if ((DistRightCheckCNT == 1) && (PulseOnR == HIGH))
      {

        SecondRightState  = (BackSensor.distanceInCm() * 10);
        PercentSecondRightState = SecondRightState;

        //          Serial.println("");
        //          Serial.print("PercentSecondRightState :");
        //          Serial.print(PercentSecondRightState);
        //          Serial.print(" MM");
        //          Serial.println("");

        lcd.setCursor(0, 3);
        lcd.print("1:");
        lcd.print(PercentSecondRightState);
        lcd.print(" ");

      }
    }
    if (bluledState == HIGH) {
      RightDifference = 0;
    } else {
      RightDifference = (PercentFirstRightState - PercentSecondRightState);
    }


    //    if ((RightDifference >= 100) || (RightDifference <= -100) && (bluledState == LOW)) {
    if ((RightDifference >= 200) || (RightDifference <= -200)) {
      RightDifferenceOn = HIGH;
      digitalWrite(bluledPin, HIGH);
      bluledState = HIGH;
    }
    else {
      RightDifferenceOn = LOW;
      digitalWrite(bluledPin, LOW);
      bluledState = LOW;
      RightDifference = 0;
      PercentFirstRightState = 0;
      PercentSecondRightState = 0;


      lcd.setCursor(6, 2);
      lcd.print(bluledState);
      lcd.print(" ");
      //      }
    }

    //  }
  }
}

//***********************************************************************************************************

void batteryFull1(){

          lcd1.setCursor(0, 1);
          lcd1.print("Batt. :");
          lcd1.setCursor(8, 1);
          lcd1.write((byte)0);
          digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");            
    BatBuzzStartTime = millis();
}

//***********************************************************************************************************

void battery3Quater1(){

          lcd1.setCursor(0, 1);
          lcd1.print("Batt. :");
          lcd1.setCursor(8, 1);
          lcd1.write((byte)1);
          digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");           
    BatBuzzStartTime = millis();
}

//***********************************************************************************************************

void batteryHalf1(){

          lcd1.setCursor(0, 1);
          lcd1.print("Batt. :");
          lcd1.setCursor(8, 1);
          lcd1.write((byte)2);
          digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");
    BatBuzzStartTime = millis();
}

//***********************************************************************************************************

void batteryQuat1(){

          lcd1.setCursor(0, 1);
          lcd1.print("Batt. :");
          lcd1.setCursor(8, 1);
          lcd1.write((byte)3);
          digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");          
    BatBuzzStartTime = millis();
}

//***********************************************************************************************************

void batteryEmpty1(){

BatBuzzProgress = millis() - BatBuzzStartTime;     // Servo Head Progress

  if (BatBuzzProgress <= BatBuzzMoveTime) {
    BatBuzzState = HIGH;
  }

  if (BatBuzzProgress >= BatBuzzMoveTime) {
    BatBuzzState = LOW;
  }

  if (BatBuzzProgress >= BatBuzzResetTime) {
    BatBuzzStartTime = millis();
  }  
  
//

//          

BatBlinkProgress = millis() - BatBlinkStartTime;     // Servo Head Progress

  if (BatBlinkProgress <= BatBlinkMoveTime) {
//    digitalWrite(ledPin, HIGH);
    BatBlinkState = HIGH;
  }

  if (BatBlinkProgress >= BatBlinkMoveTime) {
//    digitalWrite(ledPin, LOW);
    BatBlinkState = LOW;
  }

  if (BatBlinkProgress >= BatBlinkResetTime) {
    BatBlinkStartTime = millis();
  }  
  
//

  if (BatBuzzState == HIGH){
          
          digitalWrite(batbuzzerPin, HIGH);

  }
  
  if (BatBuzzState == LOW){
  
  
  digitalWrite(batbuzzerPin, LOW);
          
  }

  if (BatBlinkState == HIGH){
    
//  digitalWrite(batbuzzerPin, HIGH);
          lcd1.setCursor(8, 1);
          lcd1.write((byte)4);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");
          lcd1.setCursor(0, 1);
          lcd1.print("Batt. Lo");

  }

  if (BatBlinkState == LOW){

//  digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(8, 1);
          lcd1.print(" ");
          lcd1.setCursor(9, 1);
          lcd1.print("     ");
          lcd1.print("  ");
          lcd1.setCursor(0, 1);
          lcd1.print("               ");

  }
          
}


//***********************************************************************************************************
void BatteryBuzzerTimer(){

}

//***********************************************************************************************************

void BatteryLowBuzzer(){
}

//***********************************************************************************************************

void BatteryVoltageNormal(){

          digitalWrite(batbuzzerPin, LOW);
          lcd1.setCursor(9, 1);
          lcd1.print(BatteryVoltage * 0.98);
          lcd1.print(" V");

}


//***********************************************************************************************************

//Interrupt routine runs if CLK goes from HIGH to LOW

void isr1 () {

  //  delay(2); // delay for Debouncing  (4)
  if (digitalRead(PinCLK1)){

    rotationdirection1 = digitalRead(PinCLK1);
  TurnDetected1 = true;
RPMCounter1 = RPMCounter1++;

}
}
//Interrupt routine runs if CLK goes from HIGH to LOW

void isr2 () {

  //  delay(2); // delay for Debouncing  (4)
  if (digitalRead(PinCLK2)){

    rotationdirection2 = digitalRead(PinCLK2);
  TurnDetected2 = true;
RPMCounter2 = RPMCounter2++;

}
}


//***********************************************************************************************************

void LeftRpmCounter(){
LeftRpmTimer();

  if (TurnDetected1 == true) {
    PrevPosition1 = WheelPosition1;  // Save previous position in variable
    WheelPosition1 = WheelPosition1 + 1;  // Increase Position by 1
    RPMCounter1 = RPMCounter1 + 1;
    RPM1 = RPM1 + 1;
//   LeftTurnCNT = LeftTurnCNT + 1;
   LeftTurnRpmCNT = LeftTurnRpmCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();

  }

//  
int WheelTimeMultiply;
int WheelDiscDevide;


WheelTimeMultiply = 30;
WheelDiscDevide = 16;

    RPM1 = (RPMCounter1a * WheelTimeMultiply);
    RPMavg1 = RPM1 + RPMCounter1 / 2;
}


//***********************************************************************************************************
//***********************************************************************************************************

void RightRpmCounter(){
RightRpmTimer();
  if (TurnDetected2 == true) {
    PrevPosition2 = WheelPosition2;  // Save previous position in variable
    WheelPosition2 = WheelPosition2 + 1;  // Increase Position by 1
    RPMCounter2 = RPMCounter2 + 1;
    RPM2 = RPM2 + 1;
    RightTurnRpmCNT = RightTurnRpmCNT + 1;
//    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();
  }
  
  
int WheelTimeMultiply;
int WheelDiscDevide;


WheelTimeMultiply = 30;
WheelDiscDevide = 16;

    RPM2 =(RPMCounter2a * WheelTimeMultiply);
    RPMavg2 = RPM2 + RPMCounter2 / 2;
}


//***********************************************************************************************************
//***********************************************************************************************************

void LeftRpmTimer(){
// ISR_count2();
 
  TimeStart1 = millis() - TimeRunner1;

  if (TimeStart1 <= 100) {
    RPM1 = 0;
    RPMCounter1 = 0;
//  attachInterrupt (4, isr1, RISING);

  }


  if (TimeStart1 >= TimeStart1a) {
//  detachInterrupt (4);
    RPMCounter1a = RPMCounter1;
    RPM1 = RPMCounter1a / 16;
    RPM1 = RPM1 / 2000;
    RPM1 = RPM1 * 30000;
        
    RPMavg1 = (RPM1 + RPM1) / 2;
    TimeRunner1 = millis();
  
  }
//  
//  Serial.print("  RPM1 : ");
//  Serial.print(RPM1);
//  Serial.println("");  
//
//  Serial.print("  RPMCounter1 : ");
//  Serial.print(RPMCounter1);
//  Serial.println("");  
//
//  Serial.print("  TimeStart1 : ");
//  Serial.print(TimeStart1);
//  Serial.println("");  
  
//    RPM1 = RPM1 / 16;
//    RPM1 = RPM1 / 2000;
//    RPM1 = RPM1 * 3000;
//    RPMavg1 = (RPM1 + RPM1) / 2;

//  if (TimeStart1 >= Reset1) {
//    RPMCounter1a = RPMCounter1;
//    RPM1 = (RPMCounter1a / 16) * 30;
//    RPMavg1 = (RPM1 + RPM1) / 2;
//    TimeRunner1 = millis();
//  }   
}


//***********************************************************************************************************
//***********************************************************************************************************

void RightRpmTimer(){
// ISR_count1();

  TimeStart2 = millis() - TimeRunner2;

  if (TimeStart2 <= 100) {
    RPM2 = 0;
    RPMCounter2 = 0;
//  attachInterrupt (5, isr2, RISING);//    RPMCounter1a = (RPMCounter1 / 16);
  }

  if (TimeStart2 >= TimeStart2a) {
//  detachInterrupt (5);//    RPMCounter1a = (RPMCounter1 / 16);
    RPMCounter2a = RPMCounter2;
    RPM2 = RPMCounter2a / 16;
    RPM2 = RPM2 / 2000;
    RPM2 = RPM2 * 3000;
    RPMavg2 = (RPM2 + RPM2) / 2;
    TimeRunner2 = millis();
  }
  
//  Serial.print("  RPM2 : ");
//  Serial.print(RPM2);
//  Serial.println("");
//  
//  Serial.print("  RPMCounter2 : ");
//  Serial.print(RPMCounter2);
//  Serial.println("");
//  
//  Serial.print("  TimeStart2 : ");
//  Serial.print(TimeStart2);
//  Serial.println("");
  

//    RPM2 = RPM2 / 16;
//    RPM2 = RPM2 / 2000;
//    RPM2 = RPM2 * 3000;
//    RPMavg2 = (RPM2 + RPM2) / 2;
    
//  if (TimeStart2 >= Reset2) {
//    RPMCounter2a = RPMCounter2;
//    RPM2 = (RPMCounter2a / 16) * 30;
//    RPMavg2 = (RPM2 + RPM2) / 2;
//    TimeRunner2 = millis();
//
//}

}

//************************************************************************************************************

void ISR_count1()
{
  MotorRightCounter++;
  MotorRightCounter1++;
//LeftRightSpeedSync();
RightTurnCNT++;
//RPMCounter2++;

}

void ISR_count2()
{
  MotorLeftCounter++;
  MotorLeftCounter1++;
//LeftRightSpeedSync();
LeftTurnCNT++;
//RPMCounter1++;

}

void ISR_timerone()
{

  
  Timer1.detachInterrupt();  //Stop the TIMER
//  Serial.print("Motor Speed Right: ");
  RightRpm = (MotorRightCounter / diskslots) * 600;   // 60.00
//  Serial.print(RightRpm);
//  Serial.print(" RPM - ");
  MotorRightCounter = 0;  //Reset counter to Zero
//  Serial.print("Motor Speed Left: ");
  LeftRpm = (MotorLeftCounter / diskslots) * 600.00;   //60.00
//  Serial.print(LeftRpm);
//  Serial.print(" RPM - ");
  MotorLeftCounter = 0;  //Reset counter to Zero

//SpeedSyncAdjustLeft = 0;
//SpeedSyncAdjustRight = 0;


////  Serial.print("LeftSelState : ");
////  Serial.print(LeftSelState);
////  Serial.println("");
////
////  Serial.print("RightSelState : ");
////  Serial.print(RightSelState);
////  Serial.println("");
////
////  Serial.print("MotorTestPinState : ");
////  Serial.print(MotorTestPinState);
////  Serial.println("");
////
////  Serial.print("SpeedValue : ");
////  Serial.print(SpeedValue);
////  Serial.println("");
////
////
////  Serial.print("  MotorLeftCounter1 : ");
////  Serial.print(MotorLeftCounter1);
////  Serial.println("");
////
////  Serial.print("  MotorRightCounter1 : ");
////  Serial.print(MotorRightCounter1);
////  Serial.println("");
////
////
////  Serial.print("  LeftTurnCNT : ");
////  Serial.print(LeftTurnCNT);
////  Serial.println("");
////
////  Serial.print("  RightTurnCNT : ");
////  Serial.print(RightTurnCNT);
////  Serial.println("");
////
////
//  Serial.print("SpeedSyncAdjust : ");
//  Serial.print(SpeedSyncAdjust);
//  Serial.println("");
//
//  Serial.print("SpeedSyncAdjustLeft : ");
//  Serial.print(SpeedSyncAdjustLeft);
//  Serial.println("");
//
//
//  Serial.print("SpeedSyncAdjustRight : ");
//  Serial.print(SpeedSyncAdjustRight);
//  Serial.println("");

//  Serial.print("FrontDistReading : ");
//  Serial.print(FrontDistReading);
//  Serial.println("");

  Timer1.attachInterrupt(ISR_timerone);  //Restart Stopped TIMER


}

//************************************************************************************************************

//***********************************************************************************************************
void LeftRightSpeedSync(){
 MotorTest();

 SpeedSyncAdjust = 0;
PreviousSpeedSyncAdjust = SpeedSyncAdjust;

if (SpeedValue >= 2){
  
while (LeftRpm != RightRpm){
 MotorTest();
ISR_timerone();
 isr1 ();
 isr2 ();
 
if (LeftRpm <= RightRpm){
SpeedSyncAdjustLeft = SpeedSyncAdjustLeft + 0.05;
}

if (LeftRpm >= RightRpm){
SpeedSyncAdjustLeft = SpeedSyncAdjustLeft - 0.05;
}


//if (RPM1 <= RPM2){
//SpeedSyncAdjustLeft = SpeedSyncAdjustLeft + 2;
//}
//
//
//if (RPM1 >= RPM2){
//SpeedSyncAdjustLeft = SpeedSyncAdjustLeft - 2;
//}

//if (RightRpm <= LeftRpm){
//SpeedSyncAdjustRight = SpeedSyncAdjustRight + 0.5;  
//}
//
//
//if (RightRpm >= LeftRpm){
//SpeedSyncAdjustRight = SpeedSyncAdjustRight - 0.65;  
//}

}

if (LeftRpm == RightRpm){
AdjustedLeftRpm = SpeedSyncAdjustLeft;
}
}

else if (SpeedValue <=1 ){

  SpeedSyncAdjustLeft = 0;
SpeedSyncAdjustRight = 0;
}
}
 
//*************************************************************************************************************

void MotorTest(){

    SpeedValue = analogRead(SpeedPotPin);
  SpeedValue = map(SpeedValue, 0, 1023, 0, 255);


//  speedForwardLeftAI = SpeedValue * 1.5;
//  speedForwardRightAI = SpeedValue;

  MotorTestPinState = digitalRead(MotorTestPin);

    if (SpeedValue >= 2){
    if ((LeftSelState == LOW) && (RightSelState == HIGH)) {

      analogWrite (LeftSpeedPin, SpeedValue * AdjustedLeftRpm);

      analogWrite(LeftForRevPin, 255);
      analogWrite(LeftMotorPowerOnPin, 255);

      analogWrite(RightForRevPin, 255);
      analogWrite(RightMotorPowerOnPin, 255);

      analogWrite (RightSpeedPin, SpeedValue + AdjustedRightRpm);
    }




    if ((RightSelState == LOW) && (LeftSelState == HIGH)) {

      analogWrite (LeftSpeedPin, SpeedValue * SpeedSyncAdjustLeft);

      analogWrite(LeftForRevPin, 0);
      analogWrite(LeftMotorPowerOnPin, 255);
      
      analogWrite(RightForRevPin, 0);
      analogWrite(RightMotorPowerOnPin, 255);

      analogWrite (RightSpeedPin, SpeedValue + SpeedSyncAdjustRight);
    }
    }


    if (SpeedValue <= 1){

SpeedSyncAdjustRight = 0;
SpeedSyncAdjustLeft = 0;
      analogWrite (LeftSpeedPin, 0);
      analogWrite (RightSpeedPin, 0);


    
    }
}


  ////***********************************************************************************************************
  //           SERIAL PRINT DELAY TIMER

void SerialPrintTimer(){
SerialPrint();
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
}
  ////***********************************************************************************************************


//***********************************************************************************************************

void setup()
{

  lcd.init();
  lcd.backlight();
  lcd1.init();
  lcd1.backlight();

lcd1.createChar(0, batteryFull);
lcd1.createChar(1, battery3Quater);
lcd1.createChar(2, batteryHalf);
lcd1.createChar(3, batteryQuat);
lcd1.createChar(4, batteryEmpty);
//lcd.createChar(5, batteryFull);

  Serial.begin(115200);


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //  ////                 GYRO MPU-6050
  //
  //  mpu6050.begin();
  //  mpu6050.calcGyroOffsets(true);

  //**************************************************************

  //  RemoteXY_Init ();

  //  pinMode (PIN_START_1, OUTPUT);


  pinMode (LeftForRevPin, OUTPUT);
  pinMode (LeftSpeedPin, OUTPUT);
  pinMode (LeftMotorPowerOnPin, OUTPUT);
  
  pinMode (RightForRevPin, OUTPUT);
  pinMode (RightSpeedPin, OUTPUT);
  pinMode (RightMotorPowerOnPin, OUTPUT);

  pinMode (whtledPin, OUTPUT);
  pinMode (grnledPin, OUTPUT);
  pinMode (bluledPin, OUTPUT);
  pinMode (redledPin, OUTPUT);

  pinMode (ledPin, OUTPUT);

  pinMode (buzzerPin, OUTPUT);
  pinMode (batbuzzerPin, OUTPUT);

  pinMode (runPin, INPUT_PULLUP);
  pinMode (run2Pin, INPUT_PULLUP);

  pinMode (LeftSelPin, INPUT_PULLUP);
  pinMode (RightSelPin, INPUT_PULLUP);
  pinMode (MotorTestPin, INPUT_PULLUP);

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                   ENCODERS

//  pinMode (PinCLK1, INPUT);
//  attachInterrupt (4, isr1, RISING);
//
//  pinMode (PinCLK2, INPUT);
//  attachInterrupt (5, isr2, RISING);

    Timer1.initialize(100000);  //Set Timer for 1 Second - 1000000
    attachInterrupt(digitalPinToInterrupt(MotorRightCountPin), ISR_count1, RISING),attachInterrupt(digitalPinToInterrupt (5), isr2, RISING);  // Increase counter 1 when sensor pin goes High 
    attachInterrupt(digitalPinToInterrupt(MotorLeftCountPin), ISR_count2, RISING),attachInterrupt(digitalPinToInterrupt (4), isr1, RISING);      // Increase counter 2 when sensor pin goes High 


    Timer1.attachInterrupt(ISR_timerone);   //  Enable the timer

  ////***********************************************************************************************************
  ////***********************************************************************************************************

  myservoScanBack.attach(12);  // create servo object to control a servo
  myservoScanFront.attach(11);  // create servo object to control a servo

  FrontSensor.attach(A0, A1); //Trigger pin , Echo pin Front SENSOR
  BackSensor.attach(A4, A5); //Trigger pin , Echo pin Rear SENSOR

  ServoMoveStartTime8 = millis();
  ServoMoveStartTime7 = millis();
  ServoMoveStartTime6 = millis();
  ServoMoveStartTime5 = millis();
  ServoMoveStartTime4 = millis();
  ServoMoveStartTime3 = millis();
  ServoMoveStartTime2 = millis();
  ServoMoveStartTime1 = millis();
  ServoMoveStartTime = millis();
  StandbyStartTime = millis();
  //  rampStartTime = millis();



BatBuzzStartTime = millis();
BatBlinkStartTime = millis();

//  TimeRunner1 = millis();
//  TimeRunner2 = millis();

lcd.clear();
}

////***********************************************************************************************************
////***********************************************************************************************************



void loop()
{
SerialPrintTimer();
//SerialPrint();
  
LeftRightSpeedSync();

LeftRpmTimer();
LeftRpmCounter();

RightRpmTimer();
RightRpmCounter();

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //           SERIAL PRINT DELAY TIMER


  ledPinState = digitalRead(ledPin);

  Progress2 = millis() - ServoMoveStartTime2;     // Servo Head Progress

  if (Progress2 <= ServoMoveTime2) {
//    digitalWrite(ledPin, HIGH);
    ledPinState = HIGH;
  }

  if (Progress2 >= ServoMoveTime2) {
//    digitalWrite(ledPin, LOW);
    ledPinState = LOW;
  }

  if (Progress2 >= ResetTime2) {
    ServoMoveStartTime2 = millis();
  }

  ////***********************************************************************************************************

lcd1.setCursor(8, 0);
lcd1.print("L: ");
lcd1.print(LeftRpm);

lcd1.setCursor(0, 0);
lcd1.print("R: ");
lcd1.print(RightRpm);



  ////***********************************************************************************************************
//              Battery Buzzer Timer


BatBuzzProgress = millis() - BatBuzzStartTime;     // Servo Head Progress

  if (BatBuzzProgress <= BatBuzzMoveTime) {
    digitalWrite(ledPin, HIGH);
    BatBuzzState = HIGH;
  }

  if (BatBuzzProgress >= BatBuzzMoveTime) {
    digitalWrite(ledPin, LOW);
    BatBuzzState = LOW;
  }

  if (BatBuzzProgress >= BatBuzzResetTime) {
    BatBuzzStartTime = millis();
  }
  
  
  
  ////***********************************************************************************************************
//              Battery Voltage Monitor

  BatteryVoltage = analogRead(BatteryVoltagePin);
  BatteryVoltage = map(BatteryVoltage, 0, 1023, 0, 14.00);

if (BatteryVoltage >= 12.50){

batteryFull1();
            
}else

if (BatteryVoltage >= 12.00){

battery3Quater1();

}else

if (BatteryVoltage >= 11.50){

batteryHalf1();

}else

if (BatteryVoltage >= 11.00){

batteryQuat1();
  
}else


if (BatteryVoltage <= 10.00){
  
batteryEmpty1();

  if (BatBuzzState == HIGH){
          
          digitalWrite(batbuzzerPin, HIGH);
  }
  
  else if (BatBuzzState == LOW){
  
  
  digitalWrite(batbuzzerPin, LOW);
          
  }
}


  ////***********************************************************************************************************
  //                   RUN STATE


  whtledState = digitalRead(whtledPin);
  redledState = digitalRead(redledPin);
  grnledState = digitalRead(grnledPin);
  bluledState = digitalRead(bluledPin);

  RunState = digitalRead(runPin);
  Run2State = digitalRead(run2Pin);

  LeftSelState = digitalRead(LeftSelPin);
  RightSelState = digitalRead(RightSelPin);

  //**************************************************************************************************************
  //                    TESTING OF MOTORS


  if (MotorTestPinState == LOW) {

MotorTest();

//  SpeedValue = analogRead(SpeedPotPin);
//  SpeedValue = map(SpeedValue, 0, 1023, 0, 255);
//
//
////  speedForwardLeftAI = SpeedValue * 1.5;
////  speedForwardRightAI = SpeedValue;
//
//  MotorTestPinState = digitalRead(MotorTestPin);
//
//    if (SpeedValue >= 20){
//    if ((LeftSelState == LOW) && (RightSelState == HIGH)) {
//
//      analogWrite (LeftSpeedPin, SpeedValue + SpeedSyncAdjustLeft);
//
//      analogWrite(LeftForRevPin, 255);
//      analogWrite(LeftMotorPowerOnPin, 255);
//
//      analogWrite(RightForRevPin, 255);
//      analogWrite(RightMotorPowerOnPin, 255);
//
//      analogWrite (RightSpeedPin, SpeedValue + SpeedSyncAdjustRight);
//    }
//
//
//
//
//    if ((RightSelState == LOW) && (LeftSelState == HIGH)) {
//
//      analogWrite (LeftSpeedPin, SpeedValue + SpeedSyncAdjustLeft);
//
//      analogWrite(LeftForRevPin, 0);
//      analogWrite(LeftMotorPowerOnPin, 255);
//      
//      analogWrite(RightForRevPin, 0);
//      analogWrite(RightMotorPowerOnPin, 255);
//
//      analogWrite (RightSpeedPin, SpeedValue + SpeedSyncAdjustRight);
//    }
//    }
//
//
//    if (SpeedValue <= 10){
//
//
//
//      analogWrite (LeftSpeedPin, SpeedValue + SpeedSyncAdjustRight);
//      analogWrite (RightSpeedPin, SpeedValue + SpeedSyncAdjustRight);
//
//
//    
//    }
  
  }




  //************************************************************************************************************
  //************************************************************************************************************
  //                SCANNER                SCANNER                SCANNER                SCANNER
  //************************************************************************************************************

  //************************** DISTANCE MEASURING IN MM ********************************************************

  FrontDistReading = (FrontSensor.distanceInCm() * 10);  // FRONT SCANNER
  FrontDistReadingL = 0;  // BOTTOM MIDDEL SCANNER
  FrontDistReadingR = 0;  // BOTTOM MIDDEL SCANNER

  RearDistReading = (BackSensor.distanceInCm() * 10);  // REAR SENSOR
  RearDistReadingL = 0;  // BOTTOM MIDDEL SCANNER
  RearDistReadingR = 0;  // BOTTOM MIDDEL SCANNER

  RearDistReadingTurnL = 0;  // BOTTOM MIDDEL SCANNER
  RearDistReadingTurnR = 0;  // BOTTOM MIDDEL SCANNER

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                     DISTANCE DETECTION CONTROL

  //      FRONT SENSOR

  //  if ((FrontDistReading >= 0) || (RearDistReading >= 0) || (Distreading6 >= 0)){
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
    RPM1 = RPM1 + 1;
//   LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();

  }


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                 ENCODER 2*

  // Runs if rotation was detected

  if (TurnDetected2) {
    PrevPosition2 = WheelPosition2;  // Save previous position in variable
    WheelPosition2 = WheelPosition2 + 1;  // Increase Position by 1
    RPMCounter2 = RPMCounter2 + 1;
    RPM2 = RPM2 + 1;
//    RightTurnCNT = RightTurnCNT + 1;
    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected

    StandbyStartTime = millis();
  }



  ////***********************************************************************************************************
  if ((MotorTestPinState == HIGH) && (BatteryVoltage >= 7)) {
  TimeRunner1 = millis();
  TimeRunner2 = millis();
    ////***********************************************************************************************************
    //                        LEFT / RIGHT REAR FORWARD CHECK TO TURN

    //if (BigLeftDist >= BigRightDist)

    //BigLeftDist
    //BigRightDist
    ////***********************************************************************************************************
    //***

    //***


    //  Serial.println("");
    //  Serial.print("Step1On :");
    //  Serial.print(Step1On);
    //  Serial.print(" ");
    //
    //
    //  Serial.println("");
    //  Serial.print("Step2On :");
    //  Serial.print(Step2On);
    //  Serial.print(" ");
    //
    //
    //  Serial.println("");
    //  Serial.print("Step3On :");
    //  Serial.print(Step3On);
    //  Serial.print(" ");
    //
    //
    //  Serial.println("");
    //  Serial.print("Step4On :");
    //  Serial.print(Step4On);
    //  Serial.print(" ");
    //
    //
    //  Serial.println("");

    //        Serial.println("");
    //      Serial.print("DistRightCheckCNT :");
    //      Serial.print(DistRightCheckCNT);
    //      Serial.println("");

    ////***********************************************************************************************************
    //                                   RUNNING AI CONTROL
    //  LeftSelState == HIGH;

    //***
    if (AngleM1 >= 5) {
      digitalWrite(grnledPin, LOW);
    }

    if (AngleM1 <= 170) {
      digitalWrite(bluledPin, LOW);
    }

    if (((BigLeftDist <= BigRightDist)) || (LeftSelState == LOW)) { // && (ScanRearRunning = LOW)) {
      LeftDifference =  0;
      digitalWrite(grnledPin, LOW);
      RightDifference = 0;
      BackScanLeftTurn();
    }

    if (((BigRightDist <= BigLeftDist)) || (RightSelState == LOW)) { // && (ScanRearRunning = LOW)){
      LeftDifference =  0;
      digitalWrite(bluledPin, LOW);
      RightDifference = 0;
      BackScanRightTurn();
    }
    //***
    //  BackScanLeftTurn();
    //  BackScanRightTurn();

    //******************************************************************************************************

    //  if ((RunState == HIGH)) {
    //     MoveStopAI();
    //  }

    ScanForward();
    LRcheckForward();
    ScanPulseTimerL();
    //  ScanPulseTimerR();
    //
    //  MoveLeftU180CNTAI();
    //  MoveLeft180CNTAI();
    //  MoveRightU180CNTAI();
    //  MoveRight180CNTAI();
    //
    if ((RunState == LOW)) {

      //ScanRear();

      //  Serial.println("");
      //  Serial.print("RunState :");
      //  Serial.print(RunState);
      //  Serial.print(" ");

      //*****************************************************************************************************

      //MoveForwardEnterAI();
      //MoveForwardEnterTurnAI();

      // if (RearDistReadingTurnL <= RearDistReadingTurnR){
      //  ScanForward();
      //MoveStopAI();
      //MoveLeft180AI();
      //MoveStopAI();
      //MoveForwardAI();
      //}
      //
      //
      //if (RearDistReadingTurnL >= RearDistReadingTurnR){
      //  ScanForward();
      //
      //MoveStopAI();
      //MoveRight180AI();
      //MoveStopAI();
      //MoveForwardAI();
      //}



      //***********************************************************************************************************
      //                                     CAR CONTROL
      LRcheckForward();

      //***********************************************************************************************************
      //                                     CAR FORWARD CONTROL

      if ( Entry ==  LOW) {
        if ((FrontDistReading >= FarSetPoint4) && ( Entry ==  LOW)) {
          MoveForwardAI();
          LeftUTurn180CNTOn = LOW;
          RightUTurn180CNTOn = LOW;
        }
        //              }
        else if ((FrontDistReading <= CloseSetPoint4) || ( Entry ==  HIGH)) {
          MoveStopAI();
        }


        //          }
        ////***********************************************************************************************************
        //                          REAR CHECK LEFT / RIGHT ENTRY CONTROL


        //FrontScanLeft180()
        //RearDistReadingTurnL
        //FrontScanRight180()
        //RearDistReadingTurnR

        if (grnledState == 1) {
          Entry = HIGH;
          MoveStopAI();
          TurnMoveLeft90AI();
          MoveStopAI();
          MoveForwardEnterAI();
          MoveStopAI();
          FrontScanLeft180();
          FrontScanRight180();
          if (RearDistReadingTurnL <= RearDistReadingTurnR) {
            TurnMoveLeft90AI();
          } else if (RearDistReadingTurnL >= RearDistReadingTurnR) {
            TurnMoveRight90AI();
          }
          MoveStopAI();
          MoveForwardEnterTurnAI();
          MoveStopAI();
          FrontScanLeft180();
          FrontScanRight180();
          if (RearDistReadingTurnL <= RearDistReadingTurnR) {
            MoveRight180AI();
          } else if (RearDistReadingTurnL >= RearDistReadingTurnR) {
            MoveLeft180AI();
          }


        }
        else {
          Entry = LOW;
          grnledState = 0;
          //          MoveForwardAI();
        }


        if (bluledState == 1) {
          Entry = HIGH;
          MoveStopAI();
          TurnMoveRight90AI();
          MoveStopAI();
          MoveForwardEnterAI();
          MoveStopAI();

          FrontScanLeft180();
          FrontScanRight180();
          if (RearDistReadingTurnL <= RearDistReadingTurnR) {
            TurnMoveLeft90AI();
          } else if (RearDistReadingTurnL >= RearDistReadingTurnR) {
            TurnMoveRight90AI();
          }
          MoveStopAI();
          MoveForwardEnterTurnAI();
          MoveStopAI();
          FrontScanLeft180();
          FrontScanRight180();
          if (RearDistReadingTurnL <= RearDistReadingTurnR) {
            MoveRight180AI();
          } else if (RearDistReadingTurnL >= RearDistReadingTurnR) {
            MoveLeft180AI();
          }


        }
        else {
          Entry = LOW;
          bluledState = 0;
          //          MoveForwardAI();

        }


        ////***********************************************************************************************************
        //                          FRONT CHECK LEFT / RIGHT 180 U - TURN

        MoveLeftU180CNTAI();
        MoveRightU180CNTAI();

        if (((FrontDistReading <= FarSetPoint4) && (FrontDistReading >= CloseSetPoint4))
            && (AngleM <= 100) && (AngleM >= 60)) {


          //      LRcheckForward();
          MoveStopAI();

          //if ((RightUTurn180CNT <= 2) && (LeftUTurn180CNT <= 2)){

          //      FrontScanLeft180();
          //      FrontScanRight180();



          //}

          ScanForward();

          lcd.setCursor(15, 2);
          lcd.print(RearDistReadingTurnL);
          lcd.print(" ");

          lcd.setCursor(7, 2);
          lcd.print(RearDistReadingTurnR);
          lcd.print(" ");

          //            if ((RearDistReadingTurnL <= RearDistReadingTurnR) && (LeftUTurn180CNT <= 0)) {
          //      if ((FrontDistReading <= FarSetPoint4) && ((LeftUTurn180CNT == 1) && (RightUTurn180CNT == 0))) {
          //      if ((FrontDistReading <= FarSetPoint4) && ((LeftUTurn180CNT == 1))) {
          if (FrontDistReading <= FarSetPoint4) {
            if ((LeftUTurn180CNT == 1) && (RightUTurn180CNT == 0)) {

              MoveRightU180AI();
              MoveStopAI();
            }
          }

          //      if ((RearDistReadingTurnL >= RearDistReadingTurnR) && (RightUTurn180CNT <= 0)) {
          //        if ((FrontDistReading >= FarSetPoint4) && ((LeftUTurn180CNT == 0) && (RightUTurn180CNT == 1))) {
          //        if ((FrontDistReading >= FarSetPoint4) && ((LeftUTurn180CNT == 0))) {
          if (FrontDistReading >= FarSetPoint4) {
            if ((LeftUTurn180CNT == 0) && (RightUTurn180CNT == 1)) {

              MoveLeftU180AI();
              MoveStopAI();

            }
          }


        }
        else {

          //        LeftUTurn180CNTOn = LOW;
          //        RightUTurn180CNTOn = LOW;

          ScanForward();
          //        MoveForwardAI();

        }



        //  Serial.println("");
        //  Serial.print("AngleM :");
        //  Serial.print(AngleM);
        //  Serial.print(" ");
//
//
//        Serial.println("");
//        Serial.print("LeftUTurn180On :");
//        Serial.print(LeftUTurn180On);
//        Serial.println("");
//
//        Serial.println("");
//        Serial.print("LeftUTurn180CNT :");
//        Serial.print(LeftUTurn180CNT);
//        Serial.println("");
//
//        Serial.println("");
//        Serial.print("RightUTurn180On :");
//        Serial.print(RightUTurn180On);
//        Serial.println("");
//
//        Serial.println("");
//        Serial.print("RightUTurn180CNT :");
//        Serial.print(RightUTurn180CNT);
//        Serial.println("");

        ////***********************************************************************************************************
        //                         FRONT LEFT / RIGHT 90 DEGREE TURN CHECK


        //        if (((FrontDistReading <= FarSetPoint4) && (FrontDistReading >= CloseSetPoint4))
        //            && (AngleM <= 100) && (AngleM >= 60)) {
        //          LRcheckForward();
        //          MoveStopAI();
        //
        //          AngleBotLeft = AngleM;
        //          AngleBotRight = AngleM;
        //
        //          myservoScanFront.write(160);
        //
        //          delay(500);
        //
        //          FrontDistReadingL = (FrontSensor.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
        //
        //          lcd.setCursor(11, 1);
        //          lcd.print(FrontDistReadingL  );
        //          lcd.print(" ");
        //
        //          myservoScanFront.write(20);
        //
        //          delay(1000);
        //
        //          FrontDistReadingR = (FrontSensor.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
        //
        //          lcd.setCursor(3, 1);
        //          lcd.print(FrontDistReadingR  );
        //          lcd.print(" ");
        //
        //          myservoScanFront.write(90);
        //          delay(500);
        //
        //          if ((FrontDistReadingL >= FrontDistReadingR) || (grnledState == HIGH)) {
        //
        //            BackScanRight();
        //
        //                    MoveLeft90AI();
        //          }
        //
        //          if ((FrontDistReadingR >= FrontDistReadingL) || (bluledState == HIGH)) {
        //
        //            BackScanLeft();
        //
        //                    MoveRight90AI();
        //
        //          }
        //        }

        //    ////***********************************************************************************************************
        //    ////***********************************************************************************************************
        //    //                          45 DEGREE MANUVER LEFT
        //
        //    if (((FrontDistReading <= FarSetPoint4L) && (FrontDistReading >= CloseSetPoint4L))
        //        && (AngleM <= 160) && (AngleM >= 100)) {
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
        //BackScanLeft();
        // if (RearDistReadingL <= 150){
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
        //    if (((FrontDistReading <= FarSetPoint4R) && (FrontDistReading >= CloseSetPoint4R))
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
        //BackScanRight();
        //
        // if (RearDistReadingL <= 150){
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

        //                      MoveForwardAI();
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
    }
    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                LCD2 OUTPUT CONTROL

    lcd.setCursor(0, 1);
    lcd.print("F: ");
    lcd.print(  FrontDistReading);
    lcd.print("  ");

    lcd.setCursor(8, 1);
    lcd.print("B: ");
    lcd.print(  RearDistReading );
    lcd.print("   ");

    //  lcd.setCursor(0, 0);
    //  lcd.print("R:");
    //  lcd.print(RearDistReadingR);
    //  lcd.print(" ");
    //
    //  lcd.setCursor(8, 0);
    //  lcd.print("L:");
    //  lcd.print(RearDistReadingL);
    //  lcd.print(" ");

    //  lcd.setCursor(8, 1);
    //  lcd.print("L: ");
    //  lcd.print(RearDistReading );
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
//    Serial.println("");
//    Serial.print("RunState :");
//    Serial.print(RunState);
//    Serial.println("");
//
//    Serial.println("");
//    Serial.print("Run2State :");
//    Serial.print(Run2State);
//    Serial.println("");
//
//    //  Serial.println("");
//    //  Serial.print("Progress2 :");
//    //  Serial.print(Progress2);
//    //  Serial.println("");
//
//
//      Serial.println("");
//      Serial.print("BatBuzzProgress :");
//      Serial.print(BatBuzzProgress);
//      Serial.println("");
//    


    //      Serial.println("");
    //      Serial.print("Distreading1 :");
    //      Serial.print(Distreading1);
    //      Serial.print(" MM");
    //      Serial.println("");
    //
    //      Serial.println("");
//
//    Serial.println("");
//    Serial.print("FrontDistReading :");
//    Serial.print(FrontDistReading);
//    Serial.print(" MM");
//    Serial.println("");
//
//    Serial.println("");
//    Serial.print("RearDistReading :");
//    Serial.print(RearDistReading);
//    Serial.print(" MM");
//    Serial.println("");
//    //
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
//
//    Serial.print("WheelPosition1: ");
//    Serial.println(WheelPosition1);
//
//    Serial.print("WheelPosition2: ");
//    Serial.println(WheelPosition2);

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
      //      //    //************************************************************************************************************
      //      ////                           TEST RAMP UP MOTOR CONTROL
      //
      //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
      //
      //      if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
      //        speedRampForwardAI = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);
      //
      //        //      MoveForwardRampTestAI();
      //
      //
      //        analogWrite (RightSpeedPin, speedRampForwardAI);
      //        analogWrite (RightSpeedPin, speedRampForwardAI);
      //
      //        digitalWrite(LeftForRevPin, HIGH);
      //        digitalWrite(RightMotorPowerOnPin, LOW);
      //        digitalWrite(RightForRevPin, HIGH);
      //        digitalWrite(RightMotorPowerOnPin, LOW);
      //
      //      }
      //
      //      //************************************************************************************************************
      //      //                           TEST RAMP DOWN MOTOR CONTROL
      //
      //
      //      //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
      //
      //      if ((rampProgres >= rampUpEndTime) && (rampProgres <= rampDnEndTime)) {
      //        speedRampForwardAI = map(rampProgres, rampUpEndTime, rampDnEndTime, stopSpeedForward, startSpeedForward);
      //
      //        //      MoveForwardRampTestAI();
      //
      //        analogWrite (RightSpeedPin, speedRampForwardAI);
      //        analogWrite (RightSpeedPin, speedRampForwardAI);
      //
      //        digitalWrite(LeftForRevPin, HIGH);
      //        digitalWrite(RightMotorPowerOnPin, LOW);
      //        digitalWrite(RightForRevPin, HIGH);
      //        digitalWrite(RightMotorPowerOnPin, LOW);
      //      }
      //
      //      if ((rampProgres >= rampDnEndTime)) {
      //
      //        LeftTurnCNT = 0;
      //        RightTurnCNT = 0;
      //
      //        rampStartTime = millis();
      //      }
      //
      //    }


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
      //  //  digitalWrite(RightMotorPowerOnPin, LOW);
      //  //  digitalWrite(lefttPin1B, LOW);
      //
      //  //  digitalWrite(RightForRevPin, LOW);
      //  //  digitalWrite(LeftForRevPin, LOW);
      //  }


      //if ((TurnDetected1 = true) && (TurnDetected2 == true)) {
      //
      //    StandbyActive = LOW;
      //
      //    digitalWrite(RightMotorPowerOnPin, LOW);
      //    digitalWrite(RightMotorPowerOnPin, LOW);
      //
      //    digitalWrite(RightForRevPin, LOW);
      //    digitalWrite(LeftForRevPin, LOW);
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

      //***********************************************************************************************************
      ////***********************************************************************************************************
      //                                         SCANNER SERVO CONTROL

      AngleM = Angle;

      Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

      if (Progress <= ServoMoveTime) {
        AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
        myservoScanFront.write(AngleM);
        myservoScanBack.write(AngleM);

      }

      if (Progress >= ServoMoveTime) {
        AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
        myservoScanFront.write(AngleM);
        myservoScanBack.write(AngleM);

      }

      if (Progress >= ResetTime) {
        ServoMoveStartTime = millis();

      }

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
      //  analogWrite (RightMotorPowerOnPin, speedRampForwardAI);
      //  analogWrite (RightMotorPowerOnPin, speedRampForwardAI);
      //
      //  digitalWrite(RightForRevPin, HIGH);
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



  }
}
