

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
LiquidCrystal_I2C lcd1(0x23, 20, 4);

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
int PercentFirstLeftState = 0;
int SecondLeftState = 0;
int PercentSecondLeftState = 0;

int FirstRightState = 0;
int PercentFirstRightState = 0;
int SecondRightState = 0;
int PercentSecondRightState = 0;

int LeftDifference = 0;
int RightDifference = 0;

int LeftDifferenceOn = LOW;
int RightDifferenceOn = LOW;

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
#define speedForwardLeftAI 120
#define speedForwardRightAI 80

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
//       REAR LEFT / RIGHT SCAN TIMER CONTROL

unsigned long ServoMoveTime3 = 3000;
unsigned long ServoMoveStartTime3;
unsigned long Progress3;
unsigned long ResetTime3 = 15000;

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
unsigned long ResetTime4 = 1000;

int PulseOnL = 0;

int StartAngleM4 = 10;
int StopAngleM4 =  165;

int StartAngle4 = 5;
int StopAngle4 =  175;

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
unsigned long ResetTime5 = 1000;

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
#define grnledPin 22
#define bluledPin 23
#define redledPin 24
#define whtledPin 25

#define LeftSelPin A14
int LeftSelState = 0;

#define RightSelPin A15
int RightSelState = 0;

int whtledState = 0;
int redledState = 0;
int grnledState = 0;
int bluledState = 0;

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
    //      Serial.println("");
    //      Serial.print("Progress5 :");
    //      Serial.print(Progress5);
    //      Serial.println("");

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

  if (Distreading4 >= FarSetPoint4)  {
    analogWrite (leftPinEN, speedForwardLeftAI);
    analogWrite (rightPinEN, speedForwardRightAI);
  }

  else if (Distreading4 <= FarSetPoint4) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
    analogWrite (rightPinEN, 0);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE STOP CONTROL AI

void MoveStopAI() {

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  LeftTurnCNT = 0;
  RightTurnCNT = 0;

  delay(1000);

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

  if (Distreading4L == Distreading4R) {
    analogWrite (leftPinEN, speedLeftLeftAI);
    analogWrite (rightPinEN, speedLeftRightAI);
  }

  else if (Distreading4L != Distreading4R) {                            //if (LeftTurnCNT >= LeftTurn90){
    analogWrite (leftPinEN, 0);
    analogWrite (rightPinEN, 0);
  }

  delay(ManuverDelay);

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

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
    //LRcheckForward();
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

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  while ((LeftTurnCNT <= LeftTurn90) || (RightTurnCNT <= LeftTurn90)) {

    digitalWrite(leftPin1A, LOW);
    digitalWrite(leftPin1B, HIGH);
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

    if (LeftTurnCNT <= LeftTurn90) {
      analogWrite (leftPinEN, speedLeftLeftAI);
    }

    if (RightTurnCNT <= LeftTurn90) {
      analogWrite (rightPinEN, speedLeftRightAI);
    }

  }

  while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)) {

    ScanPulseTimerL();
    ScanPulseTimerR();
    ScanRearTimer();

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

    if (LeftTurnCNT <= LeftCntFor) {
      analogWrite (leftPinEN, speedForwardLeftAI);
    }

    else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (leftPinEN, 0);
      digitalWrite(grnledPin, LOW);
    }

    if (RightTurnCNT <= RightCntFor) {
      analogWrite (rightPinEN, speedForwardRightAI);
    }

    else if (RightTurnCNT >= RightCntFor) {                           // if (RightTurnCNT >= RightTurn90){
      analogWrite (rightPinEN, 0);
      digitalWrite(bluledPin, LOW);
    }

    LeftDifference =  0;
    RightDifference = 0;

  }

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
    //LRcheckForward();
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

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

  while ((LeftTurnCNT <= RightTurn90) || (RightTurnCNT <= RightTurn90)) {

    digitalWrite(leftPin1A, HIGH);
    digitalWrite(leftPin1B, LOW);
    digitalWrite(rightPin2A, LOW);
    digitalWrite(rightPin2B, HIGH);

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

    if (RightTurnCNT <= RightTurn90) {
      analogWrite (rightPinEN, speedRightRightAI);
    }

  }

  while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)) {

    ScanPulseTimerL();
    ScanPulseTimerR();
    ScanRearTimer();

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

    if (LeftTurnCNT <= LeftCntFor) {
      analogWrite (leftPinEN, speedForwardLeftAI);
    }

    else if (LeftTurnCNT >= LeftCntFor) {                            //if (LeftTurnCNT >= LeftTurn90){
      analogWrite (leftPinEN, 0);
      digitalWrite(grnledPin, LOW);
    }

    if (RightTurnCNT <= RightCntFor) {
      analogWrite (rightPinEN, speedForwardRightAI);
    }

    else if (RightTurnCNT >= RightCntFor) {
      analogWrite (rightPinEN, 0);
      digitalWrite(bluledPin, LOW);
    }


    LeftDifference =  0;
    RightDifference = 0;
  }


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

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE 500mm CONTROL AI


void Move500AI() {

  LeftCntFor = 46;
  RightCntFor = 46;

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

  //while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)){

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

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

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

  //while ((LeftTurnCNT <= LeftCntFor) || (RightTurnCNT <= RightCntFor)){

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

  analogWrite (leftPinEN, 0);
  analogWrite (rightPinEN, 0);

}
//}

//***********************************************************************************************************
//                               MOVE 200mm LEFT CONTROL AI

void Move200LeftAI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

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

  while (Distreading5L <= 150) {

    ScanLeft();

    if (Distreading5L <= 150) {

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

}

//***********************************************************************************************************
//                               MOVE 200mm RIGHT CONTROL AI


void Move200RightAI() {

  ScanPulseTimerL();
  ScanPulseTimerR();
  ScanRearTimer();

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

  while (Distreading5R <= 150) {

    ScanRight();

    if (Distreading5R <= 150) {

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


  //  delay(ManuverDelay);

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

  }

  if (Progress >= ServoMoveTime) {
    AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
    myservoScanFront.write(AngleM);

  }

  if (Progress >= ResetTime) {
    ServoMoveStartTime = millis();

  }

}

//***********************************************************************************************************

////***********************************************************************************************************
//                                         SCANNER SERVO CONTROL

void ScanForwardMan() {
  //
  //  AngleM = Angle;
  //
  //  Progress = millis() - ServoMoveStartTime;     // Servo Head Progress
  //
  //  if (Progress <= ServoMoveTime) {
  //    AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
  //    myservoScanFront.write(AngleM);
  //
  //  }
  //
  //  if (Progress >= ServoMoveTime) {
  //    AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
  //    myservoScanFront.write(AngleM);
  //
  //  }
  //
  //  if (Progress >= ResetTime) {
  //    ServoMoveStartTime = millis();
  //
  //  }
  //  //  delay(1000);
  //
  }

  //***********************************************************************************************************

  ////***********************************************************************************************************
  ////                                      SCAN REAR (LEFT / RIGHT)
  ////***********************************************************************************************************
  //                                         SCANNER REAR SERVO CONTROL

  void ScanRear() {

    ScanForward();
    Scan4StepControlTimer();
    LeftScan2StepControlTimer();
    RightScan2StepControlTimer();

    //if (RearScanOn == HIGH){
    //
    //ScanRearRunning = HIGH;
    //
    //  AngleM1 = Angle1;
    //
    //  Progress1 = millis() - ServoMoveStartTime1;     // Servo Head Progress
    //
    //  if (Progress1 <= ServoMoveTime1) {
    //    AngleM1 = map(Progress1, 0, ServoMoveTime1, StopAngleM1, StartAngleM1);
    //    myservoScanBack.write(AngleM1);
    //
    //  }
    //
    //  if (Progress1 >= ServoMoveTime1) {
    //    AngleM1 = map(Progress1, ServoMoveTime1, ResetTime1, StartAngleM1, StopAngleM1);
    //    myservoScanBack.write(AngleM1);
    //
    //  }
    //
    //  if (Progress1 >= ResetTime1) {
    //    ServoMoveStartTime1 = millis();
    ////ScanRearRunning = LOW;
    ////RearScanOn = LOW;
    //  }

    //      ScanLeft();
    //      ScanRight();

    //  StepScan1OnLeft = HIGH;


    //if (StepScan1OnLeft == HIGH){
    myservoScanBack.write(5);

    delay(1000);

    BigLeftDist  = (mysensor2.distanceInCm() * 10);

    lcd1.setCursor(8, 0);
    lcd1.print("BL:");
    lcd1.print(BigLeftDist);
    lcd1.print(" ");
    //}
    delay(1000);


    //if (StepScan1OnLeft == HIGH){

    myservoScanBack.write(170);

    delay(1000);

    BigRightDist  = (mysensor2.distanceInCm() * 10);

    delay(1000);

    lcd1.setCursor(0, 0);
    lcd1.print("BR:");
    lcd1.print(BigRightDist);
    lcd1.print(" ");
    //}
    // ScanRearRunning = LOW;


    //**

    if (((BigLeftDist <= BigRightDist))) {  // && (ScanRearRunning = LOW))) {

      CheckLeft();

    }


    if (((BigRightDist <= BigLeftDist))) {  // && (ScanRearRunning = LOW))){

      CheckRight();

    }

  }
  //***********************************************************************************************************
  //***********************************************************************************************************
  //                                      CHECK LEFT


  void CheckLeft() {

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
    myservoScanBack.write(5);
    //  }

    //    if (StepScan2OnLeft == HIGH) {

    delay(1000);
    ScanRearRunning = LOW;
    ScanLeftTurn();
    //    }
  }


  ////***********************************************************************************************************
  //***********************************************************************************************************
  //                                      CHECK RIGHT


  void CheckRight() {

    ScanForward();
    Scan4StepControlTimer();
    LeftScan2StepControlTimer();
    RightScan2StepControlTimer();
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
    ScanRightTurn();
    //  }
  }


  ////***********************************************************************************************************

  //***********************************************************************************************************
  //                                      SCAN LEFT


  void ScanLeft() {

    //ScanRearTimer();
    // ScanPulseTimerL();
    // LRcheckForward();

    myservoScanBack.write(5);

    delay(1000);

    Distreading5L  = (mysensor2.distanceInCm() * 10);
    //    BigLeftDist = Distreading5L;

    //    lcd1.setCursor(8, 0);
    //    lcd1.print("BL:");
    //    lcd1.print(Distreading5L);
    //    lcd1.print(" ");

    delay(1000);
  }


  ////***********************************************************************************************************
  ////***********************************************************************************************************
  ////                                      SCAN RIGHT


  void ScanRight() {

    //ScanRearTimer();
    // ScanPulseTimerL();
    // LRcheckForward();

    myservoScanBack.write(170);

    delay(1000);

    Distreading5R  = (mysensor2.distanceInCm() * 10);
    //    BigRightDist = Distreading5R;


    delay(1000);
  }

  ////***********************************************************************************************************

  ////***********************************************************************************************************
  //                                         REAR SCANNER SCAN TIMER CONTROL

  void ScanRearTimer() {

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
    }

    if (Progress4 >= ServoMoveTime4) {
      AngleM4 = map(Progress4, ServoMoveTime4, ResetTime4, StartAngleM4, StopAngleM4);
      PulseOnL = LOW;
      digitalWrite(redledPin, LOW);

    }

    if (Progress4 >= ResetTime4) {
      ServoMoveStartTime4 = millis();

    }

  }

  //***********************************************************************************************************
  ////***********************************************************************************************************
  //                                    RIGHT SCANNER SCAN PULSE TIMER CONTROL

  void ScanPulseTimerR() {

    AngleM5 = Angle5;

    Progress5 = millis() - ServoMoveStartTime5;     // Servo Head Progress

    if (Progress5 <= ServoMoveTime5) {
      AngleM5 = map(Progress5, 0, ServoMoveTime5, StopAngleM5, StartAngleM5);
      PulseOnR = HIGH;
      digitalWrite(whtledPin, HIGH);
    }

    if (Progress5 >= ServoMoveTime5) {
      AngleM5 = map(Progress5, ServoMoveTime5, ResetTime5, StartAngleM5, StopAngleM5);
      PulseOnR = LOW;
      digitalWrite(whtledPin, LOW);

    }

    if (Progress5 >= ResetTime5) {
      ServoMoveStartTime5 = millis();

    }

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

  //                        LEFT / RIGHT REAR FORWARD CHECK TO TURN


  void LRcheckForward() {

    //          SCAN LEFT / RIGHT PULSE GENERATOR
    //

    ScanRearTimer();
    ScanPulseTimerL();
    ScanPulseTimerR();


  }

  ////***********************************************************************************************************

  void ScanLeftTurn() {

    digitalWrite(grnledPin, LOW);

    ScanRearTimer();
    ScanPulseTimerL();
    ScanPulseTimerR();
    if (BigLeftDist <= BigRightDist) {

      if ((ScanRearRunning == LOW) && (RearScanOn == LOW)) {
        myservoScanBack.write(5);
        AngleM1 = 3;
        //delay(250);
      }

      if ((AngleM1 <= 10) && (AngleM1 >= 0)) {
        if (PulseOnL == HIGH)
        {

          LeftCntOn = HIGH;

          //        Serial.println("");
          //        Serial.print("DistLeftCheckCNT :");
          //        Serial.print(DistLeftCheckCNT);
          //        Serial.println("");

        } else if (PulseOnL == LOW) {
          LeftCntOn = LOW;
        }


        if (LeftCntOn != LastDistLeftCheckCNT) {
          if (LeftCntOn == HIGH) {
            DistLeftCheckCNT ++;
          } else {


          }
          delay(50);
        }

        LastDistLeftCheckCNT = LeftCntOn;


        if (DistLeftCheckCNT == 2)
        {
          DistLeftCheckCNT = 0;
        }


        if (((DistLeftCheckCNT == 0) && (PulseOnL == HIGH)) && ((AngleM1 <= 5) && (AngleM1 >= 0)))
        {

          LeftDifferenceOn = LOW;
          digitalWrite(grnledPin, LOW);
          grnledState = LOW;
          LeftDifference = 0;

          FirstLeftState  = (mysensor2.distanceInCm() * 10);
          PercentFirstLeftState = FirstLeftState;

          //          Serial.println("");
          //          Serial.print("FirstLeftState :");
          //          Serial.print(FirstLeftState);
          //          Serial.print(" MM");
          //          Serial.println("");

          lcd1.setCursor(8, 1);
          lcd1.print("0:");
          lcd1.print(FirstLeftState);
          lcd1.print(" ");
          lcd1.print(LeftDifferenceOn);
          lcd1.print(" ");

        }

        if (AngleM1 >= 5) {
          digitalWrite(grnledPin, LOW);
        }

        if (((DistLeftCheckCNT == 1) && (PulseOnL == HIGH)) && ((AngleM1 <= 5) && (AngleM1 >= 0)))
        {

          SecondLeftState  = (mysensor2.distanceInCm() * 10);
          PercentSecondLeftState = SecondLeftState;

          //          Serial.println("");
          //          Serial.print("SecondLeftState :");
          //          Serial.print(SecondLeftState);
          //          Serial.print(" MM");
          //          Serial.println("");

          lcd1.setCursor(8, 1);
          lcd1.print("1:");
          lcd1.print(SecondLeftState);
          lcd1.print(" ");
        }
      }



      if (grnledState == HIGH) {
        LeftDifference = 0;
      } else {
        LeftDifference = (PercentFirstLeftState - PercentSecondLeftState);
      }


      if ((LeftDifference >= 200) || (LeftDifference <= -200) && (grnledState == LOW)) {
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

        //    if ((LeftDifference <= 200) && (LeftDifference >= -200)) {
        //      //    LeftDifferenceOn = LOW;
        //      digitalWrite(grnledPin, LOW);
        //      //  grnledState = LOW;

        lcd1.setCursor(14, 1);
        lcd1.print(grnledState);
        lcd1.print(" ");

      }


      //  }
    }
    //if ((grnledState == HIGH) && (PulseOnL == HIGH)){
    //     digitalWrite(grnledPin, LOW);
    //}

  }
  //***********************************************************************************************************
  ////***********************************************************************************************************

  void ScanRightTurn() {

    digitalWrite(bluledPin, LOW);

    ScanPulseTimerR();
    ScanRearTimer();

    //    SCAN RIGHT

    if (BigRightDist <= BigLeftDist) {

      if ((ScanRearRunning == LOW)) { // && (RearScanOn == LOW)){
        myservoScanBack.write(170);
        AngleM1 = 175;
        //delay(250);
      }

      //        if (AngleM1 >= 160){

      if ((AngleM1 >= 170) && (AngleM1 <= 180)) {
        if (PulseOnR == HIGH)
        {
          RightCntOn = HIGH;

          //        Serial.println("");
          //        Serial.print("DistRightCheckCNT :");
          //        Serial.print(DistRightCheckCNT);
          //        Serial.println("");
        } else if (PulseOnR == LOW) {

          RightCntOn = LOW;
        }


        //        if (myservoScanBack >= 160) {
        // LeftDifference =  0;
        if (RightCntOn != LastDistRightCheckCNT) {
          if (RightCntOn == HIGH) {
            DistRightCheckCNT ++;
          } else {

          }
          delay(5);
        }
        //        }

        LastDistRightCheckCNT = RightCntOn;

        if (DistRightCheckCNT == 2)
        {
          DistRightCheckCNT = 0;
        }

        if (((DistRightCheckCNT == 0) && (PulseOnR == HIGH)) && ((AngleM1 >= 170) && (AngleM1 <= 180)))
        {

          RightDifferenceOn = LOW;
          digitalWrite(bluledPin, LOW);
          bluledState = LOW;
          RightDifference = 0;

          FirstRightState  = (mysensor2.distanceInCm() * 10);
          PercentFirstRightState = FirstRightState;

          //          Serial.println("");
          //          Serial.print("PercentFirstRightState :");
          //          Serial.print(PercentFirstRightState);
          //          Serial.print(" MM");
          //          Serial.println("");

          lcd1.setCursor(0, 1);
          lcd1.print("0:");
          lcd1.print(PercentFirstRightState);
          lcd1.print(" ");

        }

        if (((DistRightCheckCNT == 1) && (PulseOnR == HIGH))  && ((AngleM1 >= 170) && (AngleM1 <= 180)))
        {

          SecondRightState  = (mysensor2.distanceInCm() * 10);
          PercentSecondRightState = SecondRightState;

          //          Serial.println("");
          //          Serial.print("PercentSecondRightState :");
          //          Serial.print(PercentSecondRightState);
          //          Serial.print(" MM");
          //          Serial.println("");

          lcd1.setCursor(0, 1);
          lcd1.print("1:");
          lcd1.print(PercentSecondRightState);
          lcd1.print(" ");

        }
      }
      if (bluledState == HIGH) {
        RightDifference = 0;
      } else {
        RightDifference = (PercentFirstRightState - PercentSecondRightState);
      }


      if ((RightDifference >= 200) || (RightDifference <= -200) && (bluledState == LOW)) {
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

        //      if ((RightDifference <= 200) && (RightDifference >= -200)) {
        //        //    RightDifferenceOn = LOW;
        //        digitalWrite(bluledPin, LOW);
        //        //  bluledState = LOW;


        lcd1.setCursor(6, 0);
        lcd1.print(bluledState);
        lcd1.print(" ");
        //      }
      }

    }
  }

  //***********************************************************************************************************

  void setup() {
    // put your setup code here, to run once:

    lcd.init();
    lcd.backlight();
    lcd1.init();
    lcd1.backlight();

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

    pinMode (whtledPin, OUTPUT);
    pinMode (grnledPin, OUTPUT);
    pinMode (bluledPin, OUTPUT);
    pinMode (redledPin, OUTPUT);

    pinMode (ledPin, OUTPUT);

    pinMode (leftPin1A, OUTPUT);      // LEFT MOTOR
    pinMode (leftPin1B, OUTPUT);      // LEFT MOTOR

    pinMode (rightPin2A, OUTPUT);      // RIGHT MOTOR
    pinMode (rightPin2B, OUTPUT);      // RIGHT MOTOR

    pinMode (runPin, INPUT_PULLUP);
    pinMode (run2Pin, INPUT_PULLUP);

    pinMode (LeftSelPin, INPUT_PULLUP);
    pinMode (RightSelPin, INPUT_PULLUP);

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

    mysensor1.attach(A0, A1); //Trigger pin , Echo pin Front SENSOR
    mysensor2.attach(A4, A5); //Trigger pin , Echo pin Rear SENSOR

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
    rampStartTime = millis();

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


    whtledState = digitalRead(whtledPin);
    redledState = digitalRead(redledPin);
    grnledState = digitalRead(grnledPin);
    bluledState = digitalRead(bluledPin);

    RunState = digitalRead(runPin);
    Run2State = digitalRead(run2Pin);

    LeftSelState = digitalRead(LeftSelPin);
    RightSelState = digitalRead(RightSelPin);



    //************************************************************************************************************
    //************************************************************************************************************
    //                SCANNER                SCANNER                SCANNER                SCANNER
    //************************************************************************************************************

    //************************** DISTANCE MEASURING IN MM ********************************************************

    Distreading4 = (mysensor1.distanceInCm() * 10);  // FRONT SCANNER
    Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
    Distreading4R = 0;  // BOTTOM MIDDEL SCANNER

    Distreading5 = (mysensor2.distanceInCm() * 10);  // REAR SENSOR
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

    //if (BigLeftDist >= BigRightDist)

    //BigLeftDist
    //BigRightDist
    ////***********************************************************************************************************

    ScanForward();
    LRcheckForward();
    ScanPulseTimerL();
    ScanPulseTimerR();

    Serial.println("");
    Serial.print("Progress6 :");
    Serial.print(Progress6);
    Serial.print(" ");

    Serial.println("");
    Serial.print("Step1On :");
    Serial.print(Step1On);
    Serial.print(" ");


    Serial.println("");
    Serial.print("Step2On :");
    Serial.print(Step2On);
    Serial.print(" ");


    Serial.println("");
    Serial.print("Step3On :");
    Serial.print(Step3On);
    Serial.print(" ");


    Serial.println("");
    Serial.print("Step4On :");
    Serial.print(Step4On);
    Serial.print(" ");


    Serial.println("");

    //        Serial.println("");
    //      Serial.print("DistRightCheckCNT :");
    //      Serial.print(DistRightCheckCNT);
    //      Serial.println("");

    ////***********************************************************************************************************
    //                                   RUNNING AI CONTROL
    //  LeftSelState == HIGH;

    //if (( ScanRearRunning = LOW)){

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
      ScanLeftTurn();
    }

    if (((BigRightDist <= BigLeftDist)) || (RightSelState == LOW)) { // && (ScanRearRunning = LOW)){
      LeftDifference =  0;
      digitalWrite(bluledPin, LOW);
      RightDifference = 0;
      ScanRightTurn();
    }
    //}

    //  if ((RunState == HIGH)) {
    //     MoveStopAI();
    //  }

    if ((RunState == LOW)) {

      ////***********************************************************************************************************
      //                                 CAR CONTROL
      //    LRcheckForward();
      //
      if (grnledState == 1) {
        MoveStopAI();
        TurnMoveLeft90AI();
        MoveStopAI();
      }
      else {
        grnledState = 0;
      }


      if (bluledState == 1) {
        MoveStopAI();
        TurnMoveRight90AI();
        MoveStopAI();

      }
      else {
        bluledState = 0;
      }

      ////      else
      if ((LeftDifference ==  LOW) && (RightDifference == LOW)) {
        if ((Distreading4 >= FarSetPoint4)) {
          //        MoveForwardAI();

        }
      }


      ////***********************************************************************************************************
      ////***********************************************************************************************************
      //                         FRONT SENSOR SWEEP


      //    if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
      //        && (AngleM <= 100) && (AngleM >= 80)) {
      //      LRcheckForward();
      //      MoveStopAI();
      //
      //      AngleBotLeft = AngleM;
      //      AngleBotRight = AngleM;
      //
      //      myservoScanFront.write(160);
      //
      //      delay(500);
      //
      //      Distreading4L = (mysensor1.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
      //
      //      lcd.setCursor(11, 1);
      //      lcd.print(Distreading4L  );
      //      lcd.print(" ");
      //
      //      myservoScanFront.write(20);
      //
      //      delay(1000);
      //
      //      Distreading4R = (mysensor1.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
      //
      //      lcd.setCursor(3, 1);
      //      lcd.print(Distreading4R  );
      //      lcd.print(" ");
      //
      //      myservoScanFront.write(90);
      //      delay(500);
      //
      //      if ((Distreading4L >= Distreading4R) || (grnledState == HIGH)) {
      //
      //        ScanRight();
      //
      //                MoveLeft90AI();
      //      }
      //
      //      if ((Distreading4R >= Distreading4L) || (bluledState == HIGH)) {
      //
      //        ScanLeft();
      //
      //                MoveRight90AI();
      //
      //      }
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


