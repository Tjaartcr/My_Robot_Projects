//**************************************************************
////                 GYRO MPU-6050

//#include <MPU6050_tockn.h>
//#include <Wire.h>
//
//MPU6050 mpu6050(Wire);

//**************************************************************
//                   LCD CONTROL
#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x23, 20, 4);

//**************************************************************
//                    DISTANCE SENSING

//#include<UltraDistSensor.h>
//
//UltraDistSensor mysensor4;   // BOTTOM CENTER SCAN SERVO DISTANCE SENSOR (CAR)
//UltraDistSensor mysensor5;   // BOTTOM LEFT DISTANCE SENSOR (CAR)
//UltraDistSensor mysensor6;   // BOTTOM RIGHT DISTANCE SENSOR (CAR)

//*******************************************

#include <Ultrasonic.h>
Ultrasonic ultrasonic(A0, A1);
//int distance;


//**************************************************************
//                    DISTANCE CONTROL

int Distreading4;    // Bottom Middel SCANNER SENSOR
int Distreading4L;    // Bottom LEFT Middel SCANNER SENSOR
int Distreading4R;   // Bottom RIGHT Middel SCANNER SENSOR (CAR)
int Distreading5;    // Bottom LEFT SENSOR (CAR)
int Distreading6;    // Bottom RIGHT SENSOR (CAR)

int MotionDetectcted = 0;

int CloseSetPoint3 = 150;         //  Centre Dist Set point Closeby
int FarSetPoint3 = 300;         //  Centre Dist Set point Far

int CloseSetPoint4 = 50;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4 = 300;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4S = 50;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4S = 500;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4L = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4L = 700;         //  Bottom Centre Dist Set point Far

int CloseSetPoint4R = 500;         // Bottom Centre Dist Set point Closeby
int FarSetPoint4R = 700;         //  Bottom Centre Dist Set point Far

int CloseSetPoint5L = 100;         // Bottom LEFT Dist Set point Closeby
int FarSetPoint5L = 400;         //  Bottom LEFT Dist Set point Far

int CloseSetPoint5R = 100;         // Bottom RIGHT Dist Set point Closeby
int FarSetPoint5R = 400;         //  Bottom RIGHT Dist Set point Far


//*************************************************************

//***********************************************
//***************************************************************************************
//                  STEPPER MOTOR CONTROL

//**********************************************
//            STEPPER MOTORS TIMER

int STEPS_PER_REV_RAMP = 100;

int StopTime = (1000);          //155000;

int Timer1 =  (1000);            //155000;

//int StopTime = (10000);          //155000;
//
//int Timer1 =  (10000 / 600);            //155000;

int Timer2 = (Timer1 * 2) + StopTime;

int Timer3 = Timer2 + StopTime;

unsigned long StepperMoveTime3 = Timer1;
unsigned long StepperMoveTime3a = Timer1 + StopTime;
unsigned long StepperMoveTime3b = Timer2;
unsigned long ResetTime3 =  Timer3;

unsigned long StepperMoveStartTime;
unsigned long StepperProgress3;


int StepOna = 0;
int StepOffa = 0;
int StepOnb = 0;
int StepOffb = 0;


int led2PinState = 0;
#define led2Pin 2

//***********************************************
//     NEW STEPPER TIMER

unsigned long ServoMoveTime6 = 100;
unsigned long ServoMoveStartTime6;
unsigned long Progress6;
unsigned long ResetTime6 = 200;

int On = 0;
int Off = 0;

// ************************************************************************
//                ROBOT CAR SPEED CONTROL

unsigned long rampUpEndTime = 10000;
unsigned long rampDnEndTime = 20000;
unsigned long rampStartTime;
unsigned long rampRunner ;
unsigned long rampProgres;

int startSpeedForward = 0;
int stopSpeedForward = 200;

#define speedForwardAI 100
#define speedForwardLeftAI 100
#define speedForwardRightAI 100

int speedRampForwardAI = 0;

#define speedBackwardAI 100
#define speedBackwardLeftAI 100
#define speedBackwardRightAI 100

#define speedRampBackwardAI 100

#define speedLeftAI 100
#define speedLeftLeftAI 150    // Turn Left Left Wheel Speed
#define speedLeftRightAI 150    // Turn Left Right Wheel Speed

#define speedRampLeftAI 100

#define speedRightAI 100
#define speedRightLeftAI 150    // Turn Right Left Wheel Speed
#define speedRightRightAI 150    // Turn Right Right Wheel Speed

#define speedRampRightLeftAI 80


//**********************************************
//                 STEPPER MOTORS TIMER

//int STEPS_PER_REV_RAMP = 0;
//
//
//unsigned long StepperMoveTime3 = 1000;
//unsigned long StepperMoveStartTime;
//unsigned long StepperProgress3;
//unsigned long ResetTime3 = 1000;
//
//int StepOn = 0;
//int StepOff = 0;
//
//int led2PinState = 0;
//#define led2Pin 2

//*****************************************************************

//*************************************************************
//           MANUVER DELAY CONTROL

#define ManuverDelay 500

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

unsigned long ServoMoveTime = 5000;
unsigned long ServoMoveStartTime;
unsigned long Progress;
unsigned long ResetTime = 10000;

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
#define ledPin 13

//***********************************************


//***************************************************************************************
//                  STEPPER MOTOR CONTROL

// Motor steps per rotation
const int STEPS_PER_REV = 100;      // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN,

#define RMS1 22
#define RMS2 23
#define RMS3 24

#define LMS1 25
#define LMS2 26
#define LMS3 27

#define leftEnaPin 8
#define LeftdirPin 7
#define LeftstepPin 6

#define RightdirPin 5
#define RightstepPin 4
#define rightEnaPin 3

//******************************************************************************************

//***********************************************
//      SCANNER Trig Echo TIME

unsigned long ServoMoveTime5 = 5;
unsigned long ServoMoveStartTime5;
unsigned long Progress5;
unsigned long ResetTime5 = 1000;

int TrigOn = 0;
int TrigOff = 0;
int EchoOn = 0;
int EchoOff = 0;


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

//*******************************************************

#define ProgRunPin 52
int ProgRunState = 0;

#define DrvRunPin 51
int DrvRunState = 0;

#define DrvRunOutPin 50
int DrvRunOutState = 0;

#define Power12vPin A12 // 12 Volt Power Enable Pin
#define Power5vPin A11 // 5 Volt Power Enable Pin 

float tempPin = A8;
int Temp = 0;

//***********************************************************************************************************
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
  pinMode (led2Pin, OUTPUT);
  //***********************************************************************
  // Setup STEPPER MOTORS

  pinMode(RightstepPin, OUTPUT);
  pinMode(RightdirPin, OUTPUT);

  pinMode(LeftstepPin, OUTPUT);
  pinMode(LeftdirPin, OUTPUT);

  pinMode(LMS1, OUTPUT);
  pinMode(LMS2, OUTPUT);
  pinMode(LMS3, OUTPUT);

  pinMode(RMS1, OUTPUT);
  pinMode(RMS2, OUTPUT);
  pinMode(RMS3, OUTPUT);

  pinMode(rightEnaPin, OUTPUT);
  pinMode(leftEnaPin, OUTPUT);

  pinMode (DrvRunOutPin, OUTPUT);


  //***********************************************************************

  pinMode(Power12vPin, OUTPUT);
  pinMode(Power5vPin, OUTPUT);

  pinMode (ProgRunPin, INPUT_PULLUP);
  pinMode (DrvRunPin, INPUT_PULLUP);

  

  ////***********************************************************************************************************
  //  ////***********************************************************************************************************
  //  //                   ENCODERS
  //
  //  pinMode (PinCLK1, INPUT);
  //  attachInterrupt (4, isr1, FALLING);
  //
  //  pinMode (PinCLK2, INPUT);
  //  attachInterrupt (5, isr2, FALLING);

  TimeRunner1 = millis();
  TimeRunner2 = millis();
//  TimeRunner3 = millis();
  
  ////***********************************************************************************************************
  ////***********************************************************************************************************

  myservoScanB.attach(A6);  // create servo object to control a servo

//  mysensor4.attach(A0, A1); //Trigger pin , Echo pin Front
//  mysensor5.attach(A2, A3); //Trigger pin , Echo pin Left
//  mysensor6.attach(A4, A5); //Trigger pin , Echo pin Right

  rampStartTime = millis();
  ServoMoveStartTime6 = micros();
  ServoMoveStartTime5 = micros();  
  ServoMoveStartTime2 = millis();
  ServoMoveStartTime = millis();
  StandbyStartTime = millis();
  StepperMoveStartTime = millis();
  
}

////***********************************************************************************************************
////***********************************************************************************************************


////***********************************************************************************************************
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

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE STOP LEFT CONTROL AI

void MoveStopLeftAI() {

  //  analogWrite (leftPin1A, 0);
  //  analogWrite (leftPin1B, 0);
  //  analogWrite (rightPin2A, 0);
  //  analogWrite (rightPin2B, 0);
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE STOP RIGHT CONTROL AI

void MoveStopRightAI() {

  //  analogWrite (leftPin1A, 0);
  //  analogWrite (leftPin1B, 0);
  //  analogWrite (rightPin2A, 0);
  //  analogWrite (rightPin2B, 0);
}

//***********************************************************************************************************
//***********************************************************************************************************
//                                      MOVE FORWARD CONTROL AI

//int CloseSetPoint4S = 50;         // Bottom Centre Dist Set point Closeby
//int FarSetPoint4S = 500

void MoveForwardAI() {


//    if (Distreading4 >= FarSetPoint4S) {

  // Set motor direction counterclockwise
  
  digitalWrite(RightdirPin, LOW);
  digitalWrite(LeftdirPin, HIGH);

//do   FrontDistSens();

////    Distreading4 = (sonar.ping_cm() * 10);  // BOTTOM MIDDEL SCANNER

  DrvRunState = digitalRead(DrvRunPin);

//while (DrvRunState == LOW) {
  
//if (On == HIGH){
    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
//}
    delayMicroseconds(1000);    //1000
//if (Off == HIGH){
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
//}    
    delayMicroseconds(1000);    //1000
              
  
//}


//}
}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE STOP CONTROL AI

void MoveStopAI() {



  digitalWrite(rightEnaPin, HIGH);
  digitalWrite(leftEnaPin, HIGH);

  delay(1000);

}


//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE BACKWARD CONTROL AI

void MoveBackwardAI() {

  // Set motor direction counterclockwise
  digitalWrite(RightdirPin, HIGH);
  digitalWrite(LeftdirPin, LOW);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor two rotations quickly
  //  for(int x = 0; x < (STEPS_PER_REV * 2); x++) {      // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,
  for (int x = 0; x < (STEPS_PER_REV); x++) {

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT 90 CONTROL AI

void MoveLeft90AI() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction counterclockwise
  digitalWrite(RightdirPin, LOW);
  digitalWrite(LeftdirPin, LOW);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor two rotations quickly
  for (int x = 0; x < (STEPS_PER_REV * 2); x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);
  }

}


//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE LEFT 45 CONTROL AI


void MoveLeft45AI() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction counterclockwise
  digitalWrite(RightdirPin, LOW);
  digitalWrite(LeftdirPin, LOW);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor two rotations quickly

  for (int x = 0; x < (STEPS_PER_REV); x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT 90 CONTROL AI

void MoveRight90AI() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction counterclockwise
  digitalWrite(RightdirPin, HIGH);
  digitalWrite(LeftdirPin, HIGH);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor two rotations quickly

  for (int x = 0; x < (STEPS_PER_REV * 2); x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE RIGHT 45 CONTROL AI

void MoveRight45AI() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction counterclockwise
  digitalWrite(RightdirPin, HIGH);
  digitalWrite(LeftdirPin, HIGH);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor two rotations quickly
  for (int x = 0; x < (STEPS_PER_REV); x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);
  }

}

//***********************************************************************************************************
//***********************************************************************************************************
//                               MOVE 500mm CONTROL AI


void Move500AI() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction clockwise
  digitalWrite(RightdirPin, LOW);
  digitalWrite(LeftdirPin, HIGH);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor one rotation slowly
  for (int x = 0; x < STEPS_PER_REV * 4.5; x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 205mm, 4.5 = 500mm,

    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);

  }

}

//***********************************************************************************************************
//                               MOVE 500mm + Offset CONTROL AI

void MoveOffset() {

  digitalWrite(LMS1, LOW);
  digitalWrite(LMS2, LOW);
  digitalWrite(LMS3, HIGH);

  digitalWrite(RMS1, LOW);
  digitalWrite(RMS2, LOW);
  digitalWrite(RMS3, HIGH);

  // Set motor direction clockwise
  digitalWrite(RightdirPin, LOW);
  digitalWrite(LeftdirPin, HIGH);

  digitalWrite(rightEnaPin, LOW);
  digitalWrite(leftEnaPin, LOW);

  // Spin motor one rotation slowly
  for (int x = 0; x < STEPS_PER_REV * 5; x++) {     // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm, 5 = 550mm,
    digitalWrite(RightstepPin, HIGH);
    digitalWrite(LeftstepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(RightstepPin, LOW);
    digitalWrite(LeftstepPin, LOW);
    delayMicroseconds(1000);

  }

}

//***********************************************************************************************************

void FrontDistSens(){

    //************************************************************************************************************
    //                SCANNER                SCANNER                SCANNER                SCANNER
    //************************************************************************************************************
//    ////***********************************************************************************************************


    ////***********************************************************************************************************
    //                                         SCANNER SERVO CONTROL

    AngleM = Angle;

    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress

    if (Progress <= ServoMoveTime) {
      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
      myservoScanB.write(AngleM);
    }

    if (Progress >= ServoMoveTime) {
      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
      myservoScanB.write(AngleM);

    }

    if (Progress >= ResetTime) {
      ServoMoveStartTime = millis();

    }

    ////***********************************************************************************************************
//    ////***********************************************************************************************************
//    //                                         SCANNER  CONTROL

    Progress5 = millis() - ServoMoveStartTime5;     // Servo Head Progress

    if (Progress5 <= ServoMoveTime5) {
TrigOn = HIGH;
    }

    if (Progress5 >= ServoMoveTime5) {
TrigOn = LOW;
    }

    if (Progress5 >= ResetTime5) {
      ServoMoveStartTime5 = millis();
    }

    ////***********************************************************************************************************
if (TrigOn == HIGH){
  
  Distreading4 = (ultrasonic.read() * 10);
    Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
    Distreading4R = 0;  // BOTTOM MIDDEL SCANNER
//    Distreading5 = (ultrasonic.read() * 10);  // BOTTOM LEFT SENSOR
//    Distreading6 = (ultrasonic.read() * 10);  // BOTTOM RIGHT SENSOR



//    Distreading4 = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
//     Distreading4 = (sonar.ping_cm() * 10);  // BOTTOM MIDDEL SCANNER
    digitalWrite(led2Pin, HIGH);

}
}



void loop() {

  
  // put your main code here, to run repeatedly:

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //           SERIAL PRINT DELAY TIMER


//  ledPinState = digitalRead(ledPin);
//
//  Progress2 = millis() - ServoMoveStartTime2;     // Servo Head Progress
//
//  if (Progress2 <= ServoMoveTime2) {
//    digitalWrite(ledPin, HIGH);
//    ledPinState = HIGH;
//  }
//
//  if (Progress2 >= ServoMoveTime2) {
//    digitalWrite(ledPin, LOW);
//    ledPinState = LOW;
//  }
//
//  if (Progress2 >= ResetTime2) {
//    ServoMoveStartTime2 = millis();
//  }

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //           STEPPER MOTORS TIMER


//  led2PinState = digitalRead(led2Pin);
//
//  StepperProgress3 = millis() - StepperMoveStartTime;     // Servo Head Progress
//
//  if (StepperProgress3 <= StepperMoveTime3) {
//    digitalWrite(led2Pin, HIGH);
//    led2PinState = HIGH;
//    StepOn = HIGH;
//    StepOff = LOW; 
//  }
//
//  if (StepperProgress3 >= StepperMoveTime3) {
//    digitalWrite(led2Pin, LOW);
//    led2PinState = LOW;
//    StepOff = HIGH;
//    StepOn = LOW;
//  }
//
//  if (StepperProgress3 >= ResetTime3) {
//    StepperMoveStartTime = millis();
//  }
//int StepOn = 0;
//int StepOff = 0;
  ////***********************************************************************************************************
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

  ProgRunState = digitalRead(ProgRunPin);
  DrvRunState = digitalRead(DrvRunPin);

  //************************************************************************************************************

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

  //  ////***********************************************************************************************************
  //  ////***********************************************************************************************************
  //  //                 ENCODER 1
  //
  //  // Runs if rotation was detected
  //
  //  if (TurnDetected1) {
  //    PrevPosition1 = WheelPosition1;  // Save previous position in variable
  //    WheelPosition1 = WheelPosition1 + 1;  // Increase Position by 1
  //    RPMCounter1 = RPMCounter1 + 1;
  //    LeftTurnCNT = LeftTurnCNT + 1;
  //    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
  //
  //    StandbyStartTime = millis();
  //
  //  }
  //
  //  TimeStart1 = millis() - TimeRunner1;
  //
  //  if (TimeStart1 <= TimeStart1a) {
  //    RPM1 = 0;
  //    RPMCounter1 = 0;
  //  }
  //
  //  if (TimeStart1 >= TimeStart1a) {
  //    RPMCounter1a = RPMCounter1;
  //  }
  //
  //  if (TimeStart1 >= Reset1) {
  //    RPM1 = (RPMCounter1a / 20) * 60;
  //    RPMavg1 = (RPM1 + RPM1) / 2;
  //    TimeRunner1 = millis();
  //    //    Serial.print("RPM1: ");
  //    //    Serial.println(RPM1);
  //    //
  //    //    Serial.print("RPMavg1: ");
  //    //    Serial.println(RPMavg1);
  //    //
  //    //    Serial.print("RPMCounter1a: ");
  //    //    Serial.println(RPMCounter1a);
  //
  //  }

  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                 ENCODER 2*

  //  // Runs if rotation was detected
  //
  //  if (TurnDetected2) {
  //    PrevPosition2 = WheelPosition2;  // Save previous position in variable
  //    WheelPosition2 = WheelPosition2 + 1;  // Increase Position by 1
  //    RPMCounter2 = RPMCounter2 + 1;
  //    RightTurnCNT = RightTurnCNT + 1;
  //    TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected
  //
  //    StandbyStartTime = millis();
  //  }
  //
  //  TimeStart2 = millis() - TimeRunner2;
  //
  //  if (TimeStart2 <= TimeStart2a) {
  //    RPM2 = 0;
  //    RPMCounter2 = 0;
  //  }
  //
  //  if (TimeStart2 >= TimeStart2a) {
  //    RPMCounter2a = RPMCounter2;
  //
  //  }
  //
  //  if (TimeStart2 >= Reset2) {
  //    RPM2 = (RPMCounter2a / 20) * 60;
  //    RPMavg2 = (RPM2 + RPM2) / 2;
  //    TimeRunner2 = millis();
  //    //    Serial.print("RPM2: ");
  //    //    Serial.println(RPM2);
  //    //
  //    //    Serial.print("RPMavg2: ");
  //    //    Serial.println(RPMavg2);
  //    //
  //    //    Serial.print("RPMCounter2a: ");
  //    //    Serial.println(RPMCounter2a);
  //
  //  }

  ////***********************************************************************************************************
  ////***********************************************************************************************************

  //  if (DrvRunState == LOW){
  //  //    //************************************************************************************************************
  ////  ////                           TEST RAMP UP MOTOR CONTROL
  ////

  ////STEPS_PER_REV_RAMP = STEPS_PER_REV;
  //
  //int x = 0;
  //
  //        rampProgres = millis() - rampStartTime;     // Servo Head Progress
  //
  //        if ((rampProgres >= 0) && (rampProgres <= rampUpEndTime)) {
  //          x = map(rampProgres, 0, rampUpEndTime, startSpeedForward, stopSpeedForward);
  //
  //  //      MoveForwardRampTestAI();
  //
  //  digitalWrite(LMS1, LOW);
  //  digitalWrite(LMS2, LOW);
  //  digitalWrite(LMS3, HIGH);
  //
  //  digitalWrite(RMS1, LOW);
  //  digitalWrite(RMS2, LOW);
  //  digitalWrite(RMS3, HIGH);
  //
  //  // Set motor direction clockwise
  //  digitalWrite(RightdirPin,LOW);
  //  digitalWrite(LeftdirPin,HIGH);
  //
  //  digitalWrite(rightEnaPin, LOW);
  //  digitalWrite(leftEnaPin, LOW);
  //
  //  // Spin motor one rotation slowly
  //  for(int x = 0; x < STEPS_PER_REV * 5; x++) {      // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,
  //    digitalWrite(RightstepPin, HIGH);
  //    digitalWrite(LeftstepPin, HIGH);
  //    delayMicroseconds(1000);
  //    digitalWrite(RightstepPin, LOW);
  //    digitalWrite(LeftstepPin, LOW);
  //    delayMicroseconds(1000);
  //
  //        }
  //
  //  //rampStartTime = millis();
  //
  //  //  delay(2000);
  //
  //
  //
  //
  //  }
  //
  ////  //int startSpeedForward = 0;
  ////  //int stopSpeedForward = 250;
  ////  //speedRampForwardAI
  ////  //unsigned long rampUpEndTime = 10000;
  ////  //unsigned long rampDnEndTime = 20000;
  ////  //unsigned long rampStartTime;
  ////  //unsigned long rampRunner ;
  ////  //unsigned long rampProgres;
  //      //************************************************************************************************************
  //  //                           TEST RAMP DOWN MOTOR CONTROL
  //
  //

  //  //      rampProgres = millis() - rampStartTime;     // Servo Head Progress
  //
  //        if ((rampProgres >= rampUpEndTime) && (rampProgres <= rampDnEndTime)) {
  //          x = map(rampProgres, rampUpEndTime, rampDnEndTime, stopSpeedForward, startSpeedForward);
  //
  //  //      MoveForwardRampTestAI();
  //
  //    digitalWrite(LMS1, LOW);
  //  digitalWrite(LMS2, LOW);
  //  digitalWrite(LMS3, HIGH);
  //
  //  digitalWrite(RMS1, LOW);
  //  digitalWrite(RMS2, LOW);
  //  digitalWrite(RMS3, HIGH);
  //
  //  // Set motor direction clockwise
  //  digitalWrite(RightdirPin,LOW);
  //  digitalWrite(LeftdirPin,HIGH);
  //
  //  digitalWrite(rightEnaPin, LOW);
  //  digitalWrite(leftEnaPin, LOW);
  //
  //   // Spin motor one rotation slowly
  //  for(int x = 0; x < STEPS_PER_REV * 5; x++) {      // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,
  //    digitalWrite(RightstepPin, HIGH);
  //    digitalWrite(LeftstepPin, HIGH);
  //    delayMicroseconds(1000);
  //    digitalWrite(RightstepPin, LOW);
  //    digitalWrite(LeftstepPin, LOW);
  //    delayMicroseconds(1000);
  //
  //  }
  //        }
  //
  //        if ((rampProgres >= rampDnEndTime)) {
  //
  //          LeftTurnCNT = 0;
  //        RightTurnCNT = 0;
  //
  //  rampStartTime = millis();
  //        }
  //
  //   }

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
  //  //      myservoScanB.write(AngleM);
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
  //                                   RUNNING AI CONTROL

  if (ProgRunState == LOW) {

    //   while (Distreading4 >= FarSetPoint4){
    //do{


    //    //      MoveForwardRampTestAI();
    //
    //      digitalWrite(LMS1, LOW);
    //    digitalWrite(LMS2, LOW);
    //    digitalWrite(LMS3, HIGH);
    //
    //    digitalWrite(RMS1, LOW);
    //    digitalWrite(RMS2, LOW);
    //    digitalWrite(RMS3, HIGH);
    //
    //    // Set motor direction clockwise
    //    digitalWrite(RightdirPin,LOW);
    //    digitalWrite(LeftdirPin,HIGH);
    //
    //    digitalWrite(rightEnaPin, LOW);
    //    digitalWrite(leftEnaPin, LOW);
    //
    //     // Spin motor one rotation slowly
    //    for(int x = 0; x < STEPS_PER_REV * 5; x++) {      // 100 STEPS = 1/2 TURN, 200 STEPS = 1 TURN = 1 TURN = 205mm,
    //      digitalWrite(RightstepPin, HIGH);
    //      digitalWrite(LeftstepPin, HIGH);
    //      delayMicroseconds(1000);
    //      digitalWrite(RightstepPin, LOW);
    //      digitalWrite(LeftstepPin, LOW);
    //      delayMicroseconds(1000);
    //
    //    }

    ////***********************************************************************************************************


    ////***********************************************************************************************************
    //                          DISTANCE (SCANNER AND HEAD SERVO) MOVEMENT CONTROL

    ////***********************************************************************************************************

FrontDistSens();

    if ((Distreading4 >= FarSetPoint4)) {

      MoveForwardAI();

    }


//    //************************************************************************************************************
//    //                SCANNER                SCANNER                SCANNER                SCANNER
//    //************************************************************************************************************
////    ////***********************************************************************************************************
//
//
//    ////***********************************************************************************************************
//    //                                         SCANNER SERVO CONTROL
//
//    AngleM = Angle;
//
//    Progress = millis() - ServoMoveStartTime;     // Servo Head Progress
//
//    if (Progress <= ServoMoveTime) {
//      AngleM = map(Progress, 0, ServoMoveTime, StopAngleM, StartAngleM);
//      myservoScanB.write(AngleM);
//    }
//
//    if (Progress >= ServoMoveTime) {
//      AngleM = map(Progress, ServoMoveTime, ResetTime, StartAngleM, StopAngleM);
//      myservoScanB.write(AngleM);
//
//    }
//
//    if (Progress >= ResetTime) {
//      ServoMoveStartTime = millis();
//
//    }
//
//    ////***********************************************************************************************************
////    ////***********************************************************************************************************
////    //                                         SCANNER  CONTROL
//
//    Progress5 = millis() - ServoMoveStartTime5;     // Servo Head Progress
//
//    if (Progress5 <= ServoMoveTime5) {
//TrigOn = HIGH;
//    }
//
//    if (Progress5 >= ServoMoveTime5) {
//TrigOn = LOW;
//    }
//
//    if (Progress5 >= ResetTime5) {
//      ServoMoveStartTime5 = millis();
//    }
//
//    ////***********************************************************************************************************
//if (TrigOn == HIGH){
//  
//  Distreading4 = (ultrasonic.read() * 10);
//    Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
//    Distreading4R = 0;  // BOTTOM MIDDEL SCANNER
////    Distreading5 = (ultrasonic.read() * 10);  // BOTTOM LEFT SENSOR
////    Distreading6 = (ultrasonic.read() * 10);  // BOTTOM RIGHT SENSOR
//
//
//
////    Distreading4 = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
////     Distreading4 = (sonar.ping_cm() * 10);  // BOTTOM MIDDEL SCANNER
//    digitalWrite(led2Pin, HIGH);
//
//}

//int TrigOn = 0;
//int TrigOff = 0;

//    ////***********************************************************************************************************
    //    //************************** DISTANCE MEASURING IN MM ********************************************************
    //
//    Distreading4 = (mysensor4.distanceInCm() * 10);  // BOTTOM MIDDEL SCANNER
//    Distreading4L = 0;  // BOTTOM MIDDEL SCANNER
//    Distreading4R = 0;  // BOTTOM MIDDEL SCANNER
//    Distreading5 = (mysensor5.distanceInCm() * 10);  // BOTTOM LEFT SENSOR
//    Distreading6 = (mysensor6.distanceInCm() * 10);  // BOTTOM RIGHT SENSOR

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
    ////***********************************************************************************************************
    //                                 CAR CONTROL

    //do {

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                         BOTTOM MIDDEL SENSOR SWEEP


    if (((Distreading4 <= FarSetPoint4) && (Distreading4 >= CloseSetPoint4))
        && (AngleM <= 100) && (AngleM >= 80)) {

      MoveStopAI();

      AngleBotLeft = AngleM;
      AngleBotRight = AngleM;

      myservoScanB.write(160);

      delay(1000);

//  Distreading4 = (ultrasonic.read() * 10);

      Distreading4L = (ultrasonic.read() * 10);  // BOTTOM MIDDEL SCANNER

      lcd.setCursor(11, 1);
      lcd.print(Distreading4L  );
      lcd.print(" ");

      delay(1000);

      myservoScanB.write(30);

      delay(1000);

      Distreading4R = (ultrasonic.read() * 10);  // BOTTOM MIDDEL SCANNER

      lcd.setCursor(3, 1);
      lcd.print(Distreading4R  );
      lcd.print(" ");

      delay(1000);

      myservoScanB.write(90);
      delay(1000);


      if (Distreading4L >= Distreading4R) {


        LeftTurnCNT = 0;
        RightTurnCNT = 0;



        delay(1000);

        MoveLeft90AI();
      }

      if (Distreading4R >= Distreading4L) {


        LeftTurnCNT = 0;
        RightTurnCNT = 0;


        delay(1000);

        MoveRight90AI();

      }

    }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                          45 DEGREE MANUVER LEFT

    if (((Distreading4 <= FarSetPoint4L) && (Distreading4 >= CloseSetPoint4L))
        && (AngleM <= 140) && (AngleM >= 100)) {

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
      LeftTurnCNT = 0;
      RightTurnCNT = 0;
    }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                          45 DEGREE MANUVER RIGHT

    if (((Distreading4 <= FarSetPoint4R) && (Distreading4 >= CloseSetPoint4R))
        && (AngleM >= 40) && (AngleM <= 80)) {

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
      LeftTurnCNT = 0;
      RightTurnCNT = 0;
    }

    ////***********************************************************************************************************
    ////***********************************************************************************************************
    //                       Testing Of Motors

//              MoveForwardAI();
//              MoveStopAI();
//              Move500AI();
//              MoveStopAI();
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

    //}
    //while (Distreading4 <= FarSetPoint4);


  }
  else
  {
    digitalWrite(leftEnaPin, HIGH);
    digitalWrite(rightEnaPin, HIGH);
  }
  ////***********************************************************************************************************
  ////***********************************************************************************************************
  //                LCD2 OUTPUT CONTROL
  //
//  lcd.setCursor(6, 0);
//  //  lcd.print("L:");
//  lcd.print(Distreading4);
//  //  lcd.print(" ");
//  //  lcd.setCursor(10, 0);
//  //  lcd.print("R:");
//  //  lcd.print(Distreading6);
//  //  lcd.print(" ");
//
//  lcd.setCursor(0, 1);
//  lcd.print("R: ");
//  lcd.print(Distreading6 );
//  lcd.print(" ");
//  lcd.setCursor(8, 1);
//  lcd.print("L: ");
//  lcd.print(Distreading5 );
//  lcd.print(" ");
//
//
//  lcd.setCursor(0, 0);
//  lcd.print("R:");
//  lcd.print(RightTurnCNT);
//  lcd.print(" ");
  //  lcd.print(TurnDetected2);
  //  lcd.print(" ");
//  lcd.setCursor(11, 0);
//  lcd.print("L:");
//  lcd.print(LeftTurnCNT);
//  lcd.print(" ");
  //  lcd.print(TurnDetected1);
  //  lcd.print(" ");



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


  //  if (ledPinState == HIGH) {

  
//      Serial.println("");
//      Serial.print("ProgRunState :");
//      Serial.print(ProgRunState);
//      Serial.println("");

  //    Serial.println("");
  //    Serial.print("Progress2 :");
  //    Serial.print(Progress2);
  //    Serial.println("");
  
//      Serial.println("");
//      Serial.print("StepperProgress3 :");
//      Serial.print(StepperProgress3);
//      Serial.println("");  
  
  //  Serial.println("");
  //  Serial.print("SDistreading1 :");
  //  Serial.print(Distreading1);
  //  Serial.print(" MM");
  //  Serial.println("");
  //
  //  Serial.println("");
  //
  Serial.println("");
  Serial.print("Distreading4 :");
  Serial.print(Distreading4);
  Serial.print(" MM");
  Serial.println("");

  //        Serial.println("");
  //        Serial.print("Distreading5 :");
  //        Serial.print(Distreading5);
  //        Serial.print(" MM");
  //        Serial.println("");
  //
  //        Serial.println("");
  //        Serial.print("Distreading6 :");
  //        Serial.print(Distreading6);
  //        Serial.print(" MM");
  //        Serial.println("");

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


  //        Serial.println("");
  //        Serial.print("DrvRunState:");
  //        Serial.print(DrvRunState);
  //        Serial.println("");
  //
  //        Serial.println("");
  //        Serial.print("rampProgres:");
  //        Serial.print(rampProgres);
  //        Serial.println("");
  //
  ////        Serial.println("");
  ////        Serial.print("x:");
  ////        Serial.print(x);
  ////        Serial.println("");
  //
  //        Serial.println("");
  //        Serial.print("STEPS_PER_REV_RAMP:");
  //        Serial.print(STEPS_PER_REV_RAMP);
  //        Serial.println("");



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
  //  }
}
