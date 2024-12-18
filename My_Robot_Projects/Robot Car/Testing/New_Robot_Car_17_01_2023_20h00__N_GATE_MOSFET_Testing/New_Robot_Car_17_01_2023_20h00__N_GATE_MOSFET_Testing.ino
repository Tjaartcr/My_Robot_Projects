#include <Servo.h>
Servo myservo0;

  // Motor control pins : L298N H bridge

  #define LeftMotorSpeedPin  9 // Left motor PWM speed control
  #define RightMotorSpeedPin  10 // Left motor PWM speed control

  #define LeftMotorOnRelayPin  4 // Left motor PWM speed control
  #define RightMotorOnRelayPin  5 // Left motor PWM speed control

  #define LeftMotorForRevRelayPin  6 // Left motor PWM speed control
  #define RightMotorForRevRelayPin  7 // Left motor PWM speed control

  #define PowerSwitchPin 8
  int PowerSwitchState;

  #define LeftSwitchPin 11
  #define RightSwitchPin 12
  
  int LeftSwitchState;
  int RightSwitchState;

  #define carSpeed 150

  #define sensPin A2

int SpeedPotPin1 = A0;

int SpeedVal1;



//*****************************************************************
//                 ENCODER 1 (LEFT)

volatile boolean TurnDetected1;  // need volatile for Interrupts
volatile boolean rotationdirection1; // CW or CCW rotation

const int PinCLK1 = 3; // 5 Generating interrupts using CLK signal

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
int LeftUTurn180 = 0;
int LeftTurn180 = 0;
int LeftUTurn180On = 0;
int LeftTurn180On = 0;
int LeftUTurn180CNT = 0;
int LeftTurn180CNT = 0;
int LeftUTurn180CNTOn = 0;
int LeftTurn180CNTOn = 0;
int LastLeftUTurn180CNT = 0;
int LastLeftTurn180CNT = 0;

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

const int PinCLK2 = 2; // 4 Generating interrupts using CLK signal

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
int RightUTurn180 = 0;
int RightTurn180 = 0;
int RightUTurn180On = 0;
int RightTurn180On = 0;
int RightUTurn180CNT = 0;
int RightTurn180CNT = 0;
int RightUTurn180CNTOn = 0;
int RightTurn180CNTOn = 0;
int LastRightUTurn180CNT = 0;
int LastRightTurn180CNT = 0;

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




    void setup() 
  { 

Serial.begin(9600);

  pinMode(LeftMotorSpeedPin, OUTPUT);
  pinMode(RightMotorSpeedPin, OUTPUT);
   
  pinMode(LeftMotorOnRelayPin, OUTPUT);
  pinMode(RightMotorOnRelayPin, OUTPUT);

  pinMode(LeftMotorForRevRelayPin, OUTPUT);
  pinMode(RightMotorForRevRelayPin, OUTPUT);

  pinMode(PowerSwitchPin, INPUT_PULLUP);

  pinMode(LeftSwitchPin, INPUT_PULLUP);
  pinMode(RightSwitchPin, INPUT_PULLUP);


digitalWrite(LeftMotorOnRelayPin, LOW);
digitalWrite(RightMotorOnRelayPin, LOW);
  


  ////***********************************************************************************************************
  //                   ENCODERS

  pinMode (PinCLK1, INPUT);
  attachInterrupt (0, isr1, RISING);

  pinMode (PinCLK2, INPUT);
  attachInterrupt (1, isr2, RISING);

  TimeRunner1 = millis();
  TimeRunner2 = millis();

  ////***********************************************************************************************************


  
  } 


  
  void loop() 
  { 

  ////***********************************************************************************************************
  //                 ENCODER 1

  // Runs if rotation was detected

  if (TurnDetected1) {
    PrevPosition1 = WheelPosition1;  // Save previous position in variable
    WheelPosition1 = WheelPosition1 + 1;  // Increase Position by 1
    RPMCounter1 = RPMCounter1 + 1;
    LeftTurnCNT = LeftTurnCNT + 1;
    TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected

//    StandbyStartTime = millis();

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

        Serial.print("LeftTurnCNT: ");
        Serial.println(LeftTurnCNT);

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

//    StandbyStartTime = millis();
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

        Serial.print("RightTurnCNT: ");
        Serial.println(RightTurnCNT);
  
  }

  ////***********************************************************************************************************

 





    if (TurnDetected1) {
      LeftTurnCNT = LeftTurnCNT + 1;
      TurnDetected1 = false; // do NOT Repeat IF loop until new rotation detected
    }
    if (TurnDetected2) {
      RightTurnCNT = RightTurnCNT + 1;
      TurnDetected2 = false; // do NOT Repeat IF loop until new rotation detected

//*******************************************************************************************************************


SpeedVal1 = analogRead(SpeedPotPin1);            // reads the value of the potentiometer (value between 0 and 1023) 
SpeedVal1 = map(SpeedVal1, 0, 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180) 

PowerSwitchState = digitalRead(PowerSwitchPin);

LeftSwitchState = digitalRead(LeftSwitchPin);
RightSwitchState = digitalRead(RightSwitchPin);

if (PowerSwitchState == LOW){
//digitalWrite(LeftMotorOnRelayPin, HIGH);
//digitalWrite(RightMotorOnRelayPin, HIGH);

if (LeftSwitchState == LOW){

digitalWrite(RightMotorOnRelayPin, LOW);
digitalWrite(RightMotorForRevRelayPin, HIGH);

digitalWrite(LeftMotorOnRelayPin, HIGH);
digitalWrite(LeftMotorForRevRelayPin, LOW);
analogWrite(LeftMotorSpeedPin, SpeedVal1);

  
}

if (RightSwitchState == LOW){

digitalWrite(LeftMotorOnRelayPin, LOW);
digitalWrite(LeftMotorForRevRelayPin  , HIGH);

digitalWrite(RightMotorOnRelayPin, HIGH);
digitalWrite(RightMotorForRevRelayPin, LOW);
analogWrite(RightMotorSpeedPin, SpeedVal1);
  
}


if ((LeftSwitchState == HIGH) && (RightSwitchState == HIGH)){

digitalWrite(LeftMotorOnRelayPin, LOW);
digitalWrite(LeftMotorForRevRelayPin, HIGH);

digitalWrite(RightMotorOnRelayPin, LOW);
digitalWrite(RightMotorForRevRelayPin, HIGH);
  
}


  
}else{


digitalWrite(LeftMotorOnRelayPin, LOW);
digitalWrite(RightMotorOnRelayPin, LOW);

}
//
//if (enBPin == HIGH){
//  B = HIGH;
//  
//}else{
//  B = LOW;
//}
//
////  analogWrite (enAPin, val1);
// 
//  digitalWrite (enAPin, LOW);
//  digitalWrite (enBPin, HIGH);
// 
//  
//  delay(1000);
//  
//  digitalWrite (enAPin, LOW);
//  digitalWrite (enBPin, LOW);
//  
//  delay(1000);
//  
//  digitalWrite (enAPin, HIGH);
//  digitalWrite (enBPin, LOW);
//  
//  delay(1000);
//  
//  digitalWrite (enAPin, LOW);
//  digitalWrite (enBPin, LOW);  
//  
//  delay(1000);
//Serial.println("A:");
//Serial.println(A);
//Serial.println("B:");
//Serial.println(B);
//   
  }
