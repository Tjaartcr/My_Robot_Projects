//#include <Adafruit_MCP23X17.h>
//Adafruit_MCP23X17 mcp;

//#include <Wire.h>
//
//#include <LiquidCrystal_I2C.h>
//
//LiquidCrystal_I2C lcd(0x25, 16, 2);
//LiquidCrystal_I2C lcd1(0x23, 20, 4);

//***********************************************
//***********************************************
//            SERIAL PRINT TIMER

unsigned long SerialPrintTime = 50;
unsigned long SerialPrintStartTime;
unsigned long SerialPrintProgress;
unsigned long SerialPrintResetTime = 1000;

int SerialPrintState;

//***********************************************
#define MoveForward 36
#define MoveReverse 37
#define MoveLeft 35
#define MoveRight 34

#define MoveRun 38
#define MoveStop 39

#define Auto_Manual 41
#define Left_Right 40

#define MoveRun 37
int MoveRunState = LOW; 

#define TestPBselect 42
int TestPBselectState = LOW; 
int TestPBselectCNT = 0;

const int TestPotPin = A15;
float TestPotPinValue = 0;
float TestPotPinValueOpposite = 0;

//************************************************
//                LEFT LEG

const int LeftHipPinA = 22;
const int LeftHipPinB = 24;
const int LeftHipPinEN = 4;
int LeftHipSpeedOffSet;

const int LeftHipPossisionSensorPin = A2;
float LeftHipPossisionSensorPinValue = 0;


const int LeftKneePinA = 26;
const int LeftKneePinB = 28;
const int LeftKneePinEN = 6;
int LeftKneeSpeedOffSet;
int LeftKneeSpeedOffSet1;
int LeftKneeSpeedOffSet2;
int LeftKneeSpeedOffSet3;
int LeftKneeSpeedOffSet4;

const int LeftKneePossisionSensorPin = A3;
float LeftKneePossisionSensorPinValue = 0;

//***********************************************

//************************************************
//                RIGHT LEG

const int RightHipPinA = 23;
const int RightHipPinB = 25;
const int RightHipPinEN = 10;
int RightHipSpeedOffSet;

const int RightHipPossisionSensorPin = A8;
float RightHipPossisionSensorPinValue = 0;


const int RightKneePinA = 27;
const int RightKneePinB = 29;
const int RightKneePinEN = 11;
int RightKneeSpeedOffSet;

const int RightKneePossisionSensorPin = A9;
float RightKneePossisionSensorPinValue = 0;



//***********************************************
//           SERIAL PRINT DELAY TIMER

void serialprintTimer() {

  SerialPrintProgress = millis() - SerialPrintStartTime;     // Servo Head Progress

  if (SerialPrintProgress <= SerialPrintTime) {
    SerialPrintState = HIGH;
  }

  if (SerialPrintProgress >= SerialPrintTime) {
    SerialPrintState = LOW;
  }

  if (SerialPrintProgress >= SerialPrintResetTime) {
    SerialPrintStartTime = millis();
  }
}

//***********************************************

void setup() {
  // put your setup code here, to run once:
  
Serial.begin(115200);
//  mcp.begin();
//mcp.begin_I2C();

pinMode(MoveForward, INPUT);
pinMode(MoveReverse, INPUT);
pinMode(MoveLeft, INPUT);
pinMode(MoveRight, INPUT);

pinMode(MoveRun, INPUT);
pinMode(MoveStop, INPUT);

pinMode(Auto_Manual, INPUT);
pinMode(Left_Right, INPUT);


pinMode(TestPBselect, INPUT);
pinMode(MoveRun, INPUT);

pinMode(RightHipPinA, OUTPUT);
pinMode(RightHipPinB, OUTPUT);

pinMode(RightKneePinA, OUTPUT);
pinMode(RightKneePinB, OUTPUT);

pinMode(LeftHipPinA, OUTPUT);
pinMode(LeftHipPinB, OUTPUT);

pinMode(LeftKneePinA, OUTPUT);
pinMode(LeftKneePinB, OUTPUT);

SerialPrintStartTime = millis();


}

void loop() {
  // put your main code here, to run repeatedly:

serialprintTimer();


TestPBselectState = digitalRead(TestPBselect);
MoveRunState = digitalRead(MoveRun);

if (TestPBselectState == HIGH){
TestPBselectCNT = TestPBselectCNT +1;
delay(250);
}

if (TestPBselectCNT >= 10){
TestPBselectCNT = 0;
}

TestPotPinValue = analogRead(TestPotPin);
TestPotPinValue = map(TestPotPinValue, 0, 1023, 0, 180);

if (TestPotPinValue <= 0){
TestPotPinValue = 0; 
}

TestPotPinValueOpposite = 180 - TestPotPinValue;

RightHipPossisionSensorPinValue = analogRead(RightHipPossisionSensorPin);
RightHipPossisionSensorPinValue = map(RightHipPossisionSensorPinValue, 1023, 0, 0, 180);

RightKneePossisionSensorPinValue = analogRead(RightKneePossisionSensorPin);
RightKneePossisionSensorPinValue = map(RightKneePossisionSensorPinValue, 3450, 970, 0, 120);

RightHipSpeedOffSet = TestPotPinValue - RightHipPossisionSensorPinValue;
if (RightHipSpeedOffSet <= -1){
RightHipSpeedOffSet = RightHipSpeedOffSet * -1; 
}
RightHipSpeedOffSet = RightHipSpeedOffSet * 2;   

if (RightHipSpeedOffSet >= 255){
RightHipSpeedOffSet = 255; 
}

if (RightHipSpeedOffSet <= 150){
RightHipSpeedOffSet = 150; 
}

RightKneeSpeedOffSet = TestPotPinValue - RightKneePossisionSensorPinValue;
if (RightKneeSpeedOffSet <= -1){
RightKneeSpeedOffSet = RightKneeSpeedOffSet * -1;  
}
RightKneeSpeedOffSet = RightKneeSpeedOffSet * 2;

if (RightKneeSpeedOffSet >= 255){
RightKneeSpeedOffSet = 255;  
}

if (RightKneeSpeedOffSet <= 230){
RightKneeSpeedOffSet = 230;  
}

if (TestPBselectCNT == 1){
if (MoveRunState == HIGH){

if (RightHipPossisionSensorPinValue >= TestPotPinValue + 10){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, HIGH);;
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
}

if (RightHipPossisionSensorPinValue <= TestPotPinValue - 10){
  
digitalWrite(RightHipPinA, HIGH);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
}

if ((RightHipPossisionSensorPinValue >= TestPotPinValue -10) && ((RightHipPossisionSensorPinValue <= TestPotPinValue +10))){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
}

if (MoveRunState == LOW){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
}
}
}



if (TestPBselectCNT == 2){
if (MoveRunState == HIGH){


if (RightKneePossisionSensorPinValue >= TestPotPinValue +10 ){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, HIGH);;
analogWrite(RightKneePinEN, RightKneeSpeedOffSet);
  
}

if (RightKneePossisionSensorPinValue <= TestPotPinValue -10){
  
digitalWrite(RightKneePinA, HIGH);
digitalWrite(RightKneePinB, LOW);
analogWrite(RightKneePinEN, RightKneeSpeedOffSet);
  
}


if ((RightKneePossisionSensorPinValue >= TestPotPinValue -10) && ((RightKneePossisionSensorPinValue <= TestPotPinValue +10))){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
}

if (MoveRunState == LOW){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
}

}
}

//*****************************************************************************************************************
//                  LEFT LEG


LeftHipPossisionSensorPinValue = analogRead(LeftHipPossisionSensorPin);
LeftHipPossisionSensorPinValue = map(LeftHipPossisionSensorPinValue, 1023, 0, 0, 180);

LeftKneePossisionSensorPinValue = analogRead(LeftKneePossisionSensorPin);
LeftKneePossisionSensorPinValue = map(LeftKneePossisionSensorPinValue, 1023, 0, 0, 120);

LeftHipSpeedOffSet = TestPotPinValue - LeftHipPossisionSensorPinValue;
if (LeftHipSpeedOffSet <= -1){
LeftHipSpeedOffSet = LeftHipSpeedOffSet * -1; 
}
LeftHipSpeedOffSet = LeftHipSpeedOffSet * 2;   

if (LeftHipSpeedOffSet >= 255){
LeftHipSpeedOffSet = 255; 
}

if (LeftHipSpeedOffSet <= 150){
LeftHipSpeedOffSet = 150; 
}



LeftKneeSpeedOffSet = TestPotPinValue - LeftKneePossisionSensorPinValue;
LeftKneeSpeedOffSet1 = LeftKneeSpeedOffSet;

if (LeftKneeSpeedOffSet <= -1){
LeftKneeSpeedOffSet = LeftKneeSpeedOffSet * -1;  
}
LeftKneeSpeedOffSet = LeftKneeSpeedOffSet * 2;

LeftKneeSpeedOffSet2 = LeftKneeSpeedOffSet;

if (LeftKneeSpeedOffSet >= 255){
LeftKneeSpeedOffSet = 255;  
}

if (LeftKneeSpeedOffSet <= 230){
LeftKneeSpeedOffSet = 230;  
}

if (TestPBselectCNT == 3){
if (MoveRunState == HIGH){


//LeftHipSpeedOffSet = TestPotPinValue - LeftHipPossisionSensorPinValue;



if (LeftHipPossisionSensorPinValue >= TestPotPinValue + 10){
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, HIGH);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);
  
}

if (LeftHipPossisionSensorPinValue <= TestPotPinValue - 10){
  
digitalWrite(LeftHipPinA, HIGH);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);
  
}


if ((LeftHipPossisionSensorPinValue >= TestPotPinValue -10) && ((LeftHipPossisionSensorPinValue <= TestPotPinValue +10))){
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);
  
}


if (MoveRunState == LOW){
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);
  
}
}
}

//****************************************************


if (TestPBselectCNT == 4){
if (MoveRunState == HIGH){

//LeftKneeSpeedOffSet = TestPotPinValue - LeftKneePossisionSensorPinValue;


if (LeftKneePossisionSensorPinValue >= TestPotPinValue +10 ){
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, HIGH);;
analogWrite(LeftKneePinEN, LeftKneeSpeedOffSet);
  
}

if (LeftKneePossisionSensorPinValue <= TestPotPinValue -10){
  
digitalWrite(LeftKneePinA, HIGH);
digitalWrite(LeftKneePinB, LOW);
analogWrite(LeftKneePinEN, LeftKneeSpeedOffSet);
  
}


if ((LeftKneePossisionSensorPinValue >= TestPotPinValue -10) && ((LeftKneePossisionSensorPinValue <= TestPotPinValue +10))){
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, LOW);;
analogWrite(LeftKneePinEN, 0);
  
}


if (MoveRunState == LOW){
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, LOW);;
analogWrite(LeftKneePinEN, 0);
  
}

}
}


//******************************************************************************************************************
//          LEFT & RIGHT HIP CONTROL BOTH SAME DIRECTION


if (TestPBselectCNT == 5){
if (MoveRunState == HIGH){


if ((RightHipPossisionSensorPinValue >= TestPotPinValue + 10) && (LeftHipPossisionSensorPinValue >= TestPotPinValue + 10)){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, HIGH);
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, HIGH);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);
  
}

if ((RightHipPossisionSensorPinValue <= TestPotPinValue - 10) && (LeftHipPossisionSensorPinValue <= TestPotPinValue - 10)){
  
digitalWrite(RightHipPinA, HIGH);
digitalWrite(RightHipPinB, LOW);
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
digitalWrite(LeftHipPinA, HIGH);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);  
}


if (((RightHipPossisionSensorPinValue >= TestPotPinValue -10) && ((RightHipPossisionSensorPinValue <= TestPotPinValue +10))) && ((LeftHipPossisionSensorPinValue >= TestPotPinValue -10) && ((LeftHipPossisionSensorPinValue <= TestPotPinValue +10)))){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);   
}


if (MoveRunState == LOW){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);  
}


}
}
//******************************************************************************************************************
//          LEFT & RIGHT HIP CONTROL OPPOTE SAME DIRECTION


if (TestPBselectCNT == 6){
if (MoveRunState == HIGH){

if ((RightHipPossisionSensorPinValue >= TestPotPinValue + 10) && (LeftHipPossisionSensorPinValue <= TestPotPinValueOpposite - 10)){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, HIGH);
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
digitalWrite(LeftHipPinA, HIGH);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);
  
}

if ((RightHipPossisionSensorPinValue <= TestPotPinValue - 10) && (LeftHipPossisionSensorPinValue >= TestPotPinValueOpposite + 10)){
  
digitalWrite(RightHipPinA, HIGH);
digitalWrite(RightHipPinB, LOW);
analogWrite(RightHipPinEN, RightHipSpeedOffSet);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, HIGH);;
analogWrite(LeftHipPinEN, LeftHipSpeedOffSet);  
}


if (((RightHipPossisionSensorPinValue >= TestPotPinValue -10) && (RightHipPossisionSensorPinValue <= TestPotPinValue +10)) && ((LeftHipPossisionSensorPinValue >= TestPotPinValueOpposite -10) && (LeftHipPossisionSensorPinValue <= TestPotPinValueOpposite +10))){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);   
}


if (MoveRunState == LOW){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
digitalWrite(LeftHipPinA, LOW);
digitalWrite(LeftHipPinB, LOW);;
analogWrite(LeftHipPinEN, 0);  
}


}
}

//******************************************************************************************************************
//          LEFT & RIGHT KNEES CONTROL BOTH SAME DIRECTION


if (TestPBselectCNT == 7){
if (MoveRunState == HIGH){


if ((RightKneePossisionSensorPinValue >= TestPotPinValue + 10) && (LeftKneePossisionSensorPinValue >= TestPotPinValue + 10)){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, HIGH);
analogWrite(RightKneePinEN, RightKneeSpeedOffSet);
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, HIGH);;
analogWrite(LeftKneePinEN, LeftKneeSpeedOffSet);
  
}

if ((RightKneePossisionSensorPinValue <= TestPotPinValue - 10) && (LeftKneePossisionSensorPinValue <= TestPotPinValue - 10)){
  
digitalWrite(RightKneePinA, HIGH);
digitalWrite(RightKneePinB, LOW);
analogWrite(RightKneePinEN, RightKneeSpeedOffSet);
  
digitalWrite(LeftKneePinA, HIGH);
digitalWrite(LeftKneePinB, LOW);;
analogWrite(LeftKneePinEN, LeftKneeSpeedOffSet);  
}


if (((RightKneePossisionSensorPinValue >= TestPotPinValue -10) && ((RightKneePossisionSensorPinValue <= TestPotPinValue +10))) && ((LeftKneePossisionSensorPinValue >= TestPotPinValue -10) && ((LeftKneePossisionSensorPinValue <= TestPotPinValue +10)))){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, LOW);;
analogWrite(LeftKneePinEN, 0);   
}


if (MoveRunState == LOW){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
digitalWrite(LeftKneePinA, LOW);
digitalWrite(LeftKneePinB, LOW);;
analogWrite(LeftKneePinEN, 0);  
}

}
}

//******************************************************************************************************************

if (SerialPrintState == HIGH){

Serial.println("");
Serial.print("TestPBselectState : ");
Serial.print(TestPBselectState);

Serial.println("");
Serial.print("TestPBselectCNT : ");
Serial.print(TestPBselectCNT);

Serial.println("");
Serial.print("MoveRunState : ");
Serial.print(MoveRunState);

Serial.println("");
Serial.print("TestPotPinValue : ");
Serial.print(TestPotPinValue);

Serial.println("");
Serial.print("TestPotPinValueOpposite : ");
Serial.print(TestPotPinValueOpposite);

Serial.println("");
Serial.print("RightHipPossisionSensorPinValue : ");
Serial.print(RightHipPossisionSensorPinValue);

Serial.println("");
Serial.print("RightKneePossisionSensorPinValue : ");
Serial.print(RightKneePossisionSensorPinValue);

Serial.println("");
Serial.print("RightHipSpeedOffSet : ");
Serial.print(RightHipSpeedOffSet);

Serial.println("");
Serial.print("RightKneeSpeedOffSet : ");
Serial.print(RightKneeSpeedOffSet);


Serial.println("");
Serial.print("LeftHipPossisionSensorPinValue : ");
Serial.print(LeftHipPossisionSensorPinValue);

Serial.println("");
Serial.print("LeftKneePossisionSensorPinValue : ");
Serial.print(LeftKneePossisionSensorPinValue);

Serial.println("");
Serial.print("LeftHipSpeedOffSet : ");
Serial.print(LeftHipSpeedOffSet);

Serial.println("");
Serial.print("LeftKneeSpeedOffSet : ");
Serial.print(LeftKneeSpeedOffSet);

Serial.println("");
Serial.print("LeftKneeSpeedOffSet1 : ");
Serial.print(LeftKneeSpeedOffSet1);

Serial.println("");
Serial.print("LeftKneeSpeedOffSet2 : ");
Serial.print(LeftKneeSpeedOffSet2);

Serial.println("");
Serial.print("LeftKneeSpeedOffSet3 : ");
Serial.print(LeftKneeSpeedOffSet3);

Serial.println("");
Serial.print("LeftKneeSpeedOffSet4 : ");
Serial.print(LeftKneeSpeedOffSet4);

//Serial.println("");
//Serial.print("TestSelectRightKneeState : ");
//Serial.print(TestSelectRightKneeState);
//
//Serial.println("");
//Serial.print("TestSelectRightHipState : ");
//Serial.print(TestSelectRightHipState);
//
//Serial.println("");
//Serial.print("TestSelectLeftKneeState : ");
//Serial.print(TestSelectLeftKneeState);
//
//Serial.println("");
//Serial.print("TestSelectLeftHipState : ");
//Serial.print(TestSelectLeftHipState);


Serial.println("");


}

}
