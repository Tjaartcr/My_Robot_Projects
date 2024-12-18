
#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x25, 16, 2);
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


#define TestPBRunning 13
int TestPBRunningState = LOW; 

#define TestPBselect 2
int TestPBselectState = LOW; 
int TestPBselectCNT = 0;

const int TestPotPin = A0;
float TestPotPinValue = 0;

const int RightHipPinA = 3;
const int RightHipPinB = 4;
const int RightHipPinEN = 5;

const int RightHipPossisionSensorPin = A1;
float RightHipPossisionSensorPinValue = 0;


const int RightKneePinA = 7;
const int RightKneePinB = 8;
const int RightKneePinEN = 6;

const int RightKneePossisionSensorPin = A2;
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


pinMode(TestPBselect, INPUT_PULLUP);
pinMode(TestPBRunning, INPUT);

pinMode(RightHipPinA, OUTPUT);
pinMode(RightHipPinB, OUTPUT);

pinMode(RightKneePinA, OUTPUT);
pinMode(RightKneePinB, OUTPUT);

SerialPrintStartTime = millis();


}

void loop() {
  // put your main code here, to run repeatedly:

serialprintTimer();
  
TestPBselectState = !digitalRead(TestPBselect);
TestPBRunningState = digitalRead(TestPBRunning);

if (TestPBselectState == HIGH){
TestPBselectCNT = TestPBselectCNT +1;

delay(100);
}


if (TestPBselectCNT >= 3){
TestPBselectCNT = 0;
}


TestPotPinValue = analogRead(TestPotPin);
TestPotPinValue = map(TestPotPinValue, 340, 1023, 0, 180);

RightHipPossisionSensorPinValue = analogRead(RightHipPossisionSensorPin);
RightHipPossisionSensorPinValue = map(RightHipPossisionSensorPinValue, 1023, 400, 0, 180);

RightKneePossisionSensorPinValue = analogRead(RightKneePossisionSensorPin);
RightKneePossisionSensorPinValue = map(RightKneePossisionSensorPinValue, 940, 340, 0, 120);


if (TestPBselectCNT == 1){
if (TestPBRunningState == HIGH){

if (RightHipPossisionSensorPinValue >= TestPotPinValue + 10){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, HIGH);;
analogWrite(RightHipPinEN, 200);
  
}

if (RightHipPossisionSensorPinValue <= TestPotPinValue - 10){
  
digitalWrite(RightHipPinA, HIGH);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 200);
  
}


if ((RightHipPossisionSensorPinValue >= TestPotPinValue -5) && ((RightHipPossisionSensorPinValue <= TestPotPinValue +5))){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
}

//if (RightHipPossisionSensorPinValue >= 170){
//  
//digitalWrite(RightHipPinA, LOW);
//digitalWrite(RightHipPinB, LOW);;
//analogWrite(RightHipPinEN, 0);
//  
//}
//
//if (RightHipPossisionSensorPinValue <= 10){
//  
//digitalWrite(RightHipPinA, LOW);
//digitalWrite(RightHipPinB, LOW);;
//analogWrite(RightHipPinEN, 0);
//  
//}

if (TestPBRunningState == LOW){
  
digitalWrite(RightHipPinA, LOW);
digitalWrite(RightHipPinB, LOW);;
analogWrite(RightHipPinEN, 0);
  
}
}
}



if (TestPBselectCNT == 2){
if (TestPBRunningState == HIGH){

if (RightKneePossisionSensorPinValue >= TestPotPinValue +10){
  
digitalWrite(RightKneePinA, HIGH);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 250);
  
}

if (RightKneePossisionSensorPinValue <= TestPotPinValue -10){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, HIGH);
analogWrite(RightKneePinEN, 250);
  
}


if ((RightKneePossisionSensorPinValue >= TestPotPinValue -5) && ((RightKneePossisionSensorPinValue <= TestPotPinValue +5))){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
}

//if (RightKneePossisionSensorPinValue >= 110){
//  
//digitalWrite(RightKneePinA, LOW);
//digitalWrite(RightKneePinB, LOW);;
//analogWrite(RightKneePinEN, 0);
//  
//}
//
//if (RightKneePossisionSensorPinValue <= 10){
//  
//digitalWrite(RightKneePinA, LOW);
//digitalWrite(RightKneePinB, LOW);;
//analogWrite(RightKneePinEN, 0);
//  
//}

if (TestPBRunningState == LOW){
  
digitalWrite(RightKneePinA, LOW);
digitalWrite(RightKneePinB, LOW);;
analogWrite(RightKneePinEN, 0);
  
}

}
}

//
//TestPBselectState = !digitalRead(TestPBselect);
//TestPBRunningState =!digitalRead(TestPBRunning);
//
//const int RightHipPossisionSensorPin = A1;
//float RightHipPossisionSensorPinValue = 0;
//
//
//const int RightKneePinA = 7;
//const int RightKneePinB = 8;
//const int RightKneePinEN = 6;
//
//const int RightKneePossisionSensorPin = A2;
//float RightKneePossisionSensorPinValue = 0;
//


if (SerialPrintState == HIGH){

Serial.println("");
Serial.print("TestPBselectState : ");
Serial.print(TestPBselectState);

Serial.println("");
Serial.print("TestPBselectCNT : ");
Serial.print(TestPBselectCNT);

Serial.println("");
Serial.print("TestPBRunningState : ");
Serial.print(TestPBRunningState);

Serial.println("");
Serial.print("TestPotPinValue : ");
Serial.print(TestPotPinValue);

Serial.println("");
Serial.print("RightHipPossisionSensorPinValue : ");
Serial.print(RightHipPossisionSensorPinValue);

Serial.println("");
Serial.print("RightKneePossisionSensorPinValue : ");
Serial.print(RightKneePossisionSensorPinValue);

}

}
