// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


//************************************ DC Motor Control *******************************************

// Motor A
 
int enA = 9;
int in1 = 4;
int in2 = 5;

// Motor B

int enB = 10;
int in3 = 6;
int in4 = 7;

 
// Motor Speed Values - Start at zero
 
int MotorSpeed1 = 0;

int MotorsBackSpeed = 0;
int MotorsForwardSpeed = 0;
int SpeedOffset = 0;
int AngleOffSet = 0;
int Delay = 0;
int ZeroAdjust = 0;

int LeftAdjFor = 0;
int RightAdjFor = 0;

int LeftAdjBack = 0;
int RightAdjBack = 0;

int MotorLeftForwardSpeed = 0;
int MotorRightForwardSpeed = 0;

int MotorLeftBackSpeed = 0;
int MotorRightBackSpeed = 0;

//************************************* FAN & DRIVE TEMPERATURE CONTROL ************************
int FanIn1  = 3;
int LedPin = 13;

float TempDrvPin = A1;
int TempDrv = 0;

int fanSpeed = 0;
int fanSpeedidle = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////

//**************************************** void CONTROL *****************************************
/////////////////////////////////////////////////////////////////////////////////////////////////

void forward(){
  digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  
}

void reverse(){
  
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  
}

void stop(){
  
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void FanStart(){
  
//  digitalWrite(FanIn1,LOW);

}

void FanStop(){
  
  digitalWrite(FanIn1,LOW);
  digitalWrite(LedPin, LOW);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////


// **************************************** SETUP ****************************************************

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
//  while (!Serial)
//    delay(10); // will pause Zero, Leonardo, etc until serial console opens
//
//  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
//  Serial.println("MPU6050 Found!");

//************************************** DC Motor Control : SETUP ***************************************

  // Set all the motor control pins to outputs
 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);



     
//*****************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////


//******************************************* FAN CONTROL SETUP ***************************************

  pinMode(FanIn1, OUTPUT);


//********************************************** MPU 6050 SETUP ***************************************

mpu.setAccelerometerRange(MPU6050_RANGE_2_G);   // +- 16, 8, 4, 2
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // +- 2000, 1000, 500, 250
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);   // 5, 10, 21, 44, 94, 184, 260

}
////////////////////////////////////////////////////////////////////////////////////////////////////////


// ********************************************* void LOOP *********************************************

void loop() {
  // put your main code here, to run repeatedly:


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");
//
//  Serial.println("");


//*********************************************  FAN CONTROL *********************************************

TempDrv = analogRead(TempDrvPin);
TempDrv = TempDrv / 10;

fanSpeed = 255;
fanSpeedidle = 150;

if (TempDrv >= 18){
 
  FanStart();
  analogWrite(FanIn1, fanSpeed);

  digitalWrite(LedPin, HIGH);

}else if (TempDrv <= 17){
  
  FanStart();
    analogWrite(FanIn1, fanSpeedidle);
    
  digitalWrite(LedPin, HIGH);
 
}else if (TempDrv <= 16){
  
  FanStop();
}


//Serial.println("TempDrv:");
//Serial.println(TempDrv);
 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////


//****************************************************** MAIN PROGRAM *****************************************

MotorSpeed1 = a.acceleration.x;
//MotorSpeed1 = map(a.acceleration.x, 0, 8, 0, 255);

if (MotorSpeed1 <= 0){
MotorSpeed1 = MotorSpeed1 * -1;   
}

else if (MotorSpeed1 >= 0){
MotorSpeed1 = MotorSpeed1 * 1;  
}

if (MotorSpeed1 <= 0){
  MotorSpeed1 = 0;
}

if (MotorSpeed1>= 255){
  MotorSpeed1 = 255;
}

MotorSpeed1 = MotorSpeed1 * 65; 

  Serial.println("MotorSpeed1: ");
  Serial.println(MotorSpeed1);

//  Serial.println("a.acceleration.x: ");
//  Serial.println(a.acceleration.x);  
  
//Serial.println("MotorLeftBackSpeed:");
//Serial.println(MotorLeftBackSpeed);
//Serial.println("MotorLeftForwardSpeed:");
//Serial.println(MotorLeftForwardSpeed);


  
  analogWrite(enA, MotorSpeed1);
  analogWrite(enB, MotorSpeed1 * 2.5);



    if (a.acceleration.x > 0.1)
  {

reverse();  
delay(100);
  }


else if (a.acceleration.x < -0.1)
  { 
  
 forward();
delay(100);       

  }


else if ((a.acceleration.x >= 80) || (a.acceleration.x <= -80)){

  stop();

}

//**********************************************************************************************************************
  
}
