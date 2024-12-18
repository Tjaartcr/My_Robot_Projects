//****************** 9 AXIS Gyro Controll ************************

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68


MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

//***************************************************************
/////////////////////////////////////////////////////////////////

//****************** DC Motor Control ***************************

// Motor A
 
int enA = 9;
int in1 = 4;
int in2 = 5;

// Motor B

int enB = 10;
int in3 = 6;
int in4 = 7;

int forwardPin = 3;
//int reversePin = 6;

 

 
int joyVert = A0; // Vertical  

 
// Motor Speed Values - Start at zero
 
int MotorSpeed1 = 0;

int forward1 = LOW;
int reverse1 = LOW;

 

 
int joyposVert = 0;

//***************************************************************
/////////////////////////////////////////////////////////////////


void setup() {
 

  Serial.begin(115200);

  Wire.begin();


//*********************** 9 AXIS Gyro Control *******************
  
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250.enableAccDLPF(true);

  myMPU9250.setAccDLPF(MPU9250_DLPF_6);  

//***************************************************************
/////////////////////////////////////////////////////////////////

//********************* DC Motor Control ************************

  // Set all the motor control pins to outputs
 
  pinMode(enA, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  pinMode(forwardPin, INPUT);
//  pinMode(reversePin, INPUT);

     
  // Start with motors disabled and direction forward
  
  // Motor A
  
  digitalWrite(enA, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  

  digitalWrite(enB, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

//***************************************************************
/////////////////////////////////////////////////////////////////

}

void loop() {

 
//********************** 9 AXIS Gyro Control *******************

  xyzFloat angle = myMPU9250.getAngles();

  Serial.print("Angle x  = ");
  Serial.print(angle.x);
  Serial.print("  |  Angle y  = ");
  Serial.print(angle.y);
  Serial.print("  |  Angle z  = ");
  Serial.println(angle.z);

  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());

  Serial.println();
  
  delay(100);

//

//***************************************************************
/////////////////////////////////////////////////////////////////

//************************ DC Motor Control *********************

Serial.println("MotorSpeed1:");
Serial.println(MotorSpeed1);
//Serial.println("forward1:");  
//Serial.println(forward1);
//Serial.println("reverse:");  
//Serial.println(reverse1);

 // Read POT for Speed Referance
 
  joyposVert = analogRead(joyVert); 

  // Set the motor speeds
 
//  analogWrite(enA, MotorSpeed1);
//  analogWrite(enB, MotorSpeed1);

  
  
//  if (joyposVert > 512)
//  {
//    MotorSpeed1 = map(joyposVert -512, 0, 512, 0, 255);
//
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//
//    digitalWrite(in3, HIGH);
//    digitalWrite(in4, LOW);
//
//  }
//  else if (joyposVert < 509)
//  { 
//     MotorSpeed1 = map(joyposVert -512, 0, 512,  0, -255);
//
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, HIGH);
//  
//  }

//***************************************************************
/////////////////////////////////////////////////////////////////

//   MotorSpeed1 = map(angle.x -10, 0, 10, 0, 255);

//   MotorSpeed1 = (angle.x * 10);
  analogWrite(enA, MotorSpeed1);
  analogWrite(enB, MotorSpeed1);

  
//     MotorSpeed1 = map(angle.x -512, 0, 512,  0, 255);
//     MotorSpeed1 = map(angle.x -512, 0, 512,  0, -255);


  
  if (angle.x < 0)
  {
//    MotorSpeed1 = map(angle.x  -2, 0, 2, 0, 100);
//     MotorSpeed1 = map(angle.x -512, 0, 512,  0, 255);
     
//  analogWrite(enA, MotorSpeed1);
//  analogWrite(enB, MotorSpeed1);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

  }
  else if (angle.x > 0)
  { 
//     MotorSpeed1 = map(angle.x -2, 0, 2, 0, -100);
//     MotorSpeed1 = map(angle.x -512, 0, 512,  0, -255);     

//  analogWrite(enA, MotorSpeed1);
//  analogWrite(enB, MotorSpeed1);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  
  }


}
