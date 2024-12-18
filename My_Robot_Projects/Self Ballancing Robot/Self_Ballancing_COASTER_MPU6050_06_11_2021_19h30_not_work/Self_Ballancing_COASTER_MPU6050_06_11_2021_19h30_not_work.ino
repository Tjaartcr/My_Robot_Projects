// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


//****************** DC Motor Control ***************************

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

int FanIn1  = 3;

float TempDrvPin = A1;
int TempDrv = 0;

int fanSpeed = 0;

//***************************************************************
/////////////////////////////////////////////////////////////////
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

}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

//  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//  Serial.print("Accelerometer range set to: ");
//  switch (mpu.getAccelerometerRange()) {
//  case MPU6050_RANGE_2_G:
//    Serial.println("+-2G");
//    break;
//  case MPU6050_RANGE_4_G:
//    Serial.println("+-4G");
//    break;
//  case MPU6050_RANGE_8_G:
//    Serial.println("+-8G");
//    break;
//  case MPU6050_RANGE_16_G:
//    Serial.println("+-16G");
//    break;
//  }
//  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//  Serial.print("Gyro range set to: ");
//  switch (mpu.getGyroRange()) {
//  case MPU6050_RANGE_250_DEG:
//    Serial.println("+- 250 deg/s");
//    break;
//  case MPU6050_RANGE_500_DEG:
//    Serial.println("+- 500 deg/s");
//    break;
//  case MPU6050_RANGE_1000_DEG:
//    Serial.println("+- 1000 deg/s");
//    break;
//  case MPU6050_RANGE_2000_DEG:
//    Serial.println("+- 2000 deg/s");
//    break;
//  }
//
//  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  Serial.print("Filter bandwidth set to: ");
//  switch (mpu.getFilterBandwidth()) {
//  case MPU6050_BAND_260_HZ:
//    Serial.println("260 Hz");
//    break;
//  case MPU6050_BAND_184_HZ:
//    Serial.println("184 Hz");
//    break;
//  case MPU6050_BAND_94_HZ:
//    Serial.println("94 Hz");
//    break;
//  case MPU6050_BAND_44_HZ:
//    Serial.println("44 Hz");
//    break;
//  case MPU6050_BAND_21_HZ:
//    Serial.println("21 Hz");
//    break;
//  case MPU6050_BAND_10_HZ:
//    Serial.println("10 Hz");
//    break;
//  case MPU6050_BAND_5_HZ:
//    Serial.println("5 Hz");
//    break;
//  }
//
//  Serial.println("");
//  delay(100);


//********************* DC Motor Control ************************

  // Set all the motor control pins to outputs
 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);



     
//***************************************************************
/////////////////////////////////////////////////////////////////


//************************* FAN CONTROL *************************

  pinMode(FanIn1, OUTPUT);


//***************************************************************

mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // +- 16, 8, 4, 2
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);      // +- 2000, 1000, 500, 250
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);   // 5, 10, 21, 44, 94, 184, 260

}



void loop() {
  // put your main code here, to run repeatedly:


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");
//
//  Serial.print("Rotation X: ");
//  Serial.print(g.gyro.x);
//  Serial.print(", Y: ");
//  Serial.print(g.gyro.y);
//  Serial.print(", Z: ");
//  Serial.print(g.gyro.z);
//  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
//  delay(500);

//**********************************************************

TempDrv = analogRead(TempDrvPin);
TempDrv = TempDrv / 10;

fanSpeed = 255;

if (TempDrv >= 18){
 
  FanStart();
  analogWrite(FanIn1, fanSpeed);

 
}else if (TempDrv <= 16){
  
  FanStop();
}


Serial.println("TempDrv:");
Serial.println(TempDrv);
 
//***************************************************************
/////////////////////////////////////////////////////////////////

SpeedOffset = 5;  
AngleOffSet = 0;
Delay = 50;
ZeroAdjust = 2 / 10;


LeftAdjFor = 0;
RightAdjFor = 0;

LeftAdjBack = 0;
RightAdjBack = 0;

if ((a.acceleration.x <= ZeroAdjust) && (a.acceleration.x >= -ZeroAdjust))
{
  a.acceleration.x = 0;
stop();

}

 

a.acceleration.x = a.acceleration.x * 10;  

  Serial.println("Acceleration X: ");
  Serial.println(a.acceleration.x);

MotorLeftForwardSpeed = a.acceleration.x * -4;
MotorRightForwardSpeed = a.acceleration.x * -4;

MotorLeftBackSpeed = a.acceleration.x * 4;
MotorRightBackSpeed = a.acceleration.x * 4;





//MotorLeftForwardSpeed = map(a.acceleration.x, 0, -80, 0, 55);
//MotorRightForwardSpeed = map(a.acceleration.x, 0, -80, 0, 55);
//
//MotorLeftBackSpeed = map(a.acceleration.x, 0, 80, 0, 55);
//MotorRightBackSpeed = map(a.acceleration.x, 0, 80, 0, 55);

MotorLeftForwardSpeed = (MotorLeftForwardSpeed + LeftAdjFor);
MotorRightForwardSpeed = (MotorRightForwardSpeed + RightAdjFor);



//if (MotorLeftForwardSpeed <= 1){
//  MotorLeftForwardSpeed = 0;
//}
//
//if (MotorRightForwardSpeed <= 1){
//  MotorRightForwardSpeed = 0;
//}

MotorLeftBackSpeed = (MotorLeftBackSpeed + LeftAdjBack);
MotorRightBackSpeed = (MotorRightBackSpeed + RightAdjBack);

//if (MotorLeftBackSpeed <= 1){
//  MotorLeftBackSpeed = 0;
//}
//
//if (MotorRightBackSpeed <= 1){
//  MotorRightBackSpeed = 0;
//}


  

  if (a.acceleration.x > 2)
  {

reverse();


  analogWrite(enA, MotorRightBackSpeed);
  analogWrite(enB, MotorLeftBackSpeed);
  
//  analogWrite(enA, 150);
//  analogWrite(enB, 150);
  
  
Serial.println("MotorLeftBackSpeed:");
Serial.println(MotorLeftBackSpeed);
Serial.println("MotorRightBackSpeed:");
Serial.println(MotorRightBackSpeed);
//Serial.println("angle:");
//Serial.println(angle.x);
  
  delay(Delay);  

  }
else if (a.acceleration.x < -2)
  { 
  
 forward();
     

  analogWrite(enA, MotorRightForwardSpeed);
  analogWrite(enB, MotorLeftForwardSpeed);

//  analogWrite(enA, 150);
//  analogWrite(enB, 150);


Serial.println("MotorLeftForwardSpeed:");
Serial.println(MotorLeftForwardSpeed);
Serial.println("MotorRightForwardSpeed:");
Serial.println(MotorRightForwardSpeed);
//Serial.println("angle:");
//Serial.println(angle.x);  

 delay(Delay);
   
  }

  delay(50);

//if ((a.acceleration.x <= 2) && (a.acceleration.x >= -2) || (a.acceleration.x >= 70) || (a.acceleration.x <= -70)){
//  stop();
//  analogWrite(enA, 0);
//  analogWrite(enB, 0);
//}

//*****************************************************
}
