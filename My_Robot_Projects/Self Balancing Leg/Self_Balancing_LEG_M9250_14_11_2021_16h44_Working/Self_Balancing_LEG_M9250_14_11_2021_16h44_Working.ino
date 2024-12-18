//****************** 9 AXIS Gyro Controll ************************

#include <MPU9250_WE.h>
#include <Wire.h>


#define MPU9250_ADDR 0x68


MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

//***************************************************************
/////////////////////////////////////////////////////////////////

#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

int SpeedOffset = 0;
int AngleOffSet = 0;
int Delay = 0;

int ZeroAdjust1 = 0;
int ZeroAdjust2 = 0;
int ZeroAdjust3 = 0;
int ZeroAdjust4 = 0;

int ZeroAdjustServ1 = 0;
int ZeroAdjustServ2 = 0;
int ZeroAdjustServ3 = 0;
int ZeroAdjustServ4 = 0;

int ServoAngle1 = 90;
int ServoAngle2 = 90;
int ServoAngle3 = 90;
int ServoAngle4 = 90;

//***************************************************************
/////////////////////////////////////////////////////////////////

void setup() {
  
  myservo.attach(3); 
  myservo1.attach(5); 
  myservo2.attach(6); 
  myservo3.attach(9); 
        
  Serial.begin(250000);

  Wire.begin();


//*********************** 9 AXIS Gyro Control *******************

    if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
//  delay(1000);

  myMPU9250.autoOffsets();

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250.enableAccDLPF(true);

  myMPU9250.setAccDLPF(MPU9250_DLPF_6);  

Wire.beginTransmission(0x68);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();

Wire.beginTransmission(0x68);
Wire.write(0x6C);
Wire.write(0x00);
Wire.endTransmission();  

Wire.beginTransmission(0x68);
Wire.write(0x1B);
Wire.write(0x00);
Wire.endTransmission();

Wire.beginTransmission(0x68);
Wire.write(0x1C);
Wire.write(0x00);
Wire.endTransmission();

//***************************************************************

  myservo1.write(90); 
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90); 

//delay(100);

}

void loop() {

//********************** 9 AXIS Gyro Control *******************

  xyzFloat angle = myMPU9250.getAngles();
  xyzFloat gValue = myMPU9250.getGValues();


//***************************************************************
/////////////////////////////////////////////////////////////////



  
  ServoAngle1 = map(angle.x, 90, -90, 0, 180); 
  ServoAngle2 = map(angle.x, 90, -90, 0, 180); 
  ServoAngle3 = map(angle.y, 90, -90, 0, 180); 
  ServoAngle4 = map(angle.y, 90, -90, 0, 180);     
   
   
   Serial.println("angle.x:");
   Serial.println(angle.x);
   Serial.println("angle.y:");
   Serial.println(angle.y);      
//   Serial.println("angle.z:");
//   Serial.println(angle.z); 
   
   Serial.println("ServoAngle1:");
   Serial.println(ServoAngle1);
   Serial.println("ServoAngle2");
   Serial.println(ServoAngle2);      
   Serial.println("ServoAngle3");
   Serial.println(ServoAngle3); 
   Serial.println("ServoAngle4");
   Serial.println(ServoAngle4); 

  myservo1.write(ServoAngle1); 
  myservo2.write(ServoAngle2); 
  myservo3.write(ServoAngle3); 
  myservo4.write(ServoAngle4); 

//delay(200);

}