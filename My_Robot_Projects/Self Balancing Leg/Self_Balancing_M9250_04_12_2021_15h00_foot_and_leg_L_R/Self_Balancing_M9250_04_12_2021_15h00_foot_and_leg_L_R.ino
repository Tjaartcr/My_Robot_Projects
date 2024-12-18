//****************** 9 AXIS Gyro Controll ************************

#include <MPU9250_WE.h>
#include <Wire.h>


#define MPU9250l_ADDR 0x68
#define MPU9250r_ADDR 0x69

MPU9250_WE myMPU9250l = MPU9250_WE(MPU9250l_ADDR);

MPU9250_WE myMPU9250r = MPU9250_WE(MPU9250r_ADDR);

//***************************************************************
/////////////////////////////////////////////////////////////////

#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

Servo myservo6;
Servo myservo7;
Servo myservo8;
Servo myservo9;
Servo myservo10;

int SpeedOffset = 0;
int AngleOffSet = 0;
int Delay = 0;

int ZeroAdjust1 = 0;
int ZeroAdjust2 = 0;
int ZeroAdjust3 = 0;
int ZeroAdjust4 = 0;

int ZeroAdjust5 = 0;
int ZeroAdjust6 = 0;
int ZeroAdjust7 = 0;
int ZeroAdjust8 = 0;

int ZeroAdjustServ1 = 0;
int ZeroAdjustServ2 = 0;
int ZeroAdjustServ3 = 0;
int ZeroAdjustServ4 = 0;

int ZeroAdjustServ5 = 0;
int ZeroAdjustServ6 = 0;
int ZeroAdjustServ7 = 0;
int ZeroAdjustServ8 = 0;

int ServoAngle1 = 90;
int ServoAngle2 = 90;
int ServoAngle3 = 90;
int ServoAngle4 = 90;

int ServoAngle6 = 90;
int ServoAngle7 = 90;
int ServoAngle8 = 90;
int ServoAngle9 = 90;

int ServoAngle3a = 90;
int ServoAngle4a = 90;

int ServoAngle7a = 90;
int ServoAngle8a = 90;

int ServoAngle5 = 0;
int ServoAngle10 = 0;

//***************************************************************
/////////////////////////////////////////////////////////////////

void setup() {
  
  myservo1.attach(3); 
  myservo2.attach(5); 
  myservo3.attach(6); 
  myservo4.attach(9); 
  myservo5.attach(10);
          
  myservo6.attach(A0); 
  myservo7.attach(A1); 
  myservo8.attach(A2); 
  myservo9.attach(A3); 
  myservo10.attach(A4);
  
  
  
  Serial.begin(250000);

  Wire.begin();


//*********************** 9 AXIS Gyro Control Left Leg *******************

    if(!myMPU9250l.init()){
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

  myMPU9250l.autoOffsets();

  myMPU9250l.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250l.enableAccDLPF(true);

  myMPU9250l.setAccDLPF(MPU9250_DLPF_6);  

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

//*********************** 9 AXIS Gyro Control Righ Leg *******************

    if(!myMPU9250r.init()){
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

  myMPU9250r.autoOffsets();

  myMPU9250r.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250r.enableAccDLPF(true);

  myMPU9250r.setAccDLPF(MPU9250_DLPF_6);  

Wire.beginTransmission(0x69);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();

Wire.beginTransmission(0x69);
Wire.write(0x6C);
Wire.write(0x00);
Wire.endTransmission();  

Wire.beginTransmission(0x69);
Wire.write(0x1B);
Wire.write(0x00);
Wire.endTransmission();

Wire.beginTransmission(0x69);
Wire.write(0x1C);
Wire.write(0x00);
Wire.endTransmission();

//***************************************************************
  
  myservo1.write(90); 
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90); 
  myservo5.write(0);

    myservo6.write(90); 
  myservo7.write(90);
  myservo8.write(90);
  myservo9.write(90); 
  myservo10.write(0);
  
//delay(100);

}

void loop() {

//********************** 9 AXIS Gyro Control Left Leg *******************

  xyzFloat anglel = myMPU9250l.getAngles();
  xyzFloat gValuel = myMPU9250l.getGValues();


//***************************************************************
/////////////////////////////////////////////////////////////////

anglel.y = anglel.y * 1.5;
anglel.x = anglel.x * 1.5;
  
  ServoAngle1 = map(anglel.x, 90, -90, 0, 180); 
  ServoAngle2 = map(anglel.x, 90, -90, 0, 180);  
  ServoAngle3 = map(anglel.x, -90, 90, 0, 180);   
  ServoAngle4 = map(anglel.x, 90, -90, 0, 180);

  ServoAngle3a = map(anglel.y, -90, 90, 0, 180); 
  ServoAngle4a = map(anglel.y, -90, 90, 0, 180);
     



ServoAngle3 = ((360 - (ServoAngle3 + ServoAngle3a)) / 2);
ServoAngle4 = ((360 - (ServoAngle4 + ServoAngle4a)) / 2);

  myservo1.write(ServoAngle1); 
  myservo3.write(ServoAngle3); 
  myservo4.write(ServoAngle4);

  
  
  gValuel.x = gValuel.x * 100;
 
  gValuel.x = gValuel.x * 1.5;
 
  ServoAngle5 = map(gValuel.x, 0, 60, 0, 20);  
 
  myservo5.write(ServoAngle5);
 
//   Serial.println("anglel.x:");
//   Serial.println(anglel.x);
//   Serial.println("anglel.y:");
//   Serial.println(anglel.y);      
//   Serial.println("anglel.z:");
//   Serial.println(anglel.z); 
   
//   Serial.println("ServoAngle1:");
//   Serial.println(ServoAngle1);
//   Serial.println("ServoAngle2");
//   Serial.println(ServoAngle2);      
//   Serial.println("ServoAngle3");
//   Serial.println(ServoAngle3); 
//   Serial.println("ServoAngle4");
//   Serial.println(ServoAngle4); 
//   Serial.println("ServoAngle3a");
//   Serial.println(ServoAngle3a); 
//   Serial.println("ServoAngle4a");
//   Serial.println(ServoAngle4a); 

   Serial.println("ServoAngle5");
   Serial.println(ServoAngle5);

/* For g-values the corrected raws are used */
  Serial.println("g-xl      = ");
  Serial.println(gValuel.x);
//  Serial.print("  |  g-yl      = ");
//  Serial.print(gValuel.y);
//  Serial.print("  |  g-zl      = ");
//  Serial.println(gValuel.z);

delay(100);

//********************** 9 AXIS Gyro Control Right Leg *******************

  xyzFloat angler = myMPU9250r.getAngles();
  xyzFloat gValuer = myMPU9250r.getGValues();


//***************************************************************
/////////////////////////////////////////////////////////////////

angler.y = angler.y * 1.5;
angler.x = angler.x * 1.5;
  
  ServoAngle6 = map(angler.x, 90, -90, 0, 180); 
  ServoAngle7 = map(angler.x, 90, -90, 0, 180);  
  ServoAngle8 = map(angler.x, -90, 90, 0, 180);   
  ServoAngle9 = map(angler.x, 90, -90, 0, 180);

  ServoAngle8a = map(angler.y, -90, 90, 0, 180); 
  ServoAngle9a = map(angler.y, -90, 90, 0, 180);
     



ServoAngle8 = ((360 - (ServoAngle8 + ServoAngle8a)) / 2);
ServoAngle9 = ((360 - (ServoAngle9 + ServoAngle9a)) / 2);

  myservo1.write(ServoAngle6); 
  myservo3.write(ServoAngle8); 
  myservo4.write(ServoAngle9);

  
  
  gValuer.x = gValuer.x * 100;
 
  gValuer.x = gValuer.x * 1.5;
 
  ServoAngle5 = map(gValuer.x, 0, 60, 0, 20);  
 
  myservo5.write(ServoAngle10);
 
//   Serial.println("angler.x:");
//   Serial.println(angler.x);
//   Serial.println("angler.y:");
//   Serial.println(angler.y);      
//   Serial.println("angler.z:");
//   Serial.println(angler.z); 
   
//   Serial.println("ServoAngle6:");
//   Serial.println(ServoAngle6);
//   Serial.println("ServoAngle7");
//   Serial.println(ServoAngle7);      
//   Serial.println("ServoAngle8");
//   Serial.println(ServoAngle8); 
//   Serial.println("ServoAngle9");
//   Serial.println(ServoAngle9); 
//   Serial.println("ServoAngle8a");
//   Serial.println(ServoAngle8a); 
//   Serial.println("ServoAngle9a");
//   Serial.println(ServoAngle9a); 

   Serial.println("ServoAngle10");
   Serial.println(ServoAngle10);

/* For g-values the corrected raws are used */
  Serial.println("g-xr      = ");
  Serial.println(gValuer.x);
//  Serial.print("  |  g-yr      = ");
//  Serial.print(gValuer.y);
//  Serial.print("  |  g-zr      = ");
//  Serial.println(gValuer.z);

delay(100);
}
