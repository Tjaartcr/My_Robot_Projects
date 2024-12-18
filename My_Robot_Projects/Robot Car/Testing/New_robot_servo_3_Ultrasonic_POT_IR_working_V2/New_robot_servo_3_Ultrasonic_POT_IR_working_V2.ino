#include <Servo.h>
Servo myservo0;



int Echo = A4;
int Trig = A5;

int EchoL = 10;
int TrigL = 4;

int EchoR = A3;
int TrigR = A2;



  // Motor control pins : L298N H bridge

  #define lazerPin 2
  
  #define enAPin  5 // Left motor PWM speed control
  #define in1Pin  7 // Left motor Direction 1
  #define in2Pin  8 // Left motor Direction 2
  #define in3Pin  9 // Right motor Direction 1
  #define in4Pin  11 // Right motor Direction 2
  #define enBPin  6 // Right motor PWM speed control

  #define carSpeed 150
  
  #define sensENA 12
  #define sensPin 13

    //
    const int leftLinePin = 22;
    const int rightLinePin = 23;
    const int centrePHsensPin = 25;
    const int centreENPin = 24;
  
  
  int rightDistance = 0, leftDistance = 0, middleDistance = 0;
  

  int val3 = 0;

  //
  int centreState = 0;

void forward(){

int potpin1 = A0;
int potpin2 = A1;

  int val1;
  int val2;

val1 = analogRead(potpin1);            // reads the value of the potentiometer (value between 0 and 1023) 
val1 = map(val1, 0, 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180) 

  val2 = analogRead(potpin2);            // reads the value of the potentiometer (value between 0 and 1023) 
  val2 = map(val2, 0, 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180) 



  analogWrite (enAPin, val1);
  analogWrite (enBPin, val2);
  digitalWrite (in1Pin, HIGH);
  digitalWrite (in2Pin, LOW);
  digitalWrite (in3Pin, LOW);
  digitalWrite (in4Pin, HIGH);

  
}

void back(){

  analogWrite (enAPin, carSpeed);
  analogWrite (enBPin, carSpeed);
  digitalWrite (in1Pin, LOW);
  digitalWrite (in2Pin, HIGH);
  digitalWrite (in3Pin, HIGH);
  digitalWrite (in4Pin, LOW);  
}

void left(){

    analogWrite (enAPin, 100);
  analogWrite (enBPin, 100);
  digitalWrite (in1Pin, LOW);
  digitalWrite (in2Pin, HIGH);
  digitalWrite (in3Pin, LOW);
  digitalWrite (in4Pin, HIGH);
}


void right(){

    analogWrite (enAPin, 100);
  analogWrite (enBPin, 100);
  digitalWrite (in1Pin, HIGH);
  digitalWrite (in2Pin, LOW);
  digitalWrite (in3Pin, HIGH);
  digitalWrite (in4Pin, LOW);
}

void stop(){

  digitalWrite (enAPin, LOW);
  digitalWrite (enBPin, LOW);
  
}


// UltraSonic Distance measurement SUB FUNCTION

int Distance_test(){
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  float Fdistance = pulseIn(Echo,HIGH);
  Fdistance = Fdistance / 58;
  return (int)Fdistance;
  
}

// UltraSonic Distance measurement SUB FUNCTION

int Distance_testL(){
  digitalWrite(TrigL, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigL, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigL, LOW);
  float FdistanceL = pulseIn(EchoL,HIGH);
  FdistanceL = FdistanceL / 58;
  return (int)FdistanceL;
  
}

// UltraSonic Distance measurement SUB FUNCTION

int Distance_testR(){
  digitalWrite(TrigR, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigR, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigR, LOW);
  float FdistanceR = pulseIn(EchoR,HIGH);
  FdistanceR = FdistanceR / 58;
  return (int)FdistanceR;
  
}


  void setup() 
  { 
myservo0.attach(3);
Serial.begin(9600);

  pinMode(lazerPin, OUTPUT);
  
  pinMode(sensPin, INPUT);
  pinMode(sensENA, OUTPUT);
  
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);

  pinMode(EchoL, INPUT);
  pinMode(TrigL, OUTPUT);

  pinMode(EchoR, INPUT);
  pinMode(TrigR, OUTPUT);
  
  
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  stop();
  
  digitalWrite(lazerPin, HIGH);
  
  pinMode (leftLinePin, INPUT);
  pinMode (rightLinePin, INPUT);
  pinMode (centrePHsensPin, INPUT);
  pinMode (centreENPin, OUTPUT);

  
  
  
  } 

  
  void loop() 
  { 

int FdistanceL; 
int FdistanceR;


  digitalWrite(sensENA, HIGH);
  val3 = digitalRead(sensPin);

//Serial.println(FdistanceL);
//Serial.println(FdistanceR);
//Serial.println(rightDistance);

//
  digitalWrite(centreENPin, HIGH);
centreState = !digitalRead(centrePHsensPin);
 
Serial.println(centreState);
 
//int FdistanceL = 0, FdistanceR = 0;

myservo0.write(90);
delay(100);

middleDistance = Distance_test();
FdistanceL = Distance_testL();
FdistanceR = Distance_testR();

forward();

if ((middleDistance < 40) || (centreState == HIGH)){         // || (leftDistance <= 20 ) || (rightDistance <= 20)){
stop();

  delay(500);
  myservo0.write(20);
  delay(500);
  rightDistance = Distance_test();
  delay(500);
  myservo0.write(90);
  delay(500);
  myservo0.write(160);
  delay(500);
  leftDistance = Distance_test();
  delay(500);
  myservo0.write(90);
  delay(10);
  
  if(rightDistance > leftDistance){
    left();
    delay(200);
  }
  else if (rightDistance < leftDistance){
    right();
    delay(200);
  }
  else if ((rightDistance <= 20) && (leftDistance <= 20)){
    back();
    delay(1000);
  }
else{
  stop();
//  delay(500);
  forward();
  }
}
else{
  stop();
//  delay(500);
  forward();
  }

}


