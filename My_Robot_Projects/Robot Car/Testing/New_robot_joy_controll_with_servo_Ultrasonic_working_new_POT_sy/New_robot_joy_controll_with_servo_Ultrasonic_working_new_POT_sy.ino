#include <Servo.h>
Servo myservo0;

int Echo = A4;
int Trig = A5;




  // Motor control pins : L298N H bridge

  #define enAPin  5 // Left motor PWM speed control
  #define in1Pin  7 // Left motor Direction 1
  #define in2Pin  8 // Left motor Direction 2
  #define in3Pin  9 // Right motor Direction 1
  #define in4Pin  11 // Right motor Direction 2
  #define enBPin  6 // Right motor PWM speed control

  #define carSpeed 150

  int rightDistance = 0, leftDistance = 0, middleDistance = 0;
  
void forward(){

  analogWrite (enAPin, carSpeed);
  analogWrite (enBPin, 160);
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

  void setup() 
  { 
myservo0.attach(3);
Serial.begin(9600);

  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);

  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  stop();
  
  } 

  
  void loop() 
  { 

myservo0.write(90);
delay(500);
middleDistance = Distance_test();

if (middleDistance <= 50){
  stop();
  delay(500);
  myservo0.write(10);
  delay(1000);
  rightDistance = Distance_test();

  delay(500);
  myservo0.write(90);
  delay(1000);
  myservo0.write(180);
  delay(1000);
  leftDistance = Distance_test();

  delay(500);
  myservo0.write(90);
  delay(100);
  if(rightDistance < leftDistance){
    right();
    delay(100);
  }
  else if (rightDistance > leftDistance){
    left();
    delay(100);
  }
  else if ((rightDistance <= 20) || (leftDistance <= 20)){
    back();
    delay(300);
  }
else{
  forward();
  }
}
else{
  forward();
  }
}


