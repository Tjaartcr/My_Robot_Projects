

#define SpeedPotPin A2
int SpeedValue;

#define MotorTestPin A3
int MotorTestPinState;


#define LeftSelPin A14
int LeftSelState = 0;

#define RightSelPin A15
int RightSelState = 0;

#define LeftForRevPin 5
#define LeftSpeedPin 6
#define LeftMotorPowerOnPin 4

#define RightForRevPin 8
#define RightSpeedPin 9
#define RightMotorPowerOnPin 10


void setup() {
  // put your setup code here, to run once:
  pinMode (LeftSelPin, INPUT_PULLUP);
  pinMode (RightSelPin, INPUT_PULLUP);
  pinMode (MotorTestPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:

//**************************************************************************************************************
//                    TESTING OF MOTORS


  LeftSelState = digitalRead(LeftSelPin);
  RightSelState = digitalRead(RightSelPin);                                              

SpeedValue = analogRead(SpeedPotPin);
SpeedValue = map(SpeedValue, 0, 1023, 0, 255);

MotorTestPinState = digitalRead(MotorTestPin);

if (MotorTestPinState == LOW){
  if ((LeftSelState == LOW) && (RightSelState == HIGH)){

  
  
//    digitalWrite(RightForRevPin, HIGH);
//    digitalWrite(RightMotorPowerOnPin, LOW);
//
//      analogWrite (RightSpeedPin, 0);
//  

      analogWrite (LeftSpeedPin, SpeedValue);

    analogWrite(LeftForRevPin, 255);
    analogWrite(LeftMotorPowerOnPin, 255);
  
}

//else 

if ((RightSelState == HIGH) && (LeftSelState == HIGH)){
    
    analogWrite(LeftForRevPin, 0);
      analogWrite(LeftMotorPowerOnPin, 0);

    analogWrite(RightForRevPin, 0);
    analogWrite(RightMotorPowerOnPin, 0);

}


if ((RightSelState == LOW) && (LeftSelState == HIGH)){



//      analogWrite (LeftSpeedPin, 0);
//
//    digitalWrite(LeftForRevPin, HIGH);
//    digitalWrite(LeftMotorPowerOnPin, LOW);
  
  
    analogWrite(RightForRevPin, 255);
    analogWrite(RightMotorPowerOnPin, 255);

      analogWrite (RightSpeedPin, SpeedValue);
  
}
  }
}
