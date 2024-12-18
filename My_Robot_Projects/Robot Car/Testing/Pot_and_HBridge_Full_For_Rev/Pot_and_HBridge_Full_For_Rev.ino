
const int speedPot = A1;  

#define ENA 5
#define IN1 11
#define IN2 10

#define ENB 6
#define IN3 9
#define IN4 8


int startTouch = 4; // Start
int stopTouch = 5; // Stop

int ledRed = 9;
int ledGreen = 10;
int ledBlue = 11;

#define carSpeed 200

int potValue1 = 0; 




void setup() { 
 
   Serial.begin(9600);
  
  
//  myservo.attach(2);  // attach servo on pin 3 to servo object

     
//  pinMode(speedPot, INPUT);    
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

   pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT); 
  
  pinMode(startTouch, INPUT); 
  pinMode(stopTouch, INPUT);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  
 
//  stop();
} 

void loop() { 

  // read the value from the sensor:
  potValue1 = analogRead(speedPot);

  potValue1 = map(potValue1, 0, 1023, -255, 255);
if ((potValue1 >= 8) && (potValue1 <= 8))
{
potValue1 = 0;
} 

if (potValue1 >= 0)
{ 

  analogWrite(ENA, potValue1);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
  analogWrite(ENB, potValue1);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH); 

}


if (potValue1 <= 0)  
{

analogWrite(ENB, -potValue1);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);    
analogWrite(ENA, -potValue1);
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
}
 
}                 
