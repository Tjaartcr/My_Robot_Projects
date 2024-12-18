
int speedPot = A0;  

#define ENA 8
#define IN1 7
#define IN2 6

int startTouch = 4; // Start
int stopTouch = 5; // Stop

int ledRed = 9;
int ledGreen = 10;
int ledBlue = 11;

#define carSpeed 200

int potValue1 = 0; 



void forward(){ 
if (startTouch == HIGH)
{ 
  analogWrite(ENA, potValue1);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
}



void stop() {
if (stopTouch == HIGH)
{ 
  digitalWrite(ENA, LOW);
//                                                                              digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 

}

void setup() { 
 
   Serial.begin(9600);
  
  
  myservo.attach(2);  // attach servo on pin 3 to servo object

     
//  pinMode(speedPot, INPUT);    
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
  pinMode(startTouch, INPUT); 
  pinMode(stopTouch, INPUT);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  
 
  stop();
} 

void loop() { 

  // read the value from the sensor:
  potValue1 = analogRead(speedPot);

map potValue1 (

    }                     
