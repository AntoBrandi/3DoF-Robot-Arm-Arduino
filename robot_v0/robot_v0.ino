#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo1;
Servo myservo2;
Servo myservo3;

int potpin = 0;// analog pin used to connect the potentiometer
int potpin1 = 1;
int potpin2 = 2;
int potpin3 = 3;

int val;    // variable to read the value from the analog pin
int val1;
int val2;
int val3;

void setup() {
  Serial.begin(9600);
  myservo.attach(8);  // attaches the servo on pin 8 to the servo object
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  Serial.println("Servo "+String(potpin)+" = "+val);
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there

  val1 = analogRead(potpin1);    
  Serial.println("Servo " + String(potpin1) + " = " + val1);    
  val1 = map(val1, 0, 1023, 0, 180);     
  myservo1.write(val1);
  delay(15);  


  val2 = analogRead(potpin2);
  Serial.println("Servo "+String(potpin2)+" = "+val2);            
  val2 = map(val2, 0, 1023, 0, 180);     
  myservo2.write(val2);
  delay(15);    


  val3 = analogRead(potpin3);   
  Serial.println("Servo "+String(potpin3)+" = "+val3);         
  val2 = map(val3, 0, 1023, 0, 180);     
  myservo3.write(val3); 
  delay(15);
                            
}
