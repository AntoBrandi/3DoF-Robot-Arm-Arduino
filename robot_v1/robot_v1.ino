/*
 * 3 DoF ROBOT WITH INVERSE KINEMATIC CONTROL
 * 
 * This software version improves the v0 version in which the control of each join has been
 * done with some potentiometer. Here, the join space ha been separated from the working space
 * and the potentiometer now controls directly the end effector in the working space.
 * The join angle are calculate automaticcaly thanks to the transormation matrix
 * 
 */
#include <Servo.h>

// list of connected servo motors
Servo base;
Servo shoulder;
Servo elbow;
Servo hand;
// list of connected potentiometer
int joyX = 0; // move the end effector alongside x direction
int joyY = 1; // move the end effector alongside y direction
int joyZ = 2; // move the end effector alongside z direction
int joyH = 3; // move the hand of the end effector
// list of connected buttons
int btnR = 7; // button on the Right of the joypad
int btnL = 8; // button on the Left of thee joypad
// list of value read from the potentiometer
int joyXval;
int joyYval;
int joyZval;
int joyHval;
// list of values read from the buttons
int btnRval;
int btnLval;
// dimension of the robot 
// denavit-hartemberg values in mm and degrees
int a2 = 10;  //mm
int a3 = 20;  //mm
int alpha1 = 90;  //deg
// X, Y, Z increment in millimeters each loop cycle
// it is referred to all possible direction for all sign of mouvement
int delta = 12; //mm
// Coordinates required of the end effector in the working space
int Wx;
int Wy;
int Wz;
int Wh;


void setup() {
  // Inizialize the servo motors and establish a connection with the output window
  base.attach(8);
  shoulder.attach(9);
  elbow.attach(10);
  hand.attach(11);
  Serial.begin(9600);  
}

void loop() {
  // Read the inputs coming from the potentiometer and buttons
  joyXval = analogRead(joyX);
  joyYval = analogRead(joyY);
  joyZval = analogRead(joyZ);
  joyHval = analogRead(joyH);
  btnRval = digitalRead(btnR);
  btnLval = digitalRead(btnL);
  // convert the values that have been read from the potentiometer in 
  // an increment/decrement of the position of the end effector in all directions in milimeters
  Wx = map(joyXval, 0, 1023, -delta, delta);
  Wy = map(joyYval, 0, 1023, -delta, delta);
  Wz = map(joyZval, 0, 1023, -delta, delta);
  Wh = map(joyHval, 0, 1023, 0, 180); // no further calculation on this value, can be directly used
  // Data are now ready to use
  // convert Wx, Wy, Wz in joint variables parameters
  
  
  
}
