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
#include <Arm_Configuration.h>
#include <Inverse_Kinematic.h>

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
double a1 = 0;   //mm
double a2 = 10;  //mm
double a3 = 20;  //mm
double alpha1 = 90;  //deg
double alpha2 = 0;   //deg
double alpha3 = 0;   //deg
double d1 = 0;   //mm
double d2 = 0;   //mm
double d3 = 0;   //mm
// X, Y, Z increment in millimeters each loop cycle
// it is referred to all possible direction for all sign of mouvement
int delta = 12; //mm
// Coordinates required of the end effector in the working space
double Wx;
double Wy;
double Wz;
double Wh;
// start and next angle of the joint of the robot
double actual_theta1 = 90;
double actual_theta2 = 90;
double actual_theta3 = 90;
double next_theta1 = 0.0;
double next_theta2 = 0.0;
double next_theta3 = 0.0;
// start and next position of the end effector 
double actual_x = 0.0;
double actual_y = 0.0;
double actual_z = 0.0;
double next_x = 0.0;
double next_y = 0.0;
double next_z = 0.0;
Inverse_Kinematic robot(a1, a2, a3, alpha1, alpha2, alpha3, d1, d2, d3);
 

void setup() {
  // Inizialize the servo motors and establish a connection with the output window
  base.attach(8);
  shoulder.attach(9);
  elbow.attach(10);
  hand.attach(11);
  Serial.begin(9600);  
  // Set the robot in the starting position
  base.write(actual_theta1);
  shoulder.write(actual_theta2);
  elbow.write(actual_theta3);
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
  // this values increment the next desired position of the robot
  next_x = actual_x + Wx;
  next_y = actual_y + Wy;
  next_z = actual_z + Wz;
  // the poit that needs to be reached is now known
  // use the inverse kinematic library to calculate the joint angles that are needed in order to reach taht point
  Arm_Configuration arm = robot.Calculate(next_x, next_y, next_z, actual_theta1, actual_theta2, actual_theta3);

  // extract the joint angles
  next_theta1 = arm.getTheta1();
  next_theta2 = arm.getTheta2();
  next_theta3 = arm.getTheta3();

  // apply the calculated anges to each joint
  base.write(next_theta1);
  shoulder.write(next_theta2);
  elbow.write(next_theta3);


  // update the parameters about the position of the robot for the next iteration
  actual_x = next_x;
  actual_y = next_y;
  actual_z = next_z;
  actual_theta1 = next_theta1;
  actual_theta2 = next_theta2;
  actual_theta3 = next_theta3;
}
