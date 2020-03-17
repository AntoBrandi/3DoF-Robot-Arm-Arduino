/*
  Arm_Configuration.cpp - Test library for the computation
  of the inverse kinematic for antropomorphous robot with 3 DoF contorlled with Arduino
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

// include this library's description file
#include "Arm_Configuration.h"


// Constructor deinition with optional parameters.
// the aim is to create the transformation matrix using the denavit-hartemberg criteria
Arm_Configuration::Arm_Configuration(double theta1, double theta2, double theta3)
{
    //initialize the actual parameters of the class
    this._theta1 = theta1;
    this._theta2 = theta2;
    this._theta3 = theta3;
}


// getters method
double Arm_Configuration::getTheta1()
{
    return this._theta1;
}
double Arm_Configuration::getTheta2()
{
    return this._theta2;
}
double Arm_Configuration::getTheta3()
{
    return this._theta3;
}