/*
  Arm_Configuration.h - Test library for the computation
  of the inverse kinematic for antropomorphous robot with 3 DoF contorlled with Arduino
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#ifndef Arm_Configuration_h
#define Arm_Configuration_h

// Cofiguration of an antropomorphous robot with 3 DoF in terms of joint angles
class Arm_Configuration
{
public:
    Arm_Configuration(double , double , double );
    double getTheta1();
    double getTheta2();
    double getTheta3();

private:
    // parameters
    double _theta1;
    double _theta2;
    double _theta3;
};


#endif