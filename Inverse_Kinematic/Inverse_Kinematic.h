/*
  Inverse_Kinematic.h - Test library for the computation
  of the inverse kinematic for antropomorphous robot with 3 DoF contorlled with Arduino
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#ifndef Inverse_Kinematic_h
#define Inverse_Kinematic_h

#include "Arm_Configuration.h"


class Inverse_Kinematic
{
    // user-accessible "public" doubleerface
    public:
        Inverse_Kinematic(double a1 = 0, double a2 = 0, double a3 = 0, double alpha1 = 0, double alpha2 = 0, double alpha3 = 0, double d1 = 0, double d2 = 0, double d3 = 0);
        void Calculate(double x, double y, double z, double stepSize);
    

    // library-accessible "private" doubleerface
    private:
    // parameters for the denavit-hartemberg criteria
        double _a1;
        double _a2;
        double _a3;
        double _alpha1;
        double _alpha2;
        double _alpha3;
        double _d1;
        double _d2;
        double _d3;
    // parameters for the next position in cardesian coordinates in the working space
        double _x;
        double _y;
        double _z;
        double _stepSize;
    // output array of the calculated joint angles
    // only 3 output angles because the robot is a 3 DoF arm
        Arm_Configuration _armConfiguration;
    // Constants to keep the mechanical limitation of the robot joints angle in degrees
        const int MAX_JOINT_ANGLE = 180;
        const int MIN_JOINT_ANGLE = 0;
};