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
        Inverse_Kinematic(double = 0, double = 0, double = 0, double = 0, double = 0, double = 0, double = 0, double = 0, double = 0);
        Arm_Configuration Calculate(double, double, double, double, double, double);
    

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
    // Constants to keep the mechanical limitation of the robot joints angle in degrees
        const int MAX_JOINT_ANGLE = 180;
        const int MIN_JOINT_ANGLE = 0;
};

#endif