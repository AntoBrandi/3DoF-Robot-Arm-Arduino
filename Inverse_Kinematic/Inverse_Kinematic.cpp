/*
  Inverse_Kinematic.cpp - Test library for the computation
  of the inverse kinematic for antropomorphous robot with 3 DoF contorlled with Arduino
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
// include the library for math calculation
#include <math.h>
#include <stdlib.h>
// include this library's description file
#include "Inverse_Kinematic.h"

// Constructor deinition with optional parameters.
// the aim is to create the transformation matrix using the denavit-hartemberg criteria
Inverse_Kinematic::Inverse_Kinematic(double a1, double a2, double a3, double alpha1, double alpha2, double alpha3, double d1, double d2, double d3)
{
    //initialize the actual parameters of the class
    _a1 = a1;
    _a2 = a2;
    _a3 = a3;
    _alpha1 = alpha1;
    _alpha2 = alpha2;
    _alpha3 = alpha3;
    _d1 = d1;
    _d2 = d2;
    _d3 = d3;
}

// execute the calculation for the joint angles given the cartesian coordinates
Arm_Configuration Inverse_Kinematic::Calculate(double x, double y, double z, double actualTheta1, double actualTheta2, double actualTheta3)
{
    double c3;
    double s3_p;
    double s3_n;
    double theta3_I;
    double theta3_II;
    double theta2_I;
    double theta2_II;
    double theta2_III;
    double theta2_IV;
    double theta1_I;
    double theta1_II;
    

    // First check if the position is reachable based on the length of the arms
    if (abs(_a2 - _a3) <= (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))) && (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))) <= (_a2 + _a3)) {
        // Then check if the required position occurs in a singularity
        if (x == 0 && y == 0) {
            return;
        }
        // solve the inverse kinematic equation
        // theta 3
        c3 = (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(_a2, 2) - pow(_a3, 2)) / (2 * _a2 * _a3);
        s3_p = sqrt(1 - pow(c3, 2));
        s3_n = -s3_p;
        theta3_I = atan2(s3_p, c3);
        theta3_II = -theta3_I;
        // theta 2
        theta2_I = atan2(
            ((_a2 + (_a3 * c3)) * z - (_a3 * s3_p * sqrt(pow(x, 2) + pow(y, 2))))
            , (((_a2 + (_a3 * c3)) * sqrt(pow(x, 2) + pow(y, 2))) + (_a3 * s3_p * z))
        );
        theta2_II = atan2(
            ((_a2 + (_a3 * c3)) * z + (_a3 * s3_p * sqrt(pow(x, 2) + pow(y, 2))))
            , (-((_a2 + (_a3 * c3)) * sqrt(pow(x, 2) + pow(y, 2))) + (_a3 * s3_p * z))
        );
        theta2_III = atan2(
            ((_a2 + (_a3 * c3)) * z - (_a3 * s3_n * sqrt(pow(x, 2) + pow(y, 2))))
            , (((_a2 + (_a3 * c3)) * sqrt(pow(x, 2) + pow(y, 2))) + (_a3 * s3_n * z))
        );
        theta2_IV = atan2(
            ((_a2 + (_a3 * c3)) * z + (_a3 * s3_n * sqrt(pow(x, 2) + pow(y, 2))))
            , (-((_a2 + (_a3 * c3)) * sqrt(pow(x, 2) + pow(y, 2))) + (_a3 * s3_n * z))
        );
        // theta 1
        theta1_I = atan2(y, x);
        theta1_II = atan2(-y, -x);
        
        // list of all the possible configurations
        Arm_Configuration config_I(theta1_I, theta2_I, theta3_I);
        Arm_Configuration config_II(theta1_I, theta2_III, theta3_II);
        Arm_Configuration config_III(theta1_II, theta2_II, theta3_I);
        Arm_Configuration config_IV(theta1_II, theta2_IV, theta3_II);

        // select the best configuration based on the mechanical limit
        if (abs(actualTheta1 - theta1_I) < abs(actualTheta1 - theta1_II)) {
            // config_I or config_II
            if (abs(actualTheta2 - theta2_I) < abs(actualTheta2 - theta2_III)) {
                // config_I
                return config_I;
            }
            else {
                // config_II
                return config_II;
            }
        }
        else {
            // config_III or config_IV
            if (abs(actualTheta2 - theta2_II) < abs(actualTheta2 - theta2_IV)) {
                // config_III
                return config_III;
            }
            else {
                // config_IV
                return config_IV;
            }
        }  
    }
    else {
        return;
    }
}


