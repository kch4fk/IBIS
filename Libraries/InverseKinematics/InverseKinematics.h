/*
  InverseKinematics.h - Library to calculate inverse kinematics of the IBIS.
  Description
  Kay Hutchinson 5/8/2019
*/

// ensure this library description is only included once
#ifndef InverseKinematics_h
#define InverseKinematics_h

// Newer Arduinos use "Arduino.h"
#if ARDUINO >= 100
// include types & constants of Wiring core API
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

// include the basic math.h library
#include <math.h>

// library interface description
class InverseKinematics
{
  // user-accessible "public" interface
  public:
    InverseKinematics(); // constructor
	
	double rLower; // lower arm length (constant) 
	
	double xLowerCalc(double tBottomServo);  // calculate x coordinate of the first joint
	double yLowerCalc(double tBottomServo);  // calculate y coordinate of the first joint
	
	double zCalc(double xUpper, double yUpper, double tBottomServo);  // calculate the required extension of the linear actuator
	
	double tTopServoCalc(double xUpper, double yUpper, double tBottomServo);  // calculate top servo angle
	
  // library-accessible "private" interface
  private:
	
	double tLowerCalc(double tBottomServo);  // calculate angle of the lower arm
	double dxUpperArmCalc(double xUpper, double tBottomServo);			 // calculate difference between xUpper and xLower
	double dyUpperArmCalc(double yUpper, double tBottomServo);			 // calculate difference between yUpper and yLower
	double rUpperCalc(double xUpper, double yUpper, double tBottomServo);  // calculate required radius of the upper arm 
	double tTopCalc(double xUpper, double yUpper, double tBottomServo);  //calculate angle of the top part of the arm
	double tUpperCalc(double xUpper, double yUpper, double tBottomServo);  //calculate the angle of the upper part of the arm
};

#endif

