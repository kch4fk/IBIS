/*
  InverseKinematics.h - Library to calculate inverse kinematics of the IBIS.
  Description
  Kay Hutchinson 5/8/2019
*/

// include core Wiring API
//#include "WProgram.h"
// newer Arduinos don't use this anymore

// include this library's description file
#include "InverseKinematics.h"

// include description files for other libraries used (if any)
#include "HardwareSerial.h"

// include basic math functions
#include <math.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

InverseKinematics::InverseKinematics()
{
  // initialize this instance's variables
  rLower = 7.8125;

  // do whatever is required to initialize the library
  Serial.begin(9600);
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

// calculates x coordinate of the lower arm
double InverseKinematics::xLowerCalc(double tBottomServo){
  double x =  rLower * cos(tLowerCalc(tBottomServo));
  return x;
}

// calculates y coordinate of the lower arm
double InverseKinematics::yLowerCalc(double tBottomServo){
  double y = rLower * sin(tLowerCalc(tBottomServo));
  return y;
}

// calculate the required extension of the linear actuator
double InverseKinematics::zCalc(double xUpper, double yUpper, double tBottomServo){
	double z = sqrt((pow(rUpperCalc(xUpper, yUpper, tBottomServo), 2))-(pow(1.5, 2))) - 9.25;
	return z;
}

// calculate top servo angle
double InverseKinematics::tTopServoCalc(double xUpper, double yUpper, double tBottomServo){
	double t = tUpperCalc(xUpper, yUpper, tBottomServo) + (M_PI/2) + atan((9.25 + zCalc(xUpper, yUpper, tBottomServo))/1.5) - tBottomServo + 0.105;
	return t;
}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

// calculates the angle of the lower arm
double InverseKinematics::tLowerCalc(double tBottomServo){
  double t = tBottomServo - 0.105;
  return t;
}

// calculate difference between xUpper and xLower
double InverseKinematics::dxUpperArmCalc(double xUpper, double tBottomServo){
	double dx = xUpper - xLowerCalc(tBottomServo);
	return dx;
}

// calculate difference between yUpper and yLower
double InverseKinematics::dyUpperArmCalc(double yUpper, double tBottomServo){
	double dy = yUpper - yLowerCalc(tBottomServo);
	return dy;
}

// calculate required radius of the upper arm 
double InverseKinematics::rUpperCalc(double xUpper, double yUpper, double tBottomServo){
	double r = sqrt((pow(dxUpperArmCalc(xUpper, tBottomServo), 2))+(pow(dyUpperArmCalc(yUpper, tBottomServo), 2)));
	return r;
}

//calculate angle of the top part of the arm
double InverseKinematics::tTopCalc(double xUpper, double yUpper, double tBottomServo){
	double t = (M_PI/2) - atan((9.25 + zCalc(xUpper, yUpper, tBottomServo))/1.5);
	return t;
}

//calculate the angle of the upper part of the arm
double InverseKinematics::tUpperCalc(double xUpper, double yUpper, double tBottomServo){
	double t = atan(dyUpperArmCalc(yUpper, tBottomServo)/dxUpperArmCalc(xUpper, tBottomServo));
	return t;
}



