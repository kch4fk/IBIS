/*
  Kinematics.h - Library to calculate forward kinematics of the IBIS.
  Description
  Kay Hutchinson 5/8/2019
*/

// include core Wiring API
//#include "WProgram.h"
// newer Arduinos don't use this anymore

// include this library's description file
#include "Kinematics.h"

// include description files for other libraries used (if any)
#include "HardwareSerial.h"

// include basic math functions
#include <math.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

Kinematics::Kinematics()
{
  // initialize this instance's variables
  rLower = 7.8125;

  // do whatever is required to initialize the library
  Serial.begin(9600);
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

// calculates x coordinate of the lower arm
double Kinematics::xLowerCalc(double tBottomServo){
  double x =  rLower * cos(tLowerCalc(tBottomServo));
  return x;
}

// calculates y coordinate of the lower arm
double Kinematics::yLowerCalc(double tBottomServo){
  double y = rLower * sin(tLowerCalc(tBottomServo));
  return y;
}

// calculates x coordinate of the upper arm
double Kinematics::xUpperCalc(double z, double tTopServo, double tBottomServo){
  double x = rUpperCalc(z) * cos(tUpperCalc(z, tTopServo, tBottomServo)) + xLowerCalc(tBottomServo);
  return x;
}

// calculates y coordinate of the upper arm
double Kinematics::yUpperCalc(double z, double tTopServo, double tBottomServo){
  double y = rUpperCalc(z) * sin(tUpperCalc(z, tTopServo, tBottomServo)) + yLowerCalc(tBottomServo);
  return y;
}

// calculates camera angle
double Kinematics::tCamCalc(double z){
	double t = atan(1.5/(9.25+z));
	return t;
} 

// calculates x coordinate of the camera
double Kinematics::xCamCalc(double z, double tTopServo, double tBottomServo){
	double x = xUpperCalc(z, tTopServo, tBottomServo) + cos((-tUpperCalc(z, tTopServo, tBottomServo) + tCamCalc(z)));
	return x;
}

// calculates y coordinate of the camera
double Kinematics::yCamCalc(double z, double tTopServo, double tBottomServo){
	double y = yUpperCalc(z, tTopServo, tBottomServo) + -sin((-tUpperCalc(z, tTopServo, tBottomServo) + tCamCalc(z)));
	return y;
}

//calculates x value of the camera sight unit vector
double Kinematics::xCamVectorCalc(double z, double tTopServo, double tBottomServo){
	double x = cos(-tUpperCalc(z, tTopServo, tBottomServo) + tCamCalc(z));
	return x;
}

// calculates y value of the camera sight unit vector
double Kinematics::yCamVectorCalc(double z, double tTopServo, double tBottomServo){
	double y = -sin(-tUpperCalc(z, tTopServo, tBottomServo) + tCamCalc(z));
	return y;
}


// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

// calculates the angle of the lower arm
double Kinematics::tLowerCalc(double tBottomServo){
  double t = tBottomServo - 0.105;
  return t;
}

// calculates the radius of the upper arm
double Kinematics::rUpperCalc(double z){
  double r = sqrt(pow(1.5, 2)+ pow(9.25+z,2));
  return r;
}

// calculates the angle of the top part of the arm
double Kinematics::tTopCalc(double z){
  double t = (M_PI/2) - atan((9.25+z)/1.5);
  return t;
}

// calculates the angle of the upper arm
double Kinematics::tUpperCalc(double z, double tTopServo, double tBottomServo){
  double t = (M_PI/2) - atan((9.25+z)/1.5) + tTopServo - M_PI + tLowerCalc(tBottomServo);
  return t;
}


