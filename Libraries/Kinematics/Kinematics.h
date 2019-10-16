/*
  Kinematics.h - Library to calculate forward kinematics of the IBIS.
  Description
  Kay Hutchinson 5/8/2019
*/

// ensure this library description is only included once
#ifndef Kinematics_h
#define Kinematics_h

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
class Kinematics
{
  // user-accessible "public" interface
  public:
    Kinematics(); // constructor
	
	double rLower; // lower arm length (constant) 
	
	double xLowerCalc(double tBottomServo);  // calculate x coordinate of the first joint
	double yLowerCalc(double tBottomServo);  // calculate y coordinate of the first joint
	double xUpperCalc(double z, double tTopServo, double tBottomServo);  // calculate x coordinate of the top of the arm
	double yUpperCalc(double z, double tTopServo, double tBottomServo);  // calculate y coordinate of the top of the arm

	double tCamCalc(double z);  // calculates camera angle
	double xCamCalc(double z, double tTopServo, double tBottomServo);  // calculates x coordinate of the camera
	double yCamCalc(double z, double tTopServo, double tBottomServo);  // calculates y coordinate of the camera
	double xCamVectorCalc(double z, double tTopServo, double tBottomServo);  //calculates x value of the camera sight unit vector
	double yCamVectorCalc(double z, double tTopServo, double tBottomServo);  // calculates y value of the camera sight unit vector

  // library-accessible "private" interface
  private:
	
	double tLowerCalc(double tBottomServo);  // calculate angle of the lower arm
	double rUpperCalc(double z);			 // calculate radius of the upper arm
	double tTopCalc(double z);				 // calculate angle of the upper arm
	double tUpperCalc(double z, double tTopServo, double tBottomServo);  // calculate angle of the upper arm 
	
};

#endif

