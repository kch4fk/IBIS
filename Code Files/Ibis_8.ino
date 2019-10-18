/*
Kay Hutchinson 10/16/19
Ibis Robotic Camera Arm

Research project to augment an exisiting Raven II Surgical Robot in the Link
Lab at UVA.

10/18 - Adding serial read processing to control the arm instead of with the foot pedal box.
*/

// Libraries
#include <math.h>
#include <Kinematics.h>
#include <InverseKinematics.h>
#include <Servo.h>


// Create instances of Kinematics and Inverse Kinematics
Kinematics kin = Kinematics();
InverseKinematics invkin = InverseKinematics();


// Actuators and motors
Servo zoomServo;   // Linear actuator expects commands like a servo so it is commanded as a servo object. 
const int zoomPin = 9;    // linear actuator is on this pin

Servo topServo;   // Top servo
const int topServoPin = 10;   // top servo is on this pin

Servo bottomServo;    // Bottom servo
const int bottomServoPin = 11;    // bottom servo is on this pin

Servo baseServo;      // Base servo
const int baseServoPin = 6;      // base servo is on this pin

// Delay time to wait for actuators to move
const int actuatorUpdateTime = 25;

// Hardware limits as determined by testing. (units are degrees)
int zoomPosMin = 40;    // minimum position of linear actuator
int zoomPosMax = 145;   // maximum position of linear actuator
int zoomPos = 45;       // current linear actuator position

int servoPosMin = 10;   // 0.17 rad
int servoPosMid = 90;   // 1.57 rad
int servoPosMax = 170;  // 2.97 rad

double servoPosMinRad = 0.17; // rad
double servoPosMidRad = 1.57; // rad
double servoPosMaxRad = 2.97; // rad

//int topServoPos = 170;   // degrees, starting pos
//int bottomServoPos = 45;  // degrees, starting pos


// IO
const int ledPin = 13;

// zoom pedals
const int xInPedal = A2;
const int xOutPedal = A3;

int xInPedalState = 0;
int xOutPedalState = 0;


// tilt pedals
const int yUpPedal = A0;
const int yDownPedal = A1;

int yUpPedalState = 0;
int yDownPedalState = 0;

// pan pedals
const int panLeftPedal = A4;
const int panRightPedal = A5;

int panLeftPedalState = 0;
int panRightPedalState = 0;


//----------------------------------
// Arm and servo positions
// Robot state
double zoom = 0.00;             // inches

double tTopServoRad = servoPosMinRad;     // radians 
double tBottomServoRad = servoPosMaxRad;  // radians

double xLower = 0.00;           // inches
double yLower = 0.00;           // inches

double xUpper = 0.00;           // inches
double yUpper = 0.00;           // inches

double xCam = 0.00;             // inches
double yCam = 0.00;             // inches
double xCamVec = 0.00;          // inches
double yCamVec = 0.00;          // inches

double tBaseServoRad = 1.57;    // radians

char command;                   // command from serial
//----------------------------------

double scalingFactor = 1.00;    // arm movement scaling factor

unsigned long time; // keep track of milliseconds since the program began

// Set up
void setup() {
  Serial.begin(9600);
  
  zoomServo.attach(zoomPin);  // attaches the servo on pin 9 to the servo object
  topServo.attach(topServoPin);
  bottomServo.attach(bottomServoPin);
  baseServo.attach(baseServoPin);

// Set pin modes (INPUT or OUTPUT)
  pinMode(ledPin, OUTPUT);
  
  pinMode(xInPedal, INPUT);
  pinMode(xOutPedal, INPUT);
  
  pinMode(yUpPedal, INPUT);
  pinMode(yDownPedal, INPUT);
  
  pinMode(panLeftPedal, INPUT);
  pinMode(panRightPedal, INPUT);

  initIBIS();
  Serial.println("IBIS initialized");
}


void loop() {

// Always update kinematics-----------
//------------------------------------
  kinUpdate(zoom, tTopServoRad, tBottomServoRad);
//------------------------------------

// Print status of IBIS over serial
  Serial.print("Time: ");
  time = millis();
  Serial.println(time);

  /*
  Serial.println("Starting Pos");
  
  Serial.print("Zoom: ");
  Serial.print(zoom);
  Serial.println(" inches");

  Serial.print("Top Servo Pos: ");
  //Serial.println(topServoPos);
  Serial.print(tTopServoRad);
  Serial.println(" radians");

  Serial.print("Bottom Servo Pos: ");
  //Serial.println(bottomServoPos);
  Serial.print(tBottomServoRad);
  Serial.println(" radians");

*/

  Serial.print("Base Servo Pos: ");
  Serial.print(tBaseServoRad);
  Serial.println(" radians"); 

  /*
  Serial.print("Upper: ( ");
  Serial.print(xUpper);
  Serial.print(", ");
  Serial.print(yUpper);
  Serial.println(")");

  Serial.print("Lower: ( ");
  Serial.print(xLower);
  Serial.print(", ");
  Serial.print(yLower);
  Serial.println(")");
*/

  Serial.print("xCam: ");
  Serial.print(xCam);
  Serial.println(" inches");

  Serial.print("yCam: ");
  Serial.print(yCam);
  Serial.println(" inches");

  Serial.print("xCamVec: ");
  Serial.print(xCamVec);
  Serial.println(" inches");

  Serial.print("yCamVec: ");
  Serial.print(yCamVec);
  Serial.println(" inches");


//-------------------------------------

/*
// Read the pedals
  xInPedalState = digitalRead(xInPedal);
  xOutPedalState = digitalRead(xOutPedal);
  
  yUpPedalState = digitalRead(yUpPedal);
  yDownPedalState = digitalRead(yDownPedal);

  panLeftPedalState = digitalRead(panLeftPedal);
  panRightPedalState = digitalRead(panRightPedal);  
// 


  // Atuate in X direction
  if (xInPedalState == HIGH) {
    // turn LED on:
    //Serial.println("x in");
    xIn();
  }

  if (xOutPedalState == HIGH) {
    // turn LED on:
    //Serial.println("x out");
    xOut();
  }

  
  // Actuate in Y direction
  if (yUpPedalState == HIGH) {
    //Serial.println("y up");
    yUp();
  }

  if (yDownPedalState == HIGH) {
    //Serial.println("y down");
    yDown();
  }


  // Pan left and right
  if (panLeftPedalState == HIGH){
    //Serial.println("pan left");
    panLeft();
  }
  if (panRightPedalState == HIGH){
    //Serial.println("pan right");
    panRight();
  }

*/


// Instead of reading the pedals, accept commands from serial
  recvCommand();


/*
  Serial.println("Starting Pos");
  Serial.println(zoom);
  Serial.print(topServoPos);
  Serial.println(tTopServoRad);
  Serial.print(bottomServoPos);
  Serial.println(tBottomServoRad);
  
  Serial.println(xUpper);
  Serial.println(yUpper);

  */

  //cameraPosition(zoom, 0.00, 0.00);
  delay(10);
  Serial.println();
   
}


void recvCommand() {
  if(Serial.available() > 0) {
    command = Serial.read();
    react(command);
  }
}


void react(char command){
  switch(command) {
    case 'I':
      Serial.println("xIn");
      xIn();
      break;
    case 'O':
      Serial.println("xOut");
      xOut();
      break;
    case 'U':
      Serial.println("yUp");
      yUp();
      break;
    case 'D':
      Serial.println("yDown");
      yDown();
      break;
    case 'L':
      Serial.println("zLeft");
      panLeft();
      break;
    case 'R':
      Serial.println("zRight");
      panRight();
      break;
  }
}



void xIn(){
  //Serial.println("xIn function");
  tBottomServoRad = tBottomServoRad - 0.02;
  tBottomServoRad = constrain(tBottomServoRad, servoPosMinRad, servoPosMaxRad);
  
  double xUpperNext = xUpper + 0.25*scalingFactor;

  invkinUpdate(xUpperNext, yUpper, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);

  delay(actuatorUpdateTime);
}



void xOut(){
  //Serial.println("xOut function");
  tBottomServoRad = tBottomServoRad + 0.02;
  tBottomServoRad = constrain(tBottomServoRad, servoPosMinRad, servoPosMaxRad);
  
  double xUpperNext = xUpper - 0.25*scalingFactor;

  invkinUpdate(xUpperNext, yUpper, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);

  delay(actuatorUpdateTime);
}



void yUp(){
  //Serial.println("yUp function");
  tBottomServoRad = tBottomServoRad + 0.02;
  tBottomServoRad = constrain(tBottomServoRad, servoPosMinRad, servoPosMaxRad);

  double yUpperNext = yUpper + 0.25*scalingFactor;

  invkinUpdate(xUpper, yUpperNext, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);

  delay(actuatorUpdateTime);
}



void yDown(){
  //Serial.println("yDown function");
  tBottomServoRad = tBottomServoRad - 0.02;
  tBottomServoRad = constrain(tBottomServoRad, servoPosMinRad, servoPosMaxRad);

  double yUpperNext = yUpper - 0.25*scalingFactor;

  invkinUpdate(xUpper, yUpperNext, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);

  delay(actuatorUpdateTime);
  
}


void panLeft(){
  //Serial.println("pan left");
  tBaseServoRad = tBaseServoRad + 0.02;
  tBaseServoRad = constrain(tBaseServoRad, servoPosMinRad, servoPosMaxRad);

  baseServoMove(tBaseServoRad);
  delay(actuatorUpdateTime);
}

void panRight(){
  //Serial.println("pan right");
  tBaseServoRad = tBaseServoRad - 0.02;
  tBaseServoRad = constrain(tBaseServoRad, servoPosMinRad, servoPosMaxRad);

  baseServoMove(tBaseServoRad);
  delay(actuatorUpdateTime);
}

// Unfinished and unsed movement functionalities-------------------------------------------------
//-----------------------------------------------------------------------------------------------

/*
void zoomIn() {
  if (zoomPos < zoomPosMax) {
    zoomPos += 1;   // increments position of linear actuator
  }
  else {
    zoomPos = zoomPosMax;   // else, position is maximum and print feedbacks
    Serial.println("Can't zoom in any farther");   
  }
  zoomServo.write(zoomPos);
  delay(100);
}

void zoomOut() {
  if (zoomPos > zoomPosMin) {
    zoomPos -= 1;   // decrements position of linear actuator
  }
  else {
    zoomPos = zoomPosMin;   // else, position is maximum and print feedbacks
    Serial.println("Can't zoom out any farther");   
  }
  zoomServo.write(zoomPos);
  delay(100);
}

void tiltUp() {
  //bottomServoPos = constrain(bottomServoPos + 1, servoPosMin, servoPosMax);
  //tBottomServoRad = degToRad(bottomServoPos);

  tBottomServoRad = tBottomServoRad + 0.02;

  invkinUpdate(xUpper, yUpper, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);

  delay(100);
}

void tiltDown() {
  Serial.println("tiltDown fn");
  Serial.println(tBottomServoRad);
  //bottomServoPos = constrain(bottomServoPos - 1, servoPosMin, servoPosMax);
  //Serial.println(bottomServoPos);
  //tBottomServoRad = degToRad(bottomServoPos);
  tBottomServoRad = tBottomServoRad - 0.02;
  Serial.println(tBottomServoRad);

  invkinUpdate(xUpper, yUpper, tBottomServoRad);

  bottomServoMove(tBottomServoRad);
  topServoMove(tTopServoRad);
  zoomServo.write(zoomPos);
  
  delay(100);
}
*/
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------


void bottomServoMove(double tBottomServo){
  Serial.println("bottomServoMove");
  double angle = 180 - radToDeg(tBottomServo);
  angle = constrain(angle, servoPosMin, servoPosMax);
  bottomServo.write(angle);
  //Serial.println(angle);
  delay(actuatorUpdateTime);
}


void topServoMove(double tTopServo){
  Serial.println("topServoMove");
  double angle = 170 - radToDeg(tTopServo);
  angle = constrain(angle, servoPosMin, servoPosMax);
  topServo.write(angle);
  //Serial.println(angle);
  delay(actuatorUpdateTime);
}


void baseServoMove(double tBaseServo){
  Serial.println("baseServoMove");
  double angle = radToDeg(tBaseServo);
  angle = constrain(angle, servoPosMin, servoPosMax);
  baseServo.write(angle);
  //Serial.println(angle);
  delay(actuatorUpdateTime);  
}


// Initialize arm to fully retracted position
void initIBIS(){
  zoomServo.write(zoomPosMin);
  topServoMove(servoPosMinRad);   
  bottomServoMove(servoPosMaxRad);
  baseServoMove(servoPosMidRad);
  delay(500);

  //kinUpdate(0.00, 0.00, 0.00);
  
}

void kinUpdate(double z, double tTopServo, double tBottomServo){
  xLower = kin.xLowerCalc(tBottomServo);
  yLower = kin.yLowerCalc(tBottomServo);

  xUpper = kin.xUpperCalc(z, tTopServo, tBottomServo);
  yUpper = kin.yUpperCalc(z, tTopServo, tBottomServo);

  xCam = kin.xCamCalc(z, tTopServo, tBottomServo);
  yCam = kin.yCamCalc(z, tTopServo, tBottomServo);
  xCamVec = kin.xCamVectorCalc(z, tTopServo, tBottomServo);
  yCamVec = kin.yCamVectorCalc(z, tTopServo, tBottomServo);

  Serial.println("kinUpdate");
  //Serial.println(xUpper);
  //Serial.println(yUpper);
}

void invkinUpdate(double xUpper, double yUpper, double tBottomServo){
   //Serial.println("inv kin update"); 
   xLower = invkin.xLowerCalc(tBottomServo);
   yLower = invkin.yLowerCalc(tBottomServo);

   tTopServoRad = invkin.tTopServoCalc(xUpper, yUpper, tBottomServo);
   tTopServoRad = constrain(tTopServoRad, servoPosMinRad, servoPosMaxRad);
   
   zoom = invkin.zCalc(xUpper, yUpper, tBottomServo);
   zoomPos = zoomPosCalc(zoom);
   zoomPos = constrain(zoomPos, zoomPosMin, zoomPosMax);
   zoom = zoomCalc(zoomPos);

   kinUpdate(zoom, tTopServoRad, tBottomServoRad);
}


// Unit conversion functions
double radToDeg(double rad){
  double deg = rad*(180/M_PI);
  return deg;
}

double degToRad(double deg){
  double rad = deg*(M_PI/180);
  return rad;
}

// Calculation functions
// calculate zoom z in inches
double zoomCalc(double zoomPos){
  double z = 0.055*zoomPos - 2.4761;
  return z;
}

// calculate zoomPos in PWM duty cycle
double zoomPosCalc(double zoom){
  double zoomPos = 45.02 + 18.1818*zoom;
  return zoomPos;
}
