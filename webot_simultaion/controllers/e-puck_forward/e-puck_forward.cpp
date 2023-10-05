// File:          e-puck_forward.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left wheel motor");
Motor *rightMotor = robot->getMotor("right wheel motor");
Camera *cam = robot->getCamera();


int main(int argc, char **argv) {
 
 // get the motor devices
 
 // set the target position of the motors

 cam->enable(10);
 leftMotor->setPosition(INFINITY);
 rightMotor->setPosition(INFINITY);
 leftMotor->setVelocity(0.0);
 rightMotor->setVelocity(0.0);

 while (robot->step(TIME_STEP) != -1){
   leftMotor->setVelocity(0.5 * -MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
 };


 delete robot;

 return 0;
}