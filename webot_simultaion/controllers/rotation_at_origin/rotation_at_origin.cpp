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
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get the motor devices
 Motor *leftMotor = robot->getMotor("left wheel motor");
 Motor *rightMotor = robot->getMotor("right wheel motor");
 Camera *camera = robot->getCamera("camera");
 // set the target position of the motors
 camera->enable(10);
 leftMotor->setVelocity(0.5 * MAX_SPEED);
 rightMotor->setVelocity(0.5 * MAX_SPEED);
 leftMotor->setPosition(9.5);
 rightMotor->setPosition(-9.5);
 camera->saveImage("C:/機視作業/Simulation/picture.png", 100);
 while (robot->step(TIME_STEP) != -1);


 delete robot;

 return 0;
}