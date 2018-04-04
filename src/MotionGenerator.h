#ifndef __MOTIONGENERATOR_H__
#define __MOTIONGENERATOR_H__

#include <stdbool.h>

/**
 * Generates the analytical solution for the trapezoidal motion.
 *
 * <p>
 * Usage:
 * // Includes
 * #include "MotionGenerator.h"
 *
 * Initialization
 *
 * @param int aVelMax maximum velocity (units/s)
 * @param int aAccMax maximum acceleration (units/s^2)
 * @param int aInitPos initial position (units)
 *

 // Define the MotionGenerator object
 MotionGenerator *trapezoidalProfile = new MotionGenerator(100, 400, 0);

 // Retrieve calculated position
 float positionRef = 1000;
 float position = trapezoidalProfile->update(positionRef)

 // Retrieve current velocity
 float velocity = trapezoidalProfile->getVelocity();

 // Retrieve current acceleration
 float acceleration = trapezoidalProfile->getAcceleration();

 // Check if profile is finished
 if (trapezoidalProfile->getFinished()) {};

 // Reset internal state
 trapezoidalProfile->reset();

 *
 * @author      AerDronix <aerdronix@gmail.com>
 * @web		https://aerdronix.wordpress.com/
 * @version     1.0 
 * @since       2016-12-22
 */

typedef struct
{
	float maxVel;
	float maxAcc;
	float initPos;
	float pos;
	float vel;
	float acc;
	float oldPos;
	float oldPosRef;
	float oldVel;

	float dBrk;
	float dAcc;
	float dVel;
	float dDec;
	float dTot;

	float tBrk;
	float tAcc;
	float tVel;
	float tDec;

	float velSt;

	float dt;

	short int signM;      	// 1 = positive change, -1 = negative change
	bool shape;      	// true = trapezoidal, false = triangular

	bool isFinished;
} MotionGeneratorData_t;


void init(MotionGeneratorData_t* data, float aMaxVel, float aMaxAcc, float aInitPos);
float update(MotionGeneratorData_t* data, float aPosRef, float dt);
float getVelocity(MotionGeneratorData_t* data);
float getAcceleration(MotionGeneratorData_t* data);

bool getFinished(MotionGeneratorData_t* data);
void setMaxVelocity(MotionGeneratorData_t* data, float aMaxVel);
void setMaxAcceleration(MotionGeneratorData_t* data, float aMaxAcc);
void setInitPosition(MotionGeneratorData_t* data, float aInitPos);
void reset(MotionGeneratorData_t* data);


#endif
