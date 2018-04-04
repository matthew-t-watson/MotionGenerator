#include "MotionGenerator.h"

#include <math.h>
#include <stdlib.h>

void calculateTrapezoidalProfile(MotionGeneratorData_t* data, float posRef, float dt);
short int sign(float aVal);

void init(MotionGeneratorData_t* data, float aMaxVel, float aMaxAcc, float aInitPos) {
    data->dt = 0;

    data->maxVel = aMaxVel;
    data->maxAcc = aMaxAcc;
    data->initPos = aInitPos;
	
	// State variables
	reset(data);
	
	// Misc
	data->signM = 1;		// 1 = positive change, -1 = negative change
	data->shape = true;   // true = trapezoidal, false = triangular
	data->isFinished = false;
}

float update(MotionGeneratorData_t* data, float posRef, float dt) {
		
	if (data->oldPosRef != posRef)  // reference changed
	{
		data->isFinished = false;
		// Shift state variables
		data->oldPosRef = posRef;
		data->oldPos = data->pos;
		data->oldVel = data->vel;
		
		// Calculate braking time and distance (in case is neeeded)
		data->tBrk = abs(data->oldVel) / data->maxAcc;
		data->dBrk = data->tBrk * abs(data->oldVel) / 2;
		
		// Caculate Sign of motion
		data->signM = sign(posRef - (data->oldPos + sign(data->oldVel)*data->dBrk));
		
		if (data->signM != sign(data->oldVel))  // means brake is needed
		{
			data->tAcc = (data->maxVel / data->maxAcc);
			data->dAcc = data->tAcc * (data->maxVel / 2);
		}
		else
		{
			data->tBrk = 0;
			data->dBrk = 0;
			data->tAcc = (data->maxVel - abs(data->oldVel)) / data->maxAcc;
			data->dAcc = data->tAcc * (data->maxVel + abs(data->oldVel)) / 2;
		}
		
		// Calculate total distance to go after braking
		data->dTot = abs(posRef - data->oldPos + data->signM*data->dBrk);
		
		data->tDec = data->maxVel / data->maxAcc;
		data->dDec = data->tDec * (data->maxVel) / 2;
		data->dVel = data->dTot - (data->dAcc + data->dDec);
		data->tVel = data->dVel / data->maxVel;
		
		if (data->tVel > 0)    // trapezoidal shape
			data->shape = true;
		else             // triangular shape
		{
			data->shape = false;
			// Recalculate distances and periods
			if (data->signM != sign(data->oldVel))  // means brake is needed
			{
				data->velSt = sqrt(data->maxAcc*(data->dTot));
				data->tAcc = (data->velSt / data->maxAcc);
				data->dAcc = data->tAcc * (data->velSt / 2);
			}
			else
			{
				data->tBrk = 0;
				data->dBrk = 0;
				data->dTot = abs(posRef - data->oldPos);      // recalculate total distance
				data->velSt = sqrt(0.5*data->oldVel*data->oldVel + data->maxAcc*data->dTot);
				data->tAcc = (data->velSt - abs(data->oldVel)) / data->maxAcc;
				data->dAcc = data->tAcc * (data->velSt + abs(data->oldVel)) / 2;
			}
			data->tDec = data->velSt / data->maxAcc;
			data->dDec = data->tDec * (data->velSt) / 2;
		}
		
	}
	
	// Calculate new setpoint
	calculateTrapezoidalProfile(data, posRef, dt);
	
	//calculateTrapezoidalProfile(setpoint);
	return data->pos;
}


void calculateTrapezoidalProfile(MotionGeneratorData_t* data, float posRef, float dt) {

	if (data->shape)   // trapezoidal shape
	{
		if (dt <= (data->tBrk+data->tAcc))
		{
			data->pos = data->oldPos + data->oldVel*dt + data->signM * 0.5*data->maxAcc*dt*dt;
			data->vel = data->oldVel + data->signM * data->maxAcc*dt;
			data->acc = data->signM * data->maxAcc;
		}
		else if (dt > (data->tBrk+data->tAcc) && dt < (data->tBrk+data->tAcc+data->tVel))
		{
			data->pos = data->oldPos + data->signM * (-data->dBrk + data->dAcc + data->maxVel*(dt-data->tBrk-data->tAcc));
			data->vel = data->signM * data->maxVel;
			data->acc = 0;
		}
		else if (dt >= (data->tBrk+data->tAcc+data->tVel) && dt < (data->tBrk+data->tAcc+data->tVel+data->tDec))
		{
			data->pos = data->oldPos + data->signM * (-data->dBrk + data->dAcc + data->dVel + data->maxVel*(dt-data->tBrk-data->tAcc-data->tVel) - 0.5*data->maxAcc*(dt-data->tBrk-data->tAcc-data->tVel)*(dt-data->tBrk-data->tAcc-data->tVel));
			data->vel = data->signM * (data->maxVel - data->maxAcc*(dt-data->tBrk-data->tAcc-data->tVel));
			data->acc = - data->signM * data->maxAcc;
		}
		else
		{
			data->pos = posRef;
			data->vel = 0;
			data->acc = 0;
			data->isFinished = true;
		}
	}
	else            // triangular shape
	{
		if (dt <= (data->tBrk+data->tAcc))
		{
			data->pos = data->oldPos + data->oldVel*dt + data->signM * 0.5*data->maxAcc*dt*dt;
			data->vel = data->oldVel + data->signM * data->maxAcc*dt;
			data->acc = data->signM * data->maxAcc;
		}
		else if (dt > (data->tBrk+data->tAcc) && dt < (data->tBrk+data->tAcc+data->tDec))
		{
			data->pos = data->oldPos + data->signM * (-data->dBrk + data->dAcc + data->velSt*(dt-data->tBrk-data->tAcc) - 0.5*data->maxAcc*(dt-data->tBrk-data->tAcc)*(dt-data->tBrk-data->tAcc));
			data->vel = data->signM * (data->velSt - data->maxAcc*(dt-data->tBrk-data->tAcc));
			data->acc = - data->signM * data->maxAcc;
		}
		else
		{
			data->pos = posRef;
			data->vel = 0;
			data->acc = 0;
			data->isFinished = true;
		}
		
	}

} 

// Getters and setters
bool getFinished(MotionGeneratorData_t* data) {
	return data->isFinished;
}

float getVelocity(MotionGeneratorData_t* data) {
	return data->vel;
}

float getAcceleration(MotionGeneratorData_t* data) {
	return data->acc;
}

void setMaxVelocity(MotionGeneratorData_t* data, float aMaxVel) {
	data->maxVel = aMaxVel;
}

void setMaxAcceleration(MotionGeneratorData_t* data, float aMaxAcc) {
	data->maxAcc = aMaxAcc;
}

void setInitPosition(MotionGeneratorData_t* data, float aInitPos) {
	data->initPos 	= aInitPos;
	data->pos 		= aInitPos;
	data->oldPos 		= aInitPos;
}

short int sign(float aVal) {
	if (aVal < 0)
		return -1;
	else if (aVal > 0)
		return 1;
	else
		return 0;
}

void reset(MotionGeneratorData_t* data) {
	// Reset all state variables	

	data->pos 		= data->initPos;
	data->oldPos 	= data->initPos;
	data->oldPosRef = 0;
	data->vel 		= 0;
	data->acc 		= 0;
	data->oldVel 	= 0;
	
	data->dBrk 		= 0;
	data->dAcc 		= 0;
	data->dVel 		= 0;
	data->dDec 		= 0;
	data->dTot 		= 0;
	
	data->tBrk 		= 0;
	data->tAcc 		= 0;
	data->tVel 		= 0;
	data->tDec 		= 0;
	
	data->velSt 		= 0;
}
