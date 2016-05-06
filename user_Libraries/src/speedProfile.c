/**
 *	Speed profile with PID
 */
 
#include "stdbool.h"
#include "stm32f4xx.h"
#include "speedProfile.h"
#include "global.h"
#include "encoder.h"
#include "pwm.h"
#include "config.h"
#include "maze.h"
#include <stdio.h>

float curSpeedX = 0;
float curSpeedW = 0;
int targetSpeedX = 0;
int targetSpeedW = 0;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputX = 0;
float pidInputW = 0;
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;
float kpX = 2;
float kdX = 4;
float kpW = 1;
float kdW = 12;						//used in straight, default 12
float kpW1 = 1;						//used for T1 and T3 in curve turn, default 1
float kdW1 = 26;
float kpW2 = 1;						//used for T2 in curve turn
float kdW2 = 36;
float accX = 50;					// acc/dec in mm/ms/ms
float decX = 50; 				 
float accW = 2; 					// cm/s^2
float decW = 2;	

int leftBaseSpeed = 0;
int rightBaseSpeed = 0;
int32_t leftEncCount = 0;
int32_t rightEncCount = 0;
int32_t leftEncOld = 0;
int32_t rightEncOld = 0;
int leftEncChange = 0;
int rightEncChange = 0;
int encChange = 0;
int32_t distanceLeft = 0;
int32_t encCount = 0;
int32_t oldEncCount = 0;
int sensorError = 0;
int sensorScale = 50;

int gyroFeedbackRatio = 5700;



void speedProfile(void)
{	
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();
}


// Updates encoder counts and distance left
void getEncoderStatus(void)
{
	leftEncCount = getLeftEncCount();
	rightEncCount = getRightEncCount();

	leftEncChange = leftEncCount - leftEncOld;
	rightEncChange = rightEncCount - rightEncOld;
	encChange = (leftEncChange + rightEncChange)/2;	 

	leftEncOld = leftEncCount;
	rightEncOld = rightEncCount;

	leftEncCount += leftEncChange;
	rightEncCount += rightEncChange;
	encCount = (leftEncCount + rightEncCount)/2;
	
	distanceLeft -= encChange;		// update distanceLeft
}


// Updates current speed based on target speed
void updateCurrentSpeed(void) {
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += ((accX*2)/100);
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= ((decX*2)/100);
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
}


void calculateMotorPwm(void) { // encoder PD controller
	int gyroFeedback = 0;
	int rotationalFeedback = 0;
	int sensorFeedback = 0;
	
  /* simple PD loop to generate base speed for both motors */	
	encoderFeedbackX = rightEncChange + leftEncChange;
	encoderFeedbackW = rightEncChange - leftEncChange;
	
	gyroFeedback = aSpeed/gyroFeedbackRatio; 	//gyroFeedbackRatio mentioned in curve turn lecture

	if(useOnlyGyroFeedback)
		rotationalFeedback = gyroFeedback;
	else if(useOnlyEncoderFeedback)
		rotationalFeedback = encoderFeedbackW;
	else
		rotationalFeedback = encoderFeedbackW + gyroFeedback;
	
	// option to include sensor feedback
		
	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - rotationalFeedback;
	
	if (useIRSensors) {
		getSensorError();
		sensorFeedback = sensorError/sensorScale;
		posErrorW += sensorFeedback;
	}
	
	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
	
	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;
	
	leftBaseSpeed = posPwmX - posPwmW;
	rightBaseSpeed = posPwmX + posPwmW;

	setLeftPwm(leftBaseSpeed);
	setRightPwm(rightBaseSpeed);	
}


/**
 *	Function: needToDecelerate
 *	Parameters: dist - encoder counts
 *				 			curSpd - counts/ms
 *				 			endSpd - counts/ms
 *	Return: positive deceleration in cm/s
 */
/*
int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd) 
{
	if (curSpd<0) 
		curSpd = -curSpd;	// take absolute value
	if (endSpd<0) 
		endSpd = -endSpd;
	if (dist<0) 
		dist = 1;			// prevent negative distance
	if (dist == 0) 
		dist = 1;  // prevent dividing by 0
	
	int estimatedDecX = ((curSpd*curSpd - endSpd*endSpd)*100/counts_to_mm(dist)/4);
	
	return (estimatedDecX < 0)? -estimatedDecX : estimatedDecX;	// cm/s/s
}
*/



void resetSpeedProfile(void)
{
	//resetEverything;
	
	//disable sensor data collecting functions running in 1ms interrupt
 	useIRSensors = 0;
	useSpeedProfile = 0;
	turnMotorOff;
	
	pidInputX = 0;
	pidInputW = 0;
	curSpeedX = 0;
	curSpeedW = 0;
	targetSpeedX = 0;
	targetSpeedW = 0;
	posErrorX = 0;
	posErrorW = 0;
	oldPosErrorX = 0;
	oldPosErrorW = 0;
  leftEncOld = 0;

	rightEncOld = 0;	
	leftEncCount = 0;
	rightEncCount = 0;
	encCount = 0;	
	oldEncCount = 0;
	leftBaseSpeed = 0;
	rightBaseSpeed = 0;
	distanceLeft = 0;

	resetLeftEncCount();
	resetRightEncCount();
}

float getDecNeeded (float d, float Vf, float Vi) {
	if (d <= 0) {
		d = 1;
	}
	
	return abs( (Vf*Vf - Vi*Vi)/d/8 );
}

// convert counts/ms to speed in mm/ms
float counts_to_mm (int counts) {
	return (counts/countspermm);
}


// convert speed in mm/s to counts/ms,
float mm_to_counts (float speed) {
	return (speed*countspermm);	// truncate from float to int
}


// get absolute value
float abs (float number) {
	return (number<0)? -number : number;
}


/**
 *	Straight movement
 */
void moveForward(int cells) {
	
	useIRSensors = 1;
	useSpeedProfile = 1;
	
	int startEncCount = (getLeftEncCount() + getRightEncCount()) / 2;
	int remainingDist = cells*cellDistance;

	while( encCount - startEncCount < cells*cellDistance ) {
		remainingDist = cells*cellDistance - (encCount - startEncCount);
		if (remainingDist < cellDistance/2) {
			useIRSensors = 0;
		}
		if (getDecNeeded(counts_to_mm(remainingDist), curSpeedX, stopSpeed) < decX) {
			targetSpeedX = moveSpeed;
		}
		else {
			targetSpeedX = stopSpeed;
		}
	}
	targetSpeedX = stopSpeed;
}

/**
 *	Move half cell at stopspeed
 */
void moveForwardHalf(void) {
	useIRSensors = 0;
	useSpeedProfile = 1;

	targetSpeedX = stopSpeed;
	int startEncCount = (getLeftEncCount() + getRightEncCount()) / 2;
	while( (getLeftEncCount() + getRightEncCount()) / 2 < startEncCount + cellDistance/2 ) {
		targetSpeedX = stopSpeed;
	}
	targetSpeedX = stopSpeed;
}


void getSensorError(void)
{
	// Both side walls
	if (LDSensor > LDvalue1 && RDSensor > RDvalue1)
		sensorError = RDSensor - LDSensor;
	// Closer to left wall
	if(LDSensor > LDvalue1)
		sensorError = LDMiddleValue - LDSensor;
	// Closer to right wall
	else if(RDSensor > RDvalue1)
		sensorError = RDSensor - RDMiddleValue;
	else
		sensorError = 0;
}

