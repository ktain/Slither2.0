/* Turning functions */
#include "turn.h"
#include "pwm.h"
#include "global.h"
#include "delay.h"
#include "stm32f4xx.h"
#include "encoder.h"
#include "speedProfile.h"
#include "maze.h"
#include "sensor_Function.h"
#include "buzzer.h"
#include "config.h"
#include <stdio.h>

int pivotTurn(int degrees) {
	resetSpeedProfile();
	useIRSensors = 0;
	targetSpeedX = 0;
	useSpeedProfile = 1;
	
	int tempAccW = accW;
	int tempDecW = decW;
	int errorFlag = EXIT_SUCCESS;
	
	accW = 100;
	decW = 100;
	int curt = millis();
	while(millis() - curt < turnDelay);

	curt = millis();
	angle = 0;
	if (degrees > 0)
		while( angle < degrees ) {
			targetSpeedW = -turnSpeed;
			if (millis() - curt > 1000) {
				errorFlag = EXIT_FAILURE;
				break;
			}
		}
	else
		while( angle > degrees ) {
			targetSpeedW = turnSpeed;
			if (millis() - curt > 1000) {
				errorFlag = EXIT_FAILURE;
				break;
			}
		}
	
	targetSpeedW = 0;
	accW = tempAccW;
	decW = tempDecW;
		
  curt = millis();
	while(millis() - curt < turnDelay);
		
	useSpeedProfile = 1;
	return errorFlag;
}



void curveTurnRight(void) {
	useSpeedProfile = 1;
	useIRSensors = 0;
	targetSpeedX = stopSpeed;
	rightEncChange = leftEncChange = 0;
	
	//int startLeftEncCount = leftEncCount;
	//int startRightEncCount = rightEncCount;
	
	int curt = millis();
	while(millis() - curt < t0) {
		// Turn earlier if too close to front wall
		readSensor();
		if (LFSensor > LFvalue2 && RFSensor > RFvalue2) {
			shortBeep(50, 4000);
			break;
		}
		delay_ms(1);
	}
	
	targetSpeedW = -speedW;
	
	curt = millis();
	while (millis() - curt < t1 + t2);
	
	targetSpeedW = 0;
	
	expectedAngle = curveRight90;

	curt = millis();
	while (millis() - curt < t3 + t4);
	useSpeedProfile = 1;
	
	rightEncChange = leftEncChange = 0;
	//setLeftEncCount(startLeftEncCount);
	//setRightEncCount(startRightEncCount);
}

void curveTurnLeft(void) {
	useSpeedProfile = 1;
	useIRSensors = 0;
	targetSpeedX = stopSpeed;
	rightEncChange = leftEncChange = 0;
	
	//int startLeftEncCount = leftEncCount;
	//int startRightEncCount = rightEncCount;
	
	int curt = millis();
	while(millis() - curt < t0) {
		// Turn earlier if too close to front wall
		readSensor();
		if (LFSensor > LFvalue2 && RFSensor > RFvalue2)			{
			shortBeep(50, 4000);
			break;
		}
		delay_ms(1);
	}
	targetSpeedW = speedW;
	
	curt = millis();
	while (millis() - curt < t1 + t2);
	targetSpeedW = 0;
	
	expectedAngle = curveLeft90;
	curt = millis();
	while (millis() - curt < t3 + t4) {
	}
	
	useSpeedProfile = 1;
	rightEncChange = leftEncChange = 0;
	//setLeftEncCount(startLeftEncCount);
	//setRightEncCount(startRightEncCount);
}
