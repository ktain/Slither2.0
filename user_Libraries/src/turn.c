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
#include <stdio.h>

void pivotTurn(int degrees) {
	useIRSensors = 0;
	useSpeedProfile = 1;
	targetSpeedX = 0;
	
	int tempAccW = accW;
	int tempDecW = decW;
	
	accW = 100;
	decW = 100;
	int curt = millis();
	while(millis() - curt < turnDelay);

	curt = millis();
	angle = 0;
	if (degrees > 0)
		while( angle < degrees ) {
			targetSpeedW = -turnSpeed;
			delay_ms(1);
			if (millis() - curt > 1000) {
				break;
			}
		}
	else
		while( angle > degrees ) {
			targetSpeedW = turnSpeed;
			delay_ms(1);
			if (millis() - curt > 1000) {
				break;
			}
		}
	
	targetSpeedW = 0;
	accW = tempAccW;
	decW = tempDecW;
  curt = millis();
	while(millis() - curt < turnDelay);
	useSpeedProfile = 1;
}



void curveTurnRight(void) {
	useSpeedProfile = 1;
	useIRSensors = 0;
	targetSpeedX = stopSpeed;
	
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
	
	curt = millis();
	while (millis() - curt < t3 + t4);
	
	useSpeedProfile = 1;
	
	//setLeftEncCount(startLeftEncCount);
	//setRightEncCount(startRightEncCount);
}

void curveTurnLeft(void) {
	useSpeedProfile = 1;
	useIRSensors = 0;
	targetSpeedX = stopSpeed;
	
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
	
	curt = millis();
	while (millis() - curt < t3 + t4);
	
	useSpeedProfile = 1;

	//setLeftEncCount(startLeftEncCount);
	//setRightEncCount(startRightEncCount);
}
