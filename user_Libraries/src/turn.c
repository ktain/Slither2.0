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
#include <stdio.h>

void pivotTurn(int degrees) {
	useIRSensors = 0;
	useSpeedProfile = 1;
	targetSpeedX = 0;
	
	int curt = millis();
	while(millis() - curt < turnDelay);

	angle = 0;
	if (degrees > 0)
		while( angle < degrees ) {
			targetSpeedW = -turnSpeed;
			delay_ms(1);
		}
	else
		while( angle > degrees ) {
			targetSpeedW = turnSpeed;
			delay_ms(1);
		}
	targetSpeedW = 0;
  curt = millis();
	while(millis() - curt < turnDelay);
	useSpeedProfile = 1;
}



void curveTurnRight(void) {
	useIRSensors = 0;
	useSpeedProfile = 1;
	targetSpeedX = stopSpeed;
		
	int curt = millis();

	while ( millis() - curt < 80 );
	
	targetSpeedW = -18*2;
	while (millis() - curt < 120 + 445);
	targetSpeedW = 0;
	while (millis() - curt < 120);
	
	curt = millis();
	while ( millis() - curt < 80 );
	
	useSpeedProfile = 1;
}

void curveTurnLeft(void) {
	useIRSensors = 0;
	useSpeedProfile = 1;
	targetSpeedX = stopSpeed;
		
	int curt = millis();

	while ( millis() - curt < 80 );

	targetSpeedW = 18*2;
	while (millis() - curt < 120 + 445);
	targetSpeedW = 0;
	while (millis() - curt < 120);
	
	curt = millis();
	while ( millis() - curt < 80 );
	
	useSpeedProfile = 1;
}
