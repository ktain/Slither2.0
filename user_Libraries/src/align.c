/* Alignment */
#include "align.h"
#include "pwm.h"
#include "global.h"
#include "delay.h"
#include "encoder.h"
#include "sensor_Function.h"

void alignFrontWall(int LSensorVal, int RSensorVal, int duration){
	int startLeftEncCount = getLeftEncCount();
	int startRightEncCount = getRightEncCount();
	int tempPwm = maxPwm;
	maxPwm = alignPwm;
	useIRSensors = 1;
	useSpeedProfile = 0;
	int startTime = millis();
	while (millis() - startTime < duration) {
		int curt = micros();
		setLeftPwm(LSensorVal - LFSensor);
		setRightPwm(RSensorVal - RFSensor);
		while(micros() - curt < alignTime);
	}
	// Ignore encoder count changes
	setLeftEncCount(startLeftEncCount);
	setRightEncCount(startRightEncCount);
	maxPwm = tempPwm;
}
