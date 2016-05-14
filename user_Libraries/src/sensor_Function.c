#include "global.h"
#include "sensor_Function.h"
#include "adc.h"
#include "delay.h"
#include "pwm.h"
#include "led.h"
#include <stdio.h>
#include "encoder.h"
#include "buzzer.h"

int reflectionRate = 1000;	//which is 1.000 (converted to ingeter)

int32_t volMeter = 0;
int32_t voltage = 0;
int32_t LFSensor = 0;
int32_t RFSensor = 0;
int32_t LDSensor = 0;
int32_t RDSensor = 0;
int32_t LSSensor = 0;
int32_t RSSensor = 0;
int32_t Outz = 0;
int32_t aSpeed = 0;	//angular velocity
int32_t angle = 0; 
int expectedAngle;
int actualAngle;

/**
 *	Read Front and Diagonal Sensors
 */
void readSensor(void)
{
	u32 curt;
	
	LFSensor = read_LF_Sensor;
	RFSensor = read_RF_Sensor;
	LDSensor = read_LD_Sensor;
	RDSensor = read_RD_Sensor;	
	
	curt = micros();
	
	// Read left and right front sensors
	LEM_F_ON;
	REM_F_ON;
	elapseMicros(60,curt);	// default 60
	LFSensor = read_LF_Sensor - LFSensor;
	RFSensor = read_RF_Sensor - RFSensor;
	LEM_F_OFF;
	REM_F_OFF;
	if(LFSensor < 0)
		LFSensor = 0;
	if(RFSensor < 0)
		RFSensor = 0;
 	elapseMicros(250,curt); // default 140

	// Read left and right diagonal sensors
	LEM_D_ON;
	REM_D_ON;
	elapseMicros(340,curt);	// default 340
	LDSensor = read_LD_Sensor - LDSensor;
	RDSensor = read_RD_Sensor - RDSensor;
  LEM_D_OFF;
	REM_D_OFF;
	if(LDSensor < 0)
		LDSensor = 0;
	if(RDSensor < 0)
		RDSensor = 0;
	/*
	elapseMicros(400,curt); // default 140
	
	// Read left and right side sensors
	LEM_S_ON;
	REM_S_ON;
	elapseMicros(150,curt);	// default 340
	LSSensor = read_LS_Sensor - LSSensor;
	RSSensor = read_RS_Sensor - RSSensor;
  LEM_S_OFF;
	REM_S_OFF;
	if(LSSensor < 0)
		LSSensor = 0;
	if(RSSensor < 0)
		RSSensor = 0;
	*/
	
	LFSensor = LFSensor*reflectionRate/1000;
	RFSensor = RFSensor*reflectionRate/1000;
	LDSensor = LDSensor*reflectionRate/1000;
	RDSensor = RDSensor*reflectionRate/1000;
	//LSSensor = LSSensor*reflectionRate/1000;
	//RSSensor = RSSensor*reflectionRate/1000;

}


/**
 *	Read Gyro
 */
void readGyro(void)
{
	//int curt = micros();								// Uncomment when not in 1ms interrupt

	int i;
	int sampleNum = 20;
	aSpeed = 0;
	for(i = 0; i < sampleNum; i++){
		aSpeed += read_Outz;
	}
	
  aSpeed *= 50000/sampleNum;	// scale by 50000 before dividing by number of samples
	aSpeed -= aSpeedScale;					// 1866 scaled by 50000
	aSpeed /= 50000;						// readjust scale
	aSpeed *= -1;								// clockwise is positive
	angle += aSpeed;
	actualAngle += aSpeed;
	
	//while( (micros() - curt) < 1000 );	// Uncomment when not in 1ms ISR
}



/**
 *	Read Voltage Meter
 */
void readVolMeter(void)
{
	volMeter = read_Vol_Meter;	//raw value 2611 == 7.00V
	voltage = volMeter/.3105; 	//actual voltage value 7.00V == 7000
															// 10K || 30K
															// 1.75V == 7V == 0x0000087E
}

/**
 *	Low Battery Alert
 */
void lowBatCheck(void)
{
	readVolMeter();
	
	// Low battery threshold of 7.00V
  if(voltage < 7000)
	{	
		// Turn on all LEDs
		ALL_LED_ON;
	}
}

void setGyroRef(void) {
	double avg_Outz = 0;
	for (int i = 0; i < 1000; i++) {
		avg_Outz += read_Outz;
		delay_ms(1);
	}
	avg_Outz /= 1000;
	aSpeedScale = avg_Outz * 50000;
}
