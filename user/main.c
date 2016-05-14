/*
Button 0
1. floodcenter. speed 70, acc 60/60, delay 150/100
2. floodcenter. speed 100, acc 90/90, delay 150/100
3. floodcenter. save. floodstart. speed 70, acc 60/60, delay 150/100
4. floodcenter. save. floodstart. speed 100, acc 90/90, delay 150/100

Button 1
1. speedrun pivot speed 200, acc 60/60, delay 100/50
2. speedrun pivot speed 300, acc 80/80, delay 50/50
3. speedrun pivot speed 500, acc 90/90, delay 50/50
4. speedrun pivot speed 500, acc 100/100, delay 0/40

Button 2
1. speedrun curve. speed 200/60, acc 60/60
2. speedrun curve. speed 400/100, acc 60/60
3. speedrun curve. speed 500/100, acc 90/90
4. speedrun curve. speed 550/100, acc 120/120

Button 3
1. save
2. load
3. experimental
4. print info
*/

#include "main.h"

/* Maze.h settings */
unsigned char cell[SIZE][SIZE] = {0};  	//  ... 0 0 0 0       0 0 0 0
																				//         DE TRACE   W S E N
																				// [row] [col]
																				// [ y ] [ x ]
unsigned char distance[SIZE][SIZE] = {0};

int xPos = 0;   // 0-15
int yPos = 0;   // 0-15
int moveCount = 0;
int traceCount = 0;

/* Configure global variables */
int maxPwm;
int alignPwm;
int alignTime;
int turnDelay;
int times;

/* Configure search variables */
bool hasFrontWall = 0;
bool hasLeftWall = 0;
bool hasRightWall = 0;
int nextMove = 0;
char orientation = 'N';

/* Configure encoder settings */
int encResolution = 2048;				// counts/mrev
int gearRatio = 5;							// 5:1
float wheelCircumference = 73.5 ;	// mm
float wheelBase = 71;						// mm
int cellDistance = 24576;
int cellDistances[16];
float countspermm = 136;
int motorToBackDist = 26;

/* Configure speed profile options */
bool useIRSensors = 0;
bool useGyro = 0;
bool useGyroCorrection = 0;
bool usePID = 0;
bool useSpeedProfile = 0;

int moveSpeed;			// speed is in counts/ms, double of actual speed
int maxSpeed;			// call speed_to_counts(maxSpeed)
int turnSpeed;		
int searchSpeed;
int stopSpeed;


// Mouse state
bool isWaiting = 0;
bool isSearching = 0;
bool isSpeedRunning = 0;
bool isCurveTurning = 0;

// Sensor Thresholds
int frontWallThresholdL = 150;		// to detect presence of a front wall
int frontWallThresholdR = 120;
int leftWallThreshold = 200;
int rightWallThreshold = 200;
int LDMiddleValue = 630;				
int RDMiddleValue = 750;
int leftPostThreshold = 100;
int rightPostThreshold = 80;
int postScale = 8;

int LFvalue1 = 3215;	// for front wall alignment, when mouse is at the center
int RFvalue1 = 2864;
int LFvalue2 = 340;		// for front wall detection during speedrun
int RFvalue2 = 310;		// 10mm

int LDvalue1 = 430;		// side sensor PID threshold
int RDvalue1 = 530;

// Pivot turn profile
int	turnLeft90;
int	turnRight90;
int	turnLeft180;
int	turnRight180;
int turnLeft45;
int turnRight45;

// Curve turn profile
int curveLeft90;
int curveRight90;

int distances[100] = {0};

// Interface
int select = 0;

// Curve turn settings
int speedW;
int t0, t1, t2, t3, t4;

// Backup
unsigned char cell_backup[SIZE][SIZE] = {0};
unsigned char distance_backup[SIZE][SIZE] = {0};

// Gyro settings
int aSpeedScale = 93323000;

void systick(void) {
	
	// check voltage
	lowBatCheck();	// check if < 7.00V
	
	readGyro();
	
	// Collect data
	if(useIRSensors) {
		readSensor();
	}
	
	// Run speed profile (with PID)
	if(useSpeedProfile) {
		speedProfile();
	}
}


int main(void) {
	
	Systick_Configuration();
	LED_Configuration();
	button_Configuration();
	usart1_Configuration(9600);
  TIM4_PWM_Init();
	Encoder_Configuration();
	buzzer_Configuration();
	ADC_Config();
	
	ALL_EM_OFF;
	ALL_LED_OFF;

	delay_ms(1000);
	shortBeep(200, 4000);	// ms, frequency
	
	isWaiting = 1;
	
	
	//Initial Speed Profile
	maxPwm = 999;
	alignPwm = 0;
	moveSpeed = 0;
	maxSpeed = 0;			
	turnSpeed = 0;
	searchSpeed = 0;
	stopSpeed = 0;
	alignTime = 0;
	turnDelay = 0;
	
	turnLeft90 = -69200;
	turnRight90 = 69500;
	turnLeft180 = -143000;
	turnRight180 = 143000;
	turnLeft45 = -36000;
	turnRight45 = 36000;

	curveLeft90 = -58200;
	curveRight90 = 51200;

	resetSpeedProfile();
	angle = 0;
	
	while(1) {
		
		if (getLeftEncCount() > encResolution) {
			if (select == 3) {
				select = 0;
			}
			else {
				select++;
			}
			resetLeftEncCount();
			shortBeep(50, 1000);
		}
		else if (getLeftEncCount() < -encResolution)  {
			if (select == 0) {
				select = 3;
			}
			else {
				select--;
			}
			resetLeftEncCount();
			shortBeep(50, 1000);
		}
		
		switch(select) {
			case 0:
				LED1_ON;
				LED2_OFF;
				LED3_OFF;
				LED4_OFF;
				break;
			case 1:
				LED1_OFF;
				LED2_ON;
				LED3_OFF;
				LED4_OFF;
				break;
			case 2:
				LED1_OFF;
				LED2_OFF;
				LED3_ON;
				LED4_OFF;
				break;
			case 3:
				LED1_OFF;
				LED2_OFF;
				LED3_OFF;
				LED4_ON;
				break;
			default:
				;
		}
		
	}
}


void button0_interrupt(void) {
	shortBeep(200, 4000);
	printf("Button 0 pressed\n\r");
	delay_ms(1000);

	waitForSignal();
	
	initializeGrid();
	visualizeGrid();
	delay_ms(100);
	
	switch (select) {
		case 0:
			alignPwm = 100;	
			turnSpeed = 40*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 40;
			postScale = 12;
			accX = 60;
			decX = 60;
			floodCenter();
			break;
		case 1:
			alignPwm = 100;
			turnSpeed = 40*2;
			searchSpeed = 100*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 40;
			postScale = 12;
			accX = 90;
			decX = 90;
			floodCenter();
			break;
		case 2:
			alignPwm = 100;
			turnSpeed = 40*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 40;
			postScale = 12;
			accX = 60;
			decX = 60;
			floodCenter();
			saveData();
			floodStart();
			break;
		case 3:
			alignPwm = 100;
			turnSpeed = 40*2;
			searchSpeed = 100*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 40;
			postScale = 12;
			accX = 90;
			decX = 90;
			floodCenter();
			saveData();
			floodStart();
			break;
		default:
			;
	}

	printf("Finished Button 0 ISR\n\r");
	
}



void button1_interrupt(void) {
	shortBeep(200, 4000);
	printf("Button 1 pressed\n\r");
	delay_ms(1000);	
	
	waitForSignal();
	
	switch (select) {
		case 0:
			alignPwm = 100;
			moveSpeed = 200*2;
			turnSpeed = 40*2;
			stopSpeed = 0*2;
			alignTime = 100;
			turnDelay = 50;
			sensorScale = 40;
			postScale = 14;
			accX = 60;
			decX = 60;
		
			break;
		case 1:
			alignPwm = 100;
			moveSpeed = 300*2;
			turnSpeed = 40*2;
			stopSpeed = 0*2;
			alignTime = 50;
			turnDelay = 50;
			sensorScale = 40;
			postScale = 14;
			accX = 80;
			decX = 100;
		
			break;
		case 2:
			alignPwm = 100;
			moveSpeed = 500*2;		
			turnSpeed = 40*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 50;
			turnDelay = 50;
			sensorScale = 40;
			postScale = 14;
			accX = 90;
			decX = 110;
		
			break;	
		case 3:
			alignPwm = 100;
			moveSpeed = 500*2;
			turnSpeed = 40*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 0;
			turnDelay = 40;
			sensorScale = 40;
			postScale = 14;
			accX = 110;
			decX = 120;
		
			break;			
		default:
			;
	}
	
	speedRun();
	
	printf("Finished Button 1 ISR\n\r");
	
}



void button2_interrupt(void) {
	shortBeep(200, 4000);
	printf("Button 2 pressed\n\r");
	delay_ms(1000);
	
	waitForSignal();
	printf("Expected angle %d, actual angle %d\n\r", expectedAngle, actualAngle);
	
		switch (select) {
		case 0:
			resetSpeedProfile();
		
			moveSpeed = 200*2;
			stopSpeed = 60*2;
			sensorScale = 25;
			postScale = 12;
			accX = 60;
			decX = 90;
		
			speedW = 65;
			t0 = 45;
			t1 = 30;
			t2 = 220;
			t3 = 30;
			t4 = 45;
		
			speedRunCurve();
		
			break;
		case 1:
			resetSpeedProfile();
		
			moveSpeed = 400*2;
			stopSpeed = 60*2;
			sensorScale = 25;
			postScale = 12;
			accX = 90;
			decX = 110;
		
			speedW = 65;
			t0 = 45;
			t1 = 30;
			t2 = 220;
			t3 = 30;
			t4 = 45;
		
			speedRunCurve();
		
			break;
		case 2:
			
			resetSpeedProfile();
		
			moveSpeed = 500*2;
			stopSpeed = 100*2;
			sensorScale = 25;
			postScale = 12;
			accX = 90;
			decX = 110;
		
			speedW = 120;
			t0 = 20;
			t1 = 40;
			t2 = 92;
			t3 = 40;
			t4 = 20;

			speedRunCurve();
		
			break;	
		case 3:
			/*
			resetSpeedProfile();
		
			moveSpeed = 1000*2;
			stopSpeed = 60*2;
			sensorScale = 40;
			accX = 120;
			decX = 130;
		
			speedW = 81;
			t0 = 60;
			t1 = 40;
			t2 = 164;
			t3 = 40;
			t4 = 60;
		
			speedRunCurve();
			*/
		
			resetSpeedProfile();
		
			moveSpeed = 550*2;
			stopSpeed = 100*2;
			sensorScale = 25;
			postScale = 12;
			accX = 110;
			decX = 120;
		
			speedW = 120;
			t0 = 20;
			t1 = 40;
			t2 = 92;
			t3 = 40;
			t4 = 20;

			speedRunCurve();
			break;			
		default:
			;
	}
	
	printf("Expected angle %d, actual angle %d\n\r", expectedAngle, actualAngle);
	printf("Finished Button 2 ISR\n\r");
}



void button3_interrupt(void) {
	shortBeep(200, 4000);
	delay_ms(1000);
	
	printf("Button 3 pressed\n\r");
	
	switch (select) {
		case 0:
			/*
			initializeGrid();
			visualizeGrid();
			delay_ms(100);
		
			alignPwm = 100;	
			turnSpeed = 40*2;
			searchSpeed = 100*2;
			stopSpeed = 55*2;
			alignTime = 100;
			turnDelay = 50;
			sensorScale = 30;
			accX = 60;
			decX = 60;
		
			floodCenterCurve();
			break;	
			*/
		
			saveData();
			beep(3);
			break;
		case 1:
			loadData();
		
			beep(3);
			break;
		case 2:
			
			/*
			resetSpeedProfile();
			actualAngle = 0;
			printf("actual angle %d\n\r", actualAngle);
		
			moveSpeed = 500*2;
			stopSpeed = 100*2;
			sensorScale = 40;
			accX = 60;
			decX = 60;
		
			speedW = 120;
			t0 = 20;
			t1 = 40;
			t2 = 92;
			t3 = 40;
			t4 = 20;
		
			moveForward(1);
			curveTurnLeft();
			//moveForward(1);
			//curveTurnLeft();
			stopSpeed = 0;
			moveForward(1);
			turnMotorOff;
			useSpeedProfile = 0;

			printf("actual angle %d\n\r", actualAngle);
			break;
			*/
			
			
			resetSpeedProfile();
		
			alignPwm = 100;
			turnSpeed = 40*2;
			searchSpeed = 60*2;
			alignTime = 100;
			turnDelay = 50;
			accX = 90;
			decX = 90;
			stopSpeed = 60*2;
			sensorScale = 40;
			
			accW = 8;
			decW = 8;
		
			speedW = 81;
			t0 = 60;
			t1 = 40;
			t2 = 164;
			t3 = 40;
			t4 = 60;
			
			
			setGyroRef();
			useGyro = 0;
			
			moveE();
			delay_ms(100);
			moveS();
			delay_ms(100);
			moveW();
			delay_ms(100);
			moveN();
		
			delay_ms(1000);
		
			moveW();
			delay_ms(100);
			moveS();
			delay_ms(100);
			moveE();
			delay_ms(100);
			moveN();	
		
			delay_ms(1000);
		
			moveS();
			delay_ms(100);
			moveN();
			
			useSpeedProfile = 0;
			turnMotorOff;
			
			useGyro = 0;
			
			break;	
			
			
		case 3:
			resetSpeedProfile();
			delay_ms(1000);
			angle = 0;
			setGyroRef();
			while(1) {
				readSensor();
				printInfo();
				delay_ms(1);
				if (getLeftEncCount() > gearRatio*encResolution || getLeftEncCount() < -gearRatio*encResolution) {
					break;
				}
			}
			delay_ms(100);
			break;
		default:
			;
	}
	
	printf("Finished Button 3 ISR\n\r");
	
}



void printInfo(void) {
	printf("LF %4d|LD %4d|RD %4d|RF %4d|LENC %9d|RENC %9d|voltage %4d|angle %4d|outZ %4d|Vref %4d|rate %4d\r\n",
					LFSensor, LDSensor, RDSensor, RFSensor, getLeftEncCount(), getRightEncCount(), voltage, angle, read_Outz, read_Vref, read_Rate);
}

