/*
Button 0
1. floodcenter. speed 70, acc 60/60, delay 200/150
2. floodcenter. speed 100, acc 60/60, delay 150/100
3. floodcenter. save. floodstart. speed 70, acc 60/60, delay 200/150
4. floodcenter. save. floodstart. speed 100, acc 60/60, delay 150/100

Button 1
1. speedrun pivot. load. floodstart. speed 200, acc 60/60, delay 100/50
2. speedrun pivot. load. floodstart. speed 300, acc 80/100, delay 50/50
3. speedrun pivot. load. floodstart. speed 500, acc 90/110, delay 50/50
4. speedrun pivot. load. floodstart. speed 500, acc 110/120, delay 0/40

Button 2
1. speedrun curve. load. floodstart. speed 200/60, acc 60/80
2. speedrun curve. load. floodstart. speed 300/100, acc 80/100
3. speedrun curve. load. floodstart. speed 500/100, acc 90/110
4. speedrun curve. load. floodstart. speed 550/100, acc 110/120

Button 3
1. save to flash
2. load from flash
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
int leftWallThreshold = 240;
int rightWallThreshold = 240;
int LDMiddleValue = 630;				
int RDMiddleValue = 630;
int leftPostThreshold = 120;
int rightPostThreshold = 100;
int postScale = 8;

int LFvalue1 = 3250;	// for front wall alignment, when mouse is at the center
int RFvalue1 = 3070;
int LFvalue2 = 460;		// for front wall detection during speedrun
int RFvalue2 = 460;		// 10mm

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
	resetSpeedProfile();
	delay_ms(100);
	
	switch (select) {
		case 0:
			alignPwm = 100;	
			turnSpeed = 40*2;
			moveSpeed = 100*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 200;
			turnDelay = 150;
			sensorScale = 60;
			postScale = 8;
			accX = 60;
			decX = 60;
		
			floodCenter();
			beep(3);
			break;
		case 1:
			alignPwm = 100;
			turnSpeed = 40*2;
			moveSpeed = 200*2;
			searchSpeed = 100*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 40;
			postScale = 12;
			accX = 60;
			decX = 60;
		
			floodCenter();
			beep(3);
			break;
		case 2:
			alignPwm = 100;
			turnSpeed = 40*2;
			moveSpeed = 200*2;
			searchSpeed = 70*2;
			stopSpeed = 0*2;
			alignTime = 200;
			turnDelay = 150;
			sensorScale = 40;
			postScale = 12;
			accX = 60;
			decX = 60;
		
			floodCenter();
			saveData();
			beep(3);
			floodStart();
			beep(3);
			break;
		case 3:
			alignPwm = 100;
			turnSpeed = 40*2;
			moveSpeed = 300*2;
			searchSpeed = 100*2;
			stopSpeed = 0*2;
			alignTime = 150;
			turnDelay = 100;
			sensorScale = 60;
			postScale = 12;
			accX = 60;
			decX = 60;
		
			floodCenter();
			saveData();
			beep(3);
			floodStart();
			beep(3);
			break;
		default:
			;
	}
	
}



void button1_interrupt(void) {
	shortBeep(200, 4000);
	printf("Button 1 pressed\n\r");
	delay_ms(1000);	
	
	waitForSignal();
	resetSpeedProfile();
	delay_ms(100);
	
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
			moveSpeed = 550*2;
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
	loadData();
	updateDistanceToStart();
	beep(3);
	
	resetSpeedProfile();
	alignPwm = 100;
	searchSpeed = 70*2;
	moveSpeed = 200*2;
	stopSpeed = 60*2;
	turnSpeed = 40*2;
	stopSpeed = 0*2;
	alignTime = 200;
	turnDelay = 150;
	sensorScale = 40;
	postScale = 12;
	accX = 60;
	decX = 60;
	
	floodStart();
	beep(3);
}



void button2_interrupt(void) {
	shortBeep(200, 4000);
	printf("Button 2 pressed\n\r");
	delay_ms(1000);
	
	waitForSignal();
	resetSpeedProfile();
	delay_ms(100);
	
		switch (select) {
		case 0:
			moveSpeed = 200*2;
			stopSpeed = 60*2;
			sensorScale = 25;
			postScale = 12;
			accX = 60;
			decX = 80;
		
			speedW = 65;
			t0 = 45;
			t1 = 30;
			t2 = 220;
			t3 = 30;
			t4 = 45;
		
			break;
		case 1:
			moveSpeed = 300*2;
			stopSpeed = 100*2;
			sensorScale = 25;
			postScale = 12;
			accX = 80;
			decX = 100;
		
			speedW = 120;
			t0 = 20;
			t1 = 40;
			t2 = 92;
			t3 = 40;
			t4 = 20;
			break;
		case 2:
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
		
			break;	
		case 3:
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

			break;			
		default:
			;
	}
	
	speedRunCurve();
	loadData();
	updateDistanceToStart();
	beep(3);
	
	alignPwm = 100;
	searchSpeed = 70*2;
	moveSpeed = 200*2;
	stopSpeed = 60*2;
	turnSpeed = 40*2;
	stopSpeed = 0*2;
	alignTime = 200;
	turnDelay = 150;
	sensorScale = 40;
	postScale = 12;
	accX = 60;
	decX = 60;
	
	floodStart();
	beep(3);
}



void button3_interrupt(void) {
	shortBeep(200, 4000);
	delay_ms(1000);
	
	printf("Button 3 pressed\n\r");
	
	switch (select) {
		case 0:
			saveData();
			beep(3);
			break;
		case 1:
			loadData();
			updateDistanceToCenter();
			visualizeGrid();
			beep(3);
			break;
		case 2:
			
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
	
}



void printInfo(void) {
	printf("LF %4d|LD %4d|RD %4d|RF %4d|LENC %9d|RENC %9d|voltage %4d|angle %4d|outZ %4d|Vref %4d|rate %4d\r\n",
					LFSensor, LDSensor, RDSensor, RFSensor, getLeftEncCount(), getRightEncCount(), voltage, angle, read_Outz, read_Vref, read_Rate);
}

