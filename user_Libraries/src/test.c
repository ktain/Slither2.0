/* Test functions */
#include "stm32f4xx.h"
#include "test.h"
#include "delay.h"
#include "sensor_Function.h"
#include "pwm.h"
#include "led.h"
#include "encoder.h"
#include "global.h"
#include "speedProfile.h"
#include "config.h"
#include "turn.h"
#include "buzzer.h"
#include "align.h"
#include "maze.h"
#include <stdio.h>


/**
 *	Hug Front Wall
 */
void hugFrontWall(int LSensorVal, int RSensorVal) {
	while (1) {
		int curt = micros(); //start to track time in order to make one adjust every 1000us
		readSensor();
		setLeftPwm(LSensorVal - LFSensor);
		setRightPwm(RSensorVal - RFSensor);
		elapseMicros(1000, curt); //elapse 1000 micro seconds
	}
}


void ledTest (void) {
	if (LFSensor > frontWallThresholdL)
		LED2_ON;
	else
		LED2_OFF;
	
	if (LDSensor > leftWallThreshold)
		LED1_ON;
	else
		LED1_OFF;
	
	if (RDSensor > rightWallThreshold)
		LED4_ON;
	else
		LED4_OFF;
	
	if (RFSensor > frontWallThresholdR)
		LED3_ON;
	else
		LED3_OFF;
}

/*	
		Function: wheelOffsetTest()
		Parameters: motor speed, ontime
		Return: right - left encoder count
 */
int wheelOffsetTest(int speed, int ontime) {
	resetLeftEncCount();
	resetRightEncCount();

	setLeftPwm(speed);
	setRightPwm(speed);    
	delay_ms(ontime);
	turnMotorOff; 
	delay_ms(100);
	
	return getRightEncCount() - getLeftEncCount();
}



/*
 * Random movements using pivot turns
 */
void randomMovement(void) {
	isWaiting = 0;
	isSearching = 1;
	isSpeedRunning = 0;
	
	int cellCount = 1;						// number of explored cells
	int turnCount = 0;
	int remainingDist = 0;				// positional distance
	bool beginCellFlag = 0;
	bool quarterCellFlag = 0;
	bool halfCellFlag = 0;
	bool threeQuarterCellFlag = 0;
	bool fullCellFlag = 0;
	bool hasFrontWall = 0;
	bool hasLeftWall = 0;
	bool hasRightWall = 0;
	int nextMove = 0;
	
	targetSpeedX = searchSpeed;
	
	while(1) {	// run forever
		remainingDist = cellCount*cellDistance - encCount;
		
		if (!beginCellFlag && (remainingDist <= cellDistance))	{	// run once
			beginCellFlag = 1;
			
			useIRSensors = 1;
			useGyro = 0;
			useSpeedProfile = 1;
			
		// If has front wall or needs to turn, decelerate to 0 within half a cell distance
		if (hasFrontWall || nextMove == TURNLEFT || nextMove == TURNRIGHT || nextMove == TURNBACK) {
			if(getDecNeeded(counts_to_mm(remainingDist), curSpeedX, stopSpeed) < decX) {
				targetSpeedX = searchSpeed;
			}
			else {
				targetSpeedX = stopSpeed;
				}
		}
		else 
			targetSpeedX = searchSpeed;
		}
		
		// Reached quarter cell
		if (!quarterCellFlag && (remainingDist <= cellDistance*3/4))	{
			quarterCellFlag = 1;
		}
		
		if (quarterCellFlag && !threeQuarterCellFlag)
			useIRSensors = 1;
		
		
		// Reached half cell
		if (!halfCellFlag && (remainingDist <= cellDistance/2)) {		// Run once
			halfCellFlag = 1;
			// Read wall and set wall flags
			if ((LFSensor > frontWallThresholdL) || (RFSensor > frontWallThresholdR))
				hasFrontWall = 1;
			if (LDSensor > leftWallThreshold)
				hasLeftWall = 1;
			if (RDSensor > rightWallThreshold)
				hasRightWall = 1;
			
			// Store destination cell's wall data
			
			// Decide next movement (search algorithm)
			while (1) {
				nextMove = (millis() % 4) + 1;
				if ((nextMove == GOFORWARD) && (!hasFrontWall))
					break;	
				if ((nextMove == TURNLEFT) && (!hasLeftWall))
					break;
				if ((nextMove == TURNRIGHT) && (!hasRightWall))
					break;
				if ((nextMove == TURNBACK) && (hasFrontWall && hasLeftWall && hasRightWall))
					break;
			}

		}
		
		// Reached three quarter cell
		if (!threeQuarterCellFlag && (remainingDist <= cellDistance*1/4)) {	// run once
			threeQuarterCellFlag = 1;
		}
		
		
		if (threeQuarterCellFlag) {
			// Check for front wall to turn off for the remaining distance
			if (hasFrontWall)
				useIRSensors = 0;
		}
		
		// Reached full cell
		if ((!fullCellFlag && (remainingDist <= 0)) || (LFSensor > 2000) || (RFSensor > 2000)) {
			fullCellFlag = 1;
			cellCount++;
			//shortBeep(200, 1000);
			
			// If has front wall, align with front wall
			if (hasFrontWall) {
				alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right value
			}
			
			
			// Reached full cell, perform next move
			if (nextMove == TURNLEFT) {
				pivotTurn(turnLeft90);
				turnCount++;
			}
			else if (nextMove == TURNRIGHT) {
				pivotTurn(turnRight90);
				turnCount++;
			}
			else if (nextMove == TURNBACK) {
				pivotTurn(turnLeft180);
				turnCount++;
			}
			else if (nextMove == GOFORWARD) {
				// Continue moving forward
			}
			
			beginCellFlag = 0;
			quarterCellFlag = 0;
			halfCellFlag = 0;
			threeQuarterCellFlag = 0;
			fullCellFlag = 0;
			hasFrontWall = 0;
			hasLeftWall = 0;
			hasRightWall = 0;
			
		}
	}
}

//Returns which direction to move in and sets the orientation to next move
int getNextDirection(void)
{
  int currDistance = distance[yPos][xPos];
  int distN = MAX_DIST;
  int distE = MAX_DIST;
  int distS = MAX_DIST;
  int distW = MAX_DIST;

  if(yPos < SIZE - 1)
    distN = distance[yPos+1][xPos];
  if(xPos < SIZE - 1)
    distE = distance[yPos][xPos+1];
  if(xPos > 0)
    distS = distance[yPos-1][xPos];
  if(yPos > 0)
    distW = distance[yPos][xPos-1];

  if(!hasNorth(cell[yPos][xPos]) && (distN == currDistance - 1))
  {
    orientation = 'N';
    return MOVEN;
  }
  if(!hasEast(cell[yPos][xPos]) && (distE == currDistance - 1))
  {
    orientation = 'E';
    return MOVEE;
  }
  if(!hasSouth(cell[yPos][xPos]) && (distS == currDistance - 1))
  {
    orientation = 'S';
    return MOVES;
  }
  if(!hasWest(cell[yPos][xPos]) && (distW == currDistance - 1))
  {
    orientation = 'W';
    return MOVEW;
  }
  return 0;
}

void speedRun(void) 
{

  int nextDir[100] = {0};
	int length = 0;
  
  xPos = 0;
  yPos = 0;
	orientation = 'N';

	// Close off untraced routes
	closeUntracedCells();
  updateDistanceToCenter();
  visualizeGrid();

	// Simulate path
	for (int i = 0; !atCenter(); i++) {
		
		// Error check
		if (distance[yPos][xPos] >= MAX_DIST) {
			beep(10);
			break;
		}
		if (orientation == 'N') {
			while (!hasNorth(cell[yPos][xPos]) && (distance[yPos + 1][xPos] == distance[yPos][xPos] - 1)) {
				length++;
				yPos++;
			}
		}
		else if (orientation == 'E') {
			while (!hasEast(cell[yPos][xPos]) && (distance[yPos][xPos + 1] == distance[yPos][xPos] - 1)) {
				length++;
				xPos++;
			}
		}
		else if (orientation == 'S') {
			while (!hasSouth(cell[yPos][xPos]) && (distance[yPos - 1][xPos] == distance[yPos][xPos] - 1)) {
				length++;
				yPos--;
			}
		}
		else if (orientation == 'W') {
			while (!hasWest(cell[yPos][xPos]) && (distance[yPos][xPos - 1] == distance[yPos][xPos] - 1)) {
				length++;
				xPos--;
			}
		}
		distances[i] = length;
		nextDir[i] = getNextDirection();
		length = 0;
	}
	
	/* Print values
	for (int i = 0; distances[i]; i++)
		printf("distances[%d] = %d | nextDir[%d] = %d\n\r", i, distances[i], i, nextDir[i]);
	*/
	
	// Run path
	delay_ms(1000);
	setGyroRef();
	
	orientation = 'N';
	
	useIRSensors = 1;
	resetSpeedProfile();
	isSpeedRunning = 1;
	useSpeedProfile = 1;
	
  for (int i = 0; distances[i] != 0; i++) {
		if (i == 0) {
			moveForward(distances[i] + (0.4667 - mm_to_counts(motorToBackDist)/cellDistance));
		}
		else if (distances[i+1] == 0) {
			int tempSpeed = stopSpeed;
			stopSpeed = 0;
			moveForward(distances[i] + 0); // +1
			stopSpeed = tempSpeed;
		}
		else {
			moveForward(distances[i]);
		}
		
		//readSensor();
		if (LFSensor > LFvalue2 && RFSensor > RFvalue2) {
			alignFrontWall(LFvalue1, RFvalue1, alignTime);
		}
		
		
    if (nextDir[i] == MOVEN) {
      moveN();
    }
    else if (nextDir[i] == MOVEE) {
      moveE();
    }
    else if (nextDir[i] == MOVES) {
      moveS();
    }
    else if (nextDir[i] == MOVEW) {
      moveW();
    }
  }
	
	useSpeedProfile = 0;
	turnMotorOff;
	beep(3);
	
	isSpeedRunning = 0;
}



void closeUntracedCells(void) {
	int j, k;
	for (j = 0; j < SIZE; j++) {
		for (k = 0; k < SIZE; k++) {
			if (!hasTrace(cell[j][k]))
      {
				cell[j][k] |= 15;
        if(j < SIZE - 1)
          cell[j+1][k] |= 4;
        if(j > 0)
          cell[j-1][k] |= 1;
        if(k < SIZE - 1)
          cell[j][k+1] |= 8;
        if(k > 0)
          cell[j][k-1] |= 2;
      }
		}
	}
}


void speedRunCurve(void) 
{
  int nextDir[100] = {0};
	int length = 0;
  
  xPos = 0;
  yPos = 0;
	orientation = 'N';

	// Close off untraced routes
	closeUntracedCells();
  updateDistanceToCenter();
  visualizeGrid();

	// Simulate path
	for (int i = 0; !atCenter(); i++) {
		
		// Error check
		if (distance[yPos][xPos] >= MAX_DIST) {
			beep(10);
			break;
		}
		
		if (orientation == 'N') {
			while (!hasNorth(cell[yPos][xPos]) && (distance[yPos + 1][xPos] == distance[yPos][xPos] - 1)) {
				length++;
				yPos++;
			}
		}
		else if (orientation == 'E') {
			while (!hasEast(cell[yPos][xPos]) && (distance[yPos][xPos + 1] == distance[yPos][xPos] - 1)) {
				length++;
				xPos++;
			}
		}
		else if (orientation == 'S') {
			while (!hasSouth(cell[yPos][xPos]) && (distance[yPos - 1][xPos] == distance[yPos][xPos] - 1)) {
				length++;
				yPos--;
			}
		}
		else if (orientation == 'W') {
			while (!hasWest(cell[yPos][xPos]) && (distance[yPos][xPos - 1] == distance[yPos][xPos] - 1)) {
				length++;
				xPos--;
			}
		}
		distances[i] = length;
		nextDir[i] = getNextDirection();
		length = 0;
	}
	
	
	/*
	//Print values
	for (int i = 0; distances[i]; i++)
		printf("distances[%d] = %d | nextDir[%d] = %d\n\r", i, distances[i], i, nextDir[i]);
	*/
	
	
	// Run path
	delay_ms(1000);
	setGyroRef();
	
	orientation = 'N';
	
	isSpeedRunning = 1;
	resetSpeedProfile();
	useIRSensors = 0;
	useSpeedProfile = 1;
	
	useIRSensors = 1;
  for (int i = 0; distances[i] != 0; i++) {
		if (i == 0) {
			moveForward(distances[i] - 0.4667 + mm_to_counts(motorToBackDist)/cellDistance);
		}
		else if (distances[i+1] == 0) {
			int tempSpeed = stopSpeed;
			stopSpeed = 0;
			moveForward(distances[i] - 0.5); // +0.5
			stopSpeed = tempSpeed;
		}
		else {
			moveForward(distances[i] - 1);
		}
		
		isCurveTurning = 1;
    if (nextDir[i] == MOVEN) {
      moveN();
    }
    else if (nextDir[i] == MOVEE) {
      moveE();
    }
    else if (nextDir[i] == MOVES) {
      moveS();
    }
    else if (nextDir[i] == MOVEW) {
      moveW();
    }
		isCurveTurning = 0;
		
  }
	
	useIRSensors = 0;
	
	useSpeedProfile = 0;
	turnMotorOff;
	beep(3);
	
	isSpeedRunning = 0;
}



bool hasFrontWallInMem(void) {
	int curBlock = cell[yPos][xPos];
	if ( (orientation == 'N' && hasNorth(curBlock)) || (orientation == 'E' && hasEast(curBlock)) ||
			 (orientation == 'S' && hasSouth(curBlock)) || (orientation == 'W' && hasWest(curBlock)) )
		return 1;
	else
		return 0;
}

void saveData(void) {
	// Save cell data
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			cell_backup[i][j] = cell[i][j];
		}
	}
	
	// Save distance data
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			distance_backup[i][j] = distance[i][j];
		}
	}
	
	return;
}

void loadData(void) {
	// Load cell data
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			cell[i][j] = cell_backup[i][j];
		}
	}
	
	// Load distance data
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			distance[i][j] = distance_backup[i][j];
		}
	}
	
	return;
}

void waitForSignal(void) {
	
	// Wait for left forward sensor signal
	while(LFSensor < 700) {
		readSensor();
		delay_ms(5);
	}
	beep(2);
	
	// Wait for right forward sensor signal
	while(RFSensor < 700) {
		readSensor();
		delay_ms(5);
	}
	beep(2);

}

