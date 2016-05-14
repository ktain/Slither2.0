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


/*
 * Flood fill search to center using pivot turns
 */
void floodCenter(void) {
	delay_ms(1000);
	setGyroRef();
	
	isSearching = 1;
	resetSpeedProfile();
	useGyro = 1;
	
	//int cellCount = 1;						// number of explored cells
	int remainingDist = 0;				// positional distance
	int accumDist = encCount;
	bool beginCellFlag = 0;
	bool quarterCellFlag = 0;
	bool halfCellFlag = 0;
	bool threeQuarterCellFlag = 0;
	bool fullCellFlag = 0;

	int distN = 0;   // distances around current position
  int distE = 0;
  int distS = 0;
  int distW = 0;
	
	// Starting cell
	xPos = 0;
	yPos = 0;
	orientation = 'N';
	
	// Place trace at starting position
  if (!hasTrace(cell[yPos][xPos])) {
    cell[yPos][xPos] |= 16;
    traceCount++;
	}
	
	targetSpeedX = searchSpeed;
	
	while(!atCenter()) {
		remainingDist = accumDist + cellDistance - encCount;
		
		// Beginning of cell
		if (!beginCellFlag && (remainingDist <= cellDistance))	{	// run once
			beginCellFlag = 1;
			useIRSensors = 1;
			useSpeedProfile = 1;
			
			// Error check
			if (distance[yPos][xPos] >= MAX_DIST) {
				if (DEBUG) {
					printf("Stuck... Can't find center.\n\r");
				}
				nextMove = STOP;
				useSpeedProfile = 0;
				turnMotorOff;
				beep(10);
				visualizeGrid();
				return;
			}
			
			// Update position
			if (orientation == 'N') {
				yPos += 1;
			}
			if (orientation == 'E') {
				xPos += 1;
			}
			if (orientation == 'S') {
				yPos -= 1;
			}
			if (orientation == 'W') {
				xPos -= 1;
			}
		
		}
		
		// Reached quarter cell
		if (!quarterCellFlag && (remainingDist <= cellDistance*3/4))	{	// run once
			quarterCellFlag = 1;
		}
		
		// Reached half cell
		if (!halfCellFlag && (remainingDist <= cellDistance/2)) {		// Run once
			halfCellFlag = 1;
			
			// Detect left and right wall
			if (LDSensor > leftWallThreshold)
				hasLeftWall = 1;
			if (RDSensor > rightWallThreshold)
				hasRightWall = 1;		
			// Detect front wall
			if ((LFSensor > frontWallThresholdL) && (RFSensor > frontWallThresholdR))
				hasFrontWall = 1;
		
			// If new cell, detect wall data
			if (!hasTrace(cell[yPos][xPos])) {
				// Store next cell's wall data
				if (DEBUG) printf("Detecting walls\n\r");
				detectWalls();
			}
			
			// Update distance for current block
			if (DEBUG) printf("Updating distance for current block\n\r");
			distance[yPos][xPos] = getMin(xPos, yPos) + 1;
			
			// Update distances for every other block
			if (DEBUG) printf("Updating distances\n\r");
			updateDistanceToCenter();
			
			// Get distances around current block
			distN = hasNorth(cell[yPos][xPos])? MAX_DIST : distance[yPos + 1][xPos];
			distE = hasEast(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos + 1];
			distS = hasSouth(cell[yPos][xPos])? MAX_DIST : distance[yPos - 1][xPos];
			distW = hasWest(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos - 1];
			if (DEBUG) printf("distN %d, distE %d, distS %d, distW %d\n\r", distN, distE, distS, distW);
			
			// Decide next movement
			if (DEBUG) printf("Deciding next movement\n\r");
			// 1. Pick the shortest route
			if ( (distN < distE) && (distN < distS) && (distN < distW) )
				nextMove = MOVEN;
			else if ( (distE < distN) && (distE < distS) && (distE < distW) )
				nextMove = MOVEE;
			else if ( (distS < distE) && (distS < distN) && (distS < distW) )
				nextMove = MOVES;
			else if ( (distW < distE) && (distW < distS) && (distW < distN) )
				nextMove = MOVEW;
			
			// 2. If multiple equally short routes, go straight if untraced
			else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]) && orientation == 'N')
				nextMove = MOVEN;
			else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]) && orientation == 'E')
				nextMove = MOVEE;
			else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]) && orientation == 'S')
				nextMove = MOVES;
			else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]) && orientation == 'W')
				nextMove = MOVEW;
			 
			// 3. Else, choose untraced route prioritizing N > E > S > W
			else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]))
				nextMove = MOVEN;
			else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]))
				nextMove = MOVEE;
			else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]))
				nextMove = MOVES;
			else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]))
				nextMove = MOVEW;
			
			
			// 4. Else, go straight if possible
			else if ( orientation == 'N' && !hasNorth(cell[yPos][xPos]) )
				nextMove = MOVEN;
			else if ( orientation == 'E' && !hasEast(cell[yPos][xPos]) )
				nextMove = MOVEE;
			else if ( orientation == 'S' && !hasSouth(cell[yPos][xPos]) )
				nextMove = MOVES;
			else if ( orientation == 'W' && !hasWest(cell[yPos][xPos]) )
				nextMove = MOVEW;
			
			 
			// 5. Otherwise prioritize N > E > S > W
			else if (!hasNorth(cell[yPos][xPos]))
				nextMove = MOVEN;
			else if (!hasEast(cell[yPos][xPos]))
				nextMove = MOVEE;
			else if (!hasSouth(cell[yPos][xPos]))
				nextMove = MOVES;
			else if (!hasWest(cell[yPos][xPos]))
				nextMove = MOVEW;
			
			else {
				if (DEBUG) {
					printf("Stuck... Can't find center.\n\r");
				}
				nextMove = STOP;
				useSpeedProfile = 0;
				turnMotorOff;
				beep(10);
				visualizeGrid();
				return;
			}
			
			if (DEBUG) printf("nextMove %d\n\r", nextMove);
			
		}
		
		// Reached half cell
		if ((remainingDist <= cellDistance/2)) {		// Run for last half
			halfCellFlag = 1;
		}
					
			// If needs to turn, decelerate to 0 within half a cell distance
			if (willTurn()) {
				if(getDecNeeded(counts_to_mm(remainingDist), curSpeedX, 0) < decX) {
					targetSpeedX = searchSpeed;
				}
				else {
					targetSpeedX = 0;
				}
			}
			else 
				targetSpeedX = searchSpeed;
		
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
		if ((!fullCellFlag && (remainingDist <= 0))) {	// run once
			if (DEBUG) printf("Reached full cell\n\r");
			fullCellFlag = 1;
			//cellCount++;
			//shortBeep(200, 1000);
			
			// Place trace
			if (!hasTrace(cell[yPos][xPos])) {
				cell[yPos][xPos] |= 16;
				traceCount++;
			}
			
			// If has front wall, align with front wall
			if (hasFrontWall) {
				alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
			}
			
			// Reached full cell, perform next move
			if (nextMove == MOVEN) {
				moveN();
			}
			else if (nextMove == MOVEE) {
				moveE();
			}
			else if (nextMove == MOVES) {
				moveS();
			}
			else if (nextMove == MOVEW) {
				moveW();
			}
			
			updateDistanceToCenter();
			simulateStraight();
			stopSpeed = 0;
			
			accumDist = encCount;
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
	
	// Finish moving across last cell
	while(remainingDist > 0) {
		if (remainingDist < cellDistance/2)
			useIRSensors = 0;
		remainingDist = accumDist + cellDistance - encCount;
		if(getDecNeeded(counts_to_mm(remainingDist), curSpeedX, 0) < decX) {
			targetSpeedX = searchSpeed;
		}
		else {
			targetSpeedX = 0;
		}
	}
	
	//shortBeep(200, 1000);
	
	// Place trace
	if (!hasTrace(cell[yPos][xPos])) {
		cell[yPos][xPos] |= 16;
		traceCount++;
	}
	
	useSpeedProfile = 0;
	turnMotorOff;
			
  //isolateDeadEnds();
	visualizeGrid();
	beep(3);
	
	isSearching = 0;
}

/*
 * Flood fill search to start using pivot turns
 */
void floodStart(void) {
	delay_ms(1000);
	setGyroRef();
	
	isSearching = 1;
	resetSpeedProfile();
	
	//int cellCount = 1;						// number of explored cells
	int remainingDist = 0;				// positional distance
	int accumDist = encCount;
	bool beginCellFlag = 0;
	bool quarterCellFlag = 0;
	bool halfCellFlag = 0;
	bool threeQuarterCellFlag = 0;
	bool fullCellFlag = 0;

	int distN = 0;   // distances around current position
  int distE = 0;
  int distS = 0;
  int distW = 0;
	
	// Current location known
	
	// Re-initialize distances to flood starting position
  int i, j, k;
  for(i = 0; i < SIZE; i++) {
    k = 0;
    for (j = 0; j < SIZE; j++) {
      distance[i][j] = k + i;
      k++;
    }
  }
	updateDistanceToStart();
	
	// Turn back
	moveBack();
	
	targetSpeedX = searchSpeed;
	
	while(!atStart()) {  // while not at starting position
		remainingDist = accumDist + cellDistance - encCount;
		
		// Beginning of cell
		if (!beginCellFlag && (remainingDist <= cellDistance))	{	// run once
			beginCellFlag = 1;
			useIRSensors = 1;
			useSpeedProfile = 1;
			
			// Error check
			if (distance[yPos][xPos] >= MAX_DIST) {
				if (DEBUG) {
					printf("Stuck... Can't find start.\n\r");
				}
				nextMove = STOP;
				useSpeedProfile = 0;
				turnMotorOff;
				beep(10);
				visualizeGrid();
				return;
			}
			
			// Update position
			if (orientation == 'N') {
				yPos += 1;
			}
			if (orientation == 'E') {
				xPos += 1;
			}
			if (orientation == 'S') {
				yPos -= 1;
			}
			if (orientation == 'W') {
				xPos -= 1;
			}
		}
		
		// Reached quarter cell
		if (!quarterCellFlag && (remainingDist <= cellDistance*3/4))	{	// run once
			quarterCellFlag = 1;	
		}
		
		// Reached half cell
		if (!halfCellFlag && (remainingDist <= cellDistance/2)) {		// Run once
			halfCellFlag = 1;
						
			// Detect left and right wall
			if (LDSensor > leftWallThreshold)
				hasLeftWall = 1;
			if (RDSensor > rightWallThreshold)
				hasRightWall = 1;		
			// Detect front wall
			if ((LFSensor > frontWallThresholdL) && (RFSensor > frontWallThresholdR))
				hasFrontWall = 1;
		
			// If new cell, detect wall data
			if (!hasTrace(cell[yPos][xPos])) {
				// Store next cell's wall data
				if (DEBUG) printf("Detecting walls\n\r");
				detectWalls();
			}
			
			// Update distance for current block
			if (DEBUG) printf("Updating distance for current block\n\r");
			distance[yPos][xPos] = getMin(xPos, yPos) + 1;
			
			// Update distances for every other block
			if (DEBUG) printf("Updating distances\n\r");
			updateDistanceToStart();
			
			// Get distances around current block
			distN = hasNorth(cell[yPos][xPos])? MAX_DIST : distance[yPos + 1][xPos];
			distE = hasEast(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos + 1];
			distS = hasSouth(cell[yPos][xPos])? MAX_DIST : distance[yPos - 1][xPos];
			distW = hasWest(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos - 1];
			if (DEBUG) printf("distN %d, distE %d, distS %d, distW %d\n\r", distN, distE, distS, distW);
			
			// Decide next movement, flooding to start
			if (DEBUG) printf("Deciding next movement\n\r");
			// 1. Pick the shortest route
			if ( (distN < distE) && (distN < distS) && (distN < distW) )
				nextMove = MOVEN;
			else if ( (distE < distN) && (distE < distS) && (distE < distW) )
				nextMove = MOVEE;
			else if ( (distS < distE) && (distS < distN) && (distS < distW) )
				nextMove = MOVES;
			else if ( (distW < distE) && (distW < distS) && (distW < distN) )
				nextMove = MOVEW;
			
			// 2. If multiple equally short routes, go straight if untraced
			else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]) && orientation == 'N')
				nextMove = MOVEN;
			else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]) && orientation == 'E')
				nextMove = MOVEE;
			else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]) && orientation == 'S')
				nextMove = MOVES;
			else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]) && orientation == 'W')
				nextMove = MOVEW;
			 
			// 3. Else, choose untraced route prioritizing W > S > E > N	
			else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]))
				nextMove = MOVEW;
			else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]))
				nextMove = MOVES;
			else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]))
				nextMove = MOVEE;
			else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]))
				nextMove = MOVEN;
			
			// 4. Else, go straight if possible
			else if ( orientation == 'N' && !hasNorth(cell[yPos][xPos]) )
				nextMove = MOVEN;
			else if ( orientation == 'E' && !hasEast(cell[yPos][xPos]) )
				nextMove = MOVEE;
			else if ( orientation == 'S' && !hasSouth(cell[yPos][xPos]) )
				nextMove = MOVES;
			else if ( orientation == 'W' && !hasWest(cell[yPos][xPos]) )
				nextMove = MOVEW;
			
			// 5. Otherwise prioritize W > S > E > N
			else if (!hasWest(cell[yPos][xPos]))
				nextMove = MOVEW;
			else if (!hasSouth(cell[yPos][xPos]))
				nextMove = MOVES;
			else if (!hasEast(cell[yPos][xPos]))
				nextMove = MOVEE;
			else if (!hasNorth(cell[yPos][xPos]))
				nextMove = MOVEN;
			
			else {
				if (DEBUG) {
					printf("Stuck... Can't find start.\n\r");
				}
				nextMove = STOP;
				useSpeedProfile = 0;
				turnMotorOff;
				beep(10);
				visualizeGrid();
				return;
			}
			
			if (DEBUG) printf("nextMove %d\n\r", nextMove);
			
		}
		
		// Reached half cell
		if ((remainingDist <= cellDistance/2)) {		// Run for last half
			halfCellFlag = 1;
		}
					
			// If needs to turn, decelerate to 0 within half a cell distance
			if (willTurn()) {
				if(getDecNeeded(counts_to_mm(remainingDist), curSpeedX, 0) < decX) {
					targetSpeedX = searchSpeed;
				}
				else {
					targetSpeedX = 0;
				}
			}
			else 
				targetSpeedX = searchSpeed;
		
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
		if ((!fullCellFlag && (remainingDist <= 0))) {	// run once
			if (DEBUG) printf("Reached full cell\n\r");
			fullCellFlag = 1;
			//cellCount++;
			//shortBeep(200, 1000);
			
			// Place trace
			if (!hasTrace(cell[yPos][xPos])) {
				cell[yPos][xPos] |= 16;
				traceCount++;
			}
			
			// If has front wall, align with front wall
			if (hasFrontWall) {
				alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
			}
			
			// Reached full cell, perform next move
			if (nextMove == MOVEN) {
				moveN();
			}
			else if (nextMove == MOVEE) {
				moveE();
			}
			else if (nextMove == MOVES) {
				moveS();
			}
			else if (nextMove == MOVEW) {
				moveW();
			}
			
			accumDist = encCount;
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
	
	// Finish moving across last cell
	while(remainingDist > 0) {
		if (remainingDist < cellDistance/2)
			useIRSensors = 0;
		remainingDist = accumDist + cellDistance - encCount;
		if(getDecNeeded(counts_to_mm(remainingDist), curSpeedX, 0) < decX) {
			targetSpeedX = searchSpeed;
		}
		else {
			targetSpeedX = 0;
		}
	}
	
	// Place trace
	if (!hasTrace(cell[yPos][xPos])) {
		cell[yPos][xPos] |= 16;
		traceCount++;
	}
	
	// Turn back
	moveBack();
	
	useSpeedProfile = 0;
	turnMotorOff;
	
	updateDistanceToCenter();
			
  //isolateDeadEnds();
	visualizeGrid();
	
	beep(3);
	
	isSearching = 0;
}

/*
 * Flood fill search to center using curve turns
 */
void floodCenterCurve(void) {

}


void isolateDeadEnds(void)
{
   // Isolate known dead ends with pseudo walls
	if (DEBUG) 
		printf("Placing pseudo walls\n\r");
	for (int i = SIZE*SIZE; i >= 0; i--) {
		for (int j = 0; j < SIZE; j++) {
			for (int k = 0; k < SIZE; k++) {
				
				// Ignore start and center ends
				if ( !((j == 0) && (k == 0)) && 
						 !( (((SIZE - 1)/2 == i) || (SIZE/2 == i)) && (((SIZE - 1)/2 == j) || (SIZE/2 == j)) )) {
					// If dead end, isolate block
					if ((hasNorth(cell[j][k]) + hasEast(cell[j][k]) +
							hasSouth(cell[j][k]) + hasWest(cell[j][k])) >= 3) {
						cell[j][k] |= 32;
						cell[j][k] |= 15;
						if (j < SIZE - 1)  // Update adjacent wall
							cell[j + 1][k] |= 4;
						if (k < SIZE - 1)  // Update adjacent wall
							cell[j][k + 1] |= 8;
						if (j > 0)         // Update adjacent wall
							cell[j - 1][k]  |= 1;
						if (k > 0)         // Update adjacent wall
							cell[j][k - 1] |= 2;
					}
				}
			}
		}
	}
	
}


// Update distances for every other block while flooding the center
// slow...
void updateDistanceToCenter() {
	bool changed;
	int temp;
  int i, j, k;
  for (i = SIZE*SIZE; i >= 0; i--) {
		changed = 0;
    for (j = 0; j < SIZE; j++) {
      for (k = 0; k < SIZE; k++) {
				if ( !((j == (SIZE/2)-1 || (j == SIZE/2)) &&
						((k == (SIZE/2)-1) || (k == SIZE/2))) ) {
					temp = distance[j][k];
					distance[j][k] = getMin(k, j) + 1;
					if (temp != distance[j][k]) {
						changed = 1;
					}
				}
      }
    }
		if (!changed) {
			break;
		}
  }
}


// Update distances for every other block while flooding the start
// Terminate when nothing changes
// slow...
void updateDistanceToStart() {
  bool changed;
	int temp;
  int i, j, k;
  for (i = SIZE*SIZE; i >= 0; i--) {
		changed = 0;
    for (j = 0; j < SIZE; j++) {
      for (k = 0; k < SIZE; k++) {
        if ( !((j == 0) && (k == 0)) ) {
					temp = distance[j][k];
					distance[j][k] = getMin(k, j) + 1;
					if (temp != distance[j][k]) {
						changed = 1;
					}
				}
      }
    }
		if (!changed) {
			break;
		}
  }
}


void detectWalls() {
  
	if (orientation == 'N') {
		if (hasFrontWall) {
			cell[yPos][xPos] |= 1;
			if (yPos < SIZE - 1)  // Update adjacent wall
				cell[yPos + 1][xPos] |= 4;
		}
		if (hasLeftWall) {
			cell[yPos][xPos] |= 8;
			if (xPos > 0)  // Update adjacent wall
				cell[yPos][xPos - 1] |= 2;
		}
		if (hasRightWall) {
			cell[yPos][xPos] |= 2;
			if (xPos < SIZE - 1)  // Update adjacent wall
				cell[yPos][xPos + 1] |= 8;
		}		
	}
	else if (orientation == 'E') {
		if (hasFrontWall) {
			cell[yPos][xPos] |= 2;
			if (xPos < SIZE - 1)  // Update adjacent wall
				cell[yPos][xPos + 1] |= 8;
		}
		if (hasLeftWall) {
		  cell[yPos][xPos] |= 1;
			if (yPos < SIZE - 1)  // Update adjacent wall
				cell[yPos + 1][xPos] |= 4;
		}
		if (hasRightWall) {
			cell[yPos][xPos] |= 4;
			if (yPos > 0)  // Update adjacent wall
				cell[yPos - 1][xPos] |= 1;
		}
	}
	else if (orientation == 'S') {
		if (hasFrontWall) {
			cell[yPos][xPos] |= 4;
			if (yPos > 0)  // Update adjacent wall
				cell[yPos - 1][xPos] |= 1;
		}
		if (hasLeftWall) {
			cell[yPos][xPos] |= 2;
			if (xPos < SIZE - 1)  // Update adjacent wall
				cell[yPos][xPos + 1] |= 8;
		}
		if (hasRightWall) {
			cell[yPos][xPos] |= 8;
			if (xPos > 0)  // Update adjacent wall
				cell[yPos][xPos - 1] |= 2;
		}
	}
	else if (orientation == 'W') {
		if (hasFrontWall) {
			cell[yPos][xPos] |= 8;
			if (xPos > 0)  // Update adjacent wall
				cell[yPos][xPos - 1] |= 2;
		}
		if (hasLeftWall) {
			cell[yPos][xPos] |= 4;
			if (yPos > 0)  // Update adjacent wall
				cell[yPos - 1][xPos] |= 1;
		}
		if (hasRightWall) {
			cell[yPos][xPos] |= 1;
			if (yPos < SIZE - 1)  // Update adjacent wall
				cell[yPos + 1][xPos] |= 4;
		}
	}	
}

bool willTurn(void) {
	if ( (orientation == 'N' && nextMove == MOVEN) || (orientation == 'E' && nextMove == MOVEE) || 
		 (orientation == 'S' && nextMove == MOVES) || (orientation == 'W' && nextMove == MOVEW) )
		return 0;
	else return 1;
}

void simulateStraight(void) {
	int xPosSim = xPos;
	int yPosSim = yPos;
	int count = 0;
	if (orientation == 'N') {
		while (!hasNorth(cell[yPosSim][xPosSim]) && hasTrace(cell[yPosSim + 1][xPosSim]) && (distance[yPosSim + 1][xPosSim] == distance[yPosSim][xPosSim] - 1)
			&& !((((SIZE - 1)/2 == xPosSim) || (SIZE/2 == xPosSim)) && (((SIZE - 1)/2 == yPosSim + 1) || (SIZE/2 == yPosSim + 1)))) {
			count++;
			yPosSim++;
		}
		if (!hasNorth(cell[yPosSim][xPosSim]) && (distance[yPosSim + 1][xPosSim] == getMin(xPosSim, yPosSim))) {
			stopSpeed = searchSpeed;
		}
		else {
			stopSpeed = 0;
		}
	}
	else if (orientation == 'E') {
		while (!hasEast(cell[yPosSim][xPosSim]) && hasTrace(cell[yPosSim][xPosSim + 1]) && (distance[yPosSim][xPosSim + 1] == distance[yPosSim][xPosSim] - 1)
			&& !((((SIZE - 1)/2 == xPosSim + 1) || (SIZE/2 == xPosSim + 1)) && (((SIZE - 1)/2 == yPosSim) || (SIZE/2 == yPosSim)))) {
			count++;
			xPosSim++;
		}
		if (!hasEast(cell[yPosSim][xPosSim]) && (distance[yPosSim][xPosSim + 1] == getMin(xPosSim, yPosSim))) {
			stopSpeed = searchSpeed;
		}
		else {
			stopSpeed = 0;
		}
	} 
	else if (orientation == 'S') {
		while (!hasSouth(cell[yPosSim][xPosSim]) && hasTrace(cell[yPosSim - 1][xPosSim]) && (distance[yPosSim - 1][xPosSim] == distance[yPosSim][xPosSim] - 1) 
			&& !((((SIZE - 1)/2 == xPosSim) || (SIZE/2 == xPosSim)) && (((SIZE - 1)/2 == yPosSim - 1) || (SIZE/2 == yPosSim - 1)))) {
			count++;
			yPosSim--;
		}
		if (!hasSouth(cell[yPosSim][xPosSim]) && (distance[yPosSim - 1][xPosSim] == getMin(xPosSim, yPosSim))) {
			stopSpeed = searchSpeed;
		}
		else {
			stopSpeed = 0;
		}
	} 
	else if (orientation == 'W') {
		while (!hasWest(cell[yPosSim][xPosSim]) && hasTrace(cell[yPosSim][xPosSim - 1]) && (distance[yPosSim][xPosSim - 1] == distance[yPosSim][xPosSim] - 1)
			&& !((((SIZE - 1)/2 == xPosSim - 1) || (SIZE/2 == xPosSim - 1)) && (((SIZE - 1)/2 == yPosSim) || (SIZE/2 == yPosSim)))) {
			count++;
			xPosSim--;
		}
		if (!hasWest(cell[yPosSim][xPosSim]) && (distance[yPosSim][xPosSim - 1] == getMin(xPosSim, yPosSim))) {
			stopSpeed = searchSpeed;
		}
		else {
			stopSpeed = 0;
		}
	} 
	
	moveForward(count);
		
	xPos = xPosSim;
	yPos = yPosSim;
	
	int distN, distE, distS, distW;
	
	// Get distances around current block
	distN = hasNorth(cell[yPos][xPos])? MAX_DIST : distance[yPos + 1][xPos];
	distE = hasEast(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos + 1];
	distS = hasSouth(cell[yPos][xPos])? MAX_DIST : distance[yPos - 1][xPos];
	distW = hasWest(cell[yPos][xPos])? MAX_DIST : distance[yPos][xPos - 1];
	if (DEBUG) printf("distN %d, distE %d, distS %d, distW %d\n\r", distN, distE, distS, distW);
	
	// Decide next movement
	if (DEBUG) printf("Deciding next movement\n\r");
	// 1. Pick the shortest route
	if ( (distN < distE) && (distN < distS) && (distN < distW) )
		nextMove = MOVEN;
	else if ( (distE < distN) && (distE < distS) && (distE < distW) )
		nextMove = MOVEE;
	else if ( (distS < distE) && (distS < distN) && (distS < distW) )
		nextMove = MOVES;
	else if ( (distW < distE) && (distW < distS) && (distW < distN) )
		nextMove = MOVEW;
	
	// 2. If multiple equally short routes, go straight if untraced
	else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]) && orientation == 'N')
		nextMove = MOVEN;
	else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]) && orientation == 'E')
		nextMove = MOVEE;
	else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]) && orientation == 'S')
		nextMove = MOVES;
	else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]) && orientation == 'W')
		nextMove = MOVEW;
	 
	// 3. Else, choose untraced route prioritizing N > E > S > W
	else if ( !hasNorth(cell[yPos][xPos]) && !hasTrace(cell[yPos + 1][xPos]))
		nextMove = MOVEN;
	else if ( !hasEast(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos + 1]))
		nextMove = MOVEE;
	else if ( !hasSouth(cell[yPos][xPos]) && !hasTrace(cell[yPos - 1][xPos]))
		nextMove = MOVES;
	else if ( !hasWest(cell[yPos][xPos]) && !hasTrace(cell[yPos][xPos - 1]))
		nextMove = MOVEW;
	
	
	// 4. Else, go straight if possible
	else if ( orientation == 'N' && !hasNorth(cell[yPos][xPos]) )
		nextMove = MOVEN;
	else if ( orientation == 'E' && !hasEast(cell[yPos][xPos]) )
		nextMove = MOVEE;
	else if ( orientation == 'S' && !hasSouth(cell[yPos][xPos]) )
		nextMove = MOVES;
	else if ( orientation == 'W' && !hasWest(cell[yPos][xPos]) )
		nextMove = MOVEW;
	
	 
	// 5. Otherwise prioritize N > E > S > W
	else if (!hasNorth(cell[yPos][xPos]))
		nextMove = MOVEN;
	else if (!hasEast(cell[yPos][xPos]))
		nextMove = MOVEE;
	else if (!hasSouth(cell[yPos][xPos]))
		nextMove = MOVES;
	else if (!hasWest(cell[yPos][xPos]))
		nextMove = MOVEW;
	
	// If has front wall, align with front wall
	if (orientation == 'N' && hasNorth(cell[yPos][xPos])) {
		alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
	}
	else if (orientation == 'E' && hasEast(cell[yPos][xPos])) {
		alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
	}
	else if (orientation == 'S' && hasSouth(cell[yPos][xPos])) {
		alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
	}
	else if (orientation == 'W' && hasWest(cell[yPos][xPos])) {
		alignFrontWall(LFvalue1, RFvalue1, alignTime);	// left, right, duration
	}
	
	// Perform next move
	if (nextMove == MOVEN) {
		moveN();
	}
	else if (nextMove == MOVEE) {
		moveE();
	}
	else if (nextMove == MOVES) {
		moveS();
	}
	else if (nextMove == MOVEW) {
		moveW();
	}
	
}
