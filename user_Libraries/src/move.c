#include "maze.h"
#include "config.h"
#include "turn.h"
#include "global.h"
#include <stdio.h>

/*
 * Filename: move.c
 * Description: Contains functions that turn to the correct orientation
 */
 
 /* Move north */
void moveN() {
	
  if ( orientation == 'N' ) {

  }
  else if ( orientation == 'W' ) {
		if (isCurveTurning)
			curveTurnRight();
    else
			pivotTurn(turnRight90);
  }
  else if ( orientation == 'E' ) {
		if (isCurveTurning)
			curveTurnLeft();
    else
			pivotTurn(turnLeft90);
  }
  else if ( orientation == 'S' ) {
			pivotTurn(turnLeft180);
  }

  // Update orientation
  orientation = 'N';

}

/* Move east */
void moveE() {
	
  if ( orientation == 'N' ) {
		if (isCurveTurning)
			curveTurnRight();
    else
			pivotTurn(turnRight90);
  }
  else if ( orientation == 'W' ) {
		pivotTurn(turnLeft180);
  }
  else if ( orientation == 'E' ) {

  }  
	else if ( orientation == 'S' ) {
		if (isCurveTurning)
			curveTurnLeft();
    else
			pivotTurn(turnLeft90);
  }

  // Update orientation
  orientation = 'E';
}

/* Move south */
void moveS() {

  if ( orientation == 'N' ) {
		pivotTurn(turnLeft180);
  }
  else if ( orientation == 'W' ) {
		if (isCurveTurning)
			curveTurnLeft();
    else
			pivotTurn(turnLeft90);
  }
  else if ( orientation == 'E' ) {
		if (isCurveTurning)
			curveTurnRight();
		else
			pivotTurn(turnRight90);
  }
  else if ( orientation == 'S' ) {

  }
	
  // Update orientation
  orientation = 'S';
	
}

/* Move west */
void moveW() {
	
  if ( orientation == 'N' ) {
		if (isCurveTurning)
			curveTurnLeft();
    else
			pivotTurn(turnLeft90);
  }
  else if ( orientation == 'W' ) {

  }
  else if ( orientation == 'E' ) {
			pivotTurn(turnLeft180);
  }
  else if ( orientation == 'S' ) {
		if (isCurveTurning)
			curveTurnRight();
    else
			pivotTurn(turnRight90);
  }

  // Update orientation
  orientation = 'W';

}


/* Move back */
void moveBack() {
  if (orientation == 'N')
    moveS();
  else if (orientation == 'E')
    moveW();
  else if (orientation == 'S')
    moveN();
  else if (orientation == 'W')
    moveE();
}
