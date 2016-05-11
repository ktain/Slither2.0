#ifndef MAZE_H
#define MAZE_H

#include "stdbool.h"

/*
 * Filename: maze.h
 * Description: Header file containing function declarations,
 *              constants, and global variables
 */
 
/* Debug option */
#define DEBUG 0

/* Graphics */
#define HIDESOUTH 1  // Hide south cell wall when printing
#define HIDEEAST 1    // Hide east cell wall when printing

/* Constants */
#define SIZE 16      // size of maze
#define MAX_DIST 252  // max distance for flood search

/* Function declarations */
void initializeGrid(void);
void readMaze(void);
void printGrid(void);
void visualizeGrid(void);
void detectWalls(void);
void isolateDeadEnds(void);

void moveN(void);
void moveE(void);
void moveS(void);
void moveW(void);
void moveBack(void);

void treeSearch(char);
void floodCenter(void);
void floodStart(void);
void floodCenterCurve(void);
void updateDistanceToCenter(void);      // to center
void updateDistanceToStart(void);  // to start

int hasNorth(int);
int hasEast(int);
int hasSouth(int);
int hasWest(int);
int hasTrace(int);
int atCenter(void);
int atStart(void);
int isDeadEnd(int) ;
int getMin(int, int);
bool willTurn(void);


/* Global variables */

extern unsigned char cell[SIZE][SIZE]; //  ... 0 0 0 0       0 0 0 0
                              //         DE TRACE   W S E N
                              // [row] [col]
                              // [ y ] [ x ]
extern unsigned char distance[SIZE][SIZE];
extern unsigned char cell_backup[SIZE][SIZE];
extern unsigned char distance_backup[SIZE][SIZE];

extern int xPos;   // 0-15
extern int yPos;   // 0-15
extern char orientation;
extern int moveCount;
extern int traceCount;
extern int nextMove;
extern bool hasFrontWall;
extern bool hasLeftWall;
extern bool hasRightWall;

#endif
