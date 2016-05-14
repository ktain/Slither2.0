#ifndef TEST_H
#define TEST_H

#include <stdbool.h>

void hugFrontWall(int LSensorVal, int RSensorVal);
void randomMovement(void);
void speedRun(void);
int getNextDirection(void);
void closeUntracedCells(void);
bool hasFrontWallInMem(void);
void speedRunCurve(void);
void saveData(void);
void loadData(void);
void waitForSignal(void);
#endif


