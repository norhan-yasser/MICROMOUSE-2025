#ifndef API_H
#define API_H

#include "movement.h"
#include "Sensors.h"
#include "config.h"

// Sensing
bool wallFront();
bool wallLeft();
bool wallRight();

// Motion
void moveForward_A(int distance);
MoveResult moveForwardWithStatus(int distance);

// بدل ما نكتبهم cpp نكتب wrappers inline هنا
inline void turnLeft() {
    turnLeft90_MPU(200, 2.0f);
}

inline void turnRight() {
    turnRight90_MPU(200, 2.0f);
}

// Helpers
int orientation(int orient, char turning);
void updateCoordinates(int orient, int *new_x, int *new_y);

#endif
