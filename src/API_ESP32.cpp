#include "API.h"

// ====== Sensing ======
static inline bool valid(uint8_t cm){ return cm>0 && cm<200; }

bool wallFront(){
  uint8_t m = VLX_readDistance(MIDDLE_VLX);  // cm
  return valid(m) && (m <= TH_FRONT);
}

bool wallLeft(){
  uint8_t l = VLX_readDistance(LEFT_VLX);
  return valid(l) && (l <= TH_SIDE);
}

bool wallRight(){
  uint8_t r = VLX_readDistance(RIGHT_VLX);
  return valid(r) && (r <= TH_SIDE);
}

void moveForward_A(int distance){
  WallFollowCfg cfg;
  cfg.basePwm     = ALGO_BASE_PWM;
  cfg.frontStop   = TH_FRONT;   // cm
  cfg.sideTarget  = 5.0f;       // cm
  cfg.targetDelta = 0.0f;

  for(int i=0;i<distance;i++){
    MoveResult r = driveCell_WF(1, cfg);
    if (r != MOVE_OK) break; // لو اتقفلت أو stalled
  }
}

// ====== Motion ======
MoveResult moveForwardWithStatus(int distance){
  WallFollowCfg cfg;
  cfg.basePwm     = ALGO_BASE_PWM;
  cfg.frontStop   = TH_FRONT;
  cfg.sideTarget  = 4.0f;
  cfg.targetDelta = 0.0f;

  MoveResult res = MOVE_OK;
  for(int i=0;i<distance;i++){
    res = driveCell_WF(1, cfg);
    if (res != MOVE_OK) break;
  }
  return res;
}

// ====== Helpers ======
int orientation(int orient, char turning) {
  if (turning == 'L') {
    orient = (orient == 0) ? 3 : orient - 1;
  } else if (turning == 'R') {
    orient = (orient + 1) % 4;
  } else if (turning == 'B') {
    orient = (orient + 2) % 4;
  }
  return orient;
}

void updateCoordinates(int orient, int *new_x, int *new_y) {
  if (orient == 0)      *new_y += 1;
  else if (orient == 1) *new_x += 1;
  else if (orient == 2) *new_y -= 1;
  else if (orient == 3) *new_x -= 1;
}
