#include "main.h"    // includes API.h and other headers
#include "claw.h"    // include all the claw specific definitions
#include "portdef.h" // All port defintions o nthe cortex

void clawMove(int speed) {
  motorSet(CLAW_M, speed);
}
