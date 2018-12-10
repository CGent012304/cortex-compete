#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_

static int   pidFlywheelRunning = 1;
static float pidFlywheelRequestedValue;

#define FLYWHEEL_TASK_DELTA 150

// Sets the speeds of the left and right motors to move LIFT
void setFly(float flyPower);

void flywheelTask();

#endif // _LIFT_H_
