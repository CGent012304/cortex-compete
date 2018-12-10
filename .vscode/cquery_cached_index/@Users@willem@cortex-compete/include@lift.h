#ifndef _LIFT_H_
#define _LIFT_H_

//#define LIFT_LEFT_M 6
//#define LIFT_RIGHT_M 7


static int   pidRunning = 1;
static float pidRequestedValue;

#define PID_LIFT_MAX       127
#define PID_LIFT_MIN     (-127)
#define PID_SENSOR_SCALE    1
#define PID_INTEGRAL_LIMIT  50

#define LIFT_TASK_DELTA 150

// Sets the speeds of the left and right motors to move LIFT
void liftMove(int speed);

// Move the lift to a given position uisng PID control adn encoder value
void liftMovePID(int speed, int position);

//
void liftPIDtask();

#endif // _LIFT_H_
