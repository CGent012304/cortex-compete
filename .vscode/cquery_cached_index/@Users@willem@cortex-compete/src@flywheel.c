#include "main.h"    // includes API.h and other headers
#include "flywheel.h"    // include all the flywheel specific definitions
#include "portdef.h" // All port defintions on the cortex
#include <math.h>

void setFly(float flyPower){
  motorSet(FLY_M_1, flyPower);
  // motorSet(FlY_M_2, flyPower); // If multipl motors on multiple Ports
                                  // include them i nthis functions

}

// helper functio nto determien the sgn of a number - we need to knwo if the TBH
// has crossed over the '0' line does changed Assignments

int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

void flywheelTask() {
  /* This function should be called in opcontrol or auto
     it should run as a tasks:
     taskRunLoop(liftPIDtask, LIFT_TASK_DELTA);

     pidFlywheelRequestedValue = ...; should be used to set the  target
  */

  float kI = .025;          //again, this is arbitrary
  float mtrOut = 0;
  float flyTarget = 800;    //still arbitrary - encoder ticks expected
  float flyVel = 0;
  float flyErr = 0;
  float flyErrLast = 0;
  float I = 0;
  float TBH = 0;
  int flyEnc = 0;
  long flyVelTime = 0;
  int flyEncLast = 0;
  long flyVelTimeLast = 0;

  // If we are using an encoder then clear it
  encoderReset(encoderFLY);

  while(true) {
    flyVelTime = micros();
    flyVel = (encoderGet(encoderFLY) - flyEncLast) / (flyVelTime - flyVelTimeLast);
    flyErr = flyTarget - flyVel;
    I += flyErr;
    mtrOut = I * kI;
    if(mtrOut > 127) {
      mtrOut = 127;
    }
    else if(mtrOut < 0) {
      // Keep the motor power positive, to prevent damaging gears or motors
      mtrOut = 0;
    }

    if(sgn(flyErr) != sgn(flyErrLast)) {
      // If the sign of the error changes, then the error crossed zero
      TBH = (mtrOut + TBH) / 2;
      mtrOut = TBH;
      flyErrLast = flyErr;
      // the last error doesn't matter unless the sign is different, so the last error is
      // only stored when necessary
    }
    setFly(mtrOut);
    flyEncLast = encoderGet(encoderFLY);
    flyVelTimeLast = flyVel;
    delay(20);
  }
}
