#include "main.h"    // includes API.h and other headers
#include "lift.h"    // include all the lift specific definitions
#include "portdef.h" // All port defintions o nthe cortex
#include <math.h>


void liftMove(int speed) {
  motorSet(LIFT_LEFT_M, -speed);
  motorSet(LIFT_RIGHT_M, speed);  // left and right need to go opposite directions

}

void liftPIDtask() {
    //PID using optical shaft encoder
    // Shaft encoder has 360 pulses per revolution
    /* THis function should be called in opcontrol or auto
       it should run as a tasks:
       taskRunLoop(liftPIDtask, LIFT_TASK_DELTA);

       pidRequestedValue = ...; should be used to set the  target
    */
    // lift PID tuning varibales
    float  pid_Kp = 2.0;
    float  pid_Ki = 0.04;
    float  pid_Kd = 0.0;

    float  pidSensorCurrentValue;

    float  pidError;
    float  pidLastError;
    float  pidIntegral;
    float  pidDerivative;
    float  pidDrive;

    // If we are using an encoder then clear it
    encoderReset(encoderLIFT);

    // Init the variables
    pidLastError  = 0;
    pidIntegral   = 0;

    while( true )
    {
        // Is PID control active ?
        if( pidRunning )
        {
            // Read the sensor value and scale
            pidSensorCurrentValue = encoderGet(encoderLIFT) * PID_SENSOR_SCALE;

            // calculate error
            pidError = pidSensorCurrentValue - pidRequestedValue;

            // integral - if Ki is not 0
            if( pid_Ki != 0 )
            {
              // If we are inside controlable window then integrate the error
              if( fabsf(pidError) < PID_INTEGRAL_LIMIT )
                 pidIntegral = pidIntegral + pidError;
              else
                 pidIntegral = 0;
            }
            else
            {
               pidIntegral = 0;
            }

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_LIFT_MAX )
              pidDrive = PID_LIFT_MAX;
            if( pidDrive < PID_LIFT_MIN )
              pidDrive = PID_LIFT_MIN;

            // send to motor
            motorSet(LIFT_LEFT_M, pidDrive);
            motorSet(LIFT_RIGHT_M, -pidDrive);
        }
        else
        {
          // clear all
          pidError      = 0;
          pidLastError  = 0;
          pidIntegral   = 0;
          pidDerivative = 0;
          motorSet(LIFT_LEFT_M,0);
          motorSet(LIFT_RIGHT_M,0);
        }

        // Run at 50Hz
        wait( 25 );
    }
}
