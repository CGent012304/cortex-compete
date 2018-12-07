/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "chassis.h"
#include "claw.h"
#include "lift.h"
#include "portdef.h" // All port defintions o nthe cortex
#include "lcd.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
 void operatorControl() {
   bool autoRun = true;

   //int power, turn;
   lcdClear(uart1);
   lcdPrint(uart1, 1, "compete - opcntrl");

   while(VEXNET_MANUAL && autoRun) {
      // CODE To test Autonomous without VEXnet switch
      // This should never be part of production code
      lcdPrint(uart1, 2, "compete - VEXNET");
      if(joystickGetDigital(1, 8, JOY_UP)) {
          autonomous();
      } else if(joystickGetDigital(1, 8, JOY_DOWN)) {
          autoRun = false;
      }
   }

   // Using PID control to set lift to a set height and should
	 taskRunLoop(liftPIDtask, LIFT_TASK_DELTA);
	 //pidRequestedValue = 400; // should be used to set the target encoder clicks
	 														// you can bind this to a button on your controller

   int power;
   int turn;
	 int left;
	 int right;

   while (1) {
         // arcade drive
				 if(ARCADE_DRIVE) {
            power = joystickGetAnalog(1, 2); // vertical axis on left joystick
            turn  = joystickGetAnalog(1, 1); // horizontal axis on left joystick
            //motorSet(2, power + turn); // set left wheels
            //motorSet(3, power - turn); // set right wheels
            chassisSet(power + turn, power - turn);
         } else {
					 /// TANK mode
					 left = joystickGetAnalog(1, 3); // vertical axis on left joystick
					 right  = joystickGetAnalog(1, 2); // horizontal axis on left joystick

					 chassisSet(left * JOY_SCALE, right * JOY_SCALE);
				 }
         // Move the claw open/close using joystick channel 4
         clawMove(joystickGetAnalog(1, 4));

         // Move the lift up and down using button 6 Up/Down
         if(joystickGetDigital(1, 6, JOY_UP)) {
           liftMove(127); // pressing up, so lift should go up
         }
         else if(joystickGetDigital(1, 6, JOY_DOWN)) {
           liftMove(-127); // pressing down, so lift should go down
         }
         else {
          liftMove(0); // no buttons are pressed, stop the lift
         }
         delay(20);
   }
 }
