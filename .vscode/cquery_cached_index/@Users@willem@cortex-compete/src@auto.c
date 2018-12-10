/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "portdef.h"
#include "lcd.h"
#include "lift.h"
#include "chassis.h"
#include "claw.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
 void autoSkill() {
   lcdClear(uart1);
   lcdPrint(uart1, 1, "Running autoSkill");
   lcdPrint(uart1, 2, "Batt: %1.3f V", (double)powerLevelMain() / 1000);
   driveForDistancePID(80, 75);
   delay(100);
   pivotTurn(1, 50, 90, 0);
   delay(200);
   return;
 }

 void autoRedLeft() {
    // Run select script
    lcdClear(uart1);
    lcdPrint(uart1, 1, "Running autoRedLeft");
    lcdPrint(uart1, 2, "Batt: %1.3f V", (double)powerLevelMain() / 1000);
    delay(200);
    return;
 }

 void autoRedRight() {
    // Run select script
    lcdClear(uart1);
    lcdPrint(uart1, 1, "Running autoRedRight");
    lcdPrint(uart1, 2, "Batt: %1.3f V", (double)powerLevelMain() / 1000);
    delay(200);
    return;
 }

 void autoBlueLeft() {
    // Run select script
    lcdClear(uart1);
    lcdPrint(uart1, 1, "Running autoBlueLeft");
    lcdPrint(uart1, 2, "Batt: %1.3f V", (double)powerLevelMain() / 1000);
    delay(200);
    return;
 }

 void autoBlueRight() {
    // Run select script
    lcdClear(uart1);
    lcdPrint(uart1, 1, "Running autoBlueRight");
    lcdPrint(uart1, 2, "Batt: %1.3f V", (double)powerLevelMain() / 1000);
    delay(200);
    return;
 }

// FIELD control autonomous function call
void autonomous() {
  lcdScriptExecute();
 }
