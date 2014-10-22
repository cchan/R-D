#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motorA,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorB,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
NonviolenceTest.c
"Together, we CAN stop motor abuse."
Testing Nonviolence.

LastUpdatedOn: 10-22-14
LastUpdatedBy: Clive
Status: IT WORKS.
*/

#include "Nonviolence.c"

task main(){
	startTask(nonviolenceTask);

	const int x = 7000;
	writeDebugStreamLine("%d %d %d",motorA,motorB,motorC);// 3 4 2...

	motor[motorA]=50;
	wait1Msec(x);

	PlaySound(soundBeepBeep);
	nonviolence(motorA,100);
	wait1Msec(x);
	PlaySound(soundBeepBeep);
	nonviolence(motorA,0);
	wait1Msec(x);
	PlaySound(soundBeepBeep);

	return;
}
