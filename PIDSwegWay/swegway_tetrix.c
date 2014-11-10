#pragma config(Hubs,  S3, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     forwardsTilt,   sensorI2CHiTechnicGyro)
#pragma config(Sensor, S2,     accel,          sensorI2CCustom)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     left,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     right,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S3_C2_1,    rearFlipper,         tServoStandard)
#pragma config(Servo,  srvo_S3_C2_2,    frontFlipper,          tServoStandard)
#pragma config(Servo,  srvo_S3_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/hitechnic-gyro.h"
#include "drivers/hitechnic-accelerometer.h"
#include "JoystickDriver.c"
#include "../lib/PID.h"

/*
SCHPEEEEEL on how this works
So Gyro measures the rotational velocity (to pretty good precision!).
Our goal is to keep the rotational position at 90 degrees vertical.

Therefore, PID! PID stands for Proportional, Integral, Derivative, which are the three components. Using
these three components, the algorithm decides how to act. Proportional is the current error. Most of the
correction involved is going to be looking at the current error, just because that makes sense. Integral
is more or less the average error, which helps account for things like gyro drift. Derivative is current
change in error - if you kick the balancer it'll have a sudden change in error, and Proportional will be
unable to cope, but Derivative will see a large change in error and help correct any oscillations.

Wikipedia has an even simpler and quite insightful way of explaining it: "P" corrects for current error,
"I" for past error, and "D" for future error. In that way, it will very effectively maintain balance.
*/

//Forward declarations
task gyros();
void initSweg();
task steer();

//Degree headings for 180-degree "lift" servos
const int REAR_LIFT_RAISED = 240;
const int REAR_LIFT_DEPRESSED = 40;
const int FRONT_LIFT_RAISED = abs(REAR_LIFT_RAISED - 255);
const int FRONT_LIFT_DEPRESSED = abs(REAR_LIFT_DEPRESSED - 255);

const float maxNeutral = 20.0; //Maximum degrees offset for driving.

//Degrees
float forwardsAngle = 0;//Current error from 90 degrees
float leftNeutral = 0;//Target angle for left wheel
float rightNeutral = 0;//Target angle for right wheel

task main() {
	//Set up precision speed control (which, incidentally, uses PID to do it :D)
	nMotorPIDSpeedCtrl[left] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[right] = mtrSpeedReg;

	//Initialize PIDs.
	PID leftPID,rightPID,accelPID;
	initPID(leftPID,3.5,0,0);
	initPID(rightPID,3.5,0,0);
	initPID(accelPID,2,0,0);

	INTR velIntr;
	initIntr(velIntr);
	//Tuning tips: http://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
	//Also, double PID loops: http://forum.arduino.cc/index.php?topic=197688.0
	//PID for physical position, not just rotational? (need accel)
	//Swag: https://www.youtube.com/watch?v=n_6p-1J551Y
	//Maybe we should try a unisphere balancing robot :) https://www.youtube.com/watch?v=bI06lujiD7E

	//Stands up and initiates gyro stuff.
	initSweg();

	//waitForStart();

	//Initialize steering, raise arms... let's start!!
	StartTask(steer);
	servo[rearFlipper] = REAR_LIFT_RAISED;
	servo[frontFlipper] = FRONT_LIFT_RAISED;
EndTimeSlice();
 	while(true) {
 		int ax,ay,az;
 		HTACreadAllAxes(accel, ax, ay, az);
 		float yvel = integrate(velIntr,ay);


		if(abs(forwardsAngle) > 50) {//If it fell over, reset and redo.
 			reset(leftPID);
			reset(rightPID);
			initSweg();
			servo[rearFlipper] = REAR_LIFT_RAISED;
			servo[frontFlipper] = FRONT_LIFT_RAISED;
 		}

		//Separate left and right PIDs to allow steering by NeutralAngle adjustment.
		motor[left] = updatePID(leftPID, leftNeutral - forwardsAngle);
		motor[right] = updatePID(rightPID, rightNeutral - forwardsAngle);

		motor[left] = -motor[right];

		wait1Msec(5);
 	}
}

//Neutralize joystick drift (sometimes it isn't zero when you release the stick, due to mechanical error)
float norm(float joyval){if(abs(joyval) < 10) return 0; else return joyval;}

task steer() {//Uses joyBtns to change speed by adjusting neutral-points for left and right.
	while(true) {
		getJoystickSettings(joystick);

		//Single-joystick drive - forward-backward is speed, left-right is turn. (Joystick vals are from -128 to +127.)
		leftNeutral = (norm(joystick.joy1_y1) + norm(joystick.joy1_x1)) * maxNeutral / 128.0;
		rightNeutral = (norm(joystick.joy1_y1) - norm(joystick.joy1_x1)) * maxNeutral / 128.0;

		wait1Msec(5);
	}
}

task gyros() {//Updates the forwardsAngle global variable (degrees) with the current deviation from 90 degrees.
	nSchedulePriority = kHighPriority;//This exemplifies my hate of ROBOTC, but it pretty much just sets it to high CPU priority.

	INTR intr;//Integrator.
	initIntr(intr);

	forwardsAngle = 0;

	while(true) {//Constantly integrating gyro rotational velocity reading in degrees/sec to get current heading in degrees.
		forwardsAngle = integrate(intr, HTGYROreadRot(forwardsTilt));
		wait1Msec(5);
	}
}

void initSweg() {
	//Stop moving.
	motor[left] = 0;
	motor[right] = 0;

	//Stand up.
	servo[rearFlipper] = REAR_LIFT_DEPRESSED;
	servo[frontFlipper] = FRONT_LIFT_DEPRESSED;

	//Wait for it to stand all the way up.
	wait1Msec(2700);

	//Recalibrate the gyro.
	StopTask(gyros);
	PlaySound(soundBeepBeep);//"Starting calibration!"
	HTGYROstartCal(forwardsTilt);
	wait1Msec(1000);//Give it time to calibrate
	PlaySound(soundBeepBeep);//"Done calibrating!"

	//K, finished with gyro calibration. Start measuring angle now.
	StartTask(gyros);

	//Not sure what these two lines are for.
	nMotorEncoder[left] = 0;
	nMotorEncoder[right] = 0;
}
