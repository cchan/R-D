/*
Nonviolence.h
"Together, we CAN stop motor abuse."
A function to alleviate instant, jerky velocity changes in motors, instead gradually accelerating them to speed.

LastUpdatedOn: 10-29-14
LastUpdatedBy: Clive
Status: Major edits - modes and struct - untested.

Todo next possibly:
Generalize for anything - smoothly move from point A to point B. Maybe callbacks - LOLZ ROBOTC DOESN'T SUPPORT THEM. Maybe reference variables.
Does this integrate nicely? Is the distance consistent? Is there a way to apply this to encoder counts rather than speeds? (test it with encoders!)
*/


/* HOW TO USE IT
Beginning of program:
	startTask(nonviolenceTask);

When moving: (instead of motor[motorsLeft] = 100;)
	nonviolence(motorsLeft, 100);//Default (NV_DEFAULT_MODE) is (at the time of this writing) NV_MODE_LINEAR.
	nonviolence(motorsLeft, 100, NV_MODE_STEP); //Steps it directly to the target speed, with total violence. Just like motor[motorsLeft] = 100;.
	nonviolence(motorsLeft, 100, NV_MODE_LINEAR); //Linearly ramps it up/down to the target speed.
	nonviolence(motorsLeft, 100, NV_MODE_LOGISTIC); //Logistically ramps it up/down to the target speed.

Note that this uses a task. The NXT is (apparently) limited to 5 tasks, so be careful of this limit!
Note also that the NXT is not actually thread-safe, even though it has multiple "threads" running "concurrently."
This also has a ton of global constants, all prefixed with NV_.
This also contains a single function, and if you ever run up against the ROBOTC 256-function limit... what are you even doing?
*/


// STUFF YOU MIGHT WANT TO DEFINE
//Must all three be positive.
const int NV_tick = 100; //Milliseconds per clocktick of the Nonviolence task.
const int NV_acc = 1; //How much to increase/decrease the motor per NV_tick milliseconds. Serves as maximum acceleration for nonlinear modes.
const int NV_max = 90; //built in PID sometimes deals badly with motor = 100

// STUFF YOU SHOULDN'T TOUCH
//# of motors
const int NV_numMotors = kNumbOfTotalMotors;

//Constants for nonviolence modes.
const int NV_MODE_STEP = 0;
const int NV_MODE_LINEAR = 1;
const int NV_MODE_LOGISTIC = 2; //Logistic! Read about it on Wikipedia.
const int NV_DEFAULT_MODE = NV_MODE_LINEAR;

//Logistic "close enough we'll just pretend it's zero"
const int NV_LOGISTIC_CLOSEENOUGH = 2;

//Motor Properties variable structure
//typedef struct{
//	signed int mtr; //motor number ("motorsLeft" and "motorsRight" are actually stored as integers)
//	signed int initial; //Initial speed
//	signed int target; //Target speed
//	signed int mode; //Mode - see above
//	bool active;
//}NV_motorproperties;
//NV_motorproperties NV[NV_numMotors]; //declares array of motors (called "NV") of NV_motorproperties's.

signed int NV_mtr[NV_numMotors];
signed int NV_initial[NV_numMotors];
signed int NV_target[NV_numMotors];
signed int NV_mode[NV_numMotors];
bool NV_active[NV_numMotors];

/*
The "NV" Motor Queue
This queue will store the motors being acted upon. Doesn't do anything until you start calling nonviolence().
When you call nonviolence(), it adds a motor to that queue, with all the properties of the path it will follow. (motornumber, initial position, target, path mode)
Next time you call nonviolence(), that motor is already in the queue, so it'll just be modified.
When nonviolenceTask runs (once every NV_tick milliseconds), it will look at all the "active" motors in the queue and increment them along their journey.

Why do we need it? To support multiple motors running at once.
Also, because there is no way to access the list of motor numbers currently available, so they must be added on sight.
Mainly, because ROBOTC is really annoying.
*/

void nonviolence(tMotor newInputMotor, int target, int mode = NV_DEFAULT_MODE){
	//Make sure we're not doing weird stuff with over-100 values.
	target = (target > NV_max)? NV_max : (target < -NV_max)? -NV_max : target;

	for(int i = 0; i < NV_numMotors; i++)
		if(NV_mtr[i] == newInputMotor || NV_active[i] == false){ //Unnecessarily initializes some motors. But it works perfectly well.
			NV_mtr[i] = newInputMotor;
			NV_initial[i] = motor[newInputMotor];
			NV_target[i] = target;
			NV_mode[i] = mode;
			NV_active[i] = true;
			break;
		}
}

task nonviolenceTask{
	for(int i = 0; i < NV_numMotors; i++) //Initializes Nonviolence motor array
		NV_active[i] = false; //By default, none of the queued motors are active.

	while(true){
		for(int i = 0; i < NV_numMotors; i++){//Check on all the motors, inc if necessary.
			if(!NV_active[i]) continue; //This queued motor isn't active. Skip.

			float initVal = NV_initial[i];
			float currVal = motor[NV_mtr[i]];
			float target = NV_target[i];

			switch(NV_mode[i]){ //Figure out what mode is being used.
				case NV_MODE_STEP: //Go directly to target.
					motor[NV_mtr[i]] = NV_target[i];
					NV_active[i] = false;
					break;
				case NV_MODE_LINEAR: //Goes directly toward target, at acceleration of NV_acc.
					writeDebugStreamLine("motorval %d  currVal %d  target %d",motor[NV_mtr[i]],currVal,target);
					if(abs(target - currVal) <= NV_acc){ //If it's so close who cares?
						motor[NV_mtr[i]] = target; //Just set it to the target.
					//	NV_active[i] = false;
					}
					else{ //If we're appreciably above/below the target
						motor[NV_mtr[i]] += NV_acc * ((currVal < target)?1:-1); //Increase/decrease the speed of the motor by NV_acc.
					}
					break;
				case NV_MODE_LOGISTIC: //Goes in a logistic curve (slow first, faster in middle, slow down as reaching it)
					if(abs(currVal - initVal) < NV_LOGISTIC_CLOSEENOUGH){//Get it off the ground, since the logistic formula will return zero if we start at zero.
						motor[NV_mtr[i]] = currVal + NV_LOGISTIC_CLOSEENOUGH * ((currVal > initVal)?1:-1);
					}
					else if(abs(target - currVal) < NV_LOGISTIC_CLOSEENOUGH){//If it's close to endpoint, let's stop.
						motor[NV_mtr[i]] = target;
						NV_active[i] = false;
					}
					else{ //Differential equation: dy/dx = maxSpeed(diff_from_end)(diff_from_beginning)/(end - start)
						float change = (currVal - initVal) * (target - currVal) / (target - initVal);
						change *= NV_acc * (target - initVal) / 4; //Normalize it so that the highest accel (in the very middle) is exactly NV_acc
						motor[NV_mtr[i]] += change;
					}//Could this result in oscillation, with high enough acc and proper conditions??
					break;
				default:
					NV_mode[i] = NV_DEFAULT_MODE; //Set it to default and wait until next time to do anything.
					break;
			}
		}//end for loop
		wait1Msec(NV_tick);
	}//end while loop
}
