#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

//declares and defines the robot class and functions

robot::robot(vec3 p, vec3 s) : position(p), speed(s) {}

void robot::forwards(float  power) {
	speed.X = power*cos((mRot + 90)*(PI / 180));
	speed.Y = -power*sin((mRot + 90)*(PI / 180));
	if (abs(power) > 0.01)
		encoder1 += power;//increments the encoder while going forwards or backwards
}
void robot::rotateBase(float rotAmount) {
	mRot += (truSpeed(3, -rotAmount) / 50);
}
void robot::stop() {
	speed = vec3(0, 0, 0);
	mRot += 0;
}
float robot::truSpeed(int degree, float value) {//see here for reference https://www.desmos.com/calculator/bhwj2xjmef
	float exponented = value;//finished value after being taken to the degree's exponent
	int divisor = 1;
	if (degree % 2 == 0) {//if degree is even (need to multiply by sign) 
		for (int i = 1; i < degree; i++) {
			exponented *= value;
			divisor *= 127;//what the value is to be divided by, increases degree based off power
		}
		return getSign(value) * (exponented) / (divisor);
	}
	else {
		for (int i = 1; i < degree; i++) {
			exponented *= value;
			divisor *= 127;//what the value is to be divided by, increases degree based off power
		}
		return (exponented) / (divisor);
	}
}
void robot::calculatePos() {
	if (!rotating && motorPower != 0) {
		//float Magnitude = ((changeInDist) * 4 * PI) / (360);//function for adding the change in inches to current posiiton
		current.deg = mRot + 90;
		current.Xpos += cos(current.deg*(PI / 180))*(encoder1 - encoderLast);//cosine of angle times magnitude RADIANS(vector trig)//NOT WORKING
		current.Ypos -= sin(current.deg*(PI / 180))*(encoder1 - encoderLast);//sine of angle times magnitude RADIANS(vector trig)//NOT WORKING
		encoderLast = encoder1;
	}

}
float robot::PID_controller() {//accelerates and decelerates robot based on location and goal. 
	float kP = 1;//remove later
	float kI = 5;//remove later
	float kD = 1.0;//remove later
				   // If we are using an encoder then clear it
	PID.lastError = 0;
	PID.integral = 0;

	if (PID.isRunning) {
		//v.r.PID.currentPos = v.r.current.Ypos;// * sensorScale;idk if i need this, probs not.
		if (abs(position.X*ppi - PID.requestedValue) > 0.001) {
			PID.error = position.X*ppi - PID.requestedValue;//calculate error
		}
		else PID.error = 0;
		if (kI != 0) {
			if (abs(PID.error) < 50) {
				PID.integral += PID.error;//used for averaging the integral amount, later in motor power divided by 25
			}
			else {
				PID.integral = 0;
			}
		}
		else {
			PID.integral = 0;
		}
		// calculate the derivative
		PID.derivative = PID.error - PID.lastError;
		PID.lastError = PID.error;
		// calculate drive (in this case, just for the chain (ripper) )
		return(((kP * PID.error) + (PID.integral / kI) + (kD * PID.derivative)) / 127);
	}
	else {
		// clear all
		PID.error = 0;
		PID.lastError = 0;
		PID.integral = 0;
		PID.derivative = 0;
		return(0);
		// Run at 50Hz
	}
}

void robot::update() {
	position = position + speed;
}

void robot::moveAround(float jAnalogX, float jAnalogY) {
	motorPower = truSpeed(3, jAnalogY) / 127;
	forwards(motorPower);
	if (abs(jAnalogX) > 10) {//checking to see if rotation should occur.
		rotating = true;
		rotateBase(jAnalogX);
	}
	else {
		rotating = false;
	}
}
void robot::PIDControlUpdate() {
	forwards(PID_controller());
	mRot = 90;
	position.Y = 69.6;
	PID.isRunning = true;
}

void robot::NavigationUpdate() {
	PID.isRunning = false;
	calculatePos();
}

void robot::TruSpeedUpdate() {
	PID.isRunning = false;
	position.Y = 69.6;
	position.X = 69.6;
	mRot = 0;

}