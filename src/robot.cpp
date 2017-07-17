#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

//declares and defines the robot class and functions

robot::robot(vec3 p, vec3 s) : position(p), speed(s) {}
float limitSmall(float noLessThan, float value) {//not really working anyways. idk
	if (abs(value) >= noLessThan)
		return value;
	else return getSign(value)* noLessThan;
}
void robot::forwards(float power) {
	float rateOfChange = 45;//constant changing the amount of initial change the acceleration goes through? maibe
	float amountOfFriction = 3;//constant changing the amount of friction for the robot

	acceleration.X = limitSmall(0.01, 2 * power*cos((ActualHeading)*(PI / 180)) / rateOfChange - amountOfFriction*speed.X);
	acceleration.Y = limitSmall(0.01, -2*power*sin((ActualHeading)*(PI / 180))/ rateOfChange - amountOfFriction*speed.Y);
	
	//if (abs(power) > 0.01)
	//	encoder1 += power;//increments the encoder while going forwards or backwards
}
void robot::rotateBase(float rotAmount) {
	mRot += (truSpeed(3, -rotAmount) / 50);
}
void robot::stop() {
	//speed = vec3(0, 0, 0);
	//mRot += 0;
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
	if (!rotating) {
		//float Magnitude = ((changeInDist) * 4 * PI) / (360);//function for adding the change in inches to current posiiton
		current.deg = ActualHeading;
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
		//PID.currentPos = current.Ypos;// * sensorScale;idk if i need this, probs not.
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
		// calculate drive (in this case, just for the robot)
		return(((kP * PID.error) + (PID.integral / kI) + (kD * PID.derivative)));
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
void robot::setVertices() {
	//gross i know, but its for calculating each vertice of the robot based off its current angle;
	//math behind is based off basic trig and 45 45 90° triangle analytic geometry
	vertices[0].X = (position.X - ( ( size / 2) * (sqrt(2)) * cos((ActualHeading - 135) * PI / 180 ) ) );
	vertices[0].Y = (position.Y + ( ( size / 2) * (sqrt(2)) * sin((ActualHeading - 135)* PI / 180 ) ) );
	vertices[1].X = (position.X + (-1 * ( (size / 2) * (sqrt(2)) * cos(180 - (ActualHeading + 7.62) * PI / 180 ) ) ) );
	vertices[1].Y = (position.Y + (-1 * ( (size / 2) * (sqrt(2)) * sin(180 - (ActualHeading + 7.62)* PI / 180 ) ) ) );
	vertices[2].X = (position.X + ( ( size / 2) * (sqrt(2)) * cos((ActualHeading - 135) * PI / 180 ) ) );
	vertices[2].Y = (position.Y - ( ( size / 2) * (sqrt(2)) * sin((ActualHeading - 135)* PI / 180 ) ) );
	vertices[3].X = (position.X - (-1 * ( (size / 2) * (sqrt(2)) * cos(180 - (ActualHeading + 7.62) * PI / 180 ) ) ) );
	vertices[3].Y = (position.Y - (-1 * ( (size / 2) * (sqrt(2)) * sin(180 - (ActualHeading + 7.62)* PI / 180 ) ) ) );

}
void robot::update() {
	
	speed = speed + acceleration.times(1.0/60.0);
	position = position + speed;
	ActualHeading = mRot + 90;
	robot::setVertices();
}

void robot::moveAround(float jAnalogX, float jAnalogY) {
	if (ArrowKeyUp) forwards(127);
	else if (ArrowKeyDown) forwards(-127);
	else if (jAnalogY != 0) forwards(truSpeed(3, jAnalogY));
	else forwards(0);

	if (RotLeft) rotateBase(-127);
	if (RotRight) rotateBase(127);

	//forwards(motorPower/127);//used to move the robot forwards or backwards
	if (abs(jAnalogX) > 10) {//checking to see if rotation should occur.
		rotating = true;
		rotateBase(jAnalogX);
	}
	else {
		rotating = false;
	}
}
void robot::deceleration(int timePassed) {
	float power = 127;
	if (abs(power) > 0) {
		power = 127/timePassed;
		forwards(power);
	}
}
void robot::PIDControlUpdate() {
	PID.isRunning = true;
	acceleration.X = 0;
	speed.X = -PID_controller()/127;
	ActualHeading = 0;
	position.Y = 69.6;
}

void robot::NavigationUpdate() {
	PID.isRunning = false;
	calculatePos();
}

void robot::TruSpeedUpdate() {
	PID.isRunning = false;
}