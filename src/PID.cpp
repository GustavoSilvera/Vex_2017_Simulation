#include "PID.h"
#include <string>

using namespace ci;
PID::PID(robot *r) {
	pid.isRunning = true;
	isInit = false;
	r->p.acceleration.X = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.position.Y = 69.6;
	pid.requestedValue = 69.6;
}
void PID::initialize(robot *r) {
	pid.isRunning = true;
	r->p.acceleration.X = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.position.Y = 69.6;
	pid.requestedValue = 69.6;
	isInit = true;
}
void PID::textOutput(robot *r) {
	static const int columns = 5;
	int tX[columns], dInBtw = 185;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
							  //tX = 1200;
	int tY = 900;
	std::string STRING;
	float DATA;
	for (int i = 0; i < columns; i++) {
		if (i == 0) { STRING = "Req:"; DATA = pid.requestedValue / ppi; }
		else if (i == 1) { STRING = "Error:"; DATA = pid.error; }
		else if (i == 2) { STRING = "Deriv:"; DATA = pid.derivative; }
		else if (i == 3) { STRING = "Intgrl:"; DATA = pid.integral; }
		else if (i == 4) { STRING = "Full:"; DATA = controller(r); }
		
		tX[0] = 0;//initialize first button
		tX[i] = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(STRING, Vec2f(tX[i] - 70, tY), Color(1, 1, 1), Font("Arial", 30));
		drawText(DATA, vec3(tX[i], tY), vec3(1, 1, 1), 30);
	}
	gl::color(1, 0, 0);
	drawText(controller(r) / (127 * ppi), vec3(r->p.position.X*ppi, 200), vec3(1, 1, 1), 30);
	gl::drawLine(cinder::Vec2f(r->p.position.X*ppi, 300), Vec2f(pid.requestedValue, 300));
	
	gl::color(1, 1, 1);
}
float PID::controller(robot *r) {
	float kP = 1;//remove later
	float kI = 5;//remove later
	float kD = 1.0;//remove later
				   // If we are using an encoder then clear it
	pid.lastError = 0;
	pid.integral = 0;

	if (pid.isRunning) {
		//pid.currentPos = current.Ypos;// * sensorScale;idk if i need this, probs not.
		if (abs(r->p.position.X*ppi - pid.requestedValue) > 0.001) {
			pid.error = r->p.position.X*ppi - pid.requestedValue;//calculate error
		}
		else pid.error = 0;
		if (kI != 0) {
			if (abs(pid.error) < 50) pid.integral += pid.error;//used for averaging the integral amount, later in motor power divided by 25
			else pid.integral = 0;
		}
		else pid.integral = 0;
		// calculate the derivative
		pid.derivative = pid.error - pid.lastError;
		pid.lastError = pid.error;
		// calculate drive (in this case, just for the robot)
		return(((kP * pid.error) + (pid.integral / kI) + (kD * pid.derivative)));
	}
	else {
		// clear all
		pid.error = 0;
		pid.lastError = 0;
		pid.integral = 0;
		pid.derivative = 0;
		return(0);
	}
}
void PID::PIDUpdate(robot *r) {
	if (!isInit) initialize(r);//checks if not initialized
	pid.isRunning = true;
	r->p.acceleration.X = -controller(r)/(127);
	r->p.mRot = 0;
	r->p.position.Y = 69.6;
}
