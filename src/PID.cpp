#include "PID.h"

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
	r->p.velocity.X = -controller(r) / 127;
	r->p.mRot = 0;
	r->p.position.Y = 69.6;
	textOutput(r);
}
//	if (s.SimRunning == s.PIDCTRL) drawText(round(v.r.PID_controller()), vec3(v.r.p.position.X*ppi, v.r.p.position.Y*ppi - 100), vec3(1, 1, 1), 30);
