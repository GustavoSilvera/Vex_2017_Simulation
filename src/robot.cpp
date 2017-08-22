#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

//declares and defines the robot class and functions

robot::robot() {
	p.position = vec3(69.6, 69.6, 0);//initial constructor position
	c.clawSize = coneRad;
	c.clawPos = c.clawSize;
	c.clawHeight = 2;
	mg.clawSize = MoGoRad;
	mg.clawPos = mg.clawSize;
	mg.clawHeight = 2.5;
}//constructor 

float limitSmall(float noLessThan, float value) {//not really working anyways. idk
	if (abs(value) >= noLessThan)
		return value;
	else return getSign(value)* noLessThan;
}

void robot::forwards(float power) {
	//konstants that should be changed later
	float rateOfChange = 45;//constant changing the amount of initial change the acceleration goes through? maibe
	//calculate acceleration taking friction into account
	float Xaccel = 2 * (power / rateOfChange) - (p.amountOfFriction*p.velocity.X);
	float Yaccel = 2 * (power / rateOfChange) - (p.amountOfFriction*p.velocity.Y);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if(abs(Xaccel) > 0.01) p.acceleration.X = Xaccel;
	else p.acceleration.X = 0;
	if (abs(Yaccel) > 0.01) p.acceleration.Y = Yaccel;
	else p.acceleration.Y = 0;
	if (abs(p.velocity.X) < 0.01) p.velocity.X = 0;
	if (abs(p.velocity.Y) < 0.01) p.velocity.Y = 0;
	//if (abs(power) > 0.01)
	//	encoder1 += power;//increments the encoder while going forwards or backwards
}

void robot::rotate(float power) {
	//konstants that should be changed later
	float rateOfChange = 15;//constant changing the amount of initial change the acceleration goes through? maibe
							//calculate acceleration taking friction into account
	float rotAccel = (power / rateOfChange) - (p.amountOfFriction*p.rotVel);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if (abs(rotAccel) > 0.3) p.rotAcceleration = rotAccel;
	else p.rotAcceleration = 0;
	if (abs(p.rotVel) < 0.1) p.rotVel = 0;

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
	if (p.rotVel == 0) {//not rotating
		//float Magnitude = ((changeInDist) * 4 * PI) / (360);//function for adding the change in inches to current posiiton
		current.deg = p.mRot;
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
		if (abs(p.position.X*ppi - PID.requestedValue) > 0.001) {
			PID.error = p.position.X*ppi - PID.requestedValue;//calculate error
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
	if (!d.reversed) {
		db.vertices[0].X = (p.position.X - ((d.size / 2) * (sqrt(2)) * cos((p.mRot - 135) * PI / 180)));
		db.vertices[0].Y = (p.position.Y + ((d.size / 2) * (sqrt(2)) * sin((p.mRot - 135)* PI / 180)));
		db.vertices[1].X = (p.position.X + (-1 * ((d.size / 2) * (sqrt(2)) * cos(180 - (p.mRot + 7.62) * PI / 180))));
		db.vertices[1].Y = (p.position.Y + (-1 * ((d.size / 2) * (sqrt(2)) * sin(180 - (p.mRot + 7.62)* PI / 180))));
		db.vertices[2].X = (p.position.X + ((d.size / 2) * (sqrt(2)) * cos((p.mRot - 135) * PI / 180)));
		db.vertices[2].Y = (p.position.Y - ((d.size / 2) * (sqrt(2)) * sin((p.mRot - 135)* PI / 180)));
		db.vertices[3].X = (p.position.X - (-1 * ((d.size / 2) * (sqrt(2)) * cos(180 - (p.mRot + 7.62) * PI / 180))));
		db.vertices[3].Y = (p.position.Y - (-1 * ((d.size / 2) * (sqrt(2)) * sin(180 - (p.mRot + 7.62)* PI / 180))));
	}
	else {//because when drawing with new vertice, entire robot gets rotated and so do all its vertices
		db.vertices[2].X = (p.position.X - ((d.size / 2) * (sqrt(2)) * cos((p.mRot - 135) * PI / 180)));
		db.vertices[2].Y = (p.position.Y + ((d.size / 2) * (sqrt(2)) * sin((p.mRot - 135)* PI / 180)));
		db.vertices[3].X = (p.position.X + (-1 * ((d.size / 2) * (sqrt(2)) * cos(180 - (p.mRot + 7.62) * PI / 180))));
		db.vertices[3].Y = (p.position.Y + (-1 * ((d.size / 2) * (sqrt(2)) * sin(180 - (p.mRot + 7.62)* PI / 180))));
		db.vertices[0].X = (p.position.X + ((d.size / 2) * (sqrt(2)) * cos((p.mRot - 135) * PI / 180)));
		db.vertices[0].Y = (p.position.Y - ((d.size / 2) * (sqrt(2)) * sin((p.mRot - 135)* PI / 180)));
		db.vertices[1].X = (p.position.X - (-1 * ((d.size / 2) * (sqrt(2)) * cos(180 - (p.mRot + 7.62) * PI / 180))));
		db.vertices[1].Y = (p.position.Y - (-1 * ((d.size / 2) * (sqrt(2)) * sin(180 - (p.mRot + 7.62)* PI / 180))));
	}
}

void robot::intake::claw(float RobSize) {
	if (grabbing) { if (clawPos > RobSize / 18) clawPos -= 0.5; }//animation for claw close
	else { if (clawPos < clawSize) clawPos += 0.5; }//animation for claw open
	if (grabbing == false) holding = -1;//reset index
}
void robot::update() {
	p.acceleration = p.acceleration + p.friction.times(-1);
	p.velocity = p.velocity + p.acceleration.times(1.0/60.0);
	if (fieldSpeed) {//weird issue with how the robot is being drawn in the field update with the origin on the bottom right rather than top left
		p.position.X -= p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
		p.position.Y += p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	}
	else {
		p.position.X += p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
		p.position.Y -= p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	}
	p.rotVel += p.rotAcceleration*(1.0 / 60.0);
	p.mRot += p.rotVel;
	p.mRot = ((p.mRot / 360) - (long)(p.mRot / 360)) * 360;//only within 360° and -360° (takes the decimal portion and discards the whole number)
	robot::setVertices();
	c.claw(d.size);
	mg.claw(d.size);
}

void robot::moveAround(float jAnalogX, float jAnalogY) {
	if (ctrl.ArrowKeyUp) forwards(127);//checking up key
	else if (ctrl.ArrowKeyDown) forwards(-127);//checking down key
	else if (jAnalogY != 0) forwards(truSpeed(3, jAnalogY));//chacking analog drawing
	else forwards(0);//welp, no movement

	if (ctrl.RotLeft) rotate(127);//checking left key
	else if (ctrl.RotRight) rotate(-127);//checking right key
	else if (abs(jAnalogX) > 10) rotate(-jAnalogX);//checking analog drawing
	else rotate(0);//welp, no rotation
}

void robot::PIDControlUpdate() {
	PID.isRunning = true;
	p.acceleration.X = 0;
	p.velocity.X = -PID_controller()/127;
	p.mRot = 0;
	p.position.Y = 69.6;
}

void robot::NavigationUpdate() {
	PID.isRunning = false;
	calculatePos();
}

void robot::TruSpeedUpdate() {
	PID.isRunning = false;
}