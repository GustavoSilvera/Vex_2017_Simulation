#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

//declares and defines the robot class and functions

robot::robot() {
	p.position = vec3(69.6, 69.6, 0);//initial constructor position
	c.clawSize = cRad;
	c.baseSize = d.size / 18;
	c.clawPos = c.clawSize;
	c.clawThick = 0.5;
	c.clawHeight = 2;
	c.clawSpeed = 0.5;
	c.liftSpeed = 0.1;//idk
	c.liftPos = 0;
	c.liftUp = false;
	c.liftDown = false;
	mg.clawSize = MGRad;
	mg.clawThick = d.size / 18;
	mg.clawPos = mg.clawSize;
	mg.clawHeight = 2.5;
	mg.clawSpeed = 0.5;
	mg.liftPos = 0;
	d.encoderBase = RESET;
	d.gyroBase = RESET;
}//constructor 

void robot::forwards(float power) {
	//konstants that should be changed later
	if(power != 0) d.basePower = power;
	
	float rateOfChange = 60;//constant changing the amount of initial change the acceleration goes through? maibe
	//calculate acceleration taking friction into account
	float Xaccel = 2 * (power / rateOfChange) - (p.amountOfFriction * p.velocity.X);
	float Yaccel = 2 * (power / rateOfChange) - (p.amountOfFriction * p.velocity.Y);
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
	d.gyroBase = (int)p.mRot;
}
void robot::physics::speedMult(float base, float rot) {
	velocity.X = getSign(velocity.X) * abs(velocity.X*base);
	velocity.Y = getSign(velocity.Y) * abs(velocity.Y*base);
	rotVel = getSign(rotVel) * abs(rotVel*rot);
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
void robot::reset() {
	p.acceleration.X = 0;
	p.acceleration.Y = 0;
	p.velocity.X = 0;
	p.velocity.Y = 0;
}
void robot::setVertices() {
	//gross i know, but its for calculating each vertice of the robot based off its current angle;
	//math behind is based off basic trig and 45 45 90° triangle analytic geometry
	float cosDist = (d.size / 2) * cos((-p.mRot + 135) * PI / 180) * sqrt(2);
	float sinDist = (d.size / 2) * sin((-p.mRot + 135) * PI / 180) * sqrt(2);
		db.vertices[0].X = p.position.X - cosDist;
		db.vertices[0].Y = p.position.Y + sinDist;
		db.vertices[1].X = p.position.X + sinDist;//flipped sin and cos
		db.vertices[1].Y = p.position.Y + cosDist;
		db.vertices[2].X = p.position.X + cosDist;
		db.vertices[2].Y = p.position.Y - sinDist;
		db.vertices[3].X = p.position.X - sinDist;//flipped sin and cos
		db.vertices[3].Y = p.position.Y - cosDist;
}

void robot::intake::claw(float RobSize, int type) {
	//janky animations for claw 
	clawSize = 0.1*liftPos + cRad;
	baseSize = 0.05*liftPos + RobSize / 18;
	clawHeight = 0.07*liftPos + 2;
	clawThick= 0.01*liftPos + 0.5;
	clawSpeed = 0.051*liftPos + 0.5;

	if (grabbing) { if (clawPos > RobSize / 18) clawPos -= clawSpeed; }//animation for claw close
	else { if (clawPos < clawSize) clawPos += clawSpeed; }//animation for claw open
	if (grabbing == false) holding = -1 - type*100;//reset index (TO EITHER -1 (for cones) or -101 (for MOGOS))
	if (liftDown  && liftPos > 0 && clawPos > clawSize) clawPos -= clawSpeed;
}

void robot::update() {
	float pastPosX = p.position.X;
	float pastPosY = p.position.Y;
	p.acceleration.X += getSign(p.acceleration.X) * coneWeight * p.frictionC 
		+ getSign(p.acceleration.X) * moGoWeight * p.frictionM;//slows down acceleration when encountering friction
	p.acceleration.Y += getSign(p.acceleration.Y) * coneWeight * p.frictionC
		+ getSign(p.acceleration.Y) * moGoWeight * p.frictionM;
	p.velocity = p.velocity + p.acceleration.times(1.0/60.0);
	//if (fieldSpeed) {//weird issue with how the robot is being drawn in the field update with the origin on the bottom right rather than top left
		///p.position.X -= p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
		p.position.Y += p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	//}
	//else {
		p.position.X += p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
		///p.position.Y -= p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	//}
	if ((p.velocity.X != 0 && p.velocity.Y != 0) && d.gyroBase == (int)p.mRot) {
		d.encoderBase += getSign(d.basePower)*sqrt(sqr(p.position.X - pastPosX)+sqr(p.position.Y - pastPosY));
	}
	p.rotVel += p.rotAcceleration*(1.0 / 60.0);
	p.mRot += p.rotVel;
	p.mRot = ((p.mRot / 360) - (long)(p.mRot / 360)) * 360;//only within 360° and -360° (takes the decimal portion and discards the whole number)
	robot::setVertices();
	c.claw(d.size, 0);
	mg.claw(d.size, 1);
	if (c.liftUp && c.liftPos < c.maxHeight) { c.liftPos += 4.5*c.liftSpeed; }
	else if (c.liftDown && c.liftPos > 0) { c.liftPos -= 8.5*c.liftSpeed; }//goes faster coming down
}

void robot::moveAround(float jAnalogX, float jAnalogY) {
	if (ctrl.ArrowKeyUp) forwards(127);//checking up key
	else if (ctrl.ArrowKeyDown) forwards(-127);//checking down key
	else if (jAnalogY != 0) forwards(truSpeed(3, -jAnalogY));//chacking analog drawing
	else forwards(0);//welp, no movement

	if (ctrl.RotLeft) rotate(127);//checking left key
	else if (ctrl.RotRight) rotate(-127);//checking right key
	else if (abs(jAnalogX) > 10) rotate(-jAnalogX);//checking analog drawing
	else rotate(0);//welp, no rotation
}
