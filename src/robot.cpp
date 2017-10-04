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
	db.distance = RESET;
	db.rotDist = RESET;
}//constructor 

void robot::forwards(float power) {
	//konstants that should be changed later
	if(power != 0) d.basePower = power;
	
	float rateOfChange = 56.15;//constant changing the amount of initial change the acceleration goes through? maibe
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
	float rateOfChange = 23.35;//constant changing the amount of initial change the acceleration goes through? maibe
							//calculate acceleration taking friction into account
	float rotAccel = 2 * (power / rateOfChange) - (p.amountOfFriction*p.rotVel);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if (abs(rotAccel) > 0.3) p.rotAcceleration = rotAccel;
	else p.rotAcceleration = 0;
	if (abs(p.rotVel) < 0.1) p.rotVel = 0;
	db.rotDist = (int)p.mRot;
}
bool robot::driveFor(float inches) {
	float power = 100 * inches;
	float rateOfChange = 56.15;//constant changing the amount of initial change the acceleration goes through? maibe
							   //calculate acceleration taking friction into account
	float Xaccel = 2 * (power / rateOfChange) - (p.amountOfFriction * p.velocity.X);
	float Yaccel = 2 * (power / rateOfChange) - (p.amountOfFriction * p.velocity.Y);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if (abs(Xaccel) > 0.01) p.acceleration.X = Xaccel;
	else p.acceleration.X = 0;
	if (abs(Yaccel) > 0.01) p.acceleration.Y = Yaccel;
	else p.acceleration.Y = 0;
	if (abs(p.velocity.X) < 0.01) p.velocity.X = 0;
	if (abs(p.velocity.Y) < 0.01) p.velocity.Y = 0;
	return true;
}
bool robot::rotFor(float degrees) {
	float power = degrees * 100;
	float rateOfChange = 23.35;//constant changing the amount of initial change the acceleration goes through? maibe
							   //calculate acceleration taking friction into account
	float rotAccel = 2 * (power / rateOfChange) - (p.amountOfFriction*p.rotVel);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if (abs(rotAccel) > 0.3) p.rotAcceleration = rotAccel;
	else p.rotAcceleration = 0;
	if (abs(p.rotVel) < 0.1) p.rotVel = 0;
	db.rotDist = (int)p.mRot;
	return true;
}
void robot::outputTextfunc() {//not weorking, all the distances are being summed and executed at once... no delays :_(
	bool ready = false;
	//testing Auton producer
	if(!ready) ready = driveFor(-6.14602); 
	if(ready) ready = rotFor(1.05199);
	if(ready) ready = driveFor(-0.500114);
	if(ready) ready = rotFor(1.54485);
	if(ready) ready = driveFor(-0.231463);
	if(ready) ready = rotFor(1.50541);
	if(ready) ready = driveFor(-0.21989);
	if(ready) ready = rotFor(1.59923);
	if(ready) ready = driveFor(-0.208895);
	if(ready) ready = rotFor(1.81966);
	if(ready) ready = driveFor(-0.198451);
	if(ready) ready = rotFor(2.16037);
	if(ready) ready = driveFor(-0.188528);
	if(ready) ready = rotFor(1.61534);
	if(ready) ready = driveFor(-0.179102);
	if(ready) ready = rotFor(2.17886);
	if(ready) ready = driveFor(-0.170147);
	if(ready) ready = rotFor(1.84551);
	if(ready) ready = driveFor(-0.161639);
	if(ready) ready = rotFor(2.42882);
	if(ready) ready = driveFor(-0.153557);
	if(ready) ready = rotFor(1.93297);
	if(ready) ready = driveFor(-0.145879);
	if(ready) ready = rotFor(2.36191);
	if(ready) ready = driveFor(-0.138585);
	if(ready) ready = rotFor(1.7194);
	if(ready) ready = driveFor(-0.131656);
	if(ready) ready = rotFor(2.00902);
	if(ready) ready = driveFor(-0.125073);
	if(ready) ready = rotFor(1.23415);
	if(ready) ready = driveFor(-0.11882);
	if(ready) ready = rotFor(1.39803);
	if(ready) ready = driveFor(-0.112879);
	if(ready) ready = rotFor(1.50372);
	if(ready) ready = driveFor(-0.107235);
	if(ready) ready = rotFor(1.55412);
	if(ready) ready = driveFor(-0.101873);
	if(ready) ready = rotFor(1.552);
	if(ready) ready = driveFor(-0.0967793);
	if(ready) ready = rotFor(1.49999);
	if(ready) ready = driveFor(-0.0919404);
	if(ready) ready = rotFor(1.40058);
	if(ready) ready = driveFor(-0.0873434);
	if(ready) ready = rotFor(1.25614);
	if(ready) ready = driveFor(-0.0829762);
	if(ready) ready = rotFor(1.06892);
	if(ready) ready = driveFor(-0.153713);
	if(ready) ready = rotFor(1.5746);
	if(ready) ready = driveFor(-0.0711417);
	if(ready) ready = rotFor(1.27146);
	if(ready) ready = driveFor(-0.13179);
	if(ready) ready = rotFor(1.56239);
	if(ready) ready = driveFor(-0.0609951);
	if(ready) ready = rotFor(1.15986);
	if(ready) ready = driveFor(-0.112993);
	if(ready) ready = rotFor(1.26667);
	if(ready) ready = driveFor(-0.101977);
	if(ready) ready = rotFor(1.26556);
	if(ready) ready = driveFor(-0.0920339);
	if(ready) ready = rotFor(1.16707);
	if(ready) ready = driveFor(-0.121503);
	if(ready) ready = rotFor(1.35723);
	if(ready) ready = driveFor(-0.0712141);
	if(ready) ready = rotFor(1.0548);
	if(ready) ready = driveFor(-0.122275);
	if(ready) ready = rotFor(1.25253);
	if(ready) ready = driveFor(-0.076577);
	if(ready) ready = rotFor(1.00263);
	if(ready) ready = driveFor(-0.104136);
	if(ready) ready = rotFor(1.02269);
	if(ready) ready = driveFor(-0.107452);
	if(ready) ready = rotFor(1.07523);
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
	p.acceleration.X += getSign(p.acceleration.X) * coneWeight * p.frictionC 
		+ getSign(p.acceleration.X) * moGoWeight * p.frictionM;//slows down acceleration when encountering friction
	p.acceleration.Y += getSign(p.acceleration.Y) * coneWeight * p.frictionC
		+ getSign(p.acceleration.Y) * moGoWeight * p.frictionM;
	p.velocity = p.velocity + p.acceleration.times(1.0/60.0);
	p.position.Y += p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	p.position.X += p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
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
	if (ctrl.ArrowKeyUp && !d.frontStop) forwards(127);//checking up key
	else if (ctrl.ArrowKeyDown && ! d.backStop) forwards(-127);//checking down key
	else if (jAnalogY != 0) forwards(truSpeed(3, -jAnalogY));//chacking analog drawing
	else forwards(0);//welp, no movement

	if (ctrl.RotLeft) rotate(127);//checking left key
	else if (ctrl.RotRight) rotate(-127);//checking right key
	else if (abs(jAnalogX) > 10) rotate(-jAnalogX);//checking analog drawing
	else rotate(0);//welp, no rotation
}
