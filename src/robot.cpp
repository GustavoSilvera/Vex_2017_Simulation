#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"


//declares and defines the robot class and functions


robot::robot() {
	c.liftPos = 0;
	c.protrusion = 0;//not protruding from baseout
	c.liftUp = false;
	c.clawSpeed = 0.5;
	c.liftDown = false;
	mg.protrusion = 0;
	mg.liftPos = 0;
	db.distance = RESET;
	db.rotDist = RESET;
	p.position = vec3(69.6, 69.6, 0);//initial robot position
	updateFeatures();
	c.clawPos = c.clawSize;
	//vector stuff
}//constructor 
void robot::updateFeatures() {
	//physical features
	c.clawSize = d.size / 6;//3
	c.baseSize = d.size / 18;//1
	c.clawThick = d.size / 36;//0.5
	c.clawSpeed = d.size / 36;//0.5;
	c.liftSpeed = d.size / 180;//0.1
	c.clawHeight = d.size / 9;//2
	mg.clawSpeed = d.size / 36;//0.5
	mg.clawSize = d.size / 4;//4
	mg.clawThick = d.size / 18;//1
	mg.clawPos = mg.clawSize;//inf
	mg.clawHeight = (1 / 2.4)*d.size;//7.5
}
void robot::forwards(float power) {
	//konstants that should be changed later
	if(power != 0) d.basePower = power;
	
	float rateOfChange = 40;//constant changing the amount of initial change the acceleration goes through? maibe
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
void robot::readScript() {//script parser
	#define MAX_LINE 100
	readyToRun = false;
	std::ifstream file("script.txt");
	while (!file.eof()) {
		char line[MAX_LINE];
		file.getline(line, MAX_LINE);
		char command[MAX_LINE];
		char num[MAX_LINE];
		char garb[MAX_LINE];

		sscanf(line, "%s %s %s", command, num, garb);
		enum action a;
		if (std::string(command) == "driveFor(") {
			a = ACTION_FWDS;
		}
		else {
			a = ACTION_ROTATE;
		}
		commands.push_back({a, atof(num)});
	}
	readyToRun = true;
}
void robot::rotate(float power) {
	//konstants that should be changed later
	float rateOfChange = 10;//constant changing the amount of initial change the acceleration goes through? maibe
							//calculate acceleration taking friction into account
	float rotAccel = 2 * (power / rateOfChange) - (p.amountOfFriction*p.rotVel);
	//limiting acceleration to 0.01, no need for further acceleration rly
	if (abs(rotAccel) > 0.3) p.rotAcceleration = rotAccel;
	else p.rotAcceleration = 0;
	if (abs(p.rotVel) < 0.1) p.rotVel = 0;
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
bool robot::directlyInPath(bool vertical, int range, vec3 pos) {//vertical lines
	vec3 origin = pos, topLeft, topRight, bottomLeft;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	float cosDist = (range / 2) * cos((-p.mRot + 135) * PI / 180) * sqrt(2);
	float sinDist = (range / 2) * sin((-p.mRot + 135) * PI / 180) * sqrt(2);
	float protrusionSin = mg.protrusion * sin((p.mRot + 90) * PI / 180)*0.75;
	float protrusionCos = mg.protrusion * cos((p.mRot + 90) * PI / 180)*0.75;
	topLeft.X = p.position.X - cosDist;
	topLeft.Y = p.position.Y + sinDist;
	topRight.X = p.position.X + sinDist;//flipped sin and cos
	topRight.Y = p.position.Y + cosDist;
	//added protrusion snippet to account for size change of base as mogo is flipped out
	bottomLeft.X = p.position.X - sinDist;// -protrusionSin;//flipped sin and cos
	bottomLeft.Y = p.position.Y - cosDist;// +protrusionCos;
	if (vertical) {
		db.slope = (topLeft.Y - bottomLeft.Y) / (topLeft.X - bottomLeft.X);//checks for vertical y intercepts
		db.Yint[1] = db.slope * (x - (topRight.X - origin.X)) + (topRight.Y - origin.Y);
	}
	else {
		db.slope = (topLeft.Y - topRight.Y) / (topLeft.X - topRight.X);//checks for horizontal y intercepts
		db.Yint[1] = db.slope * (x - (bottomLeft.X - origin.X)) + (bottomLeft.Y - origin.Y);
	}	
	db.Yint[0] = db.slope * (x - (topLeft.X - origin.X)) + (topLeft.Y - origin.Y);//both vertical and horizontal use vertices 0
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(db.Yint[0]) != getSign(db.Yint[1]));//works for telling me if between the two lines
}
void robot::setVertices() {
	//gross i know, but its for calculating each vertice of the robot based off its current angle;
	//math behind is based off basic trig and 45 45 90° triangle analytic geometry
	float cosDist = (d.size / 2) * cos((-p.mRot + 135) * PI / 180) * sqrt(2);
	float sinDist = (d.size / 2) * sin((-p.mRot + 135) * PI / 180) * sqrt(2);
	float protrusionSin = mg.protrusion * sin((p.mRot+90) * PI / 180);
	float protrusionCos = mg.protrusion * cos((p.mRot+90) * PI / 180);
	float mogoProp = 2.5*(mg.clawSize) / d.size;//proportion of MOGO size to robot
		db.vertices[0].X = p.position.X - cosDist;
		db.vertices[0].Y = p.position.Y + sinDist;
		db.vertices[1].X = p.position.X + sinDist;//flipped sin and cos
		db.vertices[1].Y = p.position.Y + cosDist;
		db.vertices[2].X = p.position.X + cosDist;
		db.vertices[2].Y = p.position.Y - sinDist;
		db.vertices[3].X = p.position.X - sinDist;//flipped sin and cos
		db.vertices[3].Y = p.position.Y - cosDist;
		//mogo verticies
		float centerCos = cos((p.mRot) * PI / 180) * 2;
		float centerSin = sin((p.mRot) * PI / 180) * 2;

		db.MGVert[0].X = p.position.X - centerCos - mogoProp*cosDist - protrusionSin;
		db.MGVert[0].Y = p.position.Y - centerSin + mogoProp*sinDist + protrusionCos;
		db.MGVert[1].X = p.position.X - centerCos + mogoProp*sinDist - protrusionSin;//flipped sin and cos
		db.MGVert[1].Y = p.position.Y - centerSin + mogoProp*cosDist + protrusionCos;
		db.MGVert[2].X = p.position.X - centerCos + mogoProp*cosDist - protrusionSin;
		db.MGVert[2].Y = p.position.Y - centerSin - mogoProp*sinDist + protrusionCos;
		db.MGVert[3].X = p.position.X - centerCos - mogoProp*sinDist - protrusionSin;//flipped sin and cos
		db.MGVert[3].Y = p.position.Y - centerSin - mogoProp*cosDist + protrusionCos;
}
void robot::intake::claw(float robSize) {
	//janky animations for claw 
	//clawSize = 0.1*liftPos + cRad;
	baseSize = 0.05*liftPos + robSize / 18;
	clawHeight = 0.07*liftPos + 2;
	clawThick = 0.01*liftPos + 0.5;
	clawSpeed = 0.051*liftPos + 0.5;

	if (grabbing) { if (clawPos > robSize / 18) clawPos -= clawSpeed; }//animation for claw close
	else { if (clawPos < clawSize) clawPos += clawSpeed; }//animation for claw open
	if (grabbing == false) holding = -1;//reset index (TO -1 (for cones) )
	if (liftDown  && liftPos > 0 && clawPos > clawSize) clawPos -= clawSpeed;
}

void robot::intake::mogo(float robSize) {
	//what i can do is change the back vertices of the robot to extend up to where the mogo protrudes to
	//this would allow me to keep the same physics i have but also have the cones and stuff interact with the mogo thing
	//k whatever ill do it below
	
	if (grabbing) { 
		if (protrusion < clawHeight) protrusion += (robSize/18)*0.3; 
		holding = -1;
	}//animation for protrusion mogo
	else { 
		if (protrusion > 0) protrusion -= (robSize/18)*0.3;
	}//animation for intruding mogo
	//if (grabbing == false) holding = -1;//reset index (TO -1 (for mogos) )
}

void robot::update() {
	p.acceleration.X += getSign(p.acceleration.X) * coneWeight * p.frictionC 
		+ getSign(p.acceleration.X) * moGoWeight * p.frictionM;//slows down acceleration when encountering friction
	p.acceleration.Y += getSign(p.acceleration.Y) * coneWeight * p.frictionC
		+ getSign(p.acceleration.Y) * moGoWeight * p.frictionM;
	p.velocity = p.velocity + p.acceleration.times(1.0/60.0);
	p.position.Y += p.velocity.Y * sin((p.mRot)*(PI / 180));//velocity scaled because of rotation
	p.position.X += p.velocity.X * cos((p.mRot)*(PI / 180));//velocity scaled because of rotation
	p.rotVel += 2*p.rotAcceleration*(1.0 / 60.0);//constant is based off realistic tests
	p.mRot += p.rotVel;
	p.mRot = ((p.mRot / 360) - (long)(p.mRot / 360)) * 360;//only within 360° and -360° (takes the decimal portion and discards the whole number)
	robot::setVertices();
	c.claw(d.size);
	mg.mogo(d.size);
	if (c.liftUp && c.liftPos < c.maxHeight) { c.liftPos += 4.5*c.liftSpeed; }
	else if (c.liftDown && c.liftPos > 0) { c.liftPos -= 8.5*c.liftSpeed; }//goes faster coming down
	updateFeatures();
}

void robot::moveAround(float jAnalogX, float jAnalogY) {
	if (ctrl.ArrowKeyUp && !d.frontStop) forwards(d.motorSpeed);//checking up key
	else if (ctrl.ArrowKeyDown && ! d.backStop) forwards(-d.motorSpeed);//checking down key
	else if (jAnalogY != 0) forwards(truSpeed(3, -jAnalogY));//chacking analog drawing
	else forwards(0);//welp, no movement

	if (ctrl.RotLeft) rotate(d.motorSpeed);//checking left key
	else if (ctrl.RotRight) rotate(-d.motorSpeed);//checking right key
	else if (abs(jAnalogX) > 10) rotate(-jAnalogX);//checking analog drawing
	else rotate(0);//welp, no rotation
}
