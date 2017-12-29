#include "field.h"

//declares and defines the field class and functions

//constructor
// Map from vex image coordinates to Robot coordinates.
vec3 V2R(vec3 vec_coord) {
	const int fieldSizeIn = 141.05;  // Field size in inches.
	return vec3(fieldSizeIn - vec_coord.X, vec_coord.Y, vec_coord.Z);
}
//initial cone values for position
field::cone initConeConfig[] = {//array for each configuration of the cone (in field.h)
										   //{initial posision (X, Y), color (Y, R, B), radii }
	{ V2R({ 2.9, 13.0 })},{ V2R({ 2.9, 23.2 })},{ V2R({ 2.9, 34.9 })},{ V2R({ 2.9, 46.7 })},
	{ V2R({ 2.9, 58.4 })},{ V2R({ 2.9, 70.2 })},{ V2R({ 13.0, 2.9 })},{ V2R({ 13.0, 13.0 })},
	{ V2R({ 13.0, 23.2 })},{ V2R({ 23.2, 2.9 })},{ V2R({ 23.2, 13.0 })},{ V2R({ 23.2, 23.2 })},
	{ V2R({ 23.2, 34.9 })},{ V2R({ 23.2, 46.7 })},{ V2R({ 23.2, 58.4 })},{ V2R({ 23.2, 70.2 })},
	{ V2R({ 34.9, 2.9 })},{ V2R({ 34.9, 23.2 })},{ V2R({ 46.7, 2.9 })},{ V2R({ 46.7, 23.2 })},
	{ V2R({ 46.7, 46.7 })},{ V2R({ 46.7, 58.4 })},{ V2R({ 58.4, 2.9 })},{ V2R({ 58.4, 23.2 })},
	{ V2R({ 58.4, 46.7 })},{ V2R({ 58.4, 70.2 })},{ V2R({ 70.2, 2.9 })},{ V2R({ 70.2, 23.2 })},
	{ V2R({ 70.2, 58.4 })},{ V2R({ 70.2, 82.1 })},{ V2R({ 82.1, 70.2 })},{ V2R({ 82.1, 93.9 })},
	{ V2R({ 82.1, 139.2 })},{ V2R({ 93.9, 82.1 })},{ V2R({ 93.9, 93.9 })},{ V2R({ 93.9, 117.5 })},
	{ V2R({ 93.9, 137.8 })},{ V2R({ 105.8, 117.5 })},{ V2R({ 105.8, 137.8 })},{ V2R({ 117.5, 93.9 })},
	{ V2R({ 117.5, 105.8 })},{ V2R({ 117.5, 117.5 })},{ V2R({ 117.5, 127.6 })},{ V2R({ 117.5, 137.8 })},
	{ V2R({ 127.6, 117.5 })},{ V2R({ 127.6, 127.6 })},{ V2R({ 127.6, 137.8 })},{ V2R({ 137.8, 82.1 })},
	{ V2R({ 137.8, 93.9 })},{ V2R({ 137.8, 105.8 })},{ V2R({ 137.8, 117.5 })},{ V2R({ 137.8, 127.6 })},
	{ V2R({ 137.8, 137.8 })}
};
int numCones = sizeof(initConeConfig) / sizeof(field::cone);
//initializing mogos and stuff
field::MoGo initMoGoConfig[] = {//array for each configuration of the mobile goal (in field.h)
	{ V2R({ 34.9, 13.0 }), 1  },{ V2R({ 13.0, 34.9 }), 2  },
	{ V2R({ 70.2, 46.7 }), 2  },{ V2R({ 46.7, 70.2 }), 1  },
	{ V2R({ 93.9, 70.2 }), 2  },{ V2R({70.2, 93.9 }), 1  },
	{ V2R({ 127.6, 105.8 }), 1  },{ V2R({105.8, 127.6 }), 2  }
};
int numMoGos = sizeof(initMoGoConfig) / sizeof(field::MoGo);
//initializing stagos (poles) and stuff
field::stago initPoleConfig[] = {
	{ V2R({ 93, 47.3 }), 4, 24},{ V2R({ 46.9, 94 }), 4, 24}
};
int numPoles = sizeof(initPoleConfig) / sizeof(field::stago);
//initialize the entire field
field::field(std::vector<robot> *r) : isInit(true) {
	field::initialize(r);
}
//initialized everything for the field, such as cone and mogo values, and robot posision
void field::initialize(std::vector<robot> *r) {
	c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
	pl.assign(&initPoleConfig[0], &initPoleConfig[numPoles]);
	const int num_robots = 4;
	
	const int initAngle[num_robots] = { 45, 225, 225, 45 };
	const int initX[num_robots] = { 15, 97, 128, 45 };
	const int initY[num_robots] = { 45, 128, 97, 15 };

	for (int i = 0; i < (*r).size(); i++) {
		(*r)[i].stopAll();
		vec3 initPos(initX[i % num_robots], initY[i % num_robots], initAngle[i % num_robots]);//sets x, y, and angle
		(*r)[i].setPos(initPos);
	}

	//(*r)[2].reset();
	//(*r)[2].p.position.X = (*r)[1].p.position.Y;//equidistant and symmetric from friend
	//(*r)[2].p.position.Y = (*r)[1].p.position.X;
	//(*r)[2].p.mRot = 225;

	fieldInit = false;
	isInit = true;//so that this only gets called ONCE when the field tab is running
}
//calculate overall score given mogos in zones and cone stacks
int field::calculateScore() {
	int score = 0;
	for (int i = 0; i < mg.size(); i++) {
		score += 2*mg[i].stacked.size();//each stacked cone is worth 2 points
	}
	for (int i = 0; i < 2; i++) {
		score += 2*pl[i].stacked.size();//each stacked cone is worth 2 points
	}
	for (int i = 0; i < 2; i++) {//calculates score from mobile goals in zones
		score += f.z[i].fivePoint.size() * 5;
		score += f.z[i].tenPoint.size() * 10;
		score += f.z[i].twentyPoint.size() * 20;
	}
	return score;
}
//returns the distance from a point to the robots edge, given the two lengths of vertice distance
float calcD2Edge(float a, float b, robot *r, float prop) {/*not taking into account the protrusion of mogo*/
	//EXPLANATION HERE:
	//prop is for proportion of size of the square, like mogo (clawsize/size)vs full robot base(1)
	float C1 = ( ( (sqr(a) - sqr(b)) / (prop*r->getSize())) + (prop*r->getSize())) / 2;
	return sqrt(abs(sqr(a) - sqr(C1)));
}
//struct for vertice distances and functions about them (inFront) (onRight)
struct dist2Vert {
	float v[4];
	float sortedV[4];//created copy of vals to not directly affect anything badly
	bool inFront() {
		return (v[0] + v[1] < v[2] + v[3]);//checking if cone is closer to the front side
	}
	bool onRight() {
		return (v[1] + v[2] < v[0] + v[3]);//checking if cone is closer to the right side
	}
	inline void SortSmallest() {//finds smallest of the list provided
		sortedV[0] = v[0];
		sortedV[1] = v[1];
		sortedV[2] = v[2];
		sortedV[3] = v[3];
		std::sort(sortedV, sortedV + sizeof(v)/sizeof(float));
	}
	inline int sortSmallVER() {
		SortSmallest();
		for (int i = 0; i < sizeof(v)/sizeof(float); i++) {
			if (sortedV[0] == v[i]) return i;//if smallest value is equal to val
		}
	}
};
//pushes the elements against the field walls
void field::element::fencePush(fence *f) {
	float d2Top = f->fieldSizeIn - pos.Y;
	float d2Right = f->fieldSizeIn - pos.X;
	if (pos.X <= (radius + f->depthIn)) //checking left side
		pos.X += (radius + f->depthIn) - pos.X;
	else if (d2Right <= (radius + f->depthIn)) //checking right side
		pos.X -= (radius + f->depthIn) - d2Right;
	if (d2Top <= (radius + f->depthIn))//checking top
		pos.Y -= (radius + f->depthIn) - d2Top;
	else if (pos.Y <= (radius + f->depthIn)) //checking bottom
		pos.Y += (radius + f->depthIn) - pos.Y;
}
//checking if current angle is within a certain range
bool withinAngle(double angle, int lowerBound, int upperBound) {
	//checks if angle is within upper and lower
	int thresh = 2;//degrees of freedom
	return (angle < upperBound - thresh && angle > lowerBound + thresh) || (angle + 180 < upperBound - thresh && angle + 180 > lowerBound + thresh || (angle + 360 < upperBound - thresh && angle + 360 > lowerBound + thresh));
	//checks both the positive and "negative" angle
}
//finds closest point from a x, y, position and the robot's edge
vec3 findClosest(robot *r, vec3 pos, dist2Vert *d2V, float prop) {
	vec3 closestPoint;
	float mogoSide = 0;
	if (prop != 1) mogoSide = r->mg.protrusion * 2;//for mogo collisions, when not dealing with entire robot
	d2V->SortSmallest();
	float d2RobotEdge = calcD2Edge(d2V->sortedV[0], d2V->sortedV[1], r, prop);//calculates the distance to the edge of the r
	if (r->directlyInPath(true, prop * r->getSize(), pos)) {//either directly in front or behing based off center x and y position
		if (d2V->inFront()) closestPoint = vec3(pos.X - (d2RobotEdge)*cos(gAngle), pos.Y + (d2RobotEdge)*sin(gAngle));//does work
		else closestPoint = vec3(pos.X + (d2RobotEdge)*cos(gAngle), pos.Y - (d2RobotEdge)*sin(gAngle));//does work
	}
	//had to inverse x and y because horiontal lines
	else if (r->directlyInPath(false, r->getSize() + mogoSide, pos)) {
		if (d2V->onRight()) closestPoint = vec3(pos.X + (d2RobotEdge)*sin(gAngle), pos.Y + (d2RobotEdge)*cos(gAngle));//does work
		else closestPoint = vec3(pos.X - (d2RobotEdge)*sin(gAngle), pos.Y - (d2RobotEdge)*cos(gAngle));//does work
	}
	else {//not directly in path finds which vertice is the closest to the cone
		int smallest_vertice = d2V->sortSmallVER();
		if(prop == 1)//dealing with entire robot collisions (not just mogo)
			closestPoint = r->db.vertices[smallest_vertice];//closest point to center will then be the vertice
		else closestPoint = r->db.MGVert[smallest_vertice];//closest point to center will then be the vertice
	}
	return closestPoint;
}
//colliding an element with a robot against the closest point
void field::element::collideWith(robot *r, vec3 closestPoint, int type, int index, int roboIndex) {
	vec3 R = (closestPoint + pos.times(-1)).times(radius / pos.distance(closestPoint)) + pos;
	pos.X -= R.X - closestPoint.X;
	pos.Y -= R.Y - closestPoint.Y;
	//pushback for robot
	float weight = 0.0;
	if (type == MOGO) weight = moGoWeight;
	else if(type == CONE) weight = coneWeight;
	else weight = 1;
	if (inPossession.find(roboIndex) == inPossession.end() && !r->mg.grabbing ) {//makes sure not to pushback robot if picking up a mogo
		r->p.position.X += weight * (R.X - closestPoint.X);
		r->p.position.Y += weight * (R.Y - closestPoint.Y); 
		if(type == STAGO) r->p.velocity = vec3(0, 0, 0);
	}
	else if (type == CONE) {
		r->p.position.X += weight * (R.X - closestPoint.X);
		r->p.position.Y += weight * (R.Y - closestPoint.Y);
	}
	else if (type == STAGO) {//still push on stago
		r->p.position.X += weight * (R.X - closestPoint.X);
		r->p.position.Y += weight * (R.Y - closestPoint.Y);
		r->p.velocity = vec3(0, 0, 0);
	}
}
//colliding an element with a robot
void field::element::robotColl(int index, robot *r, int type, fence *f, int roboIndex) {
	//collisions from robot
	float d2Robot = pos.distance(r->p.position);
	dist2Vert ver;
	bool inPositionMoGo = false;
	if (pos.Z < height && d2Robot < renderRad * r->getSize()) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
		for (int i = 0; i < sizeof(ver.v)/sizeof(float); i++) {
			ver.v[i] = pos.distance(r->db.vertices[i]);//defines all the distance variables
		}	
		vec3 closestPoint = findClosest(r, pos, &ver, 1);//calculates the closest point given the vertices
		float d2closestPoint = pos.distance(closestPoint);
		if (type == MOGO &&//is mogo?
			abs(r->mg.protrusion - 7.5)<0.5 &&//mogo is at low position
			ver.inFront() &&//in front
			r->directlyInPath(true, r->getSize() / 4, pos) && //directly in front or back
			d2closestPoint >=0.95* radius && //isnt going inside full r
			d2closestPoint <= 1.5*radius) {
			inPositionMoGo = true;
			if (!r->mg.grabbing) {
				r->mg.holding = index+100;
				inPossession.insert(roboIndex);//only locks in when bringing mogo up (grabbing == false)
			}
		}
		else if (!inPositionMoGo && d2closestPoint < radius) {//touching
			collideWith(r, closestPoint, type, index, roboIndex);
		}
	}
	float mogoProp = 2.5*(r->mg.size) / r->getSize();//proportion of MOGO size to robot
	if (ver.inFront()) {//only do mogo(mech) calcs if in front
		float d2MoGo = pos.distance(
			vec3(
				r->p.position.X + r->mg.protrusion * cos((r->p.mRot) * PI / 180),
				r->p.position.Y + r->mg.protrusion * sin((r->p.mRot) * PI / 180)
			));
		if (pos.Z < height && d2MoGo < renderRad * r->getSize()/2) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
			dist2Vert verMoGo;
			for (int i = 0; i < sizeof(verMoGo.v) / sizeof(float); i++) {
				verMoGo.v[i] = pos.distance(r->db.MGVert[i]);
			}
			vec3 closestPointMOGO = findClosest(r, pos, &verMoGo, mogoProp);//calculates the closest point given the vertices
			bool mogoInReady = (type == MOGO && r->directlyInPath(true, r->mg.size, pos));
			if (!mogoInReady) {
				if (pos.distance(closestPointMOGO) <= radius) {//touching
					collideWith(r, closestPointMOGO, type, index, roboIndex);
				}
			}
		}
	}
	if (type == MOGO && r->mg.holding == index+100) {//specific robot that is HOLDING the mogo (else randomly switches)
		if (inPossession.find(roboIndex) != inPossession.end()) {//when doing the fancy animations (brings mogo into r)
			pos.X = r->p.position.X + r->mg.protrusion * cos((r->p.mRot) * PI / 180) * 2;
			pos.Y = r->p.position.Y + r->mg.protrusion * sin((r->p.mRot) * PI / 180) * 2;
		}
		if (abs(r->mg.protrusion - 7.5) < 0.5 && r->mg.grabbing) {//when bringing the mogo down
			inPossession.erase(roboIndex);//no longer locked onto mogo
			r->mg.holding = -1;//not holding anything (AFTER PUTS MOGO ON GROUND)
		}
	}
}
//collisions between element to element
void field::element::collision(element *e) {//collisions from element->element
	//ELEMENT *E IS WHAT IS BEING MOVED, NOT THE OTHER WAY AROUND
	float distance = pos.distance(e->pos);
	if (distance <= coefMag * (radius + e->radius)) {//touching
		vec3 normal = vec3(e->pos.X - pos.X, e->pos.Y - pos.Y);
		e->pos = e->pos + normal.times(((radius + e->radius) - distance) / distance);//push of the cones
		/*maths:
		distance*? = overlap;
		overlap = 2*cradius - distance
		therefore: ? = (2*cradius - distance)/distance;
		*/
	}
}
//overall physics between an dlement and the other elements and robots
void field::physics(int index, element *e, robot *r, int type, int roboIndex) {
	e->robotColl(index, r, type, &f, roboIndex);//collisions with fence
	e->fencePush(&f);//pushes the element from the fence if touching
	if (e->pos.Z <= c[0].height) {//assuming general cone height when on ground (dosent interact with grounded objects)
		for (int k = 0; k < c.size(); k++) {//collisions with other cones
			if (c[k].pos.Z < e->height){//had to add the landed, because the gravity would push it down further
				if (k != index) e->collision(&c[k]);
				else if (type != 0) e->collision(&c[k]);
			}
		}
	}
	if (e->pos.Z <= mg[1].height) {//assuming general mogo height when on ground (dosent interact with grounded objects)
		for (int m = 0; m < mg.size(); m++) {//collisions with other mogos
			if (mg[m].pos.Z < e->height) {//makes sure is within height of physics mattering
				if (m != index) e->collision(&mg[m]);
				else if (type != 1) e->collision(&mg[m]);
			}
		}
	}
}
//used to check if mogo is within the zones
float field::fence::poleEquation(float xPoint, float yPoint, float slope, float value) {
	return slope * (value - xPoint) + yPoint;//y-y1 = m(x-x1), and all slopes are negative (in this case)
}
//crossing the robot with the zones over the 10pt and 20pt poles
void field::fence::robotPole(robot *r) {
	//basically just slows down the robot if its centre passes the poles
	if (r->p.position.Y <= poleEquation(0, 23.2, -1, r->p.position.X)) //crossed 20 point pole
		r->p.speedMult(0.5, 0.9);
	else if (r->p.position.Y <= poleEquation(0, 46.7, -1, r->p.position.X))//crossed 10 point pole
		r->p.speedMult(0.75, 0.9);
	if (r->p.position.Y >= poleEquation(140.05, 117.5, -1, r->p.position.X)) //crossed 20 point pole
		r->p.speedMult(0.5, 0.9);
	else if (r->p.position.Y >= poleEquation(140.05, 93.9, -1, r->p.position.X)) //crossed 10 point pole
		r->p.speedMult(0.75, 0.9);
}
//pushing the robot against the field fence
void field::fence::wallPush(robot *r) {
	//deals with robot's wall boundaries 
	for (int i = 0; i < 4; i++) {
		int dir = 1;
		float d2Top = fieldSizeIn - r->db.vertices[i].Y;
		float d2Right = fieldSizeIn - r->db.vertices[i].X;
		float angleMult = 2;
		if (r->p.velocity.X < 0 && r->p.velocity.Y < 0) dir = -1;//if going backwards, reverse the angular rotation
		if (d2Right <= (depthIn)) {//checking RIGHT side
			r->p.position.X -= (depthIn) - d2Right;
			r->p.velocity.X = 0;
			if (withinAngle(r->p.mRot, 0, 90) || 
				withinAngle(r->p.mRot, 180, 270))  r->p.mRot -= dir*angleMult*cos(gAngle);
			else if (withinAngle(r->p.mRot, 90, 180) || 
				withinAngle(r->p.mRot, 270, 360)) r->p.mRot += dir*angleMult*cos(gAngle);
		}
		else if (r->db.vertices[i].X <= (depthIn)) {//checking LEFT side
			r->p.position.X += (depthIn)-r->db.vertices[i].X;
			r->p.velocity.X = 0;
			if (withinAngle(r->p.mRot, 90, 180) || 
				withinAngle(r->p.mRot, 270, 360))  r->p.mRot -= dir*angleMult*cos(gAngle);
			else if (withinAngle(r->p.mRot, 180, 270) || 
				withinAngle(r->p.mRot, 0, 90)) r->p.mRot += dir*angleMult*cos(gAngle);
		}
		if (d2Top <= (depthIn)) {//checking top
			r->p.position.Y -= (depthIn)-d2Top;
			r->p.velocity.Y = 0;
			if (withinAngle(r->p.mRot, 90, 180) || 
				withinAngle(r->p.mRot, 270, 360))  r->p.mRot += dir*angleMult*sin(gAngle);
			else if (withinAngle(r->p.mRot, 0, 90) || 
				withinAngle(r->p.mRot, 180, 270)) r->p.mRot -= dir*angleMult*sin(gAngle);
		}
		else if (r->db.vertices[i].Y <= (depthIn)) {//checking bottom side
			r->p.position.Y += (depthIn)-r->db.vertices[i].Y;
			r->p.velocity.Y = 0;
			if (withinAngle(r->p.mRot, 270, 360) || 
				withinAngle(r->p.mRot, 90, 180))  r->p.mRot -= dir*angleMult*sin(gAngle);
			else if (withinAngle(r->p.mRot, 180, 270) || 
				withinAngle(r->p.mRot, 0, 90)) r->p.mRot += dir*angleMult*sin(gAngle);
		}
	}
}
//checking if cone is being grabbed by the robot
void field::cone::coneGrab(robot *r, int index, int robIndex) {
	vec3 idealSpot = vec3(//perf
		(r->p.position.X + (r->getSize() / 2) * cos((-r->p.mRot) * PI / 180) * sqrt(2)),
		(r->p.position.Y - (r->getSize() / 2) * sin((-r->p.mRot) * PI / 180) * sqrt(2)));
	bool inPosition = (pos.distance(idealSpot) <= radius);//within range of in frontness
	if (r->c.grabbing && index < numCones && ((r->c.holding == index) || (r->c.holding == -1))) {
		if (inPosition && abs(r->c.liftPos - pos.Z) < height) {//makes sure lift is within grabbing distance
			r->c.holding = index;
			grabbingRobotIndex = robIndex;
			pos.X = (r->p.position.X + (r->getSize() / 2) * cos((-r->p.mRot) * PI / 180) * sqrt(2));//works
			pos.Y = (r->p.position.Y - (r->getSize() / 2) * sin((-r->p.mRot) * PI / 180) * sqrt(2));//works
			if ((pos.Z < r->c.maxHeight) || (pos.Z > 0)) {
				pos.Z = r->c.liftPos;//LATER: add something to try to pickyp from center
				landed = false;
			}
		}
	}
}
//checking if mogo is being grabbed by the robot
void field::MoGo::mogoGrab(robot *r, int index) {
	vec3 idealSpot = vec3(//perf
		(r->p.position.X - (r->getSize() / 2) * cos((-r->p.mRot) * PI / 180) * sqrt(2)),
		(r->p.position.Y + (r->getSize() / 2) * sin((-r->p.mRot) * PI / 180) * sqrt(2)));
	bool inPosition = (pos.distance(idealSpot) <= radius*0.7);//within range of behindness
	if (r->mg.grabbing  && ((r->mg.holding == index + 100) || (r->mg.holding == -101))) {
		if (inPosition) {//makes sure lift is within grabbing distance
			r->mg.holding = index+100;
			pos.X = (r->p.position.X - (r->getSize() / 2) * cos((-r->p.mRot) * PI / 180) * sqrt(2));//works
			pos.Y = (r->p.position.Y + (r->getSize() / 2) * sin((-r->p.mRot) * PI / 180) * sqrt(2));//works
		}
	}
}
//when cone is in freefall falling onto a mogo, stago, or just pole
void field::fallingOn(cone *fall, robot *r, int index) {
	if (!fall->landed && !r->c.grabbing) {
		for (int mog = 0; mog < mg.size(); mog++) {
			mg[mog].stacked.erase(index);
			if (!fall->landed) {
				if (fall->pos.distance(mg[mog].pos) <= cRad) {//added constant to widen range where can drop and stack
					if (fall->pos.Z > mg[mog].height + 4 + (mg[mog].stacked.size()* fall->height)) {//had to increase very high, because updates the grabvity effect before sets hadlanded to true
						fall->pos.Z += -32 / 12;//gravity?
						fall->landed = false;//still in air
						fall->fellOn = -1;//cone hasent fallen on anything yet (or ground)
					}
					else {
						fall->landed = true; //LANDED
						fall->grabbingRobotIndex = -1;//resets grabber index
						mg[mog].stacked.insert(index);
						fall->fellOn = mog + MOGO * 100;//cone has fallen on specific mogo(added 100s place value)
					}
				}
			}
			else break;
		}
		for (int pol = 0; pol < pl.size(); pol++) {
			pl[pol].stacked.erase(index);
			if (!fall->landed) {
				if (fall->pos.distance(pl[pol].pos) <= cRad) {//added constant to widen range where can drop and stack
					if (fall->pos.Z > pl[pol].height + 4 + (pl[pol].stacked.size()* fall->height)) {//had to increase very high, because updates the grabvity effect before sets hadlanded to true
						fall->pos.Z += -32 / 12;//gravity?
						fall->landed = false;//still in air
						fall->fellOn = -1;//cone hasent fallen on anything yet (or ground)
					}
					else {
						fall->landed = true;//LANDED
						fall->grabbingRobotIndex = -1;//resets grabber index
						pl[pol].stacked.insert(index);
						fall->fellOn = pol + STAGO * 100;//cone has fallen on specific pole (added 200s place value)
					}
				}
			}
			else break;
		}
		if (!fall->landed) fall->pos.Z -= 32 / 12;
	}
}
//posisioning the cone to fall to the center of its goal and funnel into the center
void field::positionFall(cone *fall) {
	float moveX, moveY;//centers cone after landing
	if (fall->fellOn <= numCones) {//if it fell on a cone
		moveX = 0.5*(fall->pos.X - c[fall->fellOn - CONE * 100].pos.X);
		moveY = 0.5*(fall->pos.Y - c[fall->fellOn - CONE * 100].pos.Y);
	}
	else if (fall->fellOn >= STAGO * 100) {//if it fell on a stagoionary goal
		moveX = 0.5*(fall->pos.X - pl[fall->fellOn - STAGO * 100].pos.X);
		moveY = 0.5*(fall->pos.Y - pl[fall->fellOn - STAGO * 100].pos.Y);
	}
	else {//between the two (MOGOS)
		moveX = 0.5*(fall->pos.X - mg[fall->fellOn - MOGO * 100].pos.X);
		moveY = 0.5*(fall->pos.Y - mg[fall->fellOn - MOGO * 100].pos.Y);
	}
	float min = 0.1;
	if (abs(moveX) > min) fall->pos.X -= moveX;
	if (abs(moveY) > min) fall->pos.Y -= moveY;
}
//checking which mogos are within which zone
void field::MoGo::zoneScore(fence *f, int index) {
	if (colour == 1) {//red
		if (pos.Y <= f->poleEquation(0, 23.2, -1, pos.X)) {//20 point
			f->z[0].twentyPoint.insert(index);
			f->z[0].fivePoint.erase(index);
			f->z[0].tenPoint.erase(index);
		}
		else if (pos.Y <= f->poleEquation(0, 46.7, -1, pos.X)) {//ten point
			f->z[0].twentyPoint.erase(index);
			f->z[0].fivePoint.erase(index);
			f->z[0].tenPoint.insert(index);
		}
		else if (pos.Y <= f->poleEquation(0, 70.2, -1, pos.X)) {//5 point
			f->z[0].twentyPoint.erase(index);
			f->z[0].fivePoint.insert(index);
			f->z[0].tenPoint.erase(index);
		}
		else {//neither
			f->z[0].twentyPoint.erase(index);
			f->z[0].fivePoint.erase(index);
			f->z[0].tenPoint.erase(index);
		}
	}
	else if (colour == 2) {//blue 
		if (pos.Y >= f->poleEquation(140.05, 117.5, -1, pos.X)) {
			f->z[1].twentyPoint.insert(index);
			f->z[1].fivePoint.erase(index);
			f->z[1].tenPoint.erase(index);
		}
		else if (pos.Y >= f->poleEquation(140.05, 93.9, -1, pos.X)) {
			f->z[1].twentyPoint.erase(index);
			f->z[1].fivePoint.erase(index);
			f->z[1].tenPoint.insert(index);
		}
		else if (pos.Y >= f->poleEquation(140.05, 70.2, -1, pos.X)) {
			f->z[1].twentyPoint.erase(index);
			f->z[1].fivePoint.insert(index);
			f->z[1].tenPoint.erase(index);
		}
		else {
			f->z[1].twentyPoint.erase(index);
			f->z[1].fivePoint.erase(index);
			f->z[1].tenPoint.erase(index);
		}
	}
}
//update task for the entire field simulation
void field::FieldUpdate(std::vector<robot> *r) {
	if (!isInit) initialize(r);
	for (int rob = 0; rob < (*r).size(); rob++) {
		f.wallPush(&(*r)[rob]);//pushed each robot against the wall
		f.robotPole(&(*r)[rob]);//[ushes each robot against the stagos
		/*for (int otherRob = 0; otherRob < (*r).size(); otherRob++) {//DO THIS LATER 
			if(rob != otherRob) (*r)[rob].collision(&(*r)[otherRob]);//robot collides with other rs
		}*/
		for (int i = 0; i < c.size(); i++) {
			//type for "cone" is 0
			int type = CONE;
			c[i].coneGrab(&(*r)[rob], i, rob);
			if (c[i].pos.Z < c[i].height) {
				physics(i, &c[i], &(*r)[rob], type, rob);
			}//only affect objects when on ground (or low enough)
			if (c[i].pos.Z > 32/12 && c[i].grabbingRobotIndex != -1) {
				fallingOn(&c[i], &(*r)[c[i].grabbingRobotIndex], i);//noice
			}
			if (c[i].grabbingRobotIndex == -1 && c[i].fellOn != -1 && c[i].landed) {
				positionFall(&c[i]);
			}
		}
		for (int i = 0; i < mg.size(); i++) {
			//type for "mogo" is 1
			int type = MOGO;
			mg[i].mogoGrab(&(*r)[rob], i);
			physics(i, &mg[i], &(*r)[rob], type, rob);//dont affect things if being lifted into the air
		}
		for (int i = 0; i < pl.size(); i++) {
			//type for "stagoionary goal" is 2
			int type = STAGO;
			//stagoGoalPush(&pl[i], &(*r)[rob], &f);
			physics(i, &pl[i], &(*r)[rob], type, rob);
		}
	}
	for (int i = 0; i < mg.size(); i++) {
		mg[i].zoneScore(&f, i);
	}
	for (int i = 0; i < pl.size(); i++) {
		pl[i].pos = initPoleConfig[i].pos; //stagoionary goal (not moving)
	}
	for (int i = 0; i < c.size(); i++) {
		c[i].radius = 0.1*c[i].pos.Z + cRad;//changes radius to enlargen when gets taller
	}




}