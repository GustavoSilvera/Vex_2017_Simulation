#include "field.h"

//declares and defines the field class and functions

//constructor
// Map from vex image coordinates to Robot coordinates.
vec3 V2R(vec3 vec_coord) {
	const int fieldSizeIn = 141.05;  // Field size in inches.
	return vec3(fieldSizeIn - vec_coord.X, vec_coord.Y, vec_coord.Z);
}

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

//initial cone values for position
field::MoGo initMoGoConfig[] = {//array for each configuration of the mobile goal (in field.h)
	{ V2R({ 34.9, 13.0 }), 1  },{ V2R({ 13.0, 34.9 }), 2  },
	{ V2R({ 70.2, 46.7 }), 2  },{ V2R({ 46.7, 70.2 }), 1  },
	{ V2R({ 93.9, 70.2 }), 2  },{ V2R({70.2, 93.9 }), 1  },
	{ V2R({ 127.6, 105.8 }), 1  },{ V2R({105.8, 127.6 }), 2  }
};
int numMoGos = sizeof(initMoGoConfig) / sizeof(field::MoGo);

field::stat initPoleConfig[] = {
	{ V2R({ 93, 47.3 }), 4, 24},{ V2R({ 46.9, 94 }), 4, 24}
};
int numPoles = sizeof(initPoleConfig) / sizeof(field::stat);
field::field(robot *r, robot *r2) : isInit(true) {
	field::initialize(r, r2);
}
//initial mogo values for position and colour
void field::initialize(robot *r, robot *r2) {
	c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
	pl.assign(&initPoleConfig[0], &initPoleConfig[numPoles]);
	r->reset();
	r->p.position.X = 35;
	r->p.position.Y = 35;
	r->p.mRot = 45;
	r2->reset();
	r2->p.position.X = 117;
	r2->p.position.Y = 117;
	r2->p.mRot = 225;
	fieldInit = false;
	isInit = true;//so that this only gets called ONCE when the field tab is running
}
//initialized everything for the field, such as cone and mogo values, and robot posision
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
float calcD2Edge(float a, float b, robot *robit, float prop) {/*not taking into account the protrusion of mogo*/
	//EXPLANATION HERE:
	//prop is for proportion of size of the square, like mogo (clawsize/size)vs full robot base(1)
	float C1 = ( ( (sqr(a) - sqr(b)) / (prop*robit->d.size)) + (prop*robit->d.size)) / 2;
	return sqrt(abs(sqr(a) - sqr(C1)));
}

//calculate distance to edge of robot
void field::element::calcD2Vertices(robot *robit) {
	/******db.vertices:*******
	0----------1
	|    r     |
	|          |
	3----------2
	**********************/
}
//calculates distance between the cone's centre and each vertice
void field::element::fencePush(fence *f) {
	float d2Top = f->fieldSizeIn - pos.Y;
	float d2Right = f->fieldSizeIn - pos.X;
	if (pos.X <= (radius + f->depthIn)) pos.X += (radius + f->depthIn) - 1.1*pos.X;////checking left side
	else if (d2Right <= (radius + f->depthIn)) pos.X -= (radius + f->depthIn) - 1.1*d2Right;//checking right side
	if (d2Top <= (radius + f->depthIn)) pos.Y -= (radius + f->depthIn) - 1.1*d2Top;//checking top
	else if (pos.Y <= (radius + f->depthIn)) pos.Y += (radius + f->depthIn) - 1.1*pos.Y;//checking bottom
}
//calculates distances to the edges of the field, and acts accordingly
bool withinAngle(double angle, int lowerBound, int upperBound) {
	//checks if angle is within upper and lower
	int thresh = 2;//degrees of freedom
	return (angle < upperBound - thresh && angle > lowerBound + thresh) || (angle + 180 < upperBound - thresh && angle + 180 > lowerBound + thresh || (angle + 360 < upperBound - thresh && angle + 360 > lowerBound + thresh));
	//checks both the positive and "negative" angle
}
void field::element::robotColl(int index, robot *robit, std::set<int> &pushCone, std::set<int> &pushMoGo, int type, fence *f) {
	//collisions from robot
	float d2V[4];
	float d2Robot = pos.distance(robit->p.position);
	if (pos.Z < height && d2Robot < renderRad * robit->d.size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
		for (int v = 0; v < 4; v++) {
			d2V[v] = pos.distance(robit->db.vertices[v]);
		}
		float d2RobotEdge;
		bool inFront = (d2V[0] + d2V[1] < d2V[2] + d2V[3]);//checking if cone is closer to the front side
		bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if cone is closer to the right side

		if (robit->directlyInPath(true, robit->d.size, pos)) {//either directly in front or behing based off center x and y position
			d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, 1);//calculates the distance to the edge of the robit
			if (inFront) closestPoint = vec3(pos.X - (d2RobotEdge)*cos(gAngle), pos.Y + (d2RobotEdge)*sin(gAngle));//does work
			else closestPoint = vec3(pos.X + (d2RobotEdge)*cos(gAngle), pos.Y - (d2RobotEdge)*sin(gAngle));//does work
		}
		//had to inverse x and y because horiontal lines
		else if (robit->directlyInPath(false, robit->d.size, pos)) {
			d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, 1);//calculates the distance to the edge of the robit
			if (onRight) closestPoint = vec3(pos.X + (d2RobotEdge)*sin(gAngle), pos.Y + (d2RobotEdge)*cos(gAngle));//does work
			else closestPoint = vec3(pos.X - (d2RobotEdge)*sin(gAngle), pos.Y - (d2RobotEdge)*cos(gAngle));//does work

		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(d2V[0], d2V[1], d2V[2], d2V[3]);
			closestPoint = robit->db.vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = pos.distance(closestPoint);
		vec3 R = (closestPoint + pos.times(-1)).times(radius / d2closestPoint) + pos;
		if (d2closestPoint <= radius || inPossession) {//touching
			if (type == MOGO &&//is mogo?
				abs(robit->mg.protrusion - 7.5)<0.5 &&//mogo is ar low position
				!inFront &&//behind
				robit->directlyInPath(true, robit->d.size / 4, pos) && //in front or back
																	   //pos.distance(robit->p.position) <= (robit->d.size*0.5+robit->mg.protrusion+3) && 
				pos.distance(robit->p.position) > robit->d.size*0.7) {
				if (!robit->mg.grabbing)inPossession = true;//only locks in when bringing mogo up (grabbing == false)
			}
			else {
				pos.X -= R.X - closestPoint.X;
				pos.Y -= R.Y - closestPoint.Y;
				bool crushingCone =
					((pos.Y <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 225, 315)) ||//checking bottom
					((f->fieldSizeIn - pos.Y <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 45, 135)) ||//checking top
					((f->fieldSizeIn - pos.X <= (radius + f->depthIn)) && (withinAngle(robit->p.mRot, 0, 45) ||//checking right P1
						withinAngle(robit->p.mRot, 315, 360))) || //checking right P2
						((pos.X <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 135, 225));//checking Left
																								   //could change crushingCone to be affected for a smaller angle, so that the reverse push only happens if almost directly crushing against the fence
				if (crushingCone) {//HAVE only affected when pushing further into fence
					int thresh = 3;//degrees of freedom
					float currentVel = sqrt(sqr(robit->p.velocity.X) + sqr(robit->p.velocity.Y));
					if (inFront) {
						if (abs(d2V[0] - d2V[1]) > thresh) {
							if (d2V[0] < d2V[1])//checking which way to rotate
								robit->p.mRot += abs(currentVel * sin(gAngle));//angle is kinda iffy still
							else if (d2V[0] > d2V[1])
								robit->p.mRot -= abs(currentVel * sin(gAngle));
						}
					}
					else if (abs(d2V[2] - d2V[3]) > thresh) {
						if (d2V[2] > d2V[3])
							robit->p.mRot -= abs(currentVel * sin(gAngle));
						else if (d2V[2] < d2V[3])
							robit->p.mRot += abs(currentVel * sin(gAngle));
					}
					robit->p.velocity.X = (R.X - closestPoint.X);
					robit->p.velocity.Y = (R.Y - closestPoint.Y);
				}
				if (index + type * 100 <= numCones) {//if the type it's touching is a cone
					pushCone.insert(index);
				}
				else {
					pushMoGo.insert(index);
				}
			}
			if (inPossession) {//when doing the fancy animations (brings mogo into robit)
				pos.X = robit->p.position.X - robit->mg.protrusion * cos((robit->p.mRot) * PI / 180) * 2;
				pos.Y = robit->p.position.Y - robit->mg.protrusion * sin((robit->p.mRot) * PI / 180) * 2;
			}
			if (abs(robit->mg.protrusion - 7.5) < 0.5 && robit->mg.grabbing) {//when bringing the mogo down
				inPossession = false;//no longer locked onto mogo
			}
		}
		else if (d2closestPoint >= radius * 1.05)
			if (index + type * 100 <= numCones) pushCone.erase(index);
			else pushMoGo.erase(index);
	}
}
void field::element::mogoColl(int index, robot *robit, std::set<int> &pushCone, std::set<int> &pushMoGo, int type, fence *f) {
	//collisions from robot
	float d2V[4];
	float d2MoGo = pos.distance(
	vec3(
		robit->p.position.X - robit->mg.protrusion * cos((robit->p.mRot) * PI / 180),
		robit->p.position.Y - robit->mg.protrusion * sin((robit->p.mRot) * PI / 180) 
	));
	if (pos.Z < height && d2MoGo < renderRad * robit->d.size/2) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
		for (int v = 0; v < 4; v++) {
			d2V[v] = pos.distance(robit->db.MGVert[v]);
		}
		float d2RobotEdge;
		bool inFront = (d2V[0] + d2V[1] < d2V[2] + d2V[3]);//checking if cone is closer to the front side
		bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if cone is closer to the right side
		float mogoProp = 2.5*(robit->mg.clawSize) / robit->d.size;//proportion of MOGO size to robot
		if (robit->directlyInPath(true, mogoProp*robit->d.size, pos)) {//either directly in front or behing based off center x and y position
			d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, mogoProp);//calculates the distance to the edge of the robit
			if (inFront) closestPointMOGO = vec3(pos.X - (d2RobotEdge)*cos(gAngle), pos.Y + (d2RobotEdge)*sin(gAngle));//does work
			else closestPointMOGO = vec3(pos.X + (d2RobotEdge)*cos(gAngle), pos.Y - (d2RobotEdge)*sin(gAngle));//does work
		}
		//had to inverse x and y because horiontal lines
		else if (robit->directlyInPath(false, robit->d.size + robit->mg.protrusion*2, pos)) {
			d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, mogoProp);//calculates the distance to the edge of the robit
			if (onRight) closestPointMOGO = vec3(pos.X + (d2RobotEdge)*sin(gAngle), pos.Y + (d2RobotEdge)*cos(gAngle));//does work
			else closestPointMOGO = vec3(pos.X - (d2RobotEdge)*sin(gAngle), pos.Y - (d2RobotEdge)*cos(gAngle));//does work
		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(d2V[0], d2V[1], d2V[2], d2V[3]);
			closestPointMOGO = robit->db.MGVert[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPointMOGO = pos.distance(closestPointMOGO);
		vec3 R = (closestPointMOGO + pos.times(-1)).times(radius / d2closestPointMOGO) + pos;
		if (d2closestPointMOGO <= radius || inPossession) {//touching
			if (type == MOGO &&//is mogo?
				abs(robit->mg.protrusion - 7.5)<0.5 &&//mogo is ar low position
				!inFront &&//behind
				robit->directlyInPath(true, robit->d.size / 4, pos) && //in front or back
																	   //pos.distance(robit->p.position) <= (robit->d.size*0.5+robit->mg.protrusion+3) && 
				pos.distance(robit->p.position) > robit->d.size*0.7) {
				if (!robit->mg.grabbing)inPossession = true;//only locks in when bringing mogo up (grabbing == false)
			}
			else {
				pos.X -= R.X - closestPointMOGO.X;
				pos.Y -= R.Y - closestPointMOGO.Y;
				bool crushingCone =
					((pos.Y <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 225, 315)) ||//checking bottom
					((f->fieldSizeIn - pos.Y <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 45, 135)) ||//checking top
					((f->fieldSizeIn - pos.X <= (radius + f->depthIn)) && (withinAngle(robit->p.mRot, 0, 45) ||//checking right P1
						withinAngle(robit->p.mRot, 315, 360))) || //checking right P2
						((pos.X <= (radius + f->depthIn)) && withinAngle(robit->p.mRot, 135, 225));//checking Left
																								   //could change crushingCone to be affected for a smaller angle, so that the reverse push only happens if almost directly crushing against the fence
				if (crushingCone) {//HAVE only affected when pushing further into fence
					int thresh = 3;//degrees of freedom
					float currentVel = sqrt(sqr(robit->p.velocity.X) + sqr(robit->p.velocity.Y));
					if (inFront) {
						if (abs(d2V[0] - d2V[1]) > thresh) {
							if (d2V[0] < d2V[1])//checking which way to rotate
								robit->p.mRot += abs(currentVel * sin(gAngle));//angle is kinda iffy still
							else if (d2V[0] > d2V[1])
								robit->p.mRot -= abs(currentVel * sin(gAngle));
						}
					}
					else if (abs(d2V[2] - d2V[3]) > thresh) {
						if (d2V[2] > d2V[3])
							robit->p.mRot -= abs(currentVel * sin(gAngle));
						else if (d2V[2] < d2V[3])
							robit->p.mRot += abs(currentVel * sin(gAngle));
					}
					robit->p.velocity.X = (R.X - closestPointMOGO.X);
					robit->p.velocity.Y = (R.Y - closestPointMOGO.Y);
				}
				if (index + type * 100 <= numCones) {//if the type it's touching is a cone
					pushCone.insert(index);
				}
				else {
					pushMoGo.insert(index);
				}
			}
			if (inPossession) {//when doing the fancy animations (brings mogo into robit)
				pos.X = robit->p.position.X - robit->mg.protrusion * cos((robit->p.mRot) * PI / 180) * 2;
				pos.Y = robit->p.position.Y - robit->mg.protrusion * sin((robit->p.mRot) * PI / 180) * 2;
			}
			if (abs(robit->mg.protrusion - 7.5) < 0.5 && robit->mg.grabbing) {//when bringing the mogo down
				inPossession = false;//no longer locked onto mogo
			}
		}
		else if (d2closestPointMOGO >= radius * 1.05)
			if (index + type * 100 <= numCones) pushCone.erase(index);
			else pushMoGo.erase(index);
	}
}

//functions for collisions between the element and the robot
void field::element::collision(element *e) {//collisions from element->element
	//ELEMENT *E IS WHAT IS BEING MOVED, NOT THE OTHER WAY AROUND
	float distance = pos.distance(e->pos);
	if (distance <= coefMag * (radius + e->radius)) {//touching
		vec3 normal = vec3(e->pos.X - pos.X, e->pos.Y - pos.Y);
		e->pos = e->pos + normal.times(((radius + e->radius) - distance) / distance);//push of the cones
		/*maths
		distance*? = overlap;
		overlap = 2*cradius - distance
		therefore: ? = (2*cradius - distance)/distance;
		*/
	}
}
//functions for collisions between the element and another element
void field::physics(int index, element *e, robot *robit, int type) {
	if (e->pos.Z <= c[0].height) {//assuming general cone height when on ground (dosent interact with grounded objects)
		for (int k = 0; k < c.size(); k++) {
			if (c[k].pos.Z < e->height){//had to add the landed, because the gravity would push it down further
				e->fencePush(&f);//pushes the cone from the fence if touching
				e->robotColl(index, robit, pushCones, pushMoGo, type, &f);
				e->mogoColl(index, robit, pushCones, pushMoGo, type, &f);
				if (k != index) e->collision(&c[k]);
				else if (type != 0) e->collision(&c[k]);
			}
		}
	}
	if (e->pos.Z <= mg[1].height) {//assuming general mogo height when on ground (dosent interact with grounded objects)
		for (int m = 0; m < mg.size(); m++) {
			if (mg[m].pos.Z < e->height) {//makes sure is within height of physics mattering
				e->fencePush(&f);//pushes the mogo from the fence if touching
				e->robotColl(index, robit, pushCones, pushMoGo, type, &f);
				e->mogoColl(index, robit, pushCones, pushMoGo, type, &f);
				if (m != index) e->collision(&mg[m]);
				else if (type != 1) e->collision(&mg[m]);
			}
		}
	}

	//LOL poles dont need to move
	/*if (e->pos.Z <= pl[1].height) {//so long as within touching distance. 
		for (int p = 0; p < pl.size(); p++) {
			if (index != p) e->collision(&pl[p]);
			else if (type != 2) e->collision(&pl[p]);
		}
	}*/
}
//function for calling all the collision functions together for el->el and el->robot
float field::fence::poleEquation(float xPoint, float yPoint, float slope, float value) {
	return slope * (value - xPoint) + yPoint;//y-y1 = m(x-x1), and all slopes are negative (in this case)
}
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
void field::fence::wallPush(robot *robit) {
	//deals with robot's wall boundaries 
	for (int i = 0; i < 4; i++) {
		float d2Top = fieldSizeIn - robit->db.vertices[i].Y;
		float d2Left = fieldSizeIn - robit->db.vertices[i].X;
		if (d2Left <= (depthIn)) {//checking RIGHT side
			robit->p.position.X -= (depthIn) - d2Left;
			robit->p.velocity.X = 0;
			if (withinAngle(robit->p.mRot, 0, 90) || withinAngle(robit->p.mRot, 180, 270))  robit->p.mRot -= 1.5*cos(gAngle);
			else if (withinAngle(robit->p.mRot, 90, 180) || withinAngle(robit->p.mRot, 270, 360)) robit->p.mRot += 1.5*cos(gAngle);
		}
		else if (robit->db.vertices[i].X <= (depthIn)) {//checking LEFT side
			robit->p.position.X += (depthIn)-robit->db.vertices[i].X;
			robit->p.velocity.X = 0;
			if (withinAngle(robit->p.mRot, 90, 180) || withinAngle(robit->p.mRot, 270, 360))  robit->p.mRot -= 1.5*cos(gAngle);
			else if (withinAngle(robit->p.mRot, 180, 270) || withinAngle(robit->p.mRot, 0, 90)) robit->p.mRot += 1.5*cos(gAngle);
		}
		if (d2Top <= (depthIn)) {//checking top
			robit->p.position.Y -= (depthIn)-d2Top;
			robit->p.velocity.Y = 0;
			if (withinAngle(robit->p.mRot, 90, 180) || withinAngle(robit->p.mRot, 270, 360))  robit->p.mRot += 1.5*sin(gAngle);
			else if (withinAngle(robit->p.mRot, 0, 90) || withinAngle(robit->p.mRot, 180, 270)) robit->p.mRot -= 1.5*sin(gAngle);
		}
		else if (robit->db.vertices[i].Y <= (depthIn)) {//checking bottom side
			robit->p.position.Y += (depthIn)-robit->db.vertices[i].Y;
			robit->p.velocity.Y = 0;
			if (withinAngle(robit->p.mRot, 270, 360) || withinAngle(robit->p.mRot, 90, 180))  robit->p.mRot += 1.5*sin(gAngle);
			else if (withinAngle(robit->p.mRot, 180, 270) || withinAngle(robit->p.mRot, 0, 90)) robit->p.mRot -= 1.5*sin(gAngle);
		}
	}
}
void field::statGoalPush(stat *pl, robot *robit, fence *f) {
	float d2obj = robit->p.position.distance(pl->pos);
	if (d2obj < renderRad * robit->d.size) {
		float d2V[4];
		for (int v = 0; v < 4; v++) {
			d2V[v] = pl->pos.distance(robit->db.vertices[v]);
		}
		bool inFront, onRight;
		vec3 closestPoint;
		float d2Edge;//different because of protrusion of mogo
		if (robit->directlyInPath(true, robit->d.size, pl->pos)) {/*in front or behind*/
			d2Edge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, 1);//calculates the distance to the edge of the robit
			inFront = (d2V[0] + d2V[1] < d2V[2] + d2V[3]);//checking if cone is closer to the front side
			if (inFront) closestPoint = vec3(pl->pos.X - (d2Edge)*cos(gAngle), pl->pos.Y + (d2Edge)*sin(gAngle));//does work
			else closestPoint = vec3(pl->pos.X + (d2Edge)*cos(gAngle), pl->pos.Y - (d2Edge)*sin(gAngle));//does work
		}
		else if (robit->directlyInPath(false, robit->d.size, pl->pos)) {
			d2Edge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit, 1);//calculates the distance to the edge of the robit
			bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if cone is closer to the right side
			if (onRight) closestPoint = vec3(pl->pos.X + (d2Edge)*sin(gAngle), pl->pos.Y + (d2Edge)*cos(gAngle));//does work
			else closestPoint = vec3(pl->pos.X - (d2Edge)*sin(gAngle), pl->pos.Y - (d2Edge)*cos(gAngle));//does work
		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(d2V[0], d2V[1], d2V[2], d2V[3]);
			closestPoint = robit->db.vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = closestPoint.distance(pl->pos);
		vec3 R = (closestPoint + pl->pos.times(-1)).times(pl->radius / d2closestPoint) + pl->pos;
		if (d2closestPoint <= pl->radius) {//touching
			robit->d.touchingPole = true;
			robit->p.position.X += (R.X - closestPoint.X);
			robit->p.position.Y += (R.Y - closestPoint.Y);
			robit->p.velocity = vec3(0, 0, 0);
			//rotation when hits the stationary goal
			int thresh = 3;//degrees of freedom
			if (robit->directlyInPath(true, robit->d.size, pl->pos)) {
				float rotScale = 0.065; //constant for rotation scaling when hits pole smaller == smoother but slower
				if (inFront) {
					if (abs(d2V[0] - d2V[1]) > thresh) {
						if (d2V[0] < d2V[1])//checking which way to rotate
							robit->p.mRot += rotScale*largest(d2V[0], d2V[1]);
						else if (d2V[0] > d2V[1])
							robit->p.mRot -= rotScale*largest(d2V[0], d2V[1]);
					}
				}
				else {
					if (abs(d2V[2] - d2V[3]) > thresh) {
						if (d2V[2] > d2V[3])
							robit->p.mRot -= rotScale*largest(d2V[2], d2V[3]);
						else if (d2V[2] < d2V[3])
							robit->p.mRot += rotScale*largest(d2V[2], d2V[3]);
					}
				}
			}
		}
		else robit->d.touchingPole = false;
	}
}
//function for making sure the robot cannot move past the fence
void field::cone::coneGrab(robot *robit, int index) {
	vec3 idealSpot = vec3(//perf
		(robit->p.position.X + (robit->d.size / 2) * cos((-robit->p.mRot) * PI / 180) * sqrt(2)),
		(robit->p.position.Y - (robit->d.size / 2) * sin((-robit->p.mRot) * PI / 180) * sqrt(2)));
	bool inPosition = (pos.distance(idealSpot) <= radius);//within range of in frontness
	if (robit->c.grabbing && index < numCones && ((robit->c.holding == index) || (robit->c.holding == -1))) {
		if (inPosition && abs(robit->c.liftPos - pos.Z) < height) {//makes sure lift is within grabbing distance
			robit->c.holding = index;
			pos.X = (robit->p.position.X + (robit->d.size / 2) * cos((-robit->p.mRot) * PI / 180) * sqrt(2));//works
			pos.Y = (robit->p.position.Y - (robit->d.size / 2) * sin((-robit->p.mRot) * PI / 180) * sqrt(2));//works
			if ((pos.Z < robit->c.maxHeight) || (pos.Z > 0)) {
				pos.Z = robit->c.liftPos;//LATER: add something to try to pickyp from center
				landed = false;
			}
		}
	}
}
void field::MoGo::mogoGrab(robot *robit, int index) {
	vec3 idealSpot = vec3(//perf
		(robit->p.position.X - (robit->d.size / 2) * cos((-robit->p.mRot) * PI / 180) * sqrt(2)),
		(robit->p.position.Y + (robit->d.size / 2) * sin((-robit->p.mRot) * PI / 180) * sqrt(2)));
	bool inPosition = (pos.distance(idealSpot) <= radius*0.7);//within range of behindness
	if (robit->mg.grabbing  && ((robit->mg.holding == index + 100) || (robit->mg.holding == -101))) {
		if (inPosition) {//makes sure lift is within grabbing distance
			robit->mg.holding = index+100;
			pos.X = (robit->p.position.X - (robit->d.size / 2) * cos((-robit->p.mRot) * PI / 180) * sqrt(2));//works
			pos.Y = (robit->p.position.Y + (robit->d.size / 2) * sin((-robit->p.mRot) * PI / 180) * sqrt(2));//works
		}
	}
}
void field::fallingOn(cone *fall, robot *robit, int index) {
	if (!fall->landed && !robit->c.grabbing) {
		for (int mog = 0; mog < mg.size(); mog++) {
			if (!fall->landed) {
				if (fall->pos.distance(mg[mog].pos) <= cRad) {//added constant to widen range where can drop and stack
					if (fall->pos.Z > mg[mog].height + 4 + (mg[mog].stacked.size()* fall->height)) {//had to increase very high, because updates the grabvity effect before sets hadlanded to true
						fall->pos.Z += -32 / 12;//gravity?
						fall->landed = false;//still in air
						mg[mog].stacked.erase(index);
						fall->fellOn = -1;//cone hasent fallen on anything yet (or ground)
					}
					else {
						fall->landed = true; //LANDED
						mg[mog].stacked.insert(index);
						fall->fellOn = mog + MOGO * 100;//cone has fallen on specific mogo(added 100s place value)
					}
				}
			}
			else break;
		}
		for (int pol = 0; pol < pl.size(); pol++) {
			if (!fall->landed) {
				if (fall->pos.distance(pl[pol].pos) <= cRad) {//added constant to widen range where can drop and stack
					if (fall->pos.Z > pl[pol].height + 4 + (pl[pol].stacked.size()* fall->height)) {//had to increase very high, because updates the grabvity effect before sets hadlanded to true
						fall->pos.Z += -32 / 12;//gravity?
						fall->landed = false;//still in air
						pl[pol].stacked.erase(index);
						fall->fellOn = -1;//cone hasent fallen on anything yet (or ground)
					}
					else {
						fall->landed = true;//LANDED
						pl[pol].stacked.insert(index);
						fall->fellOn = pol + STAT * 100;//cone has fallen on specific pole (added 200s place value)
					}
				}
			}
			else break;
		}
		if (!fall->landed) fall->pos.Z -= 32 / 12;
	}
	else if (fall->landed && fall->fellOn != -1) {
		float moveX, moveY;//centers cone after landing
		if (fall->fellOn <= numCones) {//if it fell on a cone
			moveX = 0.5*(fall->pos.X - c[fall->fellOn - CONE * 100].pos.X);
			moveY = 0.5*(fall->pos.Y - c[fall->fellOn - CONE * 100].pos.Y);
		}
		else if (fall->fellOn >= STAT*100) {//if it fell on a stationary goal
			moveX = 0.5*(fall->pos.X - pl[fall->fellOn - STAT * 100].pos.X);
			moveY = 0.5*(fall->pos.Y - pl[fall->fellOn - STAT * 100].pos.Y);
		}
		else {//between the two (MOGOS)
			moveX = 0.5*(fall->pos.X - mg[fall->fellOn - MOGO * 100].pos.X);
			moveY = 0.5*(fall->pos.Y - mg[fall->fellOn - MOGO * 100].pos.Y);
		}
		float min = 0.1;
		if (abs(moveX) > min) fall->pos.X -= moveX;
		if (abs(moveY) > min) fall->pos.Y -= moveY;
	}
}
void field::MoGo::zoneScore(fence *f, int index) {
	if (colour == 1) {//red
		if (pos.Y <= f->poleEquation(0, 23.2, -1, pos.X)) {
			f->z[0].twentyPoint.insert(index);
			f->z[0].fivePoint.erase(index);
			f->z[0].tenPoint.erase(index);
		}
		else if (pos.Y <= f->poleEquation(0, 46.7, -1, pos.X)) {
			f->z[0].twentyPoint.erase(index);
			f->z[0].fivePoint.erase(index);
			f->z[0].tenPoint.insert(index);
		}
		else if (pos.Y <= f->poleEquation(0, 70.2, -1, pos.X)) {
			f->z[0].twentyPoint.erase(index);
			f->z[0].fivePoint.insert(index);
			f->z[0].tenPoint.erase(index);
		}
		else {
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
//function for having a 'grabbed' element lock in place

void field::FieldUpdate(robot *robit, robot *r2) {
	if (!isInit) initialize(robit, r2);
	f.wallPush(robit);
	f.wallPush(r2);
	f.robotPole(robit); 
	f.robotPole(r2);
	for (int i = 0; i < c.size(); i++) {
		//type for "cone" is 0
		int type = CONE;
		c[i].radius = 0.1*c[i].pos.Z + cRad;//changes radius to enlargen when gets taller
		c[i].coneGrab(robit, i);
		c[i].coneGrab(r2, i);
		if (c[i].pos.Z < c[i].height) { 
			physics(i, &c[i], robit, type);
			physics(i, &c[i], r2, type);
		}//only affect objects when on ground (or low enough)
		if (c[i].pos.Z > 0) {
			fallingOn(&c[i], robit, i);//noice
			fallingOn(&c[i], r2, i);//noice
		}
	}
	for (int i = 0; i < mg.size(); i++) {
		//type for "mogo" is 1
		int type = MOGO;
		//mg[i].radius = 0.1*mg[i].pos.Z + MGRad;//use better scalar than 0.1, but eventually remove this (because dont rly want to pick up mogos
		mg[i].mogoGrab(robit, i);
		mg[i].mogoGrab(r2, i);
		mg[i].zoneScore(&f, i);
		physics(i, &mg[i], robit, type);//dont affect things if being lifted into the air
		physics(i, &mg[i], r2, type);//dont affect things if being lifted into the air
	}
	for (int i = 0; i < pl.size(); i++) {
		//type for stationary goal is 2
		int type = STAT;
		pl[0].pos = initPoleConfig[0].pos; //stationary goal (not moving)
		pl[1].pos = initPoleConfig[1].pos; //stationary goal (not moving)
		statGoalPush(&pl[i], robit, &f);
		physics(i, &pl[i], robit, type);
		statGoalPush(&pl[i], r2, &f);
		physics(i, &pl[i], r2, type);
	}
	robit->p.frictionC = pushCones.size();
	robit->p.frictionM = pushMoGo.size();
	r2->p.frictionC = pushCones.size();
	r2->p.frictionM = pushMoGo.size();
}
//update task for the entire field simulation