#include "field.h"

//declares and defines the field class and functions

//constructor
field::element initConeConfig[numCones] = {//array for each configuration of the cone (in field.h)
										   //{initial posision (X, Y), color (Y, R, B), radii }
	{ { 2.9, 13.0 }, 0, cRad, cHeight},{ { 2.9, 23.2 }, 0, cRad, cHeight},{ { 2.9, 34.9 }, 0, cRad, cHeight},{ { 2.9, 46.7 }, 0, cRad, cHeight},
	{ { 2.9, 58.4 }, 0, cRad, cHeight},{ { 2.9, 70.2 }, 0, cRad, cHeight},{ { 13.0, 2.9 }, 0, cRad, cHeight},{ { 13.0, 13.0 }, 0, cRad, cHeight},
	{ { 13.0, 23.2 }, 0, cRad, cHeight},{ { 23.2, 2.9 }, 0, cRad, cHeight},{ { 23.2, 13.0 }, 0, cRad, cHeight},{ { 23.2, 23.2 }, 0, cRad, cHeight},
	{ { 23.2, 34.9 }, 0, cRad, cHeight},{ { 23.2, 46.7 }, 0, cRad, cHeight},{ { 23.2, 58.4 }, 0, cRad, cHeight},{ { 23.2, 70.2 }, 0, cRad, cHeight},
	{ { 34.9, 2.9 }, 0, cRad, cHeight},{ { 34.9, 23.2 }, 0, cRad, cHeight},{ { 46.7, 2.9 }, 0, cRad, cHeight},{ { 46.7, 23.2 }, 0, cRad, cHeight},
	{ { 46.7, 46.7 }, 0, cRad, cHeight},{ { 46.7, 58.4 }, 0, cRad, cHeight},{ { 58.4, 2.9 }, 0, cRad, cHeight},{ { 58.4, 23.2 }, 0, cRad, cHeight},
	{ { 58.4, 46.7 }, 0, cRad, cHeight},{ { 58.4, 70.2 }, 0, cRad, cHeight},{ { 70.2, 2.9 }, 0, cRad, cHeight},{ { 70.2, 23.2 }, 0, cRad, cHeight},
	{ { 70.2, 58.4 }, 0, cRad, cHeight},{ { 70.2, 82.1 }, 0, cRad, cHeight},{ { 82.1, 70.2 }, 0, cRad, cHeight},{ { 82.1, 93.9 }, 0, cRad, cHeight},
	{ { 82.1, 139.2 }, 0, cRad, cHeight},{ { 93.9, 82.1 }, 0, cRad, cHeight},{ { 93.9, 93.9 }, 0, cRad, cHeight},{ { 93.9, 117.5 }, 0, cRad, cHeight},
	{ { 93.9, 137.8 }, 0, cRad, cHeight},{ { 105.8, 117.5 }, 0, cRad, cHeight},{ { 105.8, 137.8 }, 0, cRad, cHeight},{ { 117.5, 93.9 }, 0, cRad, cHeight},
	{ { 117.5, 105.8 }, 0, cRad, cHeight},{ { 117.5, 117.5 }, 0, cRad, cHeight},{ { 117.5, 127.6 }, 0, cRad, cHeight},{ { 117.5, 137.8 }, 0, cRad, cHeight},
	{ { 127.6, 117.5 }, 0, cRad, cHeight},{ { 127.6, 127.6 }, 0, cRad, cHeight},{ { 127.6, 137.8 }, 0, cRad, cHeight},{ { 137.8, 82.1 }, 0, cRad, cHeight},
	{ { 137.8, 93.9 }, 0, cRad, cHeight},{ { 137.8, 105.8 }, 0, cRad, cHeight},{ { 137.8, 117.5 }, 0, cRad, cHeight},{ { 137.8, 127.6 }, 0, cRad, cHeight},
	{ { 137.8, 137.8 }, 0, cRad, cHeight}
};
//initial cone values for position
field::element initMoGoConfig[numMoGos] = {//array for each configuration of the mobile goal (in field.h)
	{ { 34.9, 13.0 }, 1 ,  MGRad, mgHeight },{ { 13.0, 34.9 }, 2 , MGRad, mgHeight },
	{ { 70.2, 46.7 }, 2 , MGRad, mgHeight },{ { 46.7, 70.2 }, 1 , MGRad, mgHeight },
	{ { 93.9, 70.2 }, 2 , MGRad, mgHeight },{ { 70.2, 93.9 }, 1 , MGRad, mgHeight },
	{ { 127.6, 105.8 }, 1 , MGRad, mgHeight },{ { 105.8, 127.6 }, 2 , MGRad, mgHeight }
};
field::element initPoleConfig[2] = {
	{ { 93, 47.3 }, 3 , 4 },{ { 46.9, 94 }, 3 , 4 }
};

field::field(robot *robit) : isInit(true) {
	field::initialize(robit);
}
//initial mogo values for position and colour
void field::initialize(robot *robit) {
	c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
	pl.assign(&initPoleConfig[0], &initPoleConfig[2]);
	robit->reset();
	robit->p.position.X = 100;
	robit->p.position.Y = 35;
	robit->p.mRot = 45;
	fieldInit = false;
	isInit = true;//so that this only gets called ONCE when the field tab is running
}
//initialized everything for the field, such as cone and mogo values, and robot posision
float calcD2Edge(float a, float b, robot *robit) {
	//EXPLANATION HERE:
	float C1 = ((((sqr(a) - sqr(b)) / robit->d.size) + robit->d.size) / 2);
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
	for (int v = 0; v < 4; v++) {
		d2V[v] = dist(pos, robit->db.vertices[v]);
	}
}
//calculates distance between the cone's centre and each vertice
bool field::element::directlyInVerticalPath(robot *robit) {//vertical lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->db.slopeV[0] = (robit->db.vertices[0].Y - robit->db.vertices[3].Y) / (robit->db.vertices[0].X - robit->db.vertices[3].X);//should be identical to slope[1] kinda redundant i guess
	robit->db.YintV[0] = robit->db.slopeV[0] * (x - (robit->db.vertices[0].X - origin.X)) + (robit->db.vertices[0].Y - origin.Y);
	robit->db.slopeV[1] = (robit->db.vertices[1].Y - robit->db.vertices[2].Y) / (robit->db.vertices[1].X - robit->db.vertices[2].X);
	robit->db.YintV[1] = robit->db.slopeV[1] * (x - (robit->db.vertices[1].X - origin.X)) + (robit->db.vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->db.YintV[0]) != getSign(robit->db.YintV[1]));//works for telling me if between the two lines
}
//checking if cone is directly in front of or behind the robit
bool field::element::directlyInHorizontalPath(robot *robit) {//horizontal lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->db.slopeH[0] = (robit->db.vertices[0].Y - robit->db.vertices[1].Y) / (robit->db.vertices[0].X - robit->db.vertices[1].X);
	robit->db.YintH[0] = robit->db.slopeH[0] * (x - (robit->db.vertices[0].X - origin.X)) + (robit->db.vertices[0].Y - origin.Y);
	robit->db.slopeH[1] = (robit->db.vertices[3].Y - robit->db.vertices[2].Y) / (robit->db.vertices[3].X - robit->db.vertices[2].X);
	robit->db.YintH[1] = robit->db.slopeH[1] * (x - (robit->db.vertices[3].X - origin.X)) + (robit->db.vertices[3].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->db.YintH[0]) != getSign(robit->db.YintH[1]));//works for telling me if between the two lines
}
//checking if cone is directly to the right of or left of the robit
void field::element::fencePush(fence *f) {
	d2E[0] = f->fieldSize - pos.Y;
	d2E[1] = f->fieldSize - pos.X;
		bool touchingRight = (pos.X <= (rad + f->depth));
	if (touchingRight) {
		tRight = true;
		pos.X += (rad + f->depth) - 1.1*pos.X;
	}
	else tRight = false;
		bool touchingTop = (d2E[0] <= (rad + f->depth));
	if (touchingTop) {//g
		tTop = true;
		pos.Y -= (rad + f->depth) - 1.1*d2E[0];
	}
	else tTop = false;
		bool touchingLeft = (d2E[1] <= (rad + f->depth));
	if (touchingLeft) {
		tLeft= true;
		pos.X -= (rad + f->depth) - 1.1*d2E[1];
	}
	else tLeft = false;
		bool touchingBottom = (pos.Y <= (rad + f->depth));
	if (touchingBottom) {//g
		tBott = true;
		pos.Y += (rad + f->depth) - 1.1*pos.Y;
	}
	else tBott = false;
	//if (pos.X > (rad + f->depth) && d2E[0] > (rad + f->depth) && d2E[1] > (rad + f->depth) && pos.Y >= (rad + f->depth)) touchingFence = false;
}
//calculates distances to the edges of the field, and acts accordingly
bool withinAngle(double angle, int lowerBound, int upperBound) {
	//checks if angle is within upper and lower
	int thresh = 2;//degrees of freedom
	angle = abs(angle);
	return (angle < upperBound - thresh && angle > lowerBound + thresh) || (angle + 180 < upperBound - thresh && angle + 180 > lowerBound + thresh || (angle + 360 < upperBound - thresh && angle + 360 > lowerBound + thresh));
	//checks both the positive and "negative" angle
}
void field::element::robotColl(int index, robot *robit, std::set<int> &pushCone, std::set<int> &pushMoGo, int type) {
	//collisions from robot
	d2Robot = dist(pos, robit->p.position);
	if (pos.Z < height && d2Robot < renderRad * robit->d.size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
		calcD2Vertices(robit);//calculate all the distances
		d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit);//calculates the distance to the edge of the robit
		bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if cone is closer to the front side
		if (directlyInVerticalPath(robit)) {//either directly in front or behing based off center x and y position
			if (inFront) closestPoint = vec3(pos.X + (d2RobotEdge)*cos(gAngle), pos.Y - (d2RobotEdge)*sin(gAngle));//does work
			else closestPoint = vec3(pos.X - (d2RobotEdge)*cos(gAngle), pos.Y + (d2RobotEdge)*sin(gAngle));//does work
		}
		//had to inverse x and y because horiontal lines
		else if (directlyInHorizontalPath(robit)) {
			bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if cone is closer to the right side
			if (onRight) closestPoint = vec3(pos.X + (d2RobotEdge)*sin(gAngle), pos.Y + (d2RobotEdge)*cos(gAngle));//does work
			else closestPoint = vec3(pos.X - (d2RobotEdge)*sin(gAngle), pos.Y - (d2RobotEdge)*cos(gAngle));//does work
		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(d2V[0], d2V[1], d2V[2], d2V[3]);
			closestPoint = robit->db.vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = dist(pos, closestPoint);
		vec3 R = (closestPoint + pos.times(-1)).times(rad / d2closestPoint) + pos;
		if (d2closestPoint <= rad) {//touching
			pos.X -= R.X - closestPoint.X;
			pos.Y -= R.Y - closestPoint.Y;
			bool crushingCone = ((tBott && withinAngle(robit->p.mRot, 225, 315)) || (tTop && withinAngle(robit->p.mRot, 45, 135)) || (tRight && (withinAngle(robit->p.mRot, 0, 45) || withinAngle(robit->p.mRot, 315, 360))) || (tLeft && withinAngle(robit->p.mRot, 135, 225)));
			//could change crushingCone to be affected for a smaller angle, so that the reverse push only happens if almost directly crushing against the fence
			if (crushingCone ) {//HAVE only affected when pushing further into fence
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
		else if(d2closestPoint >=rad * 1.05) 
			if (index + type * 100 <= numCones) pushCone.erase(index);
			else pushMoGo.erase(index);
	}
}
//functions for collisions between the element and the robot
void field::element::collision(int index, element *e) {
	//collisions from element->element
	if (e->pos.Z < e->height) {//so long as within touching distance. 
		float distance = dist(pos, e->pos);
		if (distance <= coefMag * (rad + e->rad)) {//touching
			vec3 normal = vec3(e->pos.X - pos.X, e->pos.Y - pos.Y);
			e->pos = e->pos + normal.times(((rad + e->rad) - distance) / distance);//push of the cones
			/*maths
			distance*? = overlap;
			overlap = 2*cRad - distance
			therefore: ? = (2*cRad - distance)/distance;
			*/
			//s.insert(index);//adds index of which cone is in the stack (being pushed)
		}
	}
}
//functions for collisions between the element and another element
void field::physics(int index, element *e, robot *robit, int type) {
	for (int k = 0; k < c.size(); k++) {
		e->fencePush(&f);//pushes the cone from the fence if touching
		e->robotColl(index, robit, pushCones, pushMoGo, type);
		if (k != index) e->collision(index, &c[k]);
		else if (type != 0) e->collision(index, &c[k]);
	}
	for (int m = 0; m < mg.size(); m++) {
		e->fencePush(&f);//pushes the cone from the fence if touching
		e->robotColl(index, robit, pushCones, pushMoGo, type);
		if (m != index) e->collision(index, &mg[m]);
		else if (type != 1) e->collision(index, &mg[m]);
	}
	for (int p = 0; p < pl.size(); p++) {
		if (index != p) e->collision(index, &pl[p]);
		else if (type != 2) e->collision(index, &pl[p]);
	}
}
//function for calling all the collision functions together for el->el and el->robot
void field::fence::wallPush(robot *robit) {
	//deals with robot's boundaries and stationary goals
	for (int i = 0; i < 4; i++) {
		d2E[0] = fieldSize - robit->db.vertices[i].Y;//distance to top
		d2E[1] = fieldSize - robit->db.vertices[i].X;//distance to far left
		if (robit->db.vertices[i].X <= (depth)) {//checking right side
			robit->p.position.X += (depth)-robit->db.vertices[i].X;
			if (withinAngle(robit->p.mRot, 0, 90) || withinAngle(robit->p.mRot, 180, 270))  robit->p.mRot += robit->p.velocity.X * cos(gAngle);
			else if (withinAngle(robit->p.mRot, 270, 360) || withinAngle(robit->p.mRot, 90, 180)) robit->p.mRot -= robit->p.velocity.X * cos(gAngle);
		}
		else if (d2E[1] <= (depth)) {//checking left side
			robit->p.position.X -= (depth)-d2E[1];
			if (withinAngle(robit->p.mRot, 90, 180) || withinAngle(robit->p.mRot, 270, 360))  robit->p.mRot -= robit->p.velocity.X * cos(gAngle);
			else if (withinAngle(robit->p.mRot, 180, 270) || withinAngle(robit->p.mRot, 0, 90)) robit->p.mRot += robit->p.velocity.X * cos(gAngle);
		}
		if (d2E[0] <= (depth)) {//checking top
			robit->p.position.Y -= (depth)-d2E[0];
			if (withinAngle(robit->p.mRot, 90, 180) || withinAngle(robit->p.mRot, 270, 360))  robit->p.mRot -= robit->p.velocity.Y * sin(gAngle);
			else if (withinAngle(robit->p.mRot, 0, 90) || withinAngle(robit->p.mRot, 0, 90)) robit->p.mRot += robit->p.velocity.Y * sin(gAngle);
		}
		else if (robit->db.vertices[i].Y <= (depth)) {//checking bottom side
			robit->p.position.Y += (depth)-robit->db.vertices[i].Y;
			if (withinAngle(robit->p.mRot, 270, 360) || withinAngle(robit->p.mRot, 90, 180))  robit->p.mRot -= robit->p.velocity.Y * sin(gAngle);
			else if (withinAngle(robit->p.mRot, 180, 270) || withinAngle(robit->p.mRot, 0, 90)) robit->p.mRot += robit->p.velocity.Y * sin(gAngle);
		}
	}
}
void field::statGoalPush(element *pl, robot *robit, fence *f) {
	float d2obj = dist(robit->p.position, pl->pos);
	if (d2obj < renderRad * robit->d.size) {
		for (int v = 0; v < 4; v++) {
			pl->d2V[v] = dist(pl->pos, robit->db.vertices[v]);
		}
		bool inFront = (pl->d2V[0] + pl->d2V[1] < pl->d2V[2] + pl->d2V[3]);//checking if cone is closer to the front side
		if (pl->directlyInVerticalPath(robit)) {
			float d2Edge = calcD2Edge(SortSmallest(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]), Sort2ndSmallest(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]), robit);//calculates the distance to the edge of the robit
			if (inFront) pl->closestPoint = vec3(pl->pos.X + (d2Edge)*cos(gAngle), pl->pos.Y - (d2Edge)*sin(gAngle));//does work
			else pl->closestPoint = vec3(pl->pos.X - (d2Edge)*cos(gAngle), pl->pos.Y + (d2Edge)*sin(gAngle));//does work
		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]);
			pl->closestPoint = robit->db.vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = dist(pl->pos, pl->closestPoint);
		vec3 R = (pl->closestPoint + pl->pos.times(-1)).times(pl->rad / d2closestPoint) + pl->pos;
		if (d2closestPoint <= pl->rad) {//touching
			robit->p.position.X += (R.X - pl->closestPoint.X);
			robit->p.position.Y += (R.Y - pl->closestPoint.Y);
			//rotation when hits the stationary goal
			int thresh = 2;//degrees of freedom
			if (inFront) {
				if (abs(pl->d2V[0] - pl->d2V[1]) > thresh) {
					if (pl->d2V[0] < pl->d2V[1])//checking which way to rotate
						robit->p.mRot += abs(robit->p.velocity.Y * sin(gAngle));
					else if (pl->d2V[0] > pl->d2V[1])
						robit->p.mRot -= abs(robit->p.velocity.Y * sin(gAngle));
				}
			}
			else  if (abs(pl->d2V[2] - pl->d2V[3]) > thresh) {
				if (pl->d2V[2] > pl->d2V[3])
					robit->p.mRot -= abs(robit->p.velocity.Y * sin(gAngle));
				else if (pl->d2V[2] < pl->d2V[3])
					robit->p.mRot += abs(robit->p.velocity.Y * sin(gAngle));
			}
		}
	}
}
//function for making sure the robot cannot move past the fence
void field::element::grabbed(robot *robit, int index, int type) {
	//float slope = (robit->db.vertices[0].Y - robit->db.vertices[3].Y) / (robit->db.vertices[0].X - robit->db.vertices[3].X);
	bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if cone is closer to the front side
	//float yInt1C = slope * (0 - (robit->db.vertices[0].X + robit->c.clawSize*cos((robit->p.mRot - 135) * PI / 180) - pos.X)) + (robit->db.vertices[0].Y + robit->c.clawSize*sin((robit->p.mRot - 135) * PI / 180) - pos.Y);
	//float yInt2C = slope * (0 - (robit->db.vertices[1].X - robit->c.clawSize*cos((robit->p.mRot - 135) * PI / 180) - pos.X)) + (robit->db.vertices[1].Y - robit->c.clawSize*sin((robit->p.mRot - 135) * PI / 180) - pos.Y);
	index = index + type * 100;
	bool inPositionFront = (
		abs(pos.X - robit->p.position.X + (robit->d.size / 2) * cos((robit->p.mRot) * PI / 180) * sqrt(2)) < 1.5 &&
		abs(pos.Y - robit->p.position.Y - (robit->d.size / 2) * sin((robit->p.mRot) * PI / 180) * sqrt(2)) < 1.5);
	bool inPositionBack = (
		abs(pos.X - robit->p.position.X - (robit->d.size / 2) * cos((robit->p.mRot) * PI / 180) * sqrt(2)) < MGRad &&
		abs(pos.Y - robit->p.position.Y + (robit->d.size / 2) * sin((robit->p.mRot) * PI / 180) * sqrt(2)) < MGRad);
	if (index < numCones && (robit->c.grabbing && robit->c.holding == index) || (robit->c.grabbing && robit->c.holding == -1)) {//holding only one CONE at once (uses INDEX rather than INDEX with mg modification) ANS INDEX IS ONLY -1
		//bool inPosition = (getSign(yInt1C) != getSign(yInt2C) /*&& (d2Robot < 0.65*robit->d.size + rad) */&& (d2RobotEdge <= 1.35*rad) && inFront);
		if (inPositionFront) {
			//robit->holding = true;//locking the entities in place
			robit->c.holding = index; //+ 100 * type;//does not affect cones (as type is 0), but makes it so that mogos have an "index" of something between 100 and 108 (out of range of cones)
			pos.X = robit->p.position.X - (robit->d.size / 2) * cos((robit->p.mRot) * PI / 180) * sqrt(2);//works
			pos.Y = robit->p.position.Y + (robit->d.size / 2) * sin((robit->p.mRot) * PI / 180) * sqrt(2);//works
			held = true;
			//checking if shoudl lift;(ASSUMES PERFECT PID)
			if(abs(robit->c.liftPos - pos.Z) < height){//makes sure lift is within grabbing distance
				if (robit->c.liftUp && pos.Z < robit->c.maxHeight || robit->c.liftDown && pos.Z > 0) 
					pos.Z = robit->c.liftPos;//LATER: add something to try to pickyp from center
			}
		}
	}
	else if ((robit->mg.grabbing && robit->mg.holding == index) || (robit->mg.grabbing && robit->mg.holding == -101)) {//holding only one MOGO at once (uses INDEX with mg modification rather than just INDEX ) AND INDEX IS ONLY -100
		//bool inPosition = (getSign(yInt1MG) != getSign(yInt2MG) && (d2Robot < 0.65*robit->d.size + rad) && (d2RobotEdge <= 1.1*rad) && !inFront);
		if (inPositionBack) {
			//robit->holding = true;//locking the entities in place
			robit->mg.holding = index;// +100 * type;//does not affect cones (as type is 0), but makes it so that mogos have an "index" of something between 100 and 108 (out of range of cones)
			pos.X = robit->p.position.X + (robit->d.size / 2) * cos((robit->p.mRot) * PI / 180) * sqrt(2);
			pos.Y = robit->p.position.Y - (robit->d.size / 2) * sin((robit->p.mRot) * PI / 180) * sqrt(2);
			held = true;
		}
	}
	else held = false;
	if (!robit->c.grabbing && pos.Z > 0 || ((!inPositionFront && !inPositionBack) && pos.Z > 0)) {
		pos.Z += -32 / 12;
		pos.Z += -32 / 12;
	}
}
//function for having a 'grabbed' element lock in place
void field::FieldUpdate(robot *robit) {
	if (!isInit) initialize(robit);
	f.wallPush(robit);
	for (int i = 0; i < c.size(); i++) {
		//type for "cone" is 0
		int type = 0;
		c[i].rad = 0.1*c[i].pos.Z + cRad;//changes radius to enlargen when gets taller
		c[i].grabbed(robit, i, type);
		if(c[i].pos.Z < c[i].height) physics(i, &c[i], robit, type);//only affect objects when on ground (or low enough)
	}
	for (int i = 0; i < mg.size(); i++) {
		//type for "mogo" is 1
		int type = 1;
		//mg[i].rad = 0.1*mg[i].pos.Z + MGRad;//use better scalar than 0.1, but eventually remove this (because dont rly want to pick up mogos
		mg[i].grabbed(robit, i, type);
		physics(i, &mg[i], robit, type);//dont affect things if being lifted into the air
	}
	for (int i = 0; i < pl.size(); i++) {
		//type for stationary goal is 2
		int type = 2;
		pl[0].pos = vec3(93, 47.3);//stationary goal (not moving)
		pl[1].pos = vec3(46.9, 94);//stationary goal (not moving)
		statGoalPush(&pl[i], robit, &f);
		physics(i, &pl[i], robit, type);
	}
	robit->p.frictionC = pushCones.size();
	robit->p.frictionM = pushMoGo.size();
}
//update task for the entire field simulation