#include "field.h"

//declares and defines the field class and functions

field::element initConeConfig[numCones] = {//array for each configuration of the cone (in field.h)
										   //{initial posision (X, Y), color (Y, R, B), radii }
	{ { 2.9, 13.0 }, 0, coneRad },{ { 2.9, 23.2 }, 0, coneRad },{ { 2.9, 34.9 }, 0, coneRad },{ { 2.9, 46.7 }, 0, coneRad },
	{ { 2.9, 58.4 }, 0, coneRad },{ { 2.9, 70.2 }, 0, coneRad },{ { 13.0, 2.9 }, 0, coneRad },{ { 13.0, 13.0 }, 0, coneRad },
	{ { 13.0, 23.2 }, 0, coneRad },{ { 23.2, 2.9 }, 0, coneRad },{ { 23.2, 13.0 }, 0, coneRad },{ { 23.2, 23.2 }, 0, coneRad },
	{ { 23.2, 34.9 }, 0, coneRad },{ { 23.2, 46.7 }, 0, coneRad },{ { 23.2, 58.4 }, 0, coneRad },{ { 23.2, 70.2 }, 0, coneRad },
	{ { 34.9, 2.9 }, 0, coneRad },{ { 34.9, 23.2 }, 0, coneRad },{ { 46.7, 2.9 }, 0, coneRad },{ { 46.7, 23.2 }, 0, coneRad },
	{ { 46.7, 46.7 }, 0, coneRad },{ { 46.7, 58.4 }, 0, coneRad },{ { 58.4, 2.9 }, 0, coneRad },{ { 58.4, 23.2 }, 0, coneRad },
	{ { 58.4, 46.7 }, 0, coneRad },{ { 58.4, 70.2 }, 0, coneRad },{ { 70.2, 2.9 }, 0, coneRad },{ { 70.2, 23.2 }, 0, coneRad },
	{ { 70.2, 58.4 }, 0, coneRad },{ { 70.2, 82.1 }, 0, coneRad },{ { 82.1, 70.2 }, 0, coneRad },{ { 82.1, 93.9 }, 0, coneRad },
	{ { 82.1, 139.2 }, 0, coneRad },{ { 93.9, 82.1 }, 0, coneRad },{ { 93.9, 93.9 }, 0, coneRad },{ { 93.9, 117.5 }, 0, coneRad },
	{ { 93.9, 137.8 }, 0, coneRad },{ { 105.8, 117.5 }, 0, coneRad },{ { 105.8, 137.8 }, 0, coneRad },{ { 117.5, 93.9 }, 0, coneRad },
	{ { 117.5, 105.8 }, 0, coneRad },{ { 117.5, 117.5 }, 0, coneRad },{ { 117.5, 127.6 }, 0, coneRad },{ { 117.5, 137.8 }, 0, coneRad },
	{ { 127.6, 117.5 }, 0, coneRad },{ { 127.6, 127.6 }, 0, coneRad },{ { 127.6, 137.8 }, 0, coneRad },{ { 137.8, 82.1 }, 0, coneRad },
	{ { 137.8, 93.9 }, 0, coneRad },{ { 137.8, 105.8 }, 0, coneRad },{ { 137.8, 117.5 }, 0, coneRad },{ { 137.8, 127.6 }, 0, coneRad },
	{ { 137.8, 137.8 }, 0, coneRad }
};
//initial cone values for position
field::element initMoGoConfig[numMoGos] = {//array for each configuration of the mobile goal (in field.h)
	{ { 34.9, 13.0 }, 1 , MoGoRad },{ { 13.0, 34.9 }, 2 ,MoGoRad },
	{ { 70.2, 46.7 }, 2 ,MoGoRad },{ { 46.7, 70.2 }, 1 ,MoGoRad },
	{ { 93.9, 70.2 }, 2 ,MoGoRad },{ { 70.2, 93.9 }, 1 ,MoGoRad },
	{ { 127.6, 105.8 }, 1 ,MoGoRad },{ { 105.8, 127.6 }, 2 ,MoGoRad }
};
field::element initPoleConfig[2] = {
	{ { 93, 47.3 }, 3 , 4 },{ { 46.9, 94 }, 3 , 4 }
};

//initial mogo values for position and colour
void field::initializeField(robot *robit) {
	c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
	pl.assign(&initPoleConfig[0], &initPoleConfig[2]);
	robit->position.X = 100;
	robit->position.Y = 35;
	robit->mRot = 45;
	initialized = true;//so that this only gets called ONCE when the field tab is running
}
//initialized everything for the field, such as cone and mogo values, and robot posision
float calcD2Edge(float a, float b, robot *robit) {
	//EXPLANATION HERE:
	float C1 = ((((sqr(a) - sqr(b)) / robit->size) + robit->size) / 2);
	return sqrt(abs(sqr(a) - sqr(C1)));
}
//calculate distance to edge of robot
void field::element::calcD2Vertices(robot *robit) {
	/******vertices:*******
	0----------1
	|    r     |
	|          |
	3----------2
	**********************/
	for (int v = 0; v < 4; v++) {
		d2V[v] = dist(pos, robit->vertices[v]);
	}
}
//calculates distance between the cone's centre and each vertice
bool field::element::directlyInVerticalPath(robot *robit) {//vertical lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeV[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);//should be identical to slope[1] kinda redundant i guess
	robit->YintV[0] = robit->slopeV[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeV[1] = (robit->vertices[1].Y - robit->vertices[2].Y) / (robit->vertices[1].X - robit->vertices[2].X);
	robit->YintV[1] = robit->slopeV[1] * (x - (robit->vertices[1].X - origin.X)) + (robit->vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintV[0]) != getSign(robit->YintV[1]));//works for telling me if between the two lines
}
//checking if cone is directly in front of or behind the robit
bool field::element::directlyInHorizontalPath(robot *robit) {//horizontal lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeH[0] = (robit->vertices[0].Y - robit->vertices[1].Y) / (robit->vertices[0].X - robit->vertices[1].X);
	robit->YintH[0] = robit->slopeH[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeH[1] = (robit->vertices[3].Y - robit->vertices[2].Y) / (robit->vertices[3].X - robit->vertices[2].X);
	robit->YintH[1] = robit->slopeH[1] * (x - (robit->vertices[3].X - origin.X)) + (robit->vertices[3].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintH[0]) != getSign(robit->YintH[1]));//works for telling me if between the two lines
}
//checking if cone is directly to the right of or left of the robit
void field::element::fencePush(fence *f) {
	d2E[0] = f->fieldSize - pos.Y;
	d2E[1] = f->fieldSize - pos.X;
	bool touchingRight = (pos.X <= (rad + f->depth));
	if (touchingRight) {
		touchingFence = true;
		pos.X += (rad + f->depth) - 1.1*pos.X;
	}
	bool touchingTop = (d2E[0] <= (rad + f->depth));
	if (touchingTop) {//g
		touchingFence = true;
		pos.Y -= (rad + f->depth) - 1.1*d2E[0];
	}
	bool touchingLeft = (d2E[1] <= (rad + f->depth));
	if (touchingLeft) {
		touchingFence = true;
		pos.X -= (rad + f->depth) - 1.1*d2E[1];
	}
	bool touchingBottom = (pos.Y <= (rad + f->depth));
	if (touchingBottom) {//g
		touchingFence = true;
		pos.Y += (rad + f->depth) - 1.1*pos.Y;
	}
	if (pos.X > (rad + f->depth) && d2E[0] > (rad + f->depth) && d2E[1] > (rad + f->depth) && pos.Y >= (rad + f->depth)) touchingFence = false;
}
//calculates distances to the edges of the field, and acts accordingly
void field::element::robotColl(int index, robot *robit, std::set<int> &s) {
	//collisions from robot
	d2Robot = dist(pos, robit->position);
	if (d2Robot < renderRad * robit->size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
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
			closestPoint = robit->vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = dist(pos, closestPoint);
		vec3 R = (closestPoint + pos.times(-1)).times(rad / d2closestPoint) + pos;
		if (d2closestPoint <= rad) {//touching
			pos.X -= R.X - closestPoint.X;
			pos.Y -= R.Y - closestPoint.Y;
			if (touchingFence ) {
				robit->velocity.X = (R.X - closestPoint.X);
				robit->velocity.Y = (R.Y - closestPoint.Y);
				int thresh = 2;//degrees of freedom
				if (inFront) {
					if (abs(d2V[0] - d2V[1]) > thresh) {
						if (d2V[0] < d2V[1])//checking which way to rotate
							robit->mRot += abs(.07 * sin(gAngle));//CHANGE 0.07 TO SOMETHING OF USE, NOT A CONSTANT
						else if (d2V[0] > d2V[1])
							robit->mRot -= abs(.07 * sin(gAngle));
					}
				}
				else if (abs(d2V[2] - d2V[3]) > thresh) {
					if (d2V[2] > d2V[3])
						robit->mRot -= abs(robit->velocity.Y * sin(gAngle));
					else if (d2V[2] < d2V[3])
						robit->mRot += abs(robit->velocity.Y * sin(gAngle));
				}
			}
		}
	}
}
//functions for collisions between the element and the robot
void field::element::collision(int index, element *e, std::set<int> &s) {
	//collisions from element->element
	float distance = dist(pos, e->pos);
	if (distance <= MAGNETS * (rad + e->rad)) {//touching
		vec3 normal = vec3(e->pos.X - pos.X, e->pos.Y - pos.Y);
		e->pos = e->pos + normal.times(((rad + e->rad) - distance) / distance);//push of the cones
		/*maths
		distance*? = overlap;
		overlap = 2*coneRad - distance
		therefore: ? = (2*coneRad - distance)/distance;
		*/
		//s.insert(index);//adds index of which cone is in the stack (being pushed)
	}
}
//functions for collisions between the element and another element
void field::physics(int index, element *e, robot *robit, int type) {
	for (int k = 0; k < c.size(); k++) {
		e->fencePush(&f);//pushes the cone from the fence if touching
		e->robotColl(index, robit, stacked);
		if (k != index) e->collision(index, &c[k], stacked);
		else if (type != 0) e->collision(index, &c[k], stacked);
	}
	for (int m = 0; m < mg.size(); m++) {
		e->fencePush(&f);//pushes the cone from the fence if touching
		e->robotColl(index, robit, stacked);
		if (m != index) e->collision(index, &mg[m], stacked);
		else if (type != 1) e->collision(index, &mg[m], stacked);
	}
	for (int p = 0; p < pl.size(); p++) {
		if (index != p) e->collision(index, &pl[p], stacked);
		else if (type != 2) e->collision(index, &pl[p], stacked);
	}
}
//function for calling all the collision functions together for el->el and el->robot
bool quadrant(double angle, int quadrant) {
	vec3 bounds;//(no greater than, no less than)
	if (quadrant == 1) bounds = vec3(89, 1);
	else if (quadrant == 2) bounds = vec3(179, 91);
	else if (quadrant == 3) bounds = vec3(269, 181);
	else if (quadrant == 4) bounds = vec3(359, 271);
	else return false;
	return (angle < bounds.X && angle > bounds.Y) || (angle < bounds.X - 360 && angle > bounds.Y - 360);
}
void field::fence::wallPush(robot *robit) {
	//deals with robot's boundaries and stationary goals
	for (int i = 0; i < 4; i++) {
		d2E[0] = fieldSize - robit->vertices[i].Y;//distance to top
		d2E[1] = fieldSize - robit->vertices[i].X;//distance to far left
		if (robit->vertices[i].X <= (depth)) {//checking right side
			robit->position.X += (depth)-robit->vertices[i].X;
			if (quadrant(robit->mRot, 1) || quadrant(robit->mRot, 3))  robit->mRot -= robit->velocity.X * cos(gAngle);
			else if (quadrant(robit->mRot, 4) || quadrant(robit->mRot, 2)) robit->mRot += robit->velocity.X * cos(gAngle);
		}
		else if (d2E[1] <= (depth)) {//checking left side
			robit->position.X -= (depth)-d2E[1];
			if (quadrant(robit->mRot, 2) || quadrant(robit->mRot, 4))  robit->mRot -= robit->velocity.X * cos(gAngle);
			else if (quadrant(robit->mRot, 3) || quadrant(robit->mRot, 1)) robit->mRot += robit->velocity.X * cos(gAngle);
		}
		if (d2E[0] <= (depth)) {//checking top
			robit->position.Y -= (depth)-d2E[0];
			if (quadrant(robit->mRot, 2) || quadrant(robit->mRot, 4))  robit->mRot -= robit->velocity.Y * sin(gAngle);
			else if (quadrant(robit->mRot, 1) || quadrant(robit->mRot, 1)) robit->mRot += robit->velocity.Y * sin(gAngle);
		}
		else if (robit->vertices[i].Y <= (depth)) {//checking bottom side
			robit->position.Y += (depth)-robit->vertices[i].Y;
			if (quadrant(robit->mRot, 4) || quadrant(robit->mRot, 2))  robit->mRot += robit->velocity.Y * sin(gAngle);
			else if (quadrant(robit->mRot, 3) || quadrant(robit->mRot, 1)) robit->mRot -= robit->velocity.Y * sin(gAngle);
		}
	}
}
void field::statGoalPush(element *pl, robot *robit, fence *f) {
	float d2obj = dist(robit->position, pl->pos);
	if (d2obj < renderRad * robit->size) {
		for (int v = 0; v < 4; v++) {
			pl->d2V[v] = dist(pl->pos, robit->vertices[v]);
		}
		bool inFront = (pl->d2V[0] + pl->d2V[1] < pl->d2V[2] + pl->d2V[3]);//checking if cone is closer to the front side
		if (pl->directlyInVerticalPath(robit)) {
			float d2Edge = calcD2Edge(SortSmallest(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]), Sort2ndSmallest(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]), robit);//calculates the distance to the edge of the robit
			if (inFront) pl->closestPoint = vec3(pl->pos.X + (d2Edge)*cos(gAngle), pl->pos.Y - (d2Edge)*sin(gAngle));//does work
			else pl->closestPoint = vec3(pl->pos.X - (d2Edge)*cos(gAngle), pl->pos.Y + (d2Edge)*sin(gAngle));//does work
		}
		else {//not directly in path finds which vertice is the closest to the cone
			int smallest_vertice = sortSmallVER(pl->d2V[0], pl->d2V[1], pl->d2V[2], pl->d2V[3]);
			pl->closestPoint = robit->vertices[smallest_vertice];//closest point to center will then be the vertice
		}
		float d2closestPoint = dist(pl->pos, pl->closestPoint);
		vec3 R = (pl->closestPoint + pl->pos.times(-1)).times(pl->rad / d2closestPoint) + pl->pos;
		if (d2closestPoint <= pl->rad) {//touching
			robit->position.X += (R.X - pl->closestPoint.X);
			robit->position.Y += (R.Y - pl->closestPoint.Y);
			//rotation when hits the stationary goal
			int thresh = 2;//degrees of freedom
			if (inFront) {
				if (abs(pl->d2V[0] - pl->d2V[1]) > thresh) {
					if (pl->d2V[0] < pl->d2V[1])//checking which way to rotate
						robit->mRot += abs(robit->velocity.Y * sin(gAngle));
					else if (pl->d2V[0] > pl->d2V[1])
						robit->mRot -= abs(robit->velocity.Y * sin(gAngle));
				}
			}
			else  if (abs(pl->d2V[2] - pl->d2V[3]) > thresh) {
				if (pl->d2V[2] > pl->d2V[3])
					robit->mRot -= abs(robit->velocity.Y * sin(gAngle));
				else if (pl->d2V[2] < pl->d2V[3])
					robit->mRot += abs(robit->velocity.Y * sin(gAngle));
			}
		}
	}
}
//function for making sure the robot cannot move past the fence
void field::element::grabbed(robot *robit, int index, int type) {
	if ((robit->grabbing && robit->holding == index + type * 100) || (robit->grabbing && robit->holding == -1)) {//holding only one entity at once
		float slope = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
		float yInt1 = slope * (0 - (robit->vertices[0].X + robit->clawSize*cos((robit->mRot - 135) * PI / 180) - pos.X)) + (robit->vertices[0].Y + robit->clawSize*sin((robit->mRot - 135) * PI / 180) - pos.Y);
		float yInt2 = slope * (0 - (robit->vertices[1].X - robit->clawSize*cos((robit->mRot - 135) * PI / 180) - pos.X)) + (robit->vertices[1].Y - robit->clawSize*sin((robit->mRot - 135) * PI / 180) - pos.Y);
		bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if cone is closer to the front side
		bool inPosition = (getSign(yInt1) != getSign(yInt2) && (d2Robot < 0.65*robit->size + rad) && (d2RobotEdge <= 1.1*rad) && inFront);
		if (inPosition) {
			//robit->holding = true;//locking the entities in place
			robit->holding = index + 100 * type;//does not affect cones (as type is 0), but makes it so that mogos have an "index" of something between 100 and 108 (out of range of cones)
			pos.X = (robit->position.X) - (robit->size*.65 * cos(gAngle));
			pos.Y = (robit->position.Y + (robit->size*.65 * sin(gAngle)));
			held = true;
		}
	}
	else held = false;
}
void field::FieldUpdate(robot *robit) {
	if (!initialized) initializeField(robit);
	f.wallPush(robit);
	for (int i = 0; i < c.size(); i++) {
		//type for "cone" is 0
		int type = 0;
		c[i].grabbed(robit, i, type);
		physics(i, &c[i], robit, type);
	}
	for (int i = 0; i < mg.size(); i++) {
		//type for "mogo" is 1
		int type = 1;
		mg[i].grabbed(robit, i, type);
		physics(i, &mg[i], robit, type);
	}
	for (int i = 0; i < pl.size(); i++) {
		//type for stationary goal is 2
		int type = 2;
		pl[0].pos = vec3(93, 47.3);//stationary goal (not moving)
		pl[1].pos = vec3(46.9, 94);//stationary goal (not moving)
		statGoalPush(&pl[i], robit, &f);
		physics(i, &pl[i], robit, type);
	}
}
//update task for the entire field simulation
field::field() : initialized(false) {
	c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
	pl.assign(&initPoleConfig[0], &initPoleConfig[2]);
}
//constructor