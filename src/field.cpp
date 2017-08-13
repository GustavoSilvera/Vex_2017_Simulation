#include "field.h"

//declares and defines the field class and functions

field::element initConeConfig[numCones] = {//array for each configuration of the cone (in field.h)
	//{initial posision (X, Y), color (Y, R, B), radii }
	{ { 2.9, 13.0 }, 0, coneRad }, { { 2.9, 23.2 }, 0, coneRad }, { { 2.9, 34.9 }, 0, coneRad }, { { 2.9, 46.7 }, 0, coneRad },
	{ { 2.9, 58.4 }, 0, coneRad }, { { 2.9, 70.2 }, 0, coneRad }, { { 13.0, 2.9 }, 0, coneRad }, { { 13.0, 13.0 }, 0, coneRad },
	{ { 13.0, 23.2 }, 0, coneRad }, { { 23.2, 2.9 }, 0, coneRad }, { { 23.2, 13.0 }, 0, coneRad }, { { 23.2, 23.2 }, 0, coneRad },
	{ { 23.2, 34.9 }, 0, coneRad }, { { 23.2, 46.7 }, 0, coneRad }, { { 23.2, 58.4 }, 0, coneRad }, { { 23.2, 70.2 }, 0, coneRad },
	{ { 34.9, 2.9 }, 0, coneRad }, { { 34.9, 23.2 }, 0, coneRad }, { { 46.7, 2.9 }, 0, coneRad }, { { 46.7, 23.2 }, 0, coneRad },
	{ { 46.7, 46.7 }, 0, coneRad }, { { 46.7, 58.4 }, 0, coneRad }, { { 58.4, 2.9 }, 0, coneRad }, { { 58.4, 23.2 }, 0, coneRad },
	{ { 58.4, 46.7 }, 0, coneRad }, { { 58.4, 70.2 }, 0, coneRad }, { { 70.2, 2.9 }, 0, coneRad }, { { 70.2, 23.2 }, 0, coneRad },
	{ { 70.2, 58.4 }, 0, coneRad }, { { 70.2, 82.1 }, 0, coneRad }, { { 82.1, 70.2 }, 0, coneRad }, { { 82.1, 93.9 }, 0, coneRad },
	{ { 82.1, 139.2 }, 0, coneRad }, { { 93.9, 82.1 }, 0, coneRad }, { { 93.9, 93.9 }, 0, coneRad }, { { 93.9, 117.5 }, 0, coneRad },
	{ { 93.9, 137.8 }, 0, coneRad }, { { 105.8, 117.5 }, 0, coneRad }, { { 105.8, 137.8 }, 0, coneRad }, { { 117.5, 93.9 }, 0, coneRad },
	{ { 117.5, 105.8 }, 0, coneRad }, { { 117.5, 117.5 }, 0, coneRad }, { { 117.5, 127.6 }, 0, coneRad }, { { 117.5, 137.8 }, 0, coneRad },
	{ { 127.6, 117.5 }, 0, coneRad }, { { 127.6, 127.6 }, 0, coneRad }, { { 127.6, 137.8 }, 0, coneRad }, { { 137.8, 82.1 }, 0, coneRad },
	{ { 137.8, 93.9 }, 0, coneRad }, { { 137.8, 105.8 }, 0, coneRad }, { { 137.8, 117.5 }, 0, coneRad }, { { 137.8, 127.6 }, 0, coneRad },
	{ { 137.8, 137.8 }, 0, coneRad }
};
//initial cone values for position
field::element initMoGoConfig[numMoGos] = {//array for each configuration of the mobile goal (in field.h)
	{ { 34.9, 13.0 }, 1 , MoGoRad }, { { 13.0, 34.9 }, 2 ,MoGoRad},
	{ { 70.2, 46.7 }, 2 ,MoGoRad}, { { 46.7, 70.2 }, 1 ,MoGoRad},
	{ { 93.9, 70.2 }, 2 ,MoGoRad}, { { 70.2, 93.9 }, 1 ,MoGoRad},
	{ { 127.6, 105.8 }, 1 ,MoGoRad}, { { 105.8, 127.6 }, 2 ,MoGoRad}
};
field::element initPoleConfig[2] = {
	{ { 93.9, 46.7 }, 3 , 4 },{ { 46.7, 93.9 }, 3 , 4 }
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
	robit->slopeV[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
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
	if (pos.X <= (rad + f->depth)) {
		pos.X += (rad + f->depth) - 1.1*pos.X;
	}
	if (d2E[0] <= (rad + f->depth)) {//g
		pos.Y -= (rad + f->depth) - 1.1*d2E[0];
	}
	if (d2E[1] <= (rad + f->depth)) {
		pos.X -= (rad + f->depth) - 1.1*d2E[1];
	}
	if (pos.Y <= (rad + f->depth)) {//g
		pos.Y += (rad + f->depth) - 1.1*pos.Y;
	}
}
//calculates distances to the edges of the field, and acts accordingly
void field::element::robotColl(int index, robot *robit, std::set<int> &s) {
	//collisions from robot
	float d2Robot = dist(pos, robit->position);
	if (d2Robot < renderRad * robit->size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
		calcD2Vertices(robit);//calculate all the distances
		float d2RobotEdge = calcD2Edge(SortSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), Sort2ndSmallest(d2V[0], d2V[1], d2V[2], d2V[3]), robit);//calculates the distance to the edge of the robit
		if (directlyInVerticalPath(robit)) {//either directly in front or behing based off center x and y position
			bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if cone is closer to the front side
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
			s.insert(index);//adds index of which cone is in the stack (being pushed)
			pos.X -= R.X - closestPoint.X;
			pos.Y -= R.Y - closestPoint.Y;
		}
		else {
			s.erase(index);//removes index of which cone is in the stack (being pushed)
		}
	}
}
//functions for collisions between the element and the robot
void field::element::collision(int index, element *e, std::set<int> &s) {
	//collisions from element->element
	float distance = dist(pos, e->pos);
	if (distance < rad + e->rad) {//touching
		vec3 normal = vec3(e->pos.X - pos.X, e->pos.Y - pos.Y);
		e->pos = e->pos + normal.times(((rad + e->rad) - distance) / distance);//push of the cones
		/*maths
		distance*? = overlap;
		overlap = 2*coneRad - distance
		therefore: ? = (2*coneRad - distance)/distance;
		*/
		s.insert(index);//adds index of which cone is in the stack (being pushed)
	}
}
//functions for collisions between the element and another element
void field::physics(int index, element *e, robot *robit) {
	e->fencePush(&f);//pushes the cone from the fence if touching
	e->robotColl(index, robit, stacked);
	for (int k = 0; k < c.size(); k++) {
		if (k != index) e->collision(index, &c[k], stacked);
	}
	for (int m = 0; m < mg.size(); m++) {
		if (m != index) e->collision(index, &mg[m], stacked);
	}
	for (int p = 0; p < pl.size(); p++) {
		e->collision(index, &pl[p], stacked);
	}
}
//function for calling all the collision functions together for el->el and el->robot
void field::fence::robotPush(robot *r) {
	//deals with robot's boundaries and stationary goals
	for (int i = 0; i < 4; i++) {
		d2E[0] = fieldSize - r->vertices[i].Y;
		d2E[1] = fieldSize - r->vertices[i].X;
		if (r->vertices[i].X <= (depth)) {//checking right side
			r->position.X += (depth) - r->vertices[i].X;
			if ((r->mRot < 90 && r->mRot > 0 ) || (r->mRot < -270 && r->mRot > -360))  r->mRot -= r->velocity.X * cos((r->mRot)*(PI / 180));
			else if ((r->mRot < 360 && r->mRot > 270) || (r->mRot < 0 && r->mRot > -90)) r->mRot += r->velocity.X * cos((r->mRot)*(PI / 180));
		}
		else if (d2E[1] <= (depth)) {//checking left side
			r->position.X -= (depth) - d2E[1];
			if ((r->mRot < 180 && r->mRot > 90) || (r->mRot < -180 && r->mRot > -270))  r->mRot -= r->velocity.X * cos((r->mRot)*(PI / 180));
			else if ((r->mRot < 270 && r->mRot > 180) || (r->mRot < -90 && r->mRot > -180)) r->mRot += r->velocity.X * cos((r->mRot)*(PI / 180));
		}
		if (d2E[0] <= (depth)) {//checking top
			r->position.Y -= (depth) - d2E[0];
			if ((r->mRot < 180 && r->mRot > 90) || (r->mRot < -180 && r->mRot > -270))  r->mRot -= r->velocity.Y * sin((r->mRot)*(PI / 180));
			else if ((r->mRot < 90 && r->mRot > 0) || (r->mRot < -270 && r->mRot > -360)) r->mRot += r->velocity.Y * sin((r->mRot)*(PI / 180));
		}
		else if (r->vertices[i].Y <= (depth)) {//checking bottom side
			r->position.Y += (depth) - r->vertices[i].Y;
			if ((r->mRot < 360 && r->mRot > 270) || (r->mRot < 0 && r->mRot > -90))  r->mRot += r->velocity.Y * sin((r->mRot)*(PI / 180));
			else if ((r->mRot < 270 && r->mRot > 180) || (r->mRot < -90 && r->mRot > -180)) r->mRot -= r->velocity.Y * sin((r->mRot)*(PI / 180));
		}
	}
	/*for (int i = 0; i < 2; i++) {
		vec3 polePos;
		if (i == 0) polePos = vec3(93.9, 46.7);
		else polePos = vec3(46.7, 93.9);
		d2StatGoal[i] = dist(r->position, polePos);
		if (d2StatGoal[i] < renderRad * r->size) {
			for (int v = 0; v < 4; v++) {
				d2V[v] = dist(polePos, r->vertices[v]);
			}

		}
	}*/
}
//function for making sure the robot cannot move past the fence
void field::FieldUpdate(robot *robit) {
	if (!initialized) initializeField(robit);
	f.robotPush(robit);
	for (int i = 0; i < c.size(); i++) {
		physics(i, &c[i], robit);
	}
	for (int i = 0; i < mg.size(); i++) {
		physics(i, &mg[i], robit);
	}
	for (int i = 0; i < pl.size(); i++) {
		pl[0].pos = vec3(93.9, 46.7);//stationary goal (not moving)
		pl[1].pos = vec3(46.7, 93.9);//stationary goal (not moving)
		physics(i, &pl[i], robit);
	}
}
//update task for the entire field simulation
 field::field() : initialized(false) {
	 c.assign(&initConeConfig[0], &initConeConfig[numCones]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	 mg.assign(&initMoGoConfig[0], &initMoGoConfig[numMoGos]);
 }
//constructor
 