#include "vec3.h"
#include "field.h"
#include "randomstuff.h"
#include "robot.h"
//declares and defines the field class and functions

field::cone initialConfiguration[54] = {//array for each configuration of the cone (in field.h)
	{ { 2.9, 2.9 }, 0, false},//position X,Y; initial heading; if tipped or not
	{ { 2.9, 13.0 }, 0, false},
	{ { 2.9, 23.2 }, 0, false},
	{ { 2.9, 34.9 }, 0, false},
	{ { 2.9, 46.7 }, 0, false},
	{ { 2.9, 58.4 }, 0, false},
	{ { 2.9, 70.2 }, 0, false},
	{ { 13.0, 2.9 }, 0, false},
	{ { 13.0, 13.0 }, 0, false},
	{ { 13.0, 23.2 }, 0, false},
	{ { 23.2, 2.9 }, 0, false},
	{ { 23.2, 13.0 }, 0, false},
	{ { 23.2, 23.2 }, 0, false},
	{ { 23.2, 34.9 }, 0, false},
	{ { 23.2, 46.7 }, 0, false},
	{ { 23.2, 58.4 }, 0, false},
	{ { 23.2, 70.2 }, 0, false},
	{ { 34.9, 2.9 }, 0, false},
	{ { 34.9, 23.2 }, 0, false},
	{ { 46.7, 2.9 }, 0, false},
	{ { 46.7, 23.2 }, 0, false},
	{ { 46.7, 46.7 }, 0, false},
	{ { 46.7, 58.4 }, 0, false},
	{ { 58.4, 2.9 }, 0, false },
	{ { 58.4, 23.2 }, 0, false },
	{ { 58.4, 46.7 }, 0, false },
	{ { 58.4, 70.2 }, 0, false },
	{ { 70.2, 2.9 }, 0, false },
	{ { 70.2, 23.2 }, 0, false },
	{ { 70.2, 58.4 }, 0, false },
	{ { 70.2, 82.1 }, 0, false },
	{ { 82.1, 70.2 }, 0, false },
	{ { 82.1, 93.9 }, 0, false },
	{ { 82.1, 139.2 }, 0, false },
	{ { 93.9, 82.1 }, 0, false },
	{ { 93.9, 93.9 }, 0, false },
	{ { 93.9, 117.5 }, 0, false },
	{ { 93.9, 137.8 }, 0, false },
	{ { 105.8, 117.5 }, 0, false },
	{ { 105.8, 137.8 }, 0, false },
	{ { 117.5, 93.9 }, 0, false },
	{ { 117.5, 105.8 }, 0, false },
	{ { 117.5, 117.5 }, 0, false },
	{ { 117.5, 127.6 }, 0, false },
	{ { 117.5, 137.8 }, 0, false },
	{ { 127.6, 117.5 }, 0, false },
	{ { 127.6, 127.6 }, 0, false },
	{ { 127.6, 137.8 }, 0, false },
	{ { 137.8, 82.1 }, 0, false },
	{ { 137.8, 93.9 }, 0, false },
	{ { 137.8, 105.8 }, 0, false },
	{ { 137.8, 117.5 }, 0, false },
	{ { 137.8, 127.6 }, 0, false },
	{ { 137.8, 137.8 }, 0, false }
}; 

void field::initializeField() {
	c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	
}

void distance2Vertices(robot *robit, struct field::cone *c) {
	/******vertices:*******
		0----------1
		|    r     |
		|          |
		3----------2
	**********************/
	for (int v = 0; v < 4; v++) {
		//calculates distance between the cone's centre and each vertice
		c->d2V[v] = sqrt(sqr(c->pos.X - robit->vertices[v].X) + sqr(c->pos.Y - robit->vertices[v].Y));
	}
}
float calculateAngle(robot *robit, struct field::cone *c) {
	float changeInX = c->pos.X - robit->position.X;
	float changeInY = c->pos.Y - robit->position.Y;
	return atan(changeInY / changeInX) * (180 / PI);
}
void findSmallestD2V(float v1, float v2, float v3, float v4, robot *robit, struct field::cone *c) {
	//supposed to return the distances to the nearest two vertices based off direction of the speed
	//WHAT TO FIX: MAKE THE BACK VERTICES DO SOMETHING IF MOVING BACKWARDS, AND SOMETIMES CONES ARE BEING CAUGHT IF CLOSE TO FRONT VERTICES
	if (robit->velocity.X > 0 && robit->velocity.Y > 0) {
		c->SmallestD2V[0] = c->d2V[0];
		c->SmallestD2V[1] = c->d2V[1];
	}
	else {
		c->SmallestD2V[0] = c->d2V[3];
		c->SmallestD2V[1] = c->d2V[2];
	}
}
float calcD2Edge(float a, float b, robot *robit) {
	//EXPLANATION HERE:
	float C1 = ((((sqr(a) - sqr(b)) / robit->size) + robit->size) / 2);
	return sqrt(abs(sqr(a) - sqr(C1)));
}
void field::FieldUpdate(robot *robit) {
	if (!initialized) {
		initializeField();
		robit->position.X = 100 ;
		robit->position.Y = 35;
		robit->mRot = 135;
		initialized = true;//so that this only gets called ONCE when the field tab is running
	}
	for (int i = 0; i < c.size(); i++) {
		float dConeToRobot = sqrt(sqr(c[i].pos.X - robit->position.X) + sqr(c[i].pos.Y - robit->position.Y));
		if (dConeToRobot < robit->size) {//within a radius around the robot of 18 inches around the center point of the body
			distance2Vertices(robit, &c[i]);//calculate all the distances
			findSmallestD2V(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3], robit, &c[i]);//figure out which vertices are the ones we care about when the robot is moving a certain direction
			float d2RobotEdge = calcD2Edge(c[i].SmallestD2V[0], c[i].SmallestD2V[1], robit);
			vec3 edgePoint = vec3(c[i].pos.X - (coneRad - d2RobotEdge)*cos(robit->ActualHeading * (PI / 180)), c[i].pos.Y + (coneRad - d2RobotEdge)*sin(robit->ActualHeading * (PI / 180)));
			vec3 penetrationDepth = c[i].pos + edgePoint.times(-1);//creates the vector between the cone's center and the perpendicular point of contact on the robot's edge (amount of penetration)
			if (robit->velocity.X < 0 && robit->velocity.Y < 0) penetrationDepth = penetrationDepth.times(-1);//pushed in the opposite direction when moving backwards (makes logical sense, yes?)
			if (d2RobotEdge <= coneRad)	c[i].pos = c[i].pos + penetrationDepth;//then touching :D
		}
		//things to do:
		//fix basic velocity physics (DONE)
		//fix basic robot->cone collisions(MOSTLY DONE)
		//have to make cones interact with other cones(NOT DONE)
		//		it makes sure that you only resolve a collision if the objects are moving towards each other.

	}
}
//make functino to compute ONLY the cones closest to the robot(by like 1 metre or something idk)

//constructor
 field::field() : initialized(false) {
	 c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
}
