#include "vec3.h"
#include "field.h"
#include "randomstuff.h"
#include "robot.h"
//declares and defines the field class and functions

field::cone initialConfiguration[1] = {//array for each configuration of the cone (in field.h)
	{ { 2.9, 2.9 }, 0, false},//position X,Y; initial heading; if tipped or not
	/*{ { 2.9, 13.0 }, 0, false},
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
	{ { 137.8, 137.8 }, 0, false }*/
}; 
void field::initializeField() {
	c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	initialized = true;//so that this only gets called ONCE when the field tab is running
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

float calcD2Edge(float a, float b, robot *robit) {
	//EXPLANATION HERE:
	float C1 = ((((sqr(a) - sqr(b)) / robit->size) + robit->size) / 2);
	return sqrt(abs(sqr(a) - sqr(C1)));
}
void inPath(robot *robit, struct field::cone *c) {
	vec3 origin = c->pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeV[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
	robit->YintV[0] = robit->slopeV[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeV[1] = (robit->vertices[1].Y - robit->vertices[2].Y) / (robit->vertices[1].X - robit->vertices[2].X);
	robit->YintV[1] = robit->slopeV[1] * (x - (robit->vertices[1].X - origin.X)) + (robit->vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	c->directlyInVerticalPath = (getSign(robit->YintV[0]) != getSign(robit->YintV[1]));//works for telling me if between the two lines
		//horizontal lines
	robit->slopeH[0] = (robit->vertices[0].Y - robit->vertices[1].Y) / (robit->vertices[0].X - robit->vertices[1].X);
	robit->YintH[0] = robit->slopeH[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeH[1] = (robit->vertices[3].Y - robit->vertices[2].Y) / (robit->vertices[3].X - robit->vertices[2].X);
	robit->YintH[1] = robit->slopeH[1] * (x - (robit->vertices[3].X - origin.X)) + (robit->vertices[3].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	c->directlyInHorizontalPath = (getSign(robit->YintH[0]) != getSign(robit->YintH[1]));//works for telling me if between the two lines

}
void field::FieldUpdate(robot *robit) {
	if (!initialized) {
		initializeField();
		robit->position.X = 100 ;
		robit->position.Y = 35;
		robit->mRot = 45;
	}
	for (int i = 0; i < c.size(); i++) {
		//optimizing tip( find a way to rid the sqrt function)
		float dConeToRobot = sqrt(sqr(c[i].pos.X - robit->position.X) + sqr(c[i].pos.Y - robit->position.Y));
		if (dConeToRobot < 3*robit->size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
			//WORKS
			inPath(robit, &c[i]);
			//distance to each vertice
			distance2Vertices(robit, &c[i]);//calculate all the distances

			bool robotMovingFwds = (robit->velocity.X >= 0 && robit->velocity.Y >= 0);

			float d2RobotEdge = calcD2Edge(SortSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]), Sort2ndSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]), robit);//calculates the distance to the edge of the robit
			
			if (c[i].directlyInVerticalPath && robotMovingFwds) {//either directly in front or behing based off center x and y position
				c[i].closestPoint = vec3(c[i].pos.X + (d2RobotEdge)*cos(robit->mRot * (PI / 180)), c[i].pos.Y - (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
			}
			else if (c[i].directlyInVerticalPath && !robotMovingFwds) {//either directly in front or behing based off center x and y position
				c[i].closestPoint = vec3(c[i].pos.X - (d2RobotEdge)*cos(robit->mRot * (PI / 180)), c[i].pos.Y + (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
			}
			//had to inverse x and y because horiontal lines
			else if (c[i].directlyInHorizontalPath && robit->rotVel > 0) {//rotating to the right
				c[i].closestPoint = vec3(c[i].pos.Y + (d2RobotEdge)*sin(robit->mRot * (PI / 180)), c[i].pos.X + (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
			}
			else if (c[i].directlyInHorizontalPath && robit->rotVel < 0) {//rotating to the right
				c[i].closestPoint = vec3(c[i].pos.Y - (d2RobotEdge)*sin(robit->mRot * (PI / 180)), c[i].pos.X - (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
			}
			else {//not directly in path
				//will return which vertice is the closest to the cone
				int smallest_vertice = sortSmallVER(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]);
				HELLO = smallest_vertice;
				c[i].closestPoint = robit->vertices[smallest_vertice];//closest point to center will then be the vertice
			}
			float d2closestPoint = sqrt(sqr(c[i].pos.X - c[i].closestPoint.X) + sqr(c[i].pos.Y - c[i].closestPoint.Y));
			vec3 R = (c[i].closestPoint + c[i].pos.times(-1)).times(coneRad / d2closestPoint) + c[i].pos;
			if (d2closestPoint < coneRad && d2closestPoint != 0) {
				c[i].pos.X -= R.X - c[i].closestPoint.X;
				c[i].pos.Y -= R.Y - c[i].closestPoint.Y;

//				c[i].pos = c[i].pos + vec3(R.X - c[i].closestPoint.X, R.Y - c[i].closestPoint.Y).times(-1);
			}
			//vec3 normal = c[i].pos + closestPoint.times(-1);//creates the vector between the cone's center and the closest point (amount of penetration)
			
			//if (!robotMovingFwds) normal = normal.times(-1);//pushed in the opposite direction when moving backwards (makes logical sense, yes?)
			
			/*if (d2RobotEdge <= coneRad) { //touching :D
				//c[i].velocity = normal;
				//c[i].touchingRobot = true;//used for adding to the list of which cones are slowing down the robot
				//c[i].heading = robit->mRot; 
				c[i].pos = c[i].pos + vec3(coneRad - closestPoint.X, coneRad - closestPoint.Y);
			}*/
			
		}
		else {
			c[i].directlyInVerticalPath = false;
			c[i].directlyInHorizontalPath = false;
		}
		//things to do:
		//fix basic velocity physics (DONE)
		//fix basic robot->cone collisions(95% DONE) add penetration and normal vector maths:
		//https://cdn.tutsplus.com/gamedev/authors/randy-gaul/custom-physics-aabb-to-circle.png
		//make some sort of increase in friction when cones touching robot (OR OTHER CONES) ew
		/*if (c[i].touchingRobot) {
			if (!(std::find(s.begin(), s.end(), i) != s.end()))
			{
				s.push_back(i);
			}
		}
		else {
				s.remove(i);
		}
		*/
		//have to make cones interact with other cones(NOT DONE)
		//		it makes sure that you only resolve a collision if the objects are moving towards each other.

	}
}

//constructor
 field::field() : initialized(false) {
	 c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
}
 