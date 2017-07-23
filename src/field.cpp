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

void field::FieldUpdate(robot *robit) {
	if (!initialized) {
		initializeField();
		robit->position.X = 100 ;
		robit->position.Y = 35;
		robit->mRot = 45;
		initialized = true;//so that this only gets called ONCE when the field tab is running
	}
	for (int i = 0; i < c.size(); i++) {
		//optimizing tip( find a way to rid the sqrt function)
		float dConeToRobot = sqrt(sqr(c[i].pos.X - robit->position.X) + sqr(c[i].pos.Y - robit->position.Y));
		if (dConeToRobot < 3*robit->size) {//within a radius around the robot of 18 inches around the center point of the body
			
			vec3 origin = c[i].pos;
			float x = 0;//finds the y intercept
			robit->slope[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
			robit->Yint[0] = robit->slope[0] * (x - robit->vertices[0].X) + robit->vertices[0].Y;
			robit->slope[1] = (robit->vertices[1].Y - robit->vertices[2].Y) / (robit->vertices[1].X - robit->vertices[2].X);
			robit->Yint[1] = robit->slope[1] * (x - robit->vertices[1].X) + robit->vertices[1].Y;
			if (getSign(robit->Yint[0]) != getSign(robit->Yint[1])) c[i].directlyInPath = true;//works for telling me if between the two lines
			else c[i].directlyInPath = false;

			distance2Vertices(robit, &c[i]);//calculate all the distances

			bool robotMovingFwds;

			if (robit->velocity.X >= 0 && robit->velocity.Y >= 0) {//(going forwards//only care about the front vertices 
				c[i].SmallestD2V[0] = c[i].d2V[0];
				c[i].SmallestD2V[1] = c[i].d2V[1];
				frontOrBack = 0;//what is made sure to be the minimum vertice 
				robotMovingFwds = false;
			}
			else {//only care about the back two vertices
				c[i].SmallestD2V[0] = c[i].d2V[2];
				c[i].SmallestD2V[1] = c[i].d2V[3];
				frontOrBack = 2;//what is made sure to be the minimum vertice
				robotMovingFwds = true;
			}

			float d2RobotEdge = calcD2Edge(c[i].SmallestD2V[0], c[i].SmallestD2V[1], robit);//calculates the distance to the edge of the robit

			if (c[i].directlyInPath && robotMovingFwds) {//either directly in front or behing based off center x and y position
				c[i].closestPoint = vec3(c[i].pos.X - (d2RobotEdge)*cos(robit->ActualHeading * (PI / 180)), c[i].pos.Y + (d2RobotEdge)*sin(robit->ActualHeading * (PI / 180)));
				if (d2RobotEdge <= coneRad) {
					c[i].pos = c[i].pos + vec3(coneRad - c[i].closestPoint.X, coneRad - c[i].closestPoint.Y).times(-1);
				}
			}
			else if (c[i].directlyInPath && !robotMovingFwds) {//either directly in front or behing based off center x and y position
				c[i].closestPoint = vec3(c[i].pos.X + (d2RobotEdge)*cos(robit->ActualHeading * (PI / 180)), c[i].pos.Y - (d2RobotEdge)*sin(robit->ActualHeading * (PI / 180)));
				if (d2RobotEdge <= coneRad) {
					c[i].pos = c[i].pos + vec3(coneRad - c[i].closestPoint.X, coneRad - c[i].closestPoint.Y).times(-1);
				}
			}
			else {//not directly in path
				//will return which vertice is the closest to the cone
				HELLO = sortSmallVER(c[i].SmallestD2V[0], c[i].SmallestD2V[1]) + frontOrBack;
				c[i].closestPoint = robit->vertices[sortSmallVER(c[i].SmallestD2V[0], c[i].SmallestD2V[1]) + frontOrBack];//closest point to center will then be the vertice
				float d2RobotVertice = sqrt(sqr(c[i].pos.X - c[i].closestPoint.X) + sqr(c[i].pos.Y - c[i].closestPoint.Y));
				if (d2RobotVertice <= coneRad) {
					c[i].pos = c[i].pos + vec3(coneRad - ()).times(-1);
				}
			}

			//vec3 normal = c[i].pos + closestPoint.times(-1);//creates the vector between the cone's center and the closest point (amount of penetration)
			
			//if (!robotMovingFwds) normal = normal.times(-1);//pushed in the opposite direction when moving backwards (makes logical sense, yes?)
			
			/*if (d2RobotEdge <= coneRad) { //touching :D
				//c[i].velocity = normal;
				//c[i].touchingRobot = true;//used for adding to the list of which cones are slowing down the robot
				//c[i].heading = robit->ActualHeading; 
				c[i].pos = c[i].pos + vec3(coneRad - closestPoint.X, coneRad - closestPoint.Y);
			}*/
			
		}
		else {
			c[i].directlyInPath = false;
		}
		/*for (int k = 0; k < c.size()-1; k++) {
			c[i].distanceToCone[k] = sqrt(sqr(c[i].pos.X - c[k].pos.X) + sqr(c[i].pos.Y - c[k].pos.Y));
			vec3 vAB = c[k].velocity + c[i].velocity.times(-1);//vAB = vA - vB
			if (c[i].distanceToCone[k] <= 2*coneRad && k != i) {
				
			}
		}*/
		//things to do:
		//fix basic velocity physics (DONE)
		//fix basic robot->cone collisions(75% DONE) add penetration and normal vector maths:
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
 