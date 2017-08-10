#include "vec3.h"
#include "field.h"
#include "randomstuff.h"
#include "robot.h"
#include <vector>
#include <algorithm> // remove and remove_if


//declares and defines the field class and functions

field::cone initialConeConfiguration[53] = {//array for each configuration of the cone (in field.h)
	//{ { 2.9, 2.9 }, 0, false},//position X,Y; initial heading; if tipped or not
	{ { 2.9, 13.0 }, 0, false },
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
field::MoGo initialMoGoConfiguration[8] = {//array for each configuration of the mobile goal (in field.h)
										//{ { 2.9, 2.9 }, 0, false},//position X,Y; initial heading; if tipped or not
	{ { 34.9, 13.0 }, true },
	{ { 13.0, 34.9 }, false },
	{ { 70.2, 46.7 }, false },
	{ { 46.7, 70.2 }, true },
	{ { 93.9, 70.2 }, false },
	{ { 70.2, 93.9 }, true },
	{ { 127.6, 105.8 }, true },
	{ { 105.8, 127.6 }, false },
};
void field::initializeField() {
	c.assign(&initialConeConfiguration[0], &initialConeConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	mg.assign(&initialMoGoConfiguration[0], &initialMoGoConfiguration[8]);
	initialized = true;//so that this only gets called ONCE when the field tab is running
}
void field::cone::calcD2Vertices(robot *robit) {
	/******vertices:*******
		0----------1
		|    r     |
		|          |
		3----------2
	**********************/
	for (int v = 0; v < 4; v++) {
		//calculates distance between the cone's centre and each vertice
		d2V[v] = sqrt(sqr(pos.X - robit->vertices[v].X) + sqr(pos.Y - robit->vertices[v].Y));
	}
}
void field::MoGo::calcD2Vertices(robot *robit) {
	/******vertices:*******
	0----------1
	|    r     |
	|          |
	3----------2
	**********************/
	for (int v = 0; v < 4; v++) {
		//calculates distance between the cone's centre and each vertice
		d2V[v] = sqrt(sqr(pos.X - robit->vertices[v].X) + sqr(pos.Y - robit->vertices[v].Y));
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
bool field::cone::directlyInVerticalPath(robot *robit) {//vertical lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeV[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
	robit->YintV[0] = robit->slopeV[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeV[1] = (robit->vertices[1].Y - robit->vertices[2].Y) / (robit->vertices[1].X - robit->vertices[2].X);
	robit->YintV[1] = robit->slopeV[1] * (x - (robit->vertices[1].X - origin.X)) + (robit->vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintV[0]) != getSign(robit->YintV[1]));//works for telling me if between the two lines
}
bool field::cone::directlyInHorizontalPath(robot *robit) {//horizontal lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeH[0] = (robit->vertices[0].Y - robit->vertices[1].Y) / (robit->vertices[0].X - robit->vertices[1].X);
	robit->YintH[0] = robit->slopeH[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeH[1] = (robit->vertices[3].Y - robit->vertices[2].Y) / (robit->vertices[3].X - robit->vertices[2].X);
	robit->YintH[1] = robit->slopeH[1] * (x - (robit->vertices[3].X - origin.X)) + (robit->vertices[3].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintH[0]) != getSign(robit->YintH[1]));//works for telling me if between the two lines
}
bool field::MoGo::directlyInVerticalPath(robot *robit) {//vertical lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeV[0] = (robit->vertices[0].Y - robit->vertices[3].Y) / (robit->vertices[0].X - robit->vertices[3].X);
	robit->YintV[0] = robit->slopeV[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeV[1] = (robit->vertices[1].Y - robit->vertices[2].Y) / (robit->vertices[1].X - robit->vertices[2].X);
	robit->YintV[1] = robit->slopeV[1] * (x - (robit->vertices[1].X - origin.X)) + (robit->vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintV[0]) != getSign(robit->YintV[1]));//works for telling me if between the two lines
}
bool field::MoGo::directlyInHorizontalPath(robot *robit) {//horizontal lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->slopeH[0] = (robit->vertices[0].Y - robit->vertices[1].Y) / (robit->vertices[0].X - robit->vertices[1].X);
	robit->YintH[0] = robit->slopeH[0] * (x - (robit->vertices[0].X - origin.X)) + (robit->vertices[0].Y - origin.Y);
	robit->slopeH[1] = (robit->vertices[3].Y - robit->vertices[2].Y) / (robit->vertices[3].X - robit->vertices[2].X);
	robit->YintH[1] = robit->slopeH[1] * (x - (robit->vertices[3].X - origin.X)) + (robit->vertices[3].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->YintH[0]) != getSign(robit->YintH[1]));//works for telling me if between the two lines
}
void field::FieldUpdate(robot *robit) {
	if (!initialized) {
		initializeField();
		robit->position.X = 100;
		robit->position.Y = 35;
		robit->mRot = 45;
	}
	for (int i = 0; i < c.size(); i++) {
		//optimizing tip( find a way to rid the sqrt function)
		float dConeToRobot = sqrt(sqr(c[i].pos.X - robit->position.X) + sqr(c[i].pos.Y - robit->position.Y));
		if (dConeToRobot < renderRad * robit->size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
			//WORKS
			//inPath(robit, &c[i]);
			//distance to each vertice
			c[i].calcD2Vertices(robit);//calculate all the distances
			bool robotMovingFwds = (robit->velocity.X >= 0 && robit->velocity.Y >= 0);
			float d2RobotEdge = calcD2Edge(SortSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]), Sort2ndSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]), robit);//calculates the distance to the edge of the robit
			if (c[i].directlyInVerticalPath(robit)) {//either directly in front or behing based off center x and y position
				bool inFront = (c[i].d2V[0] + c[i].d2V[1] < c[i].d2V[3] + c[i].d2V[3]);//checking if cone is closer to the front side
				if (inFront)
					c[i].closestPoint = vec3(c[i].pos.X + (d2RobotEdge)*cos(robit->mRot * (PI / 180)), c[i].pos.Y - (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
				else
					c[i].closestPoint = vec3(c[i].pos.X - (d2RobotEdge)*cos(robit->mRot * (PI / 180)), c[i].pos.Y + (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
			}
			//had to inverse x and y because horiontal lines
			else if (c[i].directlyInHorizontalPath(robit)) {//rotating to the right
				bool onRight = (c[i].d2V[1] + c[i].d2V[2] < c[i].d2V[0] + c[i].d2V[3]);//checking if cone is closer to the right side
				if (onRight)
					c[i].closestPoint = vec3(c[i].pos.X + (d2RobotEdge)*sin(robit->mRot * (PI / 180)), c[i].pos.Y + (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
				else
					c[i].closestPoint = vec3(c[i].pos.X - (d2RobotEdge)*sin(robit->mRot * (PI / 180)), c[i].pos.Y - (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
			}
			else {//not directly in path finds which vertice is the closest to the cone
				int smallest_vertice = sortSmallVER(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]);
				HELLO = smallest_vertice;//used for drawing the closest point (debugging)
				c[i].closestPoint = robit->vertices[smallest_vertice];//closest point to center will then be the vertice
			}
			float d2closestPoint = sqrt(sqr(c[i].pos.X - c[i].closestPoint.X) + sqr(c[i].pos.Y - c[i].closestPoint.Y));
			vec3 R = (c[i].closestPoint + c[i].pos.times(-1)).times(coneRad / d2closestPoint) + c[i].pos;
			if (d2closestPoint < coneRad && d2closestPoint != 0) {
				c[i].pos.X -= R.X - c[i].closestPoint.X;
				c[i].pos.Y -= R.Y - c[i].closestPoint.Y;
			}
		}
	
		//collisions from cone->cone
		for (int k = 0; k < c.size(); k++) {//for each cone (not including itself)
			if (k != i) {
				float dCone2Cone = sqrt(sqr(c[i].pos.X - c[k].pos.X) + sqr(c[i].pos.Y - c[k].pos.Y));
				if (dCone2Cone < 2 * coneRad) {//touching
					vec3 distance = vec3(c[k].pos.X - c[i].pos.X, c[k].pos.Y - c[i].pos.Y);
					c[k].pos = c[k].pos + distance.times(((2 * coneRad) - dCone2Cone) / dCone2Cone);//push of the cones
					/*maths
					distance*? = overlap;
						overlap = 2*coneRad - distance
					therefore: ? = (2*coneRad - distance)/distance;
					*/
					s.push_back(k);
				}
				else {
					s.erase(std::remove(s.begin(), s.end(), k), s.end());
				}
				int size = s.size();
			}
		}
		//dont need collisions from cone to mogo b/c i have mogo to cone, which works just fine
		//things to do:
		//fix basic velocity physics (DONE)
		//fix tracking of closest point (for normal v calculation) (DONE)
		//make cone to cone physics (DONE)
		//make some sort of increase in friction when cones touching robot (OR OTHER CONES) ew (5%)
		//make physics for mobile goals(DONE)
		//make collisions for mogo->mogo, and mogo->cone(DONE)
	}
	for (int i = 0; i < mg.size(); i++) {//mobile goals
		float dMoGoToRobot = sqrt(sqr(mg[i].pos.X - robit->position.X) + sqr(mg[i].pos.Y - robit->position.Y));
		if (dMoGoToRobot < renderRad * robit->size) {//within a radius around the robot of 18 inches around the center point of the bodyvec3 origin = c[i].pos;//calculattes yintercepts for each cone relative to their position
			//distance to each vertice
			mg[i].calcD2Vertices(robit);//calculate all the distances
			bool robotMovingFwds = (robit->velocity.X >= 0 && robit->velocity.Y >= 0);//if the robot is moving forwards or stopped
			float d2RobotEdge = calcD2Edge(SortSmallest(mg[i].d2V[0], mg[i].d2V[1], mg[i].d2V[2], mg[i].d2V[3]), Sort2ndSmallest(mg[i].d2V[0], mg[i].d2V[1], mg[i].d2V[2], mg[i].d2V[3]), robit);//calculates the distance to the edge of the robit
			if (mg[i].directlyInVerticalPath(robit)) {//either directly in front or behing based off center x and y position
				bool inFront = (mg[i].d2V[0] + mg[i].d2V[1] < mg[i].d2V[3] + mg[i].d2V[3]);//checking if cone is closer to the front side
				if (inFront)
					mg[i].closestPoint = vec3(mg[i].pos.X + (d2RobotEdge)*cos(robit->mRot * (PI / 180)), mg[i].pos.Y - (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
				else
					mg[i].closestPoint = vec3(mg[i].pos.X - (d2RobotEdge)*cos(robit->mRot * (PI / 180)), mg[i].pos.Y + (d2RobotEdge)*sin(robit->mRot * (PI / 180)));//does work
			}
			//had to inverse x and y because horiontal lines
			else if (mg[i].directlyInHorizontalPath(robit)) {//rotating to the right
				bool onRight = (mg[i].d2V[1] + mg[i].d2V[2] < mg[i].d2V[0] + mg[i].d2V[3]);//checking if cone is closer to the right side
				if (onRight)
					mg[i].closestPoint = vec3(mg[i].pos.X + (d2RobotEdge)*sin(robit->mRot * (PI / 180)), mg[i].pos.Y + (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
				else
					mg[i].closestPoint = vec3(mg[i].pos.X - (d2RobotEdge)*sin(robit->mRot * (PI / 180)), mg[i].pos.Y - (d2RobotEdge)*cos(robit->mRot * (PI / 180)));//does work
			}
			else {//not directly in path finds which vertice is the closest to the cone
				int smallest_vertice = sortSmallVER(mg[i].d2V[0], mg[i].d2V[1], mg[i].d2V[2], mg[i].d2V[3]);
				mg[i].closestPoint = robit->vertices[smallest_vertice];//closest point to center will then be the vertice
			}
			float d2closestPoint = sqrt(sqr(mg[i].pos.X - mg[i].closestPoint.X) + sqr(mg[i].pos.Y - mg[i].closestPoint.Y));
			vec3 R = (mg[i].closestPoint + mg[i].pos.times(-1)).times(MoGoRad / d2closestPoint) + mg[i].pos;
			if (d2closestPoint < MoGoRad && d2closestPoint != 0) {
				mg[i].pos.X -= R.X - mg[i].closestPoint.X;
				mg[i].pos.Y -= R.Y - mg[i].closestPoint.Y;
			}
		}
		//collisions from MoGo->MoGo
		for (int m = 0; m < mg.size(); m++) {//for each cone (not including itself)
			if (m != i) {
				float dMoGo2MoGo = sqrt(sqr(mg[i].pos.X - mg[m].pos.X) + sqr(mg[i].pos.Y - mg[m].pos.Y));
				if (dMoGo2MoGo < 2 * MoGoRad) {//touching
					vec3 distance = vec3(mg[m].pos.X - mg[i].pos.X, mg[m].pos.Y - mg[i].pos.Y);
					mg[m].pos = mg[m].pos + distance.times(((2 * MoGoRad) - dMoGo2MoGo) / dMoGo2MoGo);//push of the cones
				}

			}
		}
		//collisions from MoGo->Cone
		for (int k = 0; k < c.size(); k++) {//for each cone (not including itself)
			float dMoGo2Cone = sqrt(sqr(mg[i].pos.X - c[k].pos.X) + sqr(mg[i].pos.Y - c[k].pos.Y));
			if (dMoGo2Cone < MoGoRad + coneRad) {//touching
				vec3 distance = vec3(c[k].pos.X - mg[i].pos.X, c[k].pos.Y - mg[i].pos.Y);
				c[k].pos = c[k].pos + distance.times((( coneRad + MoGoRad) - dMoGo2Cone) / dMoGo2Cone);//push of the cones
			}

		}
	}
}

//constructor
 field::field() : initialized(false) {
	 c.assign(&initialConeConfiguration[0], &initialConeConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
	 mg.assign(&initialMoGoConfiguration[0], &initialMoGoConfiguration[8]);

 }
 