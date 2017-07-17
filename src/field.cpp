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
		1----------2
		|    r     |
		|          |
		4----------3
	**********************/
	for (int v = 0; v < 4; v++) {
		//calculates distance between the cone's centre and each vertice
		c->d2V[v] = sqrt(sqr(c->pos.X - robit->vertices[v].X) + sqr(c->pos.Y - robit->vertices[v].Y));
	}
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
		if (dConeToRobot < robit->size) {//within a radius around the robot of 18 inches around the main body
			distance2Vertices(robit, &c[i]);//calculate all the distances
			//shortest distance: (a)
				float a = SortSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]);//sorts the smallest of the distances
			//2nd shortest distance: (b)
				float b = Sort2ndSmallest(c[i].d2V[0], c[i].d2V[1], c[i].d2V[2], c[i].d2V[3]);
				//EXPLANATION HERE:
				float C1 = ( ( ( ( sqr ( a ) - sqr ( b ) ) / robit->size ) + robit->size ) / 2 );
				float d2RobotEdge = sqrt( sqr ( a ) - sqr ( C1 ) );
				bool touchingFrontRobot = (a == c[i].d2V[0] || a == c[i].d2V[1]) && (b == c[i].d2V[0] || b == c[i].d2V[1]);//if smallest made triangle is with the 1st and 2nd vertices
				bool touchingFrontVertices = (abs(c[i].d2V[0] - coneRad) < .15 || abs(c[i].d2V[1] - coneRad) < .15);//if shortest distance possible from cone to 1st and 2nd vertices
				bool touchingBackRobot = (a == c[i].d2V[2] || a == c[i].d2V[3]) && (b == c[i].d2V[2] || b == c[i].d2V[3]);//if smallest made triangle is with the 3rd and 4th vertices
				bool touchingBackVertices = (abs( c[i].d2V[2] - coneRad) < .15 || abs(c[i].d2V[3] - coneRad) < .15);//if shortest distance possible from cone to 3rd and 4th vertices
				if (d2RobotEdge <= coneRad) {
					//then touching :D
					bool movingForwards = (robit->acceleration.X > 0 || robit->acceleration.Y > 0);
					bool movingBackwards = ((robit->acceleration.X < 0 || robit->acceleration.Y < 0));
					if (movingForwards && (touchingFrontRobot || touchingFrontVertices) ) {//if going forwards(positive)
						c[i].pos.X += robit->speed.X;
						c[i].pos.Y += robit->speed.Y;
					}
					else if (movingBackwards && (touchingBackRobot || touchingBackVertices)) {//if going backwards(negative)
						c[i].pos.X += robit->speed.X;
						c[i].pos.Y += robit->speed.Y;
					}
					if (d2RobotEdge < coneRad) {//WHAAAA
						//works for pushing stray cones away from the robot if they get too close
						c[i].pos.X += getSign(robit->speed.X) * abs(d2RobotEdge - coneRad);
						c[i].pos.Y += getSign(robit->speed.Y) * abs(d2RobotEdge - coneRad);
						//current issue, if cone is already too inside, it'll push them MORE inside
					}
					
				}
		}
		else {
			for (int v = 0; v < 4; v++) {
				c[i].d2V[v] = 1000;//given a garbage number of being 1000 units away, just to not have to be rendered
			}
		}


		/*
		float dConeToRobot = sqrt(sqr(c[i].pos.X - robit->position.X) + sqr(c[i].pos.Y - robit->position.Y));
		if (dConeToRobot < ( robit->size/2 + coneRad) && movingTowardsCone(robit, &c[i])) {
			//robot is touching a cone
			//AND MOVING TOWARDS IT
			c[i].pos.X += robit->speed.X;
			c[i].pos.Y += robit->speed.Y;

		}*///technically works, but not very well... i guess... kinda 
	}
}
//make functino to compute ONLY the cones closest to the robot(by like 1 metre or something idk)

//constructor
 field::field() : initialized(false) {
	 c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
}
