#include "vec3.h"
#include "field.h"
#include "randomstuff.h"
#include "robot.cpp"
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
void field::FieldUpdate(robot *robit) {
	if (!initialized) {
		initializeField();
		robit->position.X = 100;
		robit->position.Y = 35;
		robit->mRot = 45;
	}
}


//constructor
 field::field() : initialized(false) {
	 c.assign(&initialConfiguration[0], &initialConfiguration[54]);//assigns valeus to the vector of cones, from first parameter (0) to last one (53)
}
