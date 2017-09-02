#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "vec3.h"
#include "randomstuff.h"
//declares the joystick class and all the data that goes with it. 
class joystick {
public:
	void getAnalog(vec3 mousePos);
	bool withinAnalogRange(vec3 mousePos);
	int analogY;
	int analogX;
	int drawX = 600;//distance from where to draw the top left vertice of the drawing in PIXELS
	int drawY = 500;
	int drawSize = 127;
	//makes image with top left cornet at (700, 300), with a size of 127
};

#endif // JOYSTICK_H