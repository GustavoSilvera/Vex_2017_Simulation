#if !defined(TRUSPEED_H)
#define TRUSPEED_H
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "joystick.h"
#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <vector>
#include <string>



//declares the class for the robot and all the data that goes with it.
//LOOKIE HERE: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
class tSpeed {
public:
	tSpeed(robot *r, joystick *j);
	static const int maxDots = 200;//maximum amount of graph particles for the truspeed sim

	void textOutput(robot *r, joystick *j);
	void TruSpeedUpdate(robot *r);
	void activate(robot *r, joystick *j, double mX, double mY);
	void graphPlot();
	struct graph {
		//graph for speed of the speed of the wheels based off position
		//for graph moving
		double RYpos[maxDots];
		double BYpos[maxDots];
		//issue: yaxis length not scaling well with particles
		int YAxLength, XAxLength;
		int drawX, drawY;
		int midpoint;//midpoint of the graph, if the graph starts 20 points down.tY;

	};
	graph gr;
	
};
#endif