#if !defined(PID_H)
#define PID_H
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "joystick.h"
#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <vector>
#include <string>

class PID {
public:
	PID(robot *r);
	bool isInit;
	static const int maxDots = 150;//maximum amount of graph particles for the truspeed sim
	struct maintainPosition {
		bool isRunning;
		int threshold;
		float error, integral, derivative, lastError;
		float currentPos;
		float requestedValue;
		//, kI, kD;
	};
	struct graph {
		int posY[maxDots];
		int posY2[maxDots];
		int posY3[maxDots];
		int YAxLength, XAxLength;
		int drawX, drawY;
		int midpoint;//midpoint of the graph, if the graph starts 20 points down.tY;
	};
	graph gr;
	void initialize(robot *r);
	volatile maintainPosition pid;
	void textOutput(robot *r);
	void graphPlot();
	void reset(robot *r);
	float controller(robot *r);//actual PID function
	void PIDUpdate(robot *r);
	bool pidVel;
};
#endif