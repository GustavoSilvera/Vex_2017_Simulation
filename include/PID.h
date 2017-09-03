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
	struct maintainPosition {
		bool isRunning;
		int threshold;
		float error, integral, derivative, lastError;
		float currentPos;
		float requestedValue;
		//, kI, kD;
	};
	void initialize(robot *r);
	volatile maintainPosition pid;
	void textOutput(robot *r);
	float controller(robot *r);//actual PID function
	void PIDUpdate(robot *r);

};
#endif