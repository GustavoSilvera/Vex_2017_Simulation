#if !defined(NAV_H)
#define NAV_H
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "joystick.h"
#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"


class nav {
public:
	nav(robot *r);
	bool isInit;	
	void initialize(robot *r);
	void textOutput(robot *r);
	void navUpdate(robot *r);
	void calculatePos(robot *r);


};
#endif