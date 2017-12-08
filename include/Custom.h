#if !defined(CUSTOM_H)
#define CUSTOM_H
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "joystick.h"
#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <vector>
#include <string>

class customize {
public:
	customize(robot *r);
	bool isInit;	
	void initialize(robot *r);
	void controlPanel(robot *r, float scalar);
	void Text();
	int size = 18;
	float scale;
};
#endif