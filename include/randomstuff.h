#if !defined(RANDOMSTUFF_H)
#define RANDOMSTUFF_H

#include "cinder/app/AppNative.h"

#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"
#include "cinder/Vector.h"
#include "cinder/Text.h"
#include "cinder/Font.h"
#include <string>
#include <sstream>
#include "cinder/gl/TextureFont.h"



//random #defines and other things that are used throughout all the files
#define ppi 7.184
#define PI 3.142
#define WindowWidth  1600
#define WindowHeight  1200
// #define numCones 54
#define cRad 3
#define MGRad 5
#define cHeight 6.82
#define mgHeight 9.6
#define statHeight 24.6
#define coneWeight 0.4
#define moGoWeight 0.7
#define coefMag 1.0//e.e
#define RESET 0
#define MAXSPEED 127
//TYPES
#define CONE 0
#define MOGO 1
#define STAGO 2
#define CONENUM 15
#define gAngle -r->p.mRot * (PI / 180)//general angle (used for everything)
#define renderRad 1.5//amount of the robot's radii that are used to calculate cone distance, smaller is more optimized (but calculates for less cones)
inline float getSign(float value) {//returns whether a number is negative or positive.
	if (value < 0.0) { return -1; }
	else if (value > 0.0) { return 1; }
	else { return 0; }
}
inline float sqr(float value) {
	return value*value;
}
inline float goTo(float current, float req, float it) {
	int dir = 1;
	if (req < current) dir = -1;
	if (abs(current - req) > 2 * it) return current + dir*it;
	else return req;
}
inline float limitTo(float limit, float value) {
	if (abs(value) < abs(limit)) return value;
	else return getSign(value) * abs(limit);
}
inline float limitFrom(float limit, float value) {
	if (abs(value) > abs(limit)) return value;
	else return getSign(value) * abs(limit);
}
inline void drawText(float text, vec3I pos, vec3I colour, int size) {//simplified way of printing variables as text to the display
	std::stringstream dummyText;
	std::string PRINT;
	dummyText << text;
	dummyText >> PRINT;
	ci::gl::drawString(PRINT, ci::Vec2f(pos.X, pos.Y), ci::Color(colour.X, colour.Y, colour.Z), ci::Font("Arial", size));
}
inline float limitSpeed(float noMoreThan, float value) {
	if (abs(value) <= noMoreThan)
		return value;
	else return getSign(value) * noMoreThan;
}
inline float limitSmall(float noLessThan, float value) {//not really working anyways. idk
	if (abs(value) >= noLessThan)
		return value;
	else return getSign(value)* noLessThan;
}

inline float largest(float v1, float v2) {
	if (v1 > v2)return v1;
	else return v2;
}

#endif  // RANDOMSTUFF_H