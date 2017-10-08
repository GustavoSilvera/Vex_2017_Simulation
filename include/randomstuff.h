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
#define coneWeight 0.3
#define moGoWeight 0.8
#define coefMag 1.0//e.e
#define RESET 0
//TYPES
#define CONE 0
#define MOGO 1
#define STAT 2

#define gAngle -robit->p.mRot * (PI / 180)
#define renderRad 1//amount of the robot's radii that are used to calculate cone distance, smaller is more optimized (but calculates for less cones)
inline float getSign(float value) {//returns whether a number is negative or positive.
	if (value < 0.0) { return -1; }
	else if (value > 0.0) { return 1; }
	else { return 0; }
}
inline float sqr(float value) {
	return value*value;
}
inline float limitTo(float limit, float value) {
	if (abs(value) < abs(limit)) return value;
	else return getSign(value) * abs(limit);
}
inline void drawText(float text, vec3I pos, vec3I colour, int size) {//simplified way of printing variables as text to the display
	std::stringstream dummyText;
	std::string PRINT;
	dummyText << text;
	dummyText >> PRINT;
	ci::gl::drawString(PRINT, ci::Vec2f(pos.X, pos.Y), ci::Color(colour.X, colour.Y, colour.Z), ci::Font("Arial", size));
}
inline float limitSmall(float noLessThan, float value) {//not really working anyways. idk
	if (abs(value) >= noLessThan)
		return value;
	else return getSign(value)* noLessThan;
}
inline float SortSmallest3(float v1, float v2, float v3) {
	float smallest = v1;//initially assumes v1 is the smallest
	if (v2 < smallest) smallest = v2;//resets to v2 if its smaller than v1
	if (v3 < smallest) smallest = v3;//resets to v3 if its smaller than v1
	return smallest;
}
inline float SortSmallest(float v1, float v2, float v3, float v4) {//finds smallest of the list provided
	float smallest = v1;//initially assumes v1 is the smallest
	if (v2 < smallest) smallest = v2;//resets to v2 if its smaller than v1
	if (v3 < smallest) smallest = v3;//resets to v3 if its smaller than v1
	if (v4 < smallest) smallest = v4;//resets to v4 if its smaller than v1
	return smallest;
	
}
inline float Sort2ndSmallest(float v1, float v2, float v3, float v4) {//finds second smallest of the list provided
	float smallest = SortSmallest(v1, v2, v3, v4);
	float Smallest2nd;
	if (v1 == smallest)	Smallest2nd = SortSmallest3(v2, v3, v4);//exclude v1 from search to the smallest
	else if (v2 == smallest) Smallest2nd = SortSmallest3(v1, v3, v4);//exclude v2 from search to the smallest
	else if (v3 == smallest) Smallest2nd = SortSmallest3(v2, v1, v4);//exclude v3 from search to the smallest
	else if (v4 == smallest) Smallest2nd = SortSmallest3(v2, v3, v1);//exclude v4 from search to the smallest
	else return 0;
	return Smallest2nd;
}
inline int sortSmallVER(float v1, float v2, float v3, float v4) {
	float smallestD2V = SortSmallest(v1, v2, v3, v4);
	if (smallestD2V == v1) return 0;
	else if (smallestD2V == v2) return 1;
	else if (smallestD2V == v3) return 2;
	else if (smallestD2V == v4) return 3;
}
inline float largest(float v1, float v2) {
	if (v1 > v2)return v1;
	else return v2;
}

#endif  // RANDOMSTUFF_H