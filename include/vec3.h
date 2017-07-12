#if !defined(VEC3_H)
#define VEC3_H

#include <cmath>
//definition for a 3d vector used throughout the entire project file
class vec3 {
public:
	vec3(double x =0.0, double y =0.0, double z = 0.0) : X(x), Y(y), Z(z) {}
	double X, Y, Z;

	vec3 times(double f) {
		return vec3(X*f, Y*f, Z*f);
	}
	double distance(vec3 v) {
		return sqrt(sqr(v.X - X) + sqr(v.Y - Y) + sqr(v.Z - Z));
	}
	vec3 operator+(vec3 v) {
		return vec3(X + v.X, Y + v.Y, Z + v.Z);
	}
private:
	static double sqr(double x) { return x*x; }
};

inline vec3 colorRGB(vec3 RGB) {//provides a "Cinder Readable" color spectrum for drawing
						 //because cinder color(), only reads numbers from 0 to 1, in the (R,G,B) values
	return(vec3((RGB.X / 255), (RGB.Y / 255), (RGB.Z / 255)));//returns rgb values within 0-1
}

#endif