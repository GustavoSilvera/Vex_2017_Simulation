#if !defined(VEC3_H)
#define VEC3_H

#include <cmath>
//definition for a 3d vector used throughout the entire project file
class vec3 {
public:
	vec3(double x = 0.0, double y = 0.0, double z = 0.0) : X(x), Y(y), Z(z) {}
	double X, Y, Z;

	vec3 times(float f) {
		return vec3(X*f, Y*f, Z*f);
	}
	float distance(vec3 v) {
		return sqrt(sqr(v.X - X) + sqr(v.Y - Y) + sqr(v.Z - Z));
	}
	vec3 operator+(vec3 v) {
		return vec3(X + v.X, Y + v.Y, Z + v.Z);
	}
private:
	static float sqr(float x) { return x*x; }
};
class vec3I {//ints
public:
	vec3I(int x = 0, int y = 0, int z = 0) : X(x), Y(y), Z(z) {}
	int  X, Y, Z;

	vec3I times(float f) {
		return vec3I(X*f, Y*f, Z*f);
	}
	vec3I operator+(vec3I v) {
		return vec3I(X + v.X, Y + v.Y, Z + v.Z);
	}
private:
	static float sqr(float x) { return x*x; }
};

#endif