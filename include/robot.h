#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "vec3.h"
//declares the class for the robot and all the data that goes with it. 
class robot {
public:
	robot(vec3 p, vec3 s);

	void forwards(float  power);
	void rotateBase(float rotAmount);
	void stop();
	float truSpeed(int degree, float value);
	void calculatePos();
	float PID_controller();
	void update();
	void moveAround(float jAnalogX, float jAnalogY);
	void NavigationUpdate();
	void PIDControlUpdate();
	void TruSpeedUpdate();

	struct maintainPosition {
		bool isRunning;
		int threshold;
		float error, integral, derivative, lastError;
		float currentPos;
		float requestedValue;
		//, kI, kD;
	};
	struct position {
		double Xpos, Ypos, deg;
	};

	ci::gl::Texture TankBase;
	float encoder1 = 0;//simulating vex encoder
	float encoderLast = 0;//last encoder value after a rotation
	bool rotating = false;
	int size = 18;
	vec3 position;//X, Y, and Z
	float mRot = 0;
	float motorPower = 0;
	vec3 speed;//diff x, diff y, diff z
	volatile struct maintainPosition PID;

	struct position current;
};
