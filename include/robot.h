#if !defined(ROBOT_H)
#define ROBOT_H

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
	void deceleration(int timePassed);
	void setVertices();
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
	float mRot = 0;
	float ActualHeading = mRot;//weird drawing issues where 0° is where 90° should be irl
	vec3 position;//X, Y, and Z
	vec3 speed;//diff x, diff y, diff z
	vec3 acceleration = vec3(0, 0, 0);
	volatile struct maintainPosition PID;
	volatile bool ArrowKeyUp = false;//if the arrow keys are triggering a forwards/backwards motion
	volatile bool ArrowKeyDown = false;//if the arrow keys are triggering a forwards/backwards motion
	volatile bool RotRight = false;//used for whether the keys are supposed to be triggering a rotation
	volatile bool RotLeft = false;//used for whether the keys are supposed to be triggering a rotation

	struct position current;
	vec3 vertices[4];
};
#endif  // ROBOT_H
