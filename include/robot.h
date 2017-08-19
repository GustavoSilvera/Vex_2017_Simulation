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
	void rotate(float power);
	float truSpeed(int degree, float value);
	void calculatePos();
	float PID_controller();
	void update();
	void moveAround(float jAnalogX, float jAnalogY);
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
	//lines
		//vertical
		float slopeV[2];
		float YintV[2];
		//horizontal
		float slopeH[2];
		float YintH[2];

	//other stuff
	ci::gl::Texture TankBase;
	float encoder1 = 0;//simulating vex encoder
	float encoderLast = 0;//last encoder value after a rotation
	bool reversed = false;
	volatile bool fieldSpeed = false;
	float amountOfFriction = 3;	//constant changing the amount of friction for the robot
	vec3 friction;
	int size = 18;
	float mRot = 0;//drawing rotation
	//claw
	float clawSize = 5;
	float clawPos = clawSize;
	int clawHeight = 1.5;
	bool grabbing = false;
	int holding = -1;
	//PHYSICS
	vec3 position;//X, Y, and Z
	vec3 velocity;//diff x, diff y, diff z
	float rotVel = 0;
	//vec3 speed;//abs value (positive) of the robit
	vec3 acceleration = vec3(0, 0, 0);
	float rotAcceleration = 0;
	//OTHER STUFF
	volatile struct maintainPosition PID;
	volatile bool ArrowKeyUp = false;//if the arrow keys are triggering a forwards/backwards motion
	volatile bool ArrowKeyDown = false;//if the arrow keys are triggering a forwards/backwards motion
	volatile bool RotRight = false;//used for whether the keys are supposed to be triggering a rotation
	volatile bool RotLeft = false;//used for whether the keys are supposed to be triggering a rotation
	struct position current;
	vec3 vertices[4];
};
#endif  // ROBOT_H
