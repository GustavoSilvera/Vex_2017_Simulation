#if !defined(ROBOT_H)
#define ROBOT_H

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "vec3.h"
//declares the class for the robot and all the data that goes with it. 
class robot {
public:
	robot();

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
	struct debug {;
	//lines
		//vertical
		float slopeV[2];
		float YintV[2];
		//horizontal
		float slopeH[2];
		float YintH[2];
		vec3 vertices[4];
	};
	debug db;
	//claw
	struct intake {
		float clawSize;
		float clawPos;
		float clawHeight;
		bool grabbing = false;
		int holding = -1;
		void claw(float Robsize);
	};
		intake c;
		intake mg;
	struct physics {
		float mRot = 0;//drawing rotation
		vec3 position;//X, Y, and Z
		vec3 velocity;//diff x, diff y, diff z
		vec3 acceleration = vec3(0, 0, 0);
		float rotVel = 0;
		float rotAcceleration = 0;
		vec3 friction;
		float amountOfFriction = 3;	//constant changing the amount of friction for the robot
	};
		physics p;
	struct details {
		bool reversed = false;
		int size = 18;
	};
		details d;
	struct controls {
		volatile bool ArrowKeyUp = false;//if the arrow keys are triggering a forwards/backwards motion
		volatile bool ArrowKeyDown = false;//if the arrow keys are triggering a forwards/backwards motion
		volatile bool RotRight = false;//used for whether the keys are supposed to be triggering a rotation
		volatile bool RotLeft = false;//used for whether the keys are supposed to be triggering a rotation
	};
		controls ctrl;
	//other stuffs
	ci::gl::Texture TankBase;
	volatile maintainPosition PID;
	struct position current;
	float encoder1 = 0;//simulating vex encoder
	float encoderLast = 0;//last encoder value after a rotation
	volatile bool fieldSpeed = false;
};
#endif  // ROBOT_H
