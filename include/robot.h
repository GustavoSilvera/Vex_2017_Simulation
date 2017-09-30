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
	void reset();
	void update();
	void moveAround(float jAnalogX, float jAnalogY);
	void setVertices();
	struct position {
		double Xpos, Ypos, deg;
	};
	struct debug {;
	//lines
		//vertical
		float slopeV;//only need 1 slope, both sides of a square are parallel
		float YintV[2];
		//horizontal
		float slopeH;//only need 1 slope, both sides of a square are parallel
		float YintH[2];
		vec3 vertices[4];
	};
	debug db;
	//claw
	struct intake {
		float clawSize;
		float baseSize;//base of the claw
		float clawPos;
		float clawThick;
		float clawHeight;
		float clawSpeed;
		bool grabbing = false;
		int holding = -1;//index for which element is being grabbed, to draw it above all the rest. 
		void claw(float Robsize, int type);
		bool liftUp, liftDown;
		float liftSpeed;
		float liftPos;
		double maxHeight = 50;//50 inches max height
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
		int frictionC = 0;
		int frictionM = 0;
		float amountOfFriction = 3;	//constant changing the amount of friction for the robot
	};
		physics p;
	struct details {
		bool reversed = false;
		float basePower;
		float encoderBase;//simulating encoder sensor
		int gyroBase;//simulating gyro sensor
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
	ci::gl::Texture CChanel;
	struct position current;
	float encoderLast = 0;//last encoder value after a rotation
	volatile bool fieldSpeed = false;
};
#endif  // ROBOT_H
