#if !defined(ROBOT_H)
#define ROBOT_H

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "vec3.h"

#include <iostream>
//declares the class for the robot and all the data that goes with it. 
class robot {
public:
	robot();

	void forwards(float power);
	void rotate(float power);
	float truSpeed(int degree, float value);
	void reset();
	void update();
	void moveAround(float jAnalogX, float jAnalogY);
	void setVertices();
	void readScript();
	bool driveFor(float inches);
	bool rotFor(float degrees);
	bool readyToRun = false;
	bool thinking = false;
	enum action  {
		ACTION_ROTATE,
		ACTION_FWDS
	};
	struct command {
		enum action a;
		double amnt;
	};
	std::vector<command> commands;

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
	//output text
		float distance;//simulating a real worl vex encoder shaft
		float rotDist;//simulating a real world vex gyro 
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
		void speedMult(float base, float rot);
		float amountOfFriction = 5;	//constant changing the amount of friction for the robot
	};
		physics p;
	struct details {
		float basePower;
		int size = 18;
		bool frontStop = false, backStop = false;
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
};
#endif  // ROBOT_H
