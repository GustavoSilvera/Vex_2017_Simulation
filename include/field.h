#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <stdlib.h>
#include <vector>
#include <set>
#include <fstream>
#include <string>
#include <iostream>
//declares the class for the robot and all the data that goes with it.
//LOOKIE HERE: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
class field {
public:
	field(robot *robit);
	std::ofstream textFile;

	typedef int mobileGoalIndex;
	typedef int coneIndex;

	std::set<mobileGoalIndex> pushMoGo;
	std::set<coneIndex> pushCones;

	void FieldUpdate(robot *r);
	void initialize(robot *r);
	bool isInit; // suggestion: construct instead of initialize

	struct fence {
		/*const*/ float fieldSizeIn = 141.05;// 140.5 + 2 * (1.27);wall thickness accounted for
	//	vec3 centre = vec3(606, 606);//in pixels
	//	float fieldEnd = centre.X + fieldSizeIn*ppi / 2;//furthest to the right the field is touching
		float depthIn = 1.27;//thickness of the vex fence
		void wallPush(robot *r);
		// float d2E[2];
		/*for stationary goals*/
		// float d2StatGoal[2];
		struct zone {
			std::set<mobileGoalIndex> tenPoint;
			std::set<mobileGoalIndex> fivePoint;
			std::set<mobileGoalIndex> twentyPoint;
		};
		zone z[2]; // red and blue
		float poleEquation(float xPoint, float yPoint, float slope, float value);//red and bleu
		void robotPole(robot *r);//collision between robot and zone poles
	};
	fence f;

	struct element {
		vec3 pos;
		int colour;//0 is yellow, 1 is red, 2 is blue 
		float radius;//size of the object;
		/*const */ float height;
		void calcD2Vertices(robot *r);
		// float d2V[4];//distance to each vertice on the robot
		// float d2Robot, d2RobotEdge;
		bool directlyInVerticalPath(robot *r);
		bool directlyInHorizontalPath(robot *r);
		//vec3 closestPoint;
		void fencePush(fence *f);
		//float d2E[2];//0 is d2 top, 1 is d2left, 
		void robotColl(int index, robot *r, std::set<int> &pushCone, std::set<int> &pushMoGo, int type, fence *f);
		void collision(element *e);
		void grabbed(robot *r, int index, int type);
		//bool held;
		void zoneScore(fence *f, int index);
		int fellOn;
		std::set<coneIndex> stacked; // for goals only, cones stacked on it.
		 bool landed;
//		bool tTop, tBott, tLeft, tRight;//booleans for if touching sides of fence
	};
	std::vector<element> c;
	std::vector<element> mg;
	std::vector<element> pl;//poles in the field
	void statGoalPush(element *pl, robot *r, fence *f);
	void physics(int index, element *e, robot *r, int type);
	void fallingOn(element *fall, robot *r, int index);
	int calculateScore();
	ci::gl::Texture MobileGoal;
	ci::gl::Texture coneTexture;
	ci::gl::Texture fieldBare;

	bool initialized;//if the field bare texture is visible or not. 
	bool fieldInit;
	//vec3 Rpos;
};