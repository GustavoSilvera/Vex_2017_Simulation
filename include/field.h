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
	field(std::vector<robot> *r);


	typedef int mobileGoalIndex;
	typedef int coneIndex;

	//std::set<mobileGoalIndex> pushMoGo;
	//std::set<coneIndex> pushCones;

	void FieldUpdate(std::vector<robot> *r);
	void initialize(std::vector<robot> *r);
	bool isInit; // suggestion: construct instead of initialize

	struct fence {
		/*const*/ float fieldSizeIn = 141.05;// 140.5 + 2 * (1.27);wall thickness accounted for
		float depthIn = 1.27;//thickness of the vex fence
		void wallPush(robot *r);
		float inFromEnd = 13.9198;
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
		element(vec3 initpos, float initradius, float initheight) : pos(initpos), radius(initradius), height(initheight), inPossession(3) {}
		vec3 pos;
		float radius;//size of the object;
		/*const*/ float height;
		void fencePush(fence *f);
		void robotColl(int index, robot *r, int type, fence *f, int roboIndex);
		void collision(element *e);
		void collideWith(robot *robit, vec3 closestPoint, int type, int index, float *d2V, int roboIndex);
		std::set<coneIndex> stacked; // for goals only, cones stacked on it.
		std::vector<bool> inPossession;//if being held or whatnot
	};
	struct cone : public element {
		cone(vec3 pos) : element(pos, cRad, cHeight), fellOn(0), landed(false) {}
		int fellOn;
		bool landed;
		void coneGrab(robot *robit, int index);
	};
	struct MoGo : public element {
		MoGo(vec3 pos, int initColour) : element(pos, MGRad, mgHeight), colour(initColour) {}
		int colour;//0 is yellow, 1 is red, 2 is blue 
		void mogoGrab(robot *robit, int index);
		void zoneScore(fence *f, int index);
	};
	struct stago : public element {
		stago(vec3 pos, float initRad, float initHeight) : element(pos, initRad, initHeight) {}
	};
	std::vector<cone> c;
	std::vector<MoGo> mg;
	std::vector<stago> pl;//poles in the field

	void physics(int index, element *e, robot *r, int type, int roboIndex);
	void fallingOn(cone *fall, robot *r, int index);
	int calculateScore();
	ci::gl::Texture MobileGoal;
	ci::gl::Texture coneTexture;
	ci::gl::Texture fieldBare;

	bool initialized;//if the field bare texture is visible or not. 
	bool fieldInit;
	//vec3 Rpos;
};