#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"

#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <vector>
#include <set>

//declares the class for the robot and all the data that goes with it.
//LOOKIE HERE: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
class field {
public:
	field(robot *robit);

	std::set<int> pushMoGo;
	std::set<int> pushCones;

	void FieldUpdate(robot *r);
	void initialize(robot *r);
	bool isInit;
	struct fence {
		float fieldSize = 141.05;// 140.5 + 2 * (1.27);wall thickness accounted for
		vec3 centre = vec3(606, 606);//in pixels
		float fieldEnd = centre.X + fieldSize*ppi / 2;//furthest to the right the field is touching
		float depth = 1.27;//thickness of the vex fence
		void wallPush(robot *r);
		float d2E[2];
		/*for stationary goals*/
		float d2StatGoal[2];
	};
	fence f;

	struct element {
		vec3 pos;
		int col;//0 is yellow, 1 is red, 2 is blue 
		float rad;//size of the object;
		float height;
		void calcD2Vertices(robot *r);
		float d2V[4];//distance to each vertice on the robot
		float d2Robot, d2RobotEdge;
		bool directlyInVerticalPath(robot *r);
		bool directlyInHorizontalPath(robot *r);
		vec3 closestPoint;
		void fencePush(fence *f);
		float d2E[2];//0 is d2 top, 1 is d2left, 
		void robotColl(int index, robot *r, std::set<int> &pushCone, std::set<int> &pushMoGo, int type);
		void collision(element *e);
		void ConeGrabbed(robot *r, int index, element *pl1, element *pl2);
		void MoGoGrabbed(robot *r, int index);//DEFAULTED FOR MOGOS ONLY
		bool held;
		bool tTop, tBott, tLeft, tRight;//booleans for if touching sides of fence
	};
	std::vector<element> c;
	std::vector<element> mg;
	std::vector<element> pl;//poles in the field
	void statGoalPush(element *pl, robot *r, fence *f);
	void physics(int index, element *e, robot *r, int type);

	ci::gl::Texture MobileGoal;
	ci::gl::Texture coneTexture;
	ci::gl::Texture fieldBare;

	bool initialized;//if the field bare texture is visible or not. 
	int HELLO;
	int pushingCones = 0;
	bool fieldInit;
};