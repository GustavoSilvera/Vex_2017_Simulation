#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"

#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"

#include <vector>
#include <list>
#include <set>

//declares the class for the robot and all the data that goes with it.
//LOOKIE HERE: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
class field {
public:
	field();

	std::set<int> stacked;

	void FieldUpdate(robot *r );
	void initializeField(robot *r);
	//static  int NUMBER_OF_CONES = 50;
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
		int rad;//size of the object;
		void calcD2Vertices(robot *r);
		float d2V[4];//distance to each vertice on the robot
		bool directlyInVerticalPath(robot *r);
		bool directlyInHorizontalPath(robot *r);
		vec3 closestPoint;
		void fencePush(fence *f);
		float d2E[2];//0 is d2 right, 1 is d2 top, 2 is d2left, 3 is d2 bottom
		void robotColl(int index, robot *r, std::set<int> &s);
		void collision(int index, element *e, std::set<int> &s);
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
};

