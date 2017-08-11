#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "vec3.h"
#include "robot.h"
#include "randomstuff.h"
#include <vector>
#include <list>
//declares the class for the robot and all the data that goes with it.
//LOOKIE HERE: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
class field {
public:
	field();

	void FieldUpdate(robot *r );
	void initializeField();
	//static  int NUMBER_OF_CONES = 50;
	struct cone {
		vec3 pos;//position
		float heading;//direction of the cone in 360° when tipped
		bool tipped;//if fallen 
		void calcD2Vertices(robot *r);
		float d2V[4];//distance to each vertice on the robot
		float distanceToCone[53];
		bool directlyInVerticalPath(robot *r);
		bool directlyInHorizontalPath(robot *r);
		vec3 closestPoint;
		void fencePush(int coneRad, double fieldSize);
		float d2E[2];//0 is d2 right, 1 is d2 top, 2 is d2left, 3 is d2 bottom
	};
	std::vector<cone> c;//vector of cones
	std::vector<int> s;//how many cones are stacked
	struct MoGo {
		vec3 pos;
		bool red;
		void calcD2Vertices(robot *r);
		float d2V[4];//distance to each vertice on the robot
		bool directlyInVerticalPath(robot *r);
		bool directlyInHorizontalPath(robot *r);
		vec3 closestPoint;
		void fencePush(int coneRad, double fieldSize);
		float d2E[2];//0 is d2 right, 1 is d2 top, 2 is d2left, 3 is d2 bottom
	};
	std::vector<MoGo> mg;//vector of mogos

	ci::gl::Texture MobileGoal;
	ci::gl::Texture coneTexture;
	ci::gl::Texture fieldBare;

	struct fence {
		float fieldSize = 141.05;// 140.5 + 2 * (1.27);wall thickness accounted for
		vec3 centre = vec3(606, 606);//in pixels
		float fieldEnd = centre.X + fieldSize*ppi / 2;//furthest to the right the field is touching

	};
	fence f;
	bool initialized;//if the field bare texture is visible or not. 
	int coneRad = 3;
	int MoGoRad = 5;
	int HELLO;
	int renderRad = 1;//amount of the robot's radii that are used to calculate cone distance, smaller is more optimized (but calculates for less cones)
	int pushingCones = 0;
};

