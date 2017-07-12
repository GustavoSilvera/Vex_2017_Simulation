#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "vec3.h"
#include "robot.cpp"
#include "robot.h"
#include <vector>
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
		int heading;//direction of the cone in 360° when tipped
		bool tipped;//if fallen 
	};
	std::vector<cone> c;//vector of cones

	ci::gl::Texture coneTexture;
	ci::gl::Texture fieldBare;
	ci::gl::Texture fieldFull;

	float fieldSize = 141.05;// 140.5 + 2 * (1.27);wall thickness accounted for
	int dFromEdge = 100;//draw distance from the start of the field verticie to the edge of the window
	bool initialized;//if the field bare texture is visible or not. 
};

