#include "Custom.h"
#include <string>
using namespace ci;

customize::customize(robot *r) {
	isInit = false;
}
void customize::initialize(robot *r) {
	r->p.acceleration.X = r->p.acceleration.X = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.position.Y = r->p.position.X = 70;
}

void customize::Text() {//function for drawing the buttons
	const int dInBtw = 50;//array for #buttons, dInBtw is distance in bwtween buttons
	const int tX = 1200;//starting x value
	struct text {
		std::string s;
		double f;
	};
	text t[] = {
		{ "Size:", size}
	};
	int i = 0;
	for (text& ti : t) {
		int tY = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(ti.s, Vec2f(scale*(tX - 70), scale*(tY)), Color(1, 1, 1), Font("Arial", scale * 30));
		drawText(ti.f, vec3I(scale*(tX), scale*(tY)), vec3I(1, 1, 1), scale * 30);
		i++;
	}
}
void customize::controlPanel(robot *r, float scalar) {
	//scale = scalar;
	//Text();
	//r->d.size = size;
	//this is ded now
}
