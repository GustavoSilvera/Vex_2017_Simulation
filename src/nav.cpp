#include "nav.h"

nav::nav(robot *r) {
	isInit = true;
}
void nav::initialize(robot *r) {
	r->p.acceleration.X = r->p.acceleration.X = 0; 
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.position.Y = r->p.position.X = 69.6;
}

void nav::calculatePos(robot *r) {
	if (r->p.rotVel == 0) {//not rotating
						//float Magnitude = ((changeInDist) * 4 * PI) / (360);//function for adding the change in inches to current posiiton
		r->current.deg = r->p.mRot;
		r->current.Xpos += cos(r->current.deg*(PI / 180))*(r->encoder1 - r->encoderLast);//cosine of angle times magnitude RADIANS(vector trig)//NOT WORKING
		r->current.Ypos -= sin(r->current.deg*(PI / 180))*(r->encoder1 - r->encoderLast);//sine of angle times magnitude RADIANS(vector trig)//NOT WORKING
		r->encoderLast = r->encoder1;
	}
}

void nav::navUpdate(robot *r) {
	if (!isInit) initialize(r);
	calculatePos(r);
}