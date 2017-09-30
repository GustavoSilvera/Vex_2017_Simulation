#include "nav.h"

nav::nav(robot *r) {
	isInit = false;
}
void nav::initialize(robot *r) {
	r->p.acceleration.X = r->p.acceleration.X = 0; 
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.position.Y = r->p.position.X = 69.6;
}

void nav::calculatePos(robot *r) {

}

void nav::navUpdate(robot *r) {
	if (!isInit) initialize(r);
	calculatePos(r);
}