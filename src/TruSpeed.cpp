#include <string>
#include "TruSpeed.h"
using namespace ci;
using namespace std;

const int tSpeed::maxDots;

//declares and defines the field class and functions
tSpeed::tSpeed(robot *r) {
	isInit = false;
	tSpeed::initialize(r);
}
void tSpeed::initialize(robot *r) {
	r->p.acceleration.X = r->p.acceleration.Y = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.rotAcceleration = 0;
	r->p.position.Y = 69.6;
	gr.YAxLength = 300;
	gr.XAxLength = 800;
	gr.drawX = 550;
	gr.drawY = 80;
	gr.midpoint = ((gr.YAxLength) / 2) + gr.drawY;
	gr.RYpos.resize(maxDots);
	gr.BYpos.resize(maxDots);
	gr.GYpos.resize(maxDots);
	gr.YYpos.resize(maxDots);
	for (int i = 0; i < maxDots; i++) {
		gr.RYpos[i] = gr.midpoint;
		gr.BYpos[i] = gr.midpoint;
		gr.GYpos[i] = gr.midpoint;
		gr.BYpos[i] = gr.midpoint;
		gr.posX[i] = gr.drawX + i * (gr.XAxLength) / maxDots;
	}
	isInit = true;
}
void tSpeed::activate(robot *r, joystick *j) {
#if 0
	for (int i = 0; i < (maxDots - 1); i++) {//red line /*XPOS*/
		gr.RYpos[i] = gr.RYpos[i + 1];
	}
	gr.RYpos[maxDots - 1] = (r->truSpeed(3, (mX - (j->drawX + j->drawSize))) / (gr.YAxLength*0.003)) + gr.midpoint;
	for (int i = 0; i < (maxDots - 1); i++) {//blue line /*YPOS*/
		gr.BYpos[i] = gr.BYpos[i + 1];
	}
	gr.BYpos[maxDots - 1] = (r->truSpeed(3, (mY - (j->drawY + j->drawSize))) / (gr.YAxLength*0.003)) + gr.midpoint;
#endif
	//basicaly the same as above but with deques
	float ySpace = (gr.YAxLength*0.003);
	gr.RYpos.pop_front();
	gr.RYpos.push_back(r->truSpeed(3, (j->analogX / ySpace)) + gr.midpoint);
	gr.BYpos.pop_front();
	gr.BYpos.push_back(r->truSpeed(3, (j->analogY / ySpace)) + gr.midpoint);
	//non tru speeds
	gr.GYpos.pop_front();
	gr.GYpos.push_back((j->analogX / ySpace) + gr.midpoint);
	gr.YYpos.pop_front();
	gr.YYpos.push_back((j->analogY / ySpace) + gr.midpoint);
}
void tSpeed::graphPlot() {
	
	//lines:
	int dSize = 2;
	gl::color(Color(1, 0, 0)); //red line (XPOSITION)
	for (int i = 0; i < maxDots; i++) {
		gl::drawSolidCircle(Vec2f(gr.posX[i], gr.RYpos[i]), dSize);
		if(i != maxDots-1)gl::drawLine(Vec2f(gr.posX[i], gr.RYpos[i]), Vec2f(gr.posX[i+1], gr.RYpos[i+1]));
	}
	gl::color(Color(0, 253, 255)); // blue line (YPOSTION)
	for (int i = 0; i < maxDots; i++) {
		gl::drawSolidCircle(Vec2f(gr.posX[i], gr.BYpos[i]), dSize);
		if (i != maxDots - 1)gl::drawLine(Vec2f(gr.posX[i], gr.BYpos[i]), Vec2f(gr.posX[i + 1], gr.BYpos[i + 1]));
	}
	gl::color(Color(255.0 / 256.0, 255.0 / 256.0, 0)); // yelow line (No-TS-Y)
	for (int i = 0; i < maxDots; i++) {
		gl::drawSolidCircle(Vec2f(gr.posX[i], gr.YYpos[i]), dSize);
		if (i != maxDots - 1)gl::drawLine(Vec2f(gr.posX[i], gr.YYpos[i]), Vec2f(gr.posX[i + 1], gr.YYpos[i + 1]));
	}
	gl::color(Color(0, 1, 0)); // green line (NO-TS-X)
	for (int i = 0; i < maxDots; i++) {
		gl::drawSolidCircle(Vec2f(gr.posX[i], gr.GYpos[i]), dSize);
		if (i != maxDots - 1)gl::drawLine(Vec2f(gr.posX[i], gr.GYpos[i]), Vec2f(gr.posX[i + 1], gr.GYpos[i + 1]));
	}
	gl::color(Color(1, 1, 1));//clear to white
}

void tSpeed::textOutput(robot *r, joystick *j) {
	gl::drawSolidRect(Rectf(gr.drawX, gr.drawY, gr.drawX + 2, gr.drawY + gr.YAxLength));
	gl::drawSolidRect(Rectf(gr.drawX, gr.midpoint - 1, gr.drawX + gr.XAxLength, gr.midpoint + 1));
	gl::drawString("127", Vec2f(gr.drawX - 30, gr.drawY), Color(1, 1, 1), Font("Arial", 20));
	gl::drawString("-127", Vec2f(gr.drawX - 35, gr.YAxLength + gr.drawY + 20), Color(1, 1, 1), Font("Arial", 20));
	gl::drawString("0", Vec2f(gr.drawX - 15, gr.midpoint), Color(1, 1, 1), Font("Arial", 20));
	gl::drawString("X: ", Vec2f(gr.drawX + 40, gr.drawY - 15), Color(1, 1, 1), Font("Arial", 20));
	gl::color(Color(1, 0, 0)); // red
	gl::drawSolidRect(Rectf(gr.drawX + 60, gr.drawY - 18, gr.drawX + 70, gr.drawY - 2));
	//defining blue line
	gl::drawString("Y: ", Vec2f(gr.drawX + 90, gr.drawY - 15), Color(1, 1, 1), Font("Arial", 20));
	gl::color(Color(0, 253, 255)); // light blue
	gl::drawSolidRect(Rectf(gr.drawX + 110, gr.drawY - 18, gr.drawX + 120, gr.drawY - 2));
	//actual X
	gl::drawString("Actual X:", Vec2f(gr.drawX + 30, gr.YAxLength + gr.drawY + 10), Color(1, 1, 1), Font("Arial", 25));
	drawText(j->analogX, vec3I(gr.drawX + 130, gr.YAxLength + gr.drawY + 10), vec3I(1, 1, 1), 25);
	//modified X
	gl::drawString("-->", Vec2f(gr.drawX + 170, gr.YAxLength + gr.drawY + 10), Color(1, 1, 1), Font("Arial", 25));
	drawText(round(r->truSpeed(3, j->analogX)), vec3I(gr.drawX + 210, gr.YAxLength + gr.drawY + 10), vec3I(1, 1, 1), 25);
	//actual Y
	gl::drawString("Actual Y:", Vec2f(gr.drawX + 30, gr.YAxLength + gr.drawY + 30), Color(1, 1, 1), Font("Arial", 25));
	drawText(j->analogY, vec3I(gr.drawX + 130, gr.YAxLength + gr.drawY + 30), vec3I(1, 1, 1), 25);
	//modified X
	gl::drawString("-->", Vec2f(gr.drawX + 170, gr.YAxLength + gr.drawY + 30), Color(1, 1, 1), Font("Arial", 25));
	drawText(round(r->truSpeed(3, j->analogY)), vec3I(gr.drawX + 210, gr.YAxLength + gr.drawY + 30), vec3I(1, 1, 1), 25);
	
}

void tSpeed::TruSpeedUpdate(robot *robit) {
	if(!isInit) initialize(robit);
	robit->reset();
	robit->p.position.X = 69.6;
	robit->p.position.Y = 69.9;
	robit->p.mRot += 1;
}
