#include <string>
#include "TruSpeed.h"
using namespace ci;
using namespace std;
//declares and defines the field class and functions
tSpeed::tSpeed(robot *r, joystick *j) {
	isInit = false;
	r->p.acceleration.X = r->p.acceleration.Y = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.mRot = 0;
	r->p.rotAcceleration = 0;
	r->p.position.Y = 69.6;
	gr.YAxLength = 300;
	gr.XAxLength = 500;
	gr.drawX = 800;
	gr.drawY = 80;
	gr.midpoint = ((gr.YAxLength) / 2) + gr.drawY;
	for (int i = 0; i < maxDots; i++) {
		gr.RYpos[i] = gr.midpoint;
		gr.BYpos[i] = gr.midpoint;
	}
}
void tSpeed::initialize(robot *r) {
	r->p.acceleration.X = r->p.acceleration.Y = 0;
	r->p.velocity.X = r->p.velocity.Y = 0;
	r->p.rotAcceleration = 0;
	r->p.mRot = 0;
	r->p.position.Y = 69.6;
	gr.YAxLength = 300;
	gr.XAxLength = 500;
	gr.drawX = 800;
	gr.drawY = 80;
	gr.midpoint = ((gr.YAxLength) / 2) + gr.drawY;
	for (int i = 0; i < maxDots; i++) {
		gr.RYpos[i] = gr.midpoint;
		gr.BYpos[i] = gr.midpoint;
	}
	isInit = true;
	
}
void tSpeed::activate(robot *r, joystick *j, double mX, double mY) {
	for (int i = 0; i < (maxDots - 1); i++) {//red line /*XPOS*/
		gr.RYpos[i] = gr.RYpos[i + 1];
	}
	gr.RYpos[maxDots - 1] = (r->truSpeed(3, (mX - (j->drawX + j->drawSize))) / (gr.YAxLength*0.003)) + gr.midpoint;
	for (int i = 0; i < (maxDots - 1); i++) {//blue line /*YPOS*/
		gr.BYpos[i] = gr.BYpos[i + 1];
	}
	gr.BYpos[maxDots - 1] = (r->truSpeed(3, (mY - (j->drawY + j->drawSize))) / (gr.YAxLength*0.003)) + gr.midpoint;
}
void tSpeed::graphPlot() {
	//axis:
	gl::drawSolidRect(Rectf(gr.drawX, gr.drawY, gr.drawX + 2, gr.drawY + gr.YAxLength));
	gl::drawSolidRect(Rectf(gr.drawX, gr.midpoint - 1, gr.drawX + gr.XAxLength, gr.midpoint + 1));
	gl::drawString("127", Vec2f(gr.drawX - 30, gr.drawY), Color(1, 1, 1), Font("Arial", 20));
	gl::drawString("-127", Vec2f(gr.drawX - 35, gr.YAxLength + gr.drawY + 20), Color(1, 1, 1), Font("Arial", 20));
	gl::drawString("0", Vec2f(gr.drawX - 15, gr.midpoint), Color(1, 1, 1), Font("Arial", 20));
	//lines:
	gl::color(Color(1, 0, 0)); // blue
	int dSize = 2;
	for (int i = 0; i < maxDots; i++) {
		int dotX = gr.drawX + i * (gr.XAxLength) / maxDots;//makes the little intervals for the X axis line
		int dotY = gr.RYpos[i];
		gl::drawSolidCircle(Vec2f(dotX, dotY), dSize);
	}
	gl::color(Color(0, 253, 255)); // light blue
	for (int i = 0; i < maxDots; i++) {
		int dotX = gr.drawX + i * (gr.XAxLength) / maxDots;//makes the little intervals for the X axis line
		int dotY = gr.BYpos[i];
		gl::drawSolidCircle(Vec2f(dotX, dotY), dSize);
	}
	gl::color(Color::white());//resets the colour 
	
}
void tSpeed::textOutput(robot *r, joystick *j) {
	//title
	//Defining red line
	gl::drawString("X: ", Vec2f(gr.drawX + 40, gr.drawY - 15), Color(1, 1, 1), Font("Arial", 20));
	gl::color(Color(1, 0, 0)); // red
	gl::drawSolidRect(Rectf(gr.drawX + 60, gr.drawY - 18, gr.drawX + 70, gr.drawY - 2));
	//defining blue line
	gl::drawString("Y: ", Vec2f(gr.drawX + 90, gr.drawY - 15), Color(1, 1, 1), Font("Arial", 20));
	gl::color(Color(0, 253, 255)); // light blue
	gl::drawSolidRect(Rectf(gr.drawX + 110, gr.drawY - 18, gr.drawX + 120, gr.drawY - 2));
	gl::color(Color::white());//resets the colour
								//other information:
								//actual X
	gl::drawString("Actual X:", Vec2f(gr.drawX + 30, gr.YAxLength + gr.drawY + 10), Color(1, 1, 1), Font("Arial", 25));
	drawText(j->analogX, vec3(gr.drawX + 130, gr.YAxLength + gr.drawY + 10), vec3(1, 1, 1), 25);
	//modified X
	gl::drawString("-->", Vec2f(gr.drawX + 170, gr.YAxLength + gr.drawY + 10), Color(1, 1, 1), Font("Arial", 25));
	drawText(round(r->truSpeed(3, j->analogX)), vec3(gr.drawX + 210, gr.YAxLength + gr.drawY + 10), vec3(1, 1, 1), 25);
	//actual Y
	gl::drawString("Actual Y:", Vec2f(gr.drawX + 30, gr.YAxLength + gr.drawY + 30), Color(1, 1, 1), Font("Arial", 25));
	drawText(j->analogY, vec3(gr.drawX + 130, gr.YAxLength + gr.drawY + 30), vec3(1, 1, 1), 25);
	//modified X
	gl::drawString("-->", Vec2f(gr.drawX + 170, gr.YAxLength + gr.drawY + 30), Color(1, 1, 1), Font("Arial", 25));
	drawText(round(r->truSpeed(3, j->analogY)), vec3(gr.drawX + 210, gr.YAxLength + gr.drawY + 30), vec3(1, 1, 1), 25);
}

void tSpeed::TruSpeedUpdate(robot *robit) {
	if(!isInit) initialize(robit);
	robit->p.velocity.X = 0;
	robit->p.velocity.Y = 0;
}
