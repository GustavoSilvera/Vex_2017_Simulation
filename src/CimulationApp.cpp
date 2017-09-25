#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"
#include "cinder/Vector.h"
#include "cinder/Text.h"
#include "cinder/Font.h"
//my own headers
#include "joystick.h"
#include "field.h"
#include "PID.h"
#include "nav.h"
#include "TruSpeed.h"
#include "robot.h"

#include "vec3.h"
#include "randomstuff.h"

//declaration for the main things, simulation and whatnot
using namespace ci;
using namespace ci::app;
using namespace std;

vec3 startPos;
vec3 mousePos;
class vex {
public:
	robot r;
	tSpeed tS;
	PID pid;
	nav n;
	field f;
	joystick j;

	vex() : tS(&r), pid(&r), n(&r), f(&r){}
};
//begin
int tX = 1200;
struct simulation {
	bool mouseClicked = false;
	int hovering = 1;//which button is being hovered over. if any
	enum SimulationType {
		PIDCTRL,
		NAVIGATION,
		TRUSPEED,
		FIELD
	};
	SimulationType SimRunning = FIELD;//in accordance to which simulation is running, 1 is PID, 2 is NAV, 3 is truspeed... etc
};
simulation s;
class CimulationApp : public AppNative {
public:
	void setup();
	void mouseDown(MouseEvent event);
	void mouseUp(MouseEvent event);
	void mouseMove(MouseEvent event);
	void keyDown(KeyEvent event);
	void keyUp(KeyEvent event);
	void update();
	void textDraw();
	void drawClaw();
	void drawRobot();
	void draw();
	vex v;
};
void CimulationApp::setup() {
	srand(time(NULL));
	gl::enableVerticalSync();
	v.r.TankBase = gl::Texture(loadImage(loadAsset("Tank Drive.png")));
	v.r.CChanel = gl::Texture(loadImage(loadAsset("CChanelSmall.png")));
	v.f.fieldBare = gl::Texture(loadImage(loadAsset("InTheZoneFieldBare.jpg")));
	v.f.coneTexture = gl::Texture(loadImage(loadAsset("InTheZoneCone.png")));
	v.f.MobileGoal = gl::Texture(loadImage(loadAsset("MoGoWhite.png")));
	setWindowSize(WindowWidth, WindowHeight);
	v.r.current.Xpos = 0;
	v.r.current.Ypos = 0;
	v.r.current.deg = 0;
	v.r.encoder1 = 0;
	v.r.encoderLast = 0;
	
}
//cinder::functions
void CimulationApp::mouseDown(MouseEvent event) {
	if (event.isLeft())	s.mouseClicked = true;
	if (s.SimRunning == s.PIDCTRL) v.pid.pid.requestedValue = event.getX();//gets whats needed for PID to activate
}
void CimulationApp::mouseUp(MouseEvent event) {
	if (event.isLeft())	s.mouseClicked = false;
}
void CimulationApp::mouseMove(MouseEvent event) {
	mousePos.X = event.getX();
	mousePos.Y = event.getY();
	if (mousePos.X >= v.j.drawX) {//optimization check, rather than going through withinanalogrange everytime
		if (v.j.withinAnalogRange(mousePos)) {
			if (s.SimRunning == s.TRUSPEED) {
				v.tS.activate(&v.r, &v.j);
			}
		}
	}
}
void CimulationApp::keyDown(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_UP)	v.r.ctrl.ArrowKeyUp = true;
	if (event.getCode() == KeyEvent::KEY_DOWN)	v.r.ctrl.ArrowKeyDown = true;
	if (event.getCode() == KeyEvent::KEY_LEFT)	v.r.ctrl.RotLeft = true;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r.ctrl.RotRight = true;
	if (event.getChar() == 'e' || event.getChar() == 'E') v.r.c.grabbing = !v.r.c.grabbing;//if want toggling, else look at a while ago
	if (event.getChar() == 'r' || event.getChar() == 'R') v.r.mg.grabbing = !v.r.mg.grabbing;//if want toggling, else look at a while ago
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r.c.liftUp = true;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r.c.liftDown = true;//left Z button
	if (event.getChar() == 'p' || event.getChar() == 'P') v.pid.pidVel = !v.pid.pidVel;//right P button
	if (event.getChar() == 'o' || event.getChar() == 'O') v.pid.reset(&v.r);

}
void CimulationApp::keyUp(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_DOWN) v.r.ctrl.ArrowKeyDown = false;
	if (event.getCode() == KeyEvent::KEY_UP) v.r.ctrl.ArrowKeyUp = false;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r.ctrl.RotRight = false;
	if (event.getCode() == KeyEvent::KEY_LEFT) v.r.ctrl.RotLeft = false;
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r.c.liftUp = false;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r.c.liftDown = false;//left Z button

}
void CimulationApp::update() {
	v.j.getAnalog(mousePos);
	v.r.update();//calls robot update function
	switch (s.SimRunning) {
	case simulation::NAVIGATION:
		tX = 1200;
		v.j.drawX = 300;
		v.j.drawY = 300;
		v.j.drawSize = 150;
		v.pid.isInit = false;
		v.f.isInit = false;
		v.tS.isInit = false;
		v.n.isInit = true;
		v.r.moveAround(v.j.analogX, v.j.analogY);
		v.pid.pid.isRunning = false;
		break;
	case simulation::PIDCTRL:
		tX = 1200;
		v.j.drawX = 600;
		v.j.drawY = 500;
		v.j.drawSize = 150;
		v.pid.isInit = true;
		v.f.isInit = false;
		v.tS.isInit = false;
		v.n.isInit = false;
		v.pid.PIDUpdate(&v.r);
		break;
	case simulation::TRUSPEED:
		tX = 1400;
		v.j.drawX = 300;
		v.j.drawY = 300;
		v.j.drawSize = 300;
		v.pid.isInit = false;
		v.f.isInit = false;
		v.tS.isInit = true;
		v.n.isInit = false;
		v.tS.TruSpeedUpdate(&v.r);
		v.pid.pid.isRunning = false;
		break;
	case simulation::FIELD:
		tX = 1200;
		v.j.drawX = v.f.f.fieldSize + 100;
		v.j.drawY = 500;
		v.j.drawSize = 150;
		v.pid.isInit = false;
		v.f.isInit = true;
		v.tS.isInit = false;
		v.n.isInit = false;
		v.f.FieldUpdate(&v.r);
		v.pid.pid.isRunning = false; 
		v.r.moveAround(v.j.analogX, v.j.analogY);
		break;
	}
	v.f.f.fieldEnd = v.f.f.centre.X + v.f.f.fieldSize*ppi / 2;
}
//for buttons
void clicky(int AMOUNT_BUTTON) {//function for clicking the buttons
	for (int i = 0; i < AMOUNT_BUTTON; i++) {//for each button in the array 
		if (mousePos.X > 100 * (i + 1) - (50) + (25 * i) &&
			mousePos.X < 100 * (i + 1) + (50) + (25 * i) &&
			mousePos.Y > 25 && mousePos.Y < 75) {//within boundaries for each button based off their index
			s.hovering = i;
			if (s.mouseClicked) {
				s.SimRunning = simulation::SimulationType(i);
			}
		}
	}
}
void buttons() {//function for drawing the buttons
	#define BUTTON_AMOUNT 4//number of buttons
	int bX[BUTTON_AMOUNT], bY = 50, dInBtw = 25;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	for (int i = 0; i < BUTTON_AMOUNT; i++) {
		bX[0] = 0;//initialize first button
		bX[i] = (i + 1) * 100;//increment x position for each button based off index
		if (i == s.SimRunning) { gl::color(0, 1, 0); }//if the button's index is equal to whichever button's index is being hovered over
		else if (i == s.hovering) { gl::color(1, 0, 0); }//if the button's index is equal to whichever button's index is being hovered over
		else { gl::color(1, 1, 1); }
		gl::drawStrokedRoundedRect(Area(bX[i] - 50 + dInBtw*i, bY - 25, bX[i] + 50 + dInBtw*i, bY + 25), 5);//ROUNDED rectangle with corner rad of 7
		gl::color(1, 1, 1);//resets colour 
		if (i == 0)gl::drawString("PID", Vec2f(bX[i] - 20, bY - 12.5), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 1)gl::drawString("NAV", Vec2f(bX[i] - 18 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 2)gl::drawString("TRUSpeed", Vec2f(bX[i] - 49 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 3)gl::drawString("Auton", Vec2f(bX[i] - 35 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 29));
		clicky(BUTTON_AMOUNT);//function for if a button is being hovered of pressed
	}
}
void CimulationApp::textDraw() {//function for drawing the buttons 
	//(	WARNING: RESOURCE HOG!!!!!!!!!!!)
	static const int rows = 11;
	int tY[rows], dInBtw = 50;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	//tX = 1200;
	string STRING;
	float DATA;
	for (int i = 0; i < rows; i++) {
		if (i == 0) { STRING = "Angle:"; DATA = v.r.p.mRot; }
		else if (i == 1) { STRING = "X Pos:"; DATA = v.r.p.position.X; }
		else if (i == 2) { STRING = "Y Pos:"; DATA = v.r.p.position.Y; }
		else if (i == 3) { STRING = "X-Vel:"; DATA = v.r.p.velocity.X; }
		else if (i == 4) { STRING = "Y-Vel:"; DATA = v.r.p.velocity.X; }
		else if (i == 5) { STRING = "X-Acc:"; DATA = v.r.p.acceleration.X; }//same as y accel
		else if (i == 6) { STRING = "Y-Acc:"; DATA = v.r.p.acceleration.Y; }//same as X accel
		else if (i == 7) { STRING = "R-Vel:"; DATA = v.r.p.rotVel; }
		else if (i == 8) { STRING = "R-Acc:"; DATA = v.r.p.rotAcceleration; }
		else if (i == 9) { STRING = "L-Pos:"; DATA = v.r.c.liftPos; }
		else if (i == 10){ STRING = "L-Acc:"; DATA = v.r.p.rotAcceleration; }

		tY[0] = 0;//initialize first button
		tY[i] = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(STRING, Vec2f(tX - 70, tY[i]), Color(1, 1, 1), Font("Arial", 30));
		drawText(DATA, vec3I(tX, tY[i]), vec3I(1, 1, 1), 30);
	}
}
void robotDebug(vex *v, bool reversed) {
	gl::color(1, 0, 0);
	for (int i = 0; i < 4; i++) {//simplified version of drawing the vertices
		if (reversed) gl::drawSolidCircle(Vec2f(v->f.f.fieldEnd - ppi * v->r.db.vertices[i].X, v->f.f.fieldEnd - ppi * v->r.db.vertices[i].Y), 5 + i);
		else gl::drawSolidCircle(Vec2f(ppi * v->r.db.vertices[i].X, ppi * v->r.db.vertices[i].Y), 5 + i);
	}
	if (reversed) {
		//vertice rectangles
		gl::drawStrokedRect(Area(v->f.f.fieldEnd - v->r.db.vertices[0].X*ppi, v->f.f.fieldEnd - v->r.db.vertices[0].Y*ppi, v->f.f.fieldEnd - v->r.db.vertices[1].X*ppi, v->f.f.fieldEnd - v->r.db.vertices[1].Y*ppi));
		gl::drawStrokedRect(Area(v->f.f.fieldEnd - v->r.db.vertices[2].X*ppi, v->f.f.fieldEnd - v->r.db.vertices[2].Y*ppi, v->f.f.fieldEnd - v->r.db.vertices[3].X*ppi, v->f.f.fieldEnd - v->r.db.vertices[3].Y*ppi));
		//vertical lines
		gl::drawLine(cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[1].X + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.fieldEnd - (v->r.db.vertices[1].Y + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[2].X - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.fieldEnd - (v->r.db.vertices[2].Y - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[0].X + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.fieldEnd - (v->r.db.vertices[0].Y + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[3].X - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.fieldEnd - (v->r.db.vertices[3].Y - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi));
		//horizontal lines
		gl::drawLine(cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[0].X + 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.fieldEnd - (v->r.db.vertices[0].Y + 300 * cos((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[1].X - 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.fieldEnd - (v->r.db.vertices[1].Y - 300 * cos((v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[2].X + 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.fieldEnd - (v->r.db.vertices[2].Y + 300 * cos((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.fieldEnd - (v->r.db.vertices[3].X - 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.fieldEnd - (v->r.db.vertices[3].Y - 300 * cos((v->r.p.mRot) * PI / 180))*ppi));
		//draw circle
		gl::drawStrokedCircle(Vec2f(v->f.f.fieldEnd - v->r.p.position.X*ppi, v->f.f.fieldEnd - v->r.p.position.Y*ppi), renderRad * v->r.d.size*ppi);
	}
	else {
		//vertice rectangles
		gl::drawStrokedRect(Area(v->r.db.vertices[0].X*ppi, v->r.db.vertices[0].Y*ppi, v->r.db.vertices[1].X*ppi, v->r.db.vertices[1].Y*ppi));
		gl::drawStrokedRect(Area(v->r.db.vertices[2].X*ppi, v->r.db.vertices[2].Y*ppi, v->r.db.vertices[3].X*ppi, v->r.db.vertices[3].Y*ppi));
		//vertical lines
		gl::drawLine(cinder::Vec2f((v->r.db.vertices[1].X + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[1].Y + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f((v->r.db.vertices[2].X - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[2].Y - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f((v->r.db.vertices[0].X + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[0].Y + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f((v->r.db.vertices[3].X - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[3].Y - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi));
		//horizontal lines
		gl::drawLine(cinder::Vec2f((v->r.db.vertices[0].X + 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[0].Y + 300 * cos((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f((v->r.db.vertices[1].X - 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[1].Y - 300 * cos((v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f((v->r.db.vertices[2].X + 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[2].Y + 300 * cos((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f((v->r.db.vertices[3].X - 300 * sin((v->r.p.mRot) * PI / 180))*ppi,
			(v->r.db.vertices[3].Y - 300 * cos((v->r.p.mRot) * PI / 180))*ppi));
		//draw circle
		gl::drawStrokedCircle(Vec2f(v->r.p.position.X*ppi, v->r.p.position.Y*ppi), renderRad* v->r.d.size*ppi);
	}
	//gl::drawLine(cinder::Vec2f(v.r.db.vertices[0].X*ppi, v.r.db.vertices[0].Y*ppi), cinder::Vec2f(v.r.db.vertices[3].X*ppi, v.r.db.vertices[3].Y*ppi));
	gl::color(1, 1, 1);//resets colour to white
}
void CimulationApp::drawClaw() {
	gl::draw(v.r.CChanel, Area((v.r.c.clawSize)*ppi, (v.r.d.size*.5 + v.r.c.baseSize)*ppi, (-v.r.c.clawSize)*ppi, (v.r.d.size*.5)*ppi));
	gl::color(222.0 / 225, 229.0 / 225, 34.0 / 225);
	gl::drawSolidRect(Area(Vec2d((v.r.c.clawPos + v.r.c.clawThick)*ppi, (v.r.d.size*.5 + v.r.c.clawHeight + v.r.c.baseSize)*ppi), Vec2d((v.r.c.clawPos - v.r.c.clawThick)*ppi, (v.r.d.size*.5 + v.r.c.baseSize)*ppi)));
	gl::drawSolidRect(Area(Vec2d((-v.r.c.clawPos - v.r.c.clawThick)*ppi, (v.r.d.size*.5 + v.r.c.clawHeight + v.r.c.baseSize)*ppi), Vec2d((-v.r.c.clawPos + v.r.c.clawThick)*ppi, (v.r.d.size*.5 + v.r.c.baseSize)*ppi)));
	gl::color(1, 1, 1);//reset colour
}
void CimulationApp::drawRobot() {
	glPushMatrix();
	if (s.SimRunning == s.FIELD) {
		gl::translate(Vec3f(v.f.f.fieldEnd - v.r.p.position.X*ppi, v.f.f.fieldEnd - v.r.p.position.Y*ppi, 0.0));//origin of rotation
		v.r.fieldSpeed = true;
	}
	else gl::translate(Vec3f(v.r.p.position.X*ppi, v.r.p.position.Y*ppi, 0.0));//origin of rotation
	gl::rotate(Vec3f(0, 0, -v.r.p.mRot - 90));//something for like 3D rotation.... ugh
											  //gl::color(160.0/255, 160.0/255, 160.0/255);GRAY
											  //base
	gl::color(1, 1, 1);
	gl::draw(v.r.TankBase, Area((-(v.r.d.size / 2))*ppi, (-(v.r.d.size / 2))*ppi, ((v.r.d.size / 2))*ppi, ((v.r.d.size / 2))*ppi));
	//mogo
	drawClaw();
	gl::color(66.0 / 255, 135.0 / 255, 224.0 / 255);
	gl::drawSolidRect(Area((-v.r.mg.clawPos - v.r.c.clawThick)*ppi, (-v.r.d.size*.5 - v.r.mg.clawHeight)*ppi, (-v.r.mg.clawPos + v.r.c.clawThick)*ppi, (-v.r.d.size*.5)*ppi));
	gl::drawSolidRect(Area((v.r.mg.clawPos + v.r.c.clawThick)*ppi, (-v.r.d.size*.5 - v.r.mg.clawHeight)*ppi, (v.r.mg.clawPos - v.r.c.clawThick)*ppi, (-v.r.d.size*.5)*ppi));

	glPopMatrix();//end of rotation code
}
void drawJoystick(robot *r, joystick *j) {
	gl::drawStrokedCircle(Vec2f(j->drawX + j->drawSize, j->drawY + j->drawSize), j->drawSize);//circle at (800px, vec3(1, 1, 1), 300px) with radius 127px
	gl::drawStrokedRect(Area(j->drawX, j->drawY, j->drawX + 2 * j->drawSize, j->drawY + 2 * j->drawSize));
	if (j->withinAnalogRange(mousePos)) {//defined in joystick.h, basically if within the drawing of the boundaries
		drawText(round(r->truSpeed(3, j->analogX)), vec3I(mousePos.X - 30, mousePos.Y + 50), vec3I(1, 1, 1), 30);
		drawText(round(r->truSpeed(3, j->analogY)), vec3I(mousePos.X + 30, mousePos.Y + 50), vec3I(1, 1, 1), 30);
	}
}
void CimulationApp::draw() {
	gl::enableAlphaBlending();//good for transparent images
	gl::clear(Color(0, 0, 0));
	/*glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_LEQUAL);
	glDepthRange(0.0f, 1.0f);
	glClearDepth(1.0f);*/
	//joystick analog drawing
	if (s.SimRunning == s.NAVIGATION || s.SimRunning == s.TRUSPEED || s.SimRunning == s.FIELD) {//only for navigation and truspeed sim
		if (s.SimRunning == s.FIELD) {
			v.j.drawX = v.f.f.fieldEnd + 20;
			v.j.drawY = getWindowHeight() / 2;
			v.r.d.reversed = true;
		}
		else {
			v.r.d.reversed = false;
			v.j.drawX = 600;
			v.j.drawY = 500;
			//robotDebug(&v, false);
		}
		drawJoystick(&v.r, &v.j);
	}
	if (s.SimRunning == s.PIDCTRL) {
		v.pid.textOutput(&v.r);
		v.pid.graphPlot();
	}
	if (s.SimRunning == s.TRUSPEED) {
		v.tS.graphPlot();//draws the graph
		v.tS.textOutput(&v.r, &v.j);//draws the text for the graph
	}
	if (s.SimRunning == s.FIELD) {//when field button is pressed
		ci::gl::pushModelView();//for drawing the field, had to be rotated based off source
		ci::gl::translate(ci::Vec3f(v.f.f.centre.X, v.f.f.centre.X, 0.0));//origin of rotation
		ci::gl::rotate(-90);//easy rotation technique
		ci::gl::draw(v.f.fieldBare, ci::Area(-1 * v.f.f.fieldSize*ppi / 2, -v.f.f.fieldSize*ppi / 2, v.f.f.fieldSize*ppi / 2, v.f.f.fieldSize*ppi / 2));
		ci::gl::popModelView();//finish drawing the field
		drawRobot();
		gl::drawString("Score:", Vec2f(700, 50), Color(1, 1, 1), Font("Arial", 50));
		drawText(v.f.calculateScore(), vec3I(850, 50), vec3I(1, 1, 1), 50);
		for (int i = 0; i < v.f.mg.size(); i++) {
			vec3 RGB;//true color value because cinder uses values from 0->1 for their colours
			if (v.f.mg[i].col == 1)/*red mogo*/			RGB = vec3(217, 38, 38);
			else if (v.f.mg[i].col == 2)/*blue mogo*/	RGB = vec3(0, 64, 255);
			gl::color(RGB.X / 255, RGB.Y / 255, RGB.Z / 255);
			gl::draw(v.f.MobileGoal, Area(
				(v.f.f.fieldEnd) - (v.f.mg[i].pos.X - MGRad) * ppi, 
				(v.f.f.fieldEnd) - (v.f.mg[i].pos.Y - MGRad) * ppi, 
				(v.f.f.fieldEnd) - (v.f.mg[i].pos.X + v.f.mg[i].rad) * ppi, 
				(v.f.f.fieldEnd) - (v.f.mg[i].pos.Y + v.f.mg[i].rad) * ppi));
		}
		//drawing each individual cone. oh my
		for (int i = 0; i < v.f.c.size(); i++) {
			//fieldend for where the end of the field is, to subtract values because: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
			//+-(cRad*ppi) for sayin' that the point pos.X and pos.Y are the center, and the cRad*ppi is 3 inches RADIUS away from the center point
			gl::color(1, 1, 1);
			if (i != v.r.c.holding)	gl::draw(v.f.coneTexture, Area(
				(v.f.f.fieldEnd) - (v.f.c[i].pos.X - v.f.c[i].rad) * ppi,
				(v.f.f.fieldEnd) - (v.f.c[i].pos.Y - v.f.c[i].rad) * ppi,
				(v.f.f.fieldEnd) - (v.f.c[i].pos.X + v.f.c[i].rad) * ppi,
				(v.f.f.fieldEnd) - (v.f.c[i].pos.Y + v.f.c[i].rad) * ppi));
		}
		if (v.r.c.holding != -1 && v.r.c.holding < numCones) { //if actually holding something (the index exists [!= -1])
			float coneX = v.f.c[v.r.c.holding].pos.X;
			float coneY = v.f.c[v.r.c.holding].pos.Y;
			gl::draw(v.f.coneTexture, Area(
				(v.f.f.fieldEnd) - (coneX - v.f.c[v.r.c.holding].rad) * ppi,
				(v.f.f.fieldEnd) - (coneY - v.f.c[v.r.c.holding].rad) * ppi,
				(v.f.f.fieldEnd) - (coneX + v.f.c[v.r.c.holding].rad) * ppi,
				(v.f.f.fieldEnd) - (coneY + v.f.c[v.r.c.holding].rad) * ppi) );
		}
		//robotDebug(&v, true);
		for (int i = 0; i < v.f.mg.size(); i++) {//drawing how many are stacked
			if (v.f.mg[i].stacked.size() > 0) {
				drawText(v.f.mg[i].stacked.size(), vec3I(
					(int)(v.f.f.fieldEnd - (v.f.mg[i].pos.X - cRad) * (int)ppi),
					(int)(v.f.f.fieldEnd - (v.f.mg[i].pos.Y + cRad) * (int)ppi)),
					vec3I(1, 1, 1), 50);
			}
		}
		for (int i = 0; i < v.f.pl.size(); i++) {//drawing how many are stacked
			if (v.f.pl[i].stacked.size() > 0) {
				drawText(v.f.pl[i].stacked.size(), vec3I(
					(int)(v.f.f.fieldEnd - (v.f.pl[i].pos.X - cRad) * (int)ppi),
					(int)(v.f.f.fieldEnd - (v.f.pl[i].pos.Y + cRad) * (int)ppi)),
					vec3I(1, 1, 1), 50);
			}
		}

		gl::color(1, 1, 1);
	}
	else drawRobot();

	gl::color(1, 0, 0);
	gl::drawSolidCircle(Vec2f(v.f.f.fieldEnd - v.f.mg[2].pos.X*ppi, v.f.f.fieldEnd - v.f.f.poleEquation(140.5, 23.2, -1, v.f.mg[2].pos.X)*ppi), 5);
	//if (v.f.c[39].landed) gl::drawString("YES", Vec2f(1000, 600), Color(1, 1, 1), Font("Arial", 30));
	//else gl::drawString("NO", Vec2f(1000, 600), Color(1, 1, 1), Font("Arial", 30));
	//drawText(v.f.f.twentyPoint[0].size(), vec3I(1000, 660), vec3I(1, 1, 1), 30);
	//drawText(v.f.mg[3].pos.Y, vec3I(1000, 500), vec3I(1, 1, 1), 30);
	//drawText(v.f.mg[3].pos.X, vec3I(1000, 400), vec3I(1, 1, 1), 30);

	gl::color(1, 1, 1);
	//USER INTERFACE
	buttons();
	gl::drawString("FPS: ", Vec2f(getWindowWidth() - 150, 30), Color(0, 1, 0), Font("Arial", 30));
	drawText(getAverageFps(), vec3I(getWindowWidth() - 90, 30), vec3I(0, 1, 0), 30);
	//if(s.SimRunning != s.TRUSPEED) textDraw();//dont run on truspeed sim, unnecessary
}
CINDER_APP_NATIVE(CimulationApp, RendererGl)
