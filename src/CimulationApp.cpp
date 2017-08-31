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
#include "robot.h"
#include "vec3.h"
#include "randomstuff.h"

//declaration for the main things, simulation and whatnot
using namespace ci;
using namespace ci::app;
using namespace std;

static const int maxDots = 250;//maximum amount of graph particles for the truspeed sim
vec3 startPos;
vec3 mousePos;
class vex {
public:
	robot r;
	field f;
	joystick j;
	vex() : r() {}
};
inline void drawText(double text, vec3 pos, vec3 colour, int size) {//simplified way of printing variables as text to the display
	stringstream dummyText;
	string PRINT;
	dummyText << text;
	dummyText >> PRINT;
	gl::drawString(PRINT, Vec2f(pos.X, pos.Y), Color(colour.X, colour.Y, colour.Z), Font("Arial", size));
}
//begin
struct simulation {
	bool mouseClicked = false;
	int hovering = 1;//which button is being hovered over. if any
	enum SimulationType {
		PIDCTRL,
		NAVIGATION,
		TRUSPEED,
		FIELD
	};
	SimulationType SimRunning = NAVIGATION;//in accordance to which simulation is running, 1 is PID, 2 is NAV, 3 is truspeed... etc
	struct graph {
		//graph for speed of the speed of the wheels based off position
		//for graph moving
		double RYpos[maxDots];
		double BYpos[maxDots];
		//issue: yaxix length not scaling well with particles
		int YAxLength = 300, XAxLength = 500;
		int drawX = 800, drawY = 80;
		int midpoint = ((YAxLength) / 2) + drawY;//midpoint of the graph, if the graph starts 20 points down.tY;
		void graphPlot(){
			//axis:
			gl::drawSolidRect(Rectf(drawX, drawY, drawX+2, drawY + YAxLength));
			gl::drawSolidRect(Rectf(drawX, midpoint - 1, drawX + XAxLength, midpoint + 1));
			gl::drawString("127", Vec2f(drawX-30, drawY), Color(1, 1, 1), Font("Arial", 20));
			gl::drawString("-127", Vec2f(drawX-35, YAxLength + drawY + 20), Color(1, 1, 1), Font("Arial", 20));
			gl::drawString("0", Vec2f(drawX-15, midpoint), Color(1, 1, 1), Font("Arial", 20));
			//lines:
			gl::color(Color(1, 0, 0)); // blue
			for (int i = 0; i < maxDots; i++) {
				int dotX = drawX + i * (XAxLength) / maxDots;//makes the little intervals for the X axis line
				int dotY = RYpos[i];
				gl::drawSolidRect(Area(dotX + 1, dotY - 1, dotX - 1, dotY + 1));
			}
			gl::color(Color(0, 253, 255)); // light blue
			for (int i = 0; i < maxDots; i++) {
				int dotX = drawX + i * (XAxLength) / maxDots;//makes the little intervals for the X axis line
				int dotY = BYpos[i];
				gl::drawSolidRect(Area(dotX + 1, dotY - 1, dotX - 1, dotY + 1));
			}
			gl::color(Color::white());//resets the colour 
		}
		void textOutput(vex *v) {
			//title
			//Defining red line
			gl::drawString("X: ", Vec2f(drawX + 40, drawY-15), Color(1, 1, 1), Font("Arial", 20));
			gl::color(Color(1, 0, 0)); // light blue
			gl::drawSolidRect(Rectf(drawX+60, drawY-18, drawX+70, drawY-2));
			//defining blue line
			gl::drawString("Y: ", Vec2f(drawX+90, drawY-15), Color(1, 1, 1), Font("Arial", 20));
			gl::color(Color(0, 253, 255)); // light blue
			gl::drawSolidRect(Rectf(drawX+110, drawY-18, drawX+120, drawY-2));
			gl::color(Color::white());//resets the colour
				  //other information:
			 //actual X
			gl::drawString("*Actual X:", Vec2f(drawX+30, YAxLength + drawY + 10), Color(1, 1, 1), Font("Arial", 25));
			drawText(v->j.analogX, vec3(drawX + 130, YAxLength + drawY + 10), vec3(1, 1, 1), 25);
			//modified X
			gl::drawString("-->", Vec2f(drawX+170, YAxLength + drawY + 10), Color(1, 1, 1), Font("Arial", 25));
			drawText(round(v->r.truSpeed(3, v->j.analogX)), vec3(drawX + 210, YAxLength + drawY + 10), vec3(1, 1, 1), 25);
			//actual Y
			gl::drawString("Actual Y:", Vec2f(drawX+30, YAxLength + drawY + 30), Color(1, 1, 1), Font("Arial", 25));
			drawText(v->j.analogY, vec3(drawX + 130, YAxLength + drawY + 30), vec3(1, 1, 1), 25);
			//modified X
			gl::drawString("-->", Vec2f(drawX+170, YAxLength + drawY + 30), Color(1, 1, 1), Font("Arial", 25));
			drawText(round(v->r.truSpeed(3, v->j.analogY)), vec3(drawX + 210, YAxLength + drawY + 30), vec3(1, 1, 1), 25);
		}
	};
	struct graph gr;
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
	void drawRobot();
	void draw();
	vex v;
};
void CimulationApp::setup() {
	srand(time(NULL));
	gl::enableVerticalSync();
	v.r.TankBase = gl::Texture(loadImage(loadAsset("Tank Drive.png")));
	v.f.fieldBare = gl::Texture(loadImage(loadAsset("InTheZoneFieldBare.jpg")));
	v.f.coneTexture = gl::Texture(loadImage(loadAsset("InTheZoneCone.png")));
	v.f.MobileGoal = gl::Texture(loadImage(loadAsset("MoGoWhite.png")));
	setWindowSize(WindowWidth, WindowHeight);
	v.r.current.Xpos = 0;
	v.r.current.Ypos = 0;
	v.r.current.deg = 0;
	v.r.encoder1 = 0;
	v.r.encoderLast = 0;
	v.r.PID.isRunning = false;
	v.r.PID.requestedValue = v.r.p.position.X*ppi;

	//for graph
	for (int i = 0; i < maxDots; i++) {
		s.gr.RYpos[i] = s.gr.midpoint;
		s.gr.BYpos[i] = s.gr.midpoint;
	}
}
//cinder::functions
void CimulationApp::mouseDown(MouseEvent event) {
	if (event.isLeft())	s.mouseClicked = true;
	if (s.SimRunning == s.PIDCTRL) v.r.PID.requestedValue = event.getX();
}
void CimulationApp::mouseUp(MouseEvent event) {
	if (event.isLeft())	s.mouseClicked = false;
}
void CimulationApp::mouseMove(MouseEvent event) {
	mousePos.X = event.getX();
	mousePos.Y = event.getY();
	if (v.j.withinAnalogRange(mousePos)) {
		if (s.SimRunning == s.TRUSPEED) {
			for (int i = 0; i < (maxDots - 1); i++) {//red line /*XPOS*/
				s.gr.RYpos[i] = s.gr.RYpos[i + 1];
			}
			s.gr.RYpos[maxDots - 1] = (v.r.truSpeed(3, (mousePos.X - (v.j.drawX + v.j.drawSize))) / (s.gr.YAxLength*0.003)) + s.gr.midpoint;
			for (int i = 0; i < (maxDots - 1); i++) {//blue line /*YPOS*/
				s.gr.BYpos[i] = s.gr.BYpos[i + 1];
			}
			s.gr.BYpos[maxDots - 1] = (v.r.truSpeed(3, (mousePos.Y - (v.j.drawY + v.j.drawSize))) / (s.gr.YAxLength*0.003)) + s.gr.midpoint;
		}
	}
}
void CimulationApp::keyDown(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_UP)	v.r.ctrl.ArrowKeyUp = true;
	if (event.getCode() == KeyEvent::KEY_DOWN)	v.r.ctrl.ArrowKeyDown = true;
	if (event.getCode() == KeyEvent::KEY_LEFT)	v.r.ctrl.RotLeft = true;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r.ctrl.RotRight = true;
	if (event.getChar() == 'e' || event.getChar() == 'E') v.r.c.grabbing = !v.r.c.grabbing;//if want toggling, else look at a while ago
	if (event.getChar() == 'r' || event.getChar() == 'R' ) v.r.mg.grabbing = !v.r.mg.grabbing;//if want toggling, else look at a while ago
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r.c.liftUp = true;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r.c.liftDown = true;//left Z button
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
		v.r.NavigationUpdate();
		v.r.moveAround(v.j.analogX, v.j.analogY);
		v.f.initialized = false;//only initialize the field elements when running FIELD
			break;
	case simulation::PIDCTRL:
		v.r.PIDControlUpdate();
		v.f.initialized = false;//only initialize the field elements when running FIELD
		break;
	case simulation::TRUSPEED:
		v.r.TruSpeedUpdate();
		v.r.moveAround(v.j.analogX, v.j.analogY);
		v.f.initialized = false;//only initialize the field elements when running FIELD
		break;
	case simulation::FIELD:
		v.f.FieldUpdate(&v.r);
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
		gl::drawStrokedRect(Area(bX[i] - 50 + dInBtw*i, bY - 25, bX[i] + 50 + dInBtw*i, bY + 25));
		gl::color(1, 1, 1);//resets colour 
		if (i == 0)gl::drawString("PID", Vec2f(bX[i] - 20, bY - 12.5), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 1)gl::drawString("NAV", Vec2f(bX[i] - 18 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 2)gl::drawString("TRUSpeed", Vec2f(bX[i] - 49 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 25));
		else if (i == 3)gl::drawString("Auton", Vec2f(bX[i] - 35 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 29));
		clicky(BUTTON_AMOUNT);//function for if a button is being hovered of pressed
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
void CimulationApp::drawRobot() {
	/*****************************ROBOT***********************************/
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
	//claw
	gl::color(222.0 / 225, 229.0 / 225, 34.0 / 225);
	
	gl::drawSolidRect(Area((v.r.c.clawPos + v.r.c.clawThick)*ppi + v.r.c.liftPos, (v.r.d.size*.5 + v.r.c.clawHeight)*ppi + v.r.c.liftPos, (v.r.c.clawPos - v.r.c.clawThick)*ppi - v.r.c.liftPos, (v.r.d.size*.5)*ppi - v.r.c.liftPos));
	gl::drawSolidRect(Area((-v.r.c.clawPos - v.r.c.clawThick)*ppi - v.r.c.liftPos, (v.r.d.size*.5 + v.r.c.clawHeight)*ppi + v.r.c.liftPos, (-v.r.c.clawPos + v.r.c.clawThick)*ppi + v.r.c.liftPos, (v.r.d.size*.5)*ppi - v.r.c.liftPos));
	//mogo
	gl::color(66.0 / 255, 135.0 / 255, 224.0 / 255);
	gl::drawSolidRect(Area((-v.r.mg.clawPos - v.r.c.clawThick)*ppi, (-v.r.d.size*.5 - v.r.mg.clawHeight)*ppi, (-v.r.mg.clawPos + v.r.c.clawThick)*ppi, (-v.r.d.size*.5)*ppi));
	gl::drawSolidRect(Area((v.r.mg.clawPos + v.r.c.clawThick)*ppi, (-v.r.d.size*.5 - v.r.mg.clawHeight)*ppi, (v.r.mg.clawPos - v.r.c.clawThick)*ppi, (-v.r.d.size*.5)*ppi));

	glPopMatrix();//end of rotation code
}
void CimulationApp::draw() {
	gl::enableAlphaBlending();//good for transparent images
	// clear out the window with black
	gl::clear(Color(0, 0, 0));
	//joystick analog drawing
	if (s.SimRunning == s.NAVIGATION || s.SimRunning == s.TRUSPEED || s.SimRunning == s.FIELD) {//only for navigation and truspeed sim
		if (s.SimRunning == s.FIELD) {
			v.j.drawX = v.f.f.fieldEnd + 20;
			v.j.drawY = getWindowHeight()/2;
			v.r.d.reversed = true;
		}
		else {
			v.r.d.reversed = false;
			v.j.drawX = 600;
			v.j.drawY = 500;
			robotDebug(&v, false);
		}
		gl::drawStrokedCircle(Vec2f(v.j.drawX+v.j.drawSize, v.j.drawY+v.j.drawSize), v.j.drawSize);//circle at (800px, vec3(1, 1, 1), 300px) with radius 127px
		gl::drawStrokedRect(Area(v.j.drawX, v.j.drawY, v.j.drawX + 2*v.j.drawSize, v.j.drawY + 2*v.j.drawSize));

		if (v.j.withinAnalogRange(mousePos)) {//defined in joystick.h, basically if within the drawing of the boundaries
			drawText(round(v.r.truSpeed(3, v.j.analogX)), vec3(mousePos.X - 30, mousePos.Y + 50), vec3(1, 1, 1), 30);
			drawText(round(v.r.truSpeed(3, v.j.analogY)), vec3(mousePos.X + 30, mousePos.Y + 50), vec3(1, 1, 1), 30);
		}
	}
	if (s.SimRunning == s.PIDCTRL) drawText(round(v.r.PID_controller()), vec3(v.r.p.position.X*ppi, v.r.p.position.Y*ppi - 100), vec3(1, 1, 1), 30);
	if (s.SimRunning == s.TRUSPEED) {
		s.gr.graphPlot();//draws the graph
		s.gr.textOutput(&v);//draws the text for the graph
	}
	if (s.SimRunning == s.FIELD) {//when field button is pressed
		gl::pushModelView();//for drawing the field, had to be rotated based off source
			gl::translate(Vec3f(v.f.f.centre.X, v.f.f.centre.X, 0.0));//origin of rotation
			gl::rotate(-90);//easy rotation technique
			gl::draw(v.f.fieldBare, Area(-v.f.f.fieldSize*ppi/2, -v.f.f.fieldSize*ppi/2, v.f.f.fieldSize*ppi/2, v.f.f.fieldSize*ppi/2));
		gl::popModelView();//finish drawing the field
		drawRobot();
		for (int i = 0; i < v.f.mg.size(); i++) {
			vec3 RGB;//true color value because cinder uses values from 0->1 for their colours
			if (v.f.mg[i].col == 1)/*red mogo*/			RGB = vec3(217, 38, 38);
			else if (v.f.mg[i].col == 2)/*blue mogo*/	RGB = vec3(0, 64, 255);
				gl::color(RGB.X/255, RGB.Y/255, RGB.Z/255);
			gl::draw(v.f.MobileGoal, Area((v.f.f.fieldEnd)-(v.f.mg[i].pos.X*ppi) - (MGRad*ppi), (v.f.f.fieldEnd)-(v.f.mg[i].pos.Y*ppi) - (MGRad * ppi), (v.f.f.fieldEnd)-(v.f.mg[i].pos.X*ppi) + (v.f.mg[i].rad * ppi), (v.f.f.fieldEnd)-(v.f.mg[i].pos.Y*ppi) + (v.f.mg[i].rad * ppi)));
		}
		//drawing each individual cone. oh my
		for (int i = 0; i < v.f.c.size(); i++) {
			//fieldend for where the end of the field is, to subtract values because: http://vexcompetition.es/wp-content/uploads/2017/04/IntheZone-Field-specifications.pdf
			//+-(cRad*ppi) for sayin' that the point pos.X and pos.Y are the center, and the cRad*ppi is 3 inches RADIUS away from the center point
			gl::color(1, 1, 1);
			if (i != v.r.c.holding)	gl::draw(v.f.coneTexture, Area((v.f.f.fieldEnd) - (v.f.c[i].pos.X*ppi) - (cRad*ppi), (v.f.f.fieldEnd) - (v.f.c[i].pos.Y*ppi) - (cRad * ppi), (v.f.f.fieldEnd) - (v.f.c[i].pos.X*ppi) + (v.f.c[i].rad * ppi), (v.f.f.fieldEnd) - (v.f.c[i].pos.Y*ppi) + (v.f.c[i].rad * ppi)));
		}
		if(v.r.c.holding != -1 && v.r.c.holding < numCones) //if actually holding something (the index exists [!= -1])
			gl::draw(v.f.coneTexture, Area(
				(v.f.f.fieldEnd) - (v.f.c[v.r.c.holding].pos.X*ppi) - (cRad*ppi), 
				(v.f.f.fieldEnd) - (v.f.c[v.r.c.holding].pos.Y*ppi) - (cRad * ppi), 
				(v.f.f.fieldEnd) - (v.f.c[v.r.c.holding].pos.X*ppi) + (v.f.c[v.r.c.holding].rad * ppi), 
				(v.f.f.fieldEnd) - (v.f.c[v.r.c.holding].pos.Y*ppi) + (v.f.c[v.r.c.holding].rad * ppi)));
		robotDebug(&v, true);
		gl::color(1, 1, 1) ;
	}
	else drawRobot();
	/*****************************MISC**********************************/
	drawText(v.r.p.mRot, vec3(600, 40), vec3(1, 1, 1), 30);
	drawText(v.r.p.position.X, vec3(600, 140), vec3(1, 1, 1), 30);
	drawText(v.r.p.position.Y, vec3(750, 140), vec3(1, 1, 1), 30);
	drawText(v.r.p.acceleration.X, vec3(1000, 340), vec3(1, 1, 1), 30);
	drawText(v.r.p.acceleration.Y, vec3(1000, 440), vec3(1, 1, 1), 30);
	drawText(v.f.stacked.size(), vec3(1000, 500), vec3(1, 1, 1), 30);
	drawText(v.r.c.holding, vec3(1000, 700), vec3(1, 1, 1), 30);
	drawText(v.r.c.liftPos, vec3(1000, 800), vec3(1, 1, 1), 30);
	//drawText(v.f.HELLO, vec3(1000, 600), vec3(1, 1, 1), 30);
	if (v.r.c.liftDown) gl::drawString("YES", Vec2f(1000, 600), Color(1, 1, 1), Font("Arial", 30));
	else gl::drawString("NO", Vec2f(1000, 600), Color(1, 1, 1), Font("Arial", 30));
	/*drawing closest point for the 0th (first) cone*/
	gl::color(1, 0, 0);
	//gl::drawStrokedCircle(Vec2f(v.f.f.fieldEnd - v.f.pl[0].pos.X*ppi, v.f.f.fieldEnd - v.f.pl[0].pos.Y*ppi), 4*ppi);
	gl::color(1, 1, 1);
	//USER INTERFACE
	buttons();
	gl::drawString("FPS: ", Vec2f(getWindowWidth() - 150, 30), Color(0, 1, 0), Font("Arial", 30));
	drawText(getAverageFps(), vec3(getWindowWidth() - 90, 30), vec3(0, 1, 0), 30);
}
CINDER_APP_NATIVE(CimulationApp, RendererGl)
