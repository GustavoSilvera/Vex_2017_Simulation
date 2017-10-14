#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"
#include "cinder/Vector.h"
#include "cinder/Text.h"
#include "cinder/Font.h"
#include <ostream>
#include <fstream>
#include <filesystem>
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
std::ofstream scriptFile;

extern int numCones;
vec3 startPos;
vec3 mousePos;
class vex {
public:
	robot r;
	robot r2;
	tSpeed tS;
	PID pid;
	nav n;
	field f;
	joystick j;

	vex() : tS(&r), pid(&r), n(&r), f(&r, &r2){}
	bool debugText = true;
	bool recording = false;
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
	void clicky(int num_buttons);
	void buttons();
	void textDraw();
	void drawClaw(robot *r);
	void drawRobot(robot *r);
	void draw();
	vex v;
};
void CimulationApp::setup() {
	srand(time(NULL));//seeds random number generator
	//gl::enableVerticalSync();
	v.r.TankBase = gl::Texture(loadImage(loadAsset("Tank Drive.png")));
	v.r.CChanel = gl::Texture(loadImage(loadAsset("CChanelSmall.png")));
	v.r2.TankBase = gl::Texture(loadImage(loadAsset("Tank Drive.png")));
	v.r2.CChanel = gl::Texture(loadImage(loadAsset("CChanelSmall.png")));
	v.f.fieldBare = gl::Texture(loadImage(loadAsset("InTheZoneFieldBare.jpg")));
	v.f.coneTexture = gl::Texture(loadImage(loadAsset("InTheZoneCone.png")));
	v.f.MobileGoal = gl::Texture(loadImage(loadAsset("MoGoWhite.png")));
	setWindowSize(WindowWidth, WindowHeight);
	v.r2.p.position = vec3(117, 117, 0);
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
	if (event.getChar() == 'm' || event.getChar() == 'M') v.debugText = !v.debugText;
	if (event.getChar() == 'n' || event.getChar() == 'N') v.r.forwards(100);//works as of rn for ~1" 
	if (event.getChar() == 'B' || event.getChar() == 'b') v.r.rotate(-100);//works as of rn as ~1°
	if (event.getChar() == 'c') v.r.readScript();
	if (event.getChar() == 'q') v.r2.thinking = true;
}
void CimulationApp::keyUp(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_DOWN) v.r.ctrl.ArrowKeyDown = false;
	if (event.getCode() == KeyEvent::KEY_UP) v.r.ctrl.ArrowKeyUp = false;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r.ctrl.RotRight = false;
	if (event.getCode() == KeyEvent::KEY_LEFT) v.r.ctrl.RotLeft = false;
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r.c.liftUp = false;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r.c.liftDown = false;//left Z button
}
bool directlyInVerticalPath(robot *robit, vec3 pos) {//vertical lines
	vec3 origin = pos;//calculattes yintercepts for each cone relative to their position
	float x = 0;//finds the y intercept
	robit->db.slopeV = (robit->db.vertices[0].Y - robit->db.vertices[3].Y) / (robit->db.vertices[0].X - robit->db.vertices[3].X);//should be identical to slope[1] kinda redundant i guess
	robit->db.YintV[0] = robit->db.slopeV * (x - (robit->db.vertices[0].X - origin.X)) + (robit->db.vertices[0].Y - origin.Y);
	robit->db.YintV[1] = robit->db.slopeV * (x - (robit->db.vertices[1].X - origin.X)) + (robit->db.vertices[1].Y - origin.Y);
	//with the y intrcepts, checks if the y intercepts are not the same sign, thus the cone (origin) is between them
	return(getSign(robit->db.YintV[0]) != getSign(robit->db.YintV[1]));//works for telling me if between the two lines
}
void CimulationApp::update() {
	int pastRot = v.r.p.mRot;
	vec3 pastPos(v.r.p.position);
	float pastTime = ci::app::getElapsedSeconds();
	int signVX = getSign(v.r.p.velocity.X), signVY = getSign(v.r.p.velocity.Y), signRot = getSign(v.r.p.rotVel);
	v.j.getAnalog(mousePos);
	v.r.update();//calls robot update function
	v.r2.update();//calls enemy robot update function
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
		v.j.drawX = v.f.f.fieldSizeIn + 100;
		v.j.drawY = 500;
		v.j.drawSize = 150;
		v.pid.isInit = false;
		v.f.isInit = true;
		v.tS.isInit = false;
		v.n.isInit = false;
		v.f.FieldUpdate(&v.r, &v.r2);
		v.pid.pid.isRunning = false;
		v.r.moveAround(v.j.analogX, v.j.analogY);
		break;
	}
	if (v.r2.thinking) {
		int i = CONENUM;
		vec3 goal(v.r.p.position);//go to nth cone
		float d2V[4];
		for (int ver = 0; ver < 4; ver++) {
			d2V[ver] = goal.distance(v.r2.db.vertices[ver]);
		}
		int dir = 1;
		bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if goal is closer to the front side
		bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if goal is closer to the right side
		if (onRight) dir = -1;

		if (!directlyInVerticalPath(&v.r2, goal)) 
			v.r2.rotate(dir * MAXSPEED);
		else v.r2.rotate(0);
		if (v.r2.p.position.distance(goal) > v.r2.d.size && inFront) {
			v.r2.c.grabbing = false;
			v.r2.forwards(MAXSPEED + 50);
		}
		else {
			v.r2.c.grabbing = true;
			v.r2.forwards(0);
		}
	}
	v.r.db.distance += getSign(v.r.d.basePower)*v.r.p.position.distance(pastPos);
	v.r.db.rotDist += getSign(v.r.p.rotVel)*(v.r.p.mRot - pastRot);
	if (v.r.readyToRun) {
		enum action {
			ACTION_ROTATE,
			ACTION_FWDS
		};
		if (v.r.commands.size() > 0) {//make this more accriate
			float maxSpeed = 127;
			if (v.r.commands[0].amnt != 0) {//at least 
				if (v.r.commands[0].a == ACTION_ROTATE) {//for rotate
					if (abs(v.r.db.rotDist) <= 0.65*abs(v.r.commands[0].amnt)) {
						v.r.p.amountOfFriction = 10;//decreases uncontrolled rotation
						v.r.rotate(getSign(v.r.commands[0].amnt) * 127);
					}
					else {
						v.r.db.rotDist = RESET;
						v.r.p.amountOfFriction = 5;//resets to normal
						v.r.commands.erase(v.r.commands.begin());//removes first element of vector
					}
				}
				else if (v.r.commands[0].a == ACTION_FWDS) {//for fwds
					if (abs(v.r.db.distance) <= abs(v.r.commands[0].amnt)) {
						v.r.forwards(getSign(v.r.commands[0].amnt) * 127);
					}
					else {
						v.r.db.distance = RESET;
						v.r.commands.erase(v.r.commands.begin());//removes first element of vector
					}
				}
			}
			else v.r.commands.erase(v.r.commands.begin());
		}
	}

	if (v.recording) {//macro recording
		//less accurate (straight line running)
		if ((v.r.p.velocity.X == 0 && v.r.p.velocity.Y == 0) ||
			(getSign(v.r.p.velocity.X) != signVX &&
				getSign(v.r.p.velocity.Y) != signVY)) {//stopped or changed direction
			if (abs(((int)(v.r.db.distance * 100)) / 100) >= 0.01) {//but still moved 
				std::stringstream dummyText2;
				std::string distance;
				dummyText2 << (((int)(v.r.db.distance * 100)) / 100);
				dummyText2 >> distance;
				scriptFile << "driveFor( " + distance + ");\n";//used for scripting

				v.r.db.distance = RESET;//resets change in position after a while
			}
		}
		if ((int)pastRot != (int)v.r.p.mRot || getSign(v.r.p.rotVel) != signRot) {//rotation changed
			if (((int)(v.r.db.distance * 100)) / 100 != 0) {//difference in distance trav
				std::stringstream dummyText2;
				std::string distance;
				dummyText2 << (((int)(v.r.db.distance * 100)) / 100);
				dummyText2 >> distance;
				scriptFile << "driveFor( " + distance + ");\n";//used for scripting
				v.r.db.distance = RESET;//resets change in position after a while
			}
			if (((int)(v.r.db.rotDist * 100)) / 100 != 0) {//difference in rotation
				std::stringstream dummyText;
				std::string newAngle;
				dummyText << getSign(v.r.p.rotVel)*((int)(v.r.db.rotDist * 100)) / 100;//difference in angle
				dummyText >> newAngle;
				scriptFile << "rotFor( " + newAngle + ");\n";//used for scripting
				v.r.db.rotDist = RESET;//resets change in position after a while
			}
		}
	}
}
//for buttons
void CimulationApp::clicky(int AMOUNT_BUTTON) {//function for clicking the buttons
	for (int i = 0; i < AMOUNT_BUTTON; i++) {//for each button in the array 
		if (mousePos.X > 100 * (i + 1) - (50) + (25 * i) &&
			mousePos.X < 100 * (i + 1) + (50) + (25 * i) &&
			mousePos.Y > 25 && mousePos.Y < 75) {//within boundaries for each button based off their index
			s.hovering = i;
			if (s.mouseClicked && i <=3 ) {
				s.SimRunning = simulation::SimulationType(i);
			}
			else if (s.mouseClicked && i == 4) {
				v.recording = true;//toggles macro recording
				scriptFile = std::ofstream("script.txt");
			}
			else if (s.mouseClicked && i == 5) {
				v.recording = false;//toggles macro recording
				scriptFile = std::ofstream("script.txt", fstream::app);

			}
		}
	}
}
void CimulationApp::buttons() {//function for drawing the buttons
	#define BUTTON_AMOUNT 6//number of buttons
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
		else if (i == 4)gl::drawString("Macro", Vec2f(bX[i] - 35 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 29));
		else if (i == 5)gl::drawString("noRec", Vec2f(bX[i] - 35 + dInBtw*i, bY - 10), Color(1, 1, 1), Font("Arial", 29));
		clicky(BUTTON_AMOUNT);//function for if a button is being hovered of pressed
	}
}
void CimulationApp::textDraw() {//function for drawing the buttons 
	//(	WARNING: RESOURCE HOG!!!!!!!!!!!)
	const int dInBtw = 50;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	struct text{
		string s;
		float f;
	};
	text t[] = { 
		{ "Angle:", v.r.p.mRot},
		{ "X Pos:", v.r.p.position.X},
		{ "Y Pos:", v.r.p.position.Y},
		{ "X-Vel:", v.r.p.velocity.X},
		{ "Y-Vel:", v.r.p.velocity.X},
		{ "X-Acc:", v.r.p.acceleration.X},//same as y accel
		{ "Y-Acc:", v.r.p.acceleration.Y},//same as X accel
		{ "R-Vel:", v.r.p.rotVel},
		{ "R-Acc:", v.r.p.rotAcceleration},
		{ "L-Pos:", v.r.c.liftPos},
		{ "L-Acc:", v.r.p.rotAcceleration}
	};
	int i = 0;
	for (text& ti : t) {
		int tY = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(ti.s, Vec2f(tX - 70, tY), Color(1, 1, 1), Font("Arial", 30));
		drawText(ti.f, vec3I(tX, tY), vec3I(1, 1, 1), 30);
		++i;
	}
}

// Map robot coordinates to screen coordinates.
Vec2f R2S2(vec3 robot_coord) {
	const double fieldSizeIn = 141.05;
	return Vec2f(100 + ppi * robot_coord.X, 100 + fieldSizeIn * ppi - ppi * robot_coord.Y);
}
Vec3f R2S3(float robot_coordX, float robot_coordY, float robot_coordZ) {//for 3d coords, usually z is 0 coords
	const double fieldSizeIn = 141.05;
	return Vec3f(100 + ppi * robot_coordX, 100 + fieldSizeIn * ppi - ppi * robot_coordY, robot_coordZ);
}
Rectf R2S4(float p1X, float p1Y, float p2X, float p2Y) {//for rectangular coords
	const double fieldSizeIn = 141.05;
	return Rectf(100 + ppi * p1X, 100 + fieldSizeIn * ppi - ppi * p1Y, 100 + ppi * p2X, 100 + fieldSizeIn * ppi - ppi * p2Y);
}

void robotDebug(vex *v, bool reversed) {
	gl::color(1, 0, 0);
	for (int i = 0; i < 4; i++) {//simplified version of drawing the vertices
		gl::drawSolidCircle(R2S2(v->r.db.vertices[i]), 5 + i);
		//else gl::drawSolidCircle(Vec2f(ppi * v->r.db.vertices[i].X, ppi * v->r.db.vertices[i].Y), 5 + i);
	}
		//vertice rectangles
		gl::drawStrokedRect(Area(v->f.f.inFromEnd*ppi + v->r.db.vertices[0].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[0].Y*ppi, v->f.f.inFromEnd*ppi + v->r.db.vertices[1].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[1].Y*ppi));
		gl::drawStrokedRect(Area(v->f.f.inFromEnd*ppi + v->r.db.vertices[2].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[2].Y*ppi, v->f.f.inFromEnd*ppi +v->r.db.vertices[3].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[3].Y*ppi));
		//vertical lines
		gl::drawLine(cinder::Vec2f(v->f.f.inFromEnd*ppi +(v->r.db.vertices[1].X + 300 * cos((v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[1].Y + 300 * sin((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[2].X - 300 * cos((v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[2].Y - 300 * sin((v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f( v->f.f.inFromEnd*ppi + (v->r.db.vertices[0].X + 300 * cos((v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[0].Y + 300 * sin((v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[3].X - 300 * cos((v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[3].Y - 300 * sin((v->r.p.mRot) * PI / 180))*ppi));
		//horizontal lines
		gl::drawLine(cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[0].X + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[0].Y + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[1].X - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[1].Y - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi));
		gl::drawLine(cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[2].X + 300 * sin((-v->r.p.mRot) * PI / 180))*ppi,
			v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[2].Y + 300 * cos((-v->r.p.mRot) * PI / 180))*ppi),
			cinder::Vec2f(v->f.f.inFromEnd*ppi + (v->r.db.vertices[3].X - 300 * sin((-v->r.p.mRot) * PI / 180))*ppi,
				v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - (v->r.db.vertices[3].Y - 300 * cos((-v->r.p.mRot) * PI / 180))*ppi));
		//draw circle
		gl::drawStrokedCircle(Vec2f(v->f.f.inFromEnd*ppi + v->r.p.position.X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.p.position.Y*ppi), renderRad * v->r.d.size*ppi);
		gl::drawStrokedRect(Area(v->f.f.inFromEnd*ppi + v->r.db.vertices[0].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[0].Y*ppi, v->f.f.inFromEnd*ppi + v->r.db.vertices[2].X*ppi, v->f.f.inFromEnd*ppi + v->f.f.fieldSizeIn*ppi - v->r.db.vertices[2].Y*ppi));
		
		gl::drawSolidCircle(Vec2f(v->f.f.inFromEnd*ppi+ppi*(v->r.p.position.X + (v->r.d.size / 2) * cos((-v->r.p.mRot) * PI / 180) * sqrt(2)),
			v->f.f.inFromEnd*ppi+v->f.f.fieldSizeIn*ppi - ppi*(v->r.p.position.Y - (v->r.d.size / 2) * sin((-v->r.p.mRot) * PI / 180) * sqrt(2))), 5);
}
void CimulationApp::drawClaw(robot *r) {
	gl::draw(v.r.CChanel, Area((r->c.clawSize)*ppi, (r->d.size*.5 + r->c.baseSize)*ppi, (-r->c.clawSize)*ppi, (r->d.size*.5)*ppi));
	gl::color(222.0 / 225, 229.0 / 225, 34.0 / 225);
	gl::drawSolidRect(Area(Vec2d((r->c.clawPos + r->c.clawThick)*ppi, (r->d.size*.5 + r->c.clawHeight + r->c.baseSize)*ppi), Vec2d((r->c.clawPos - r->c.clawThick)*ppi, (r->d.size*.5 + r->c.baseSize)*ppi)));
	gl::drawSolidRect(Area(Vec2d((-r->c.clawPos - r->c.clawThick)*ppi, (r->d.size*.5 + r->c.clawHeight + r->c.baseSize)*ppi), Vec2d((-r->c.clawPos + r->c.clawThick)*ppi, (r->d.size*.5 + r->c.baseSize)*ppi)));
	gl::color(1, 1, 1);//reset colour
}
void CimulationApp::drawRobot(robot *r) {
	glPushMatrix();
	///robotDebug(&v, true);
	//had to modify the y because the origin is bottom left hand corner
	gl::translate(Vec3f(R2S3(r->p.position.X, r->p.position.Y, 0.0)));//origin of rotation
	gl::rotate(Vec3f(0, 0, -r->p.mRot - 90));//something for like 3D rotation.... ugh
	gl::color(1, 1, 1);
	gl::draw(r->TankBase, Area((-(r->d.size / 2))*ppi, (-(r->d.size / 2))*ppi, ((r->d.size / 2))*ppi, ((r->d.size / 2))*ppi));
	//mogo
	drawClaw(r);
	gl::color(66.0 / 255, 135.0 / 255, 224.0 / 255);
	gl::drawSolidRect(Area((-r->mg.clawPos - r->c.clawThick)*ppi, (-r->d.size*.5 - r->mg.clawHeight)*ppi, (-r->mg.clawPos + r->c.clawThick)*ppi, (-r->d.size*.5)*ppi));
	gl::drawSolidRect(Area((r->mg.clawPos + r->c.clawThick)*ppi, (-r->d.size*.5 - r->mg.clawHeight)*ppi, (r->mg.clawPos - r->c.clawThick)*ppi, (-r->d.size*.5)*ppi));

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
			v.j.drawX = 100 + v.f.f.fieldSizeIn*ppi + 20;
			v.j.drawY = getWindowHeight() / 2;
		}
		else {
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
		if (v.recording) {
			int size = 10;//pixels in which to draw the rectangles width
			gl::color(1, 0, 0);
			gl::drawSolidRect(Area(0, 0, size, getWindowHeight()));
			gl::drawSolidRect(Area(0, 0, getWindowWidth(), size));
			gl::drawSolidRect(Area(0, getWindowHeight() - size, getWindowWidth(), getWindowHeight()));
			gl::drawSolidRect(Area(getWindowWidth() - size, 0, getWindowWidth(), getWindowHeight()));
			gl::color(1, 1, 1);//reset to white
		}
		ci::gl::draw(v.f.fieldBare, ci::Area(v.f.f.inFromEnd*ppi, v.f.f.inFromEnd*ppi, v.f.f.inFromEnd*ppi + v.f.f.fieldSizeIn*ppi, v.f.f.inFromEnd*ppi + v.f.f.fieldSizeIn*ppi ));
		drawRobot(&v.r);//drawing robot 1
		drawRobot(&v.r2);//drawing oppposing robot
		gl::drawString("Score:", Vec2f(700, 50), Color(1, 1, 1), Font("Arial", 50));
		drawText(v.f.calculateScore(), vec3I(850, 50), vec3I(1, 1, 1), 50);

		for (int i = 0; i < v.f.mg.size(); i++) {
			vec3 RGB;//true color value because cinder uses values from 0->1 for their colours
			if (v.f.mg[i].colour == 1)/*red mogo*/			RGB = vec3(217, 38, 38);
			else if (v.f.mg[i].colour == 2)/*blue mogo*/	RGB = vec3(0, 64, 255);
			gl::color(RGB.X / 255, RGB.Y / 255, RGB.Z / 255);
			gl::draw(v.f.MobileGoal, Area(R2S4(
				(v.f.mg[i].pos.X - MGRad),
				(v.f.mg[i].pos.Y - MGRad),
				(v.f.mg[i].pos.X + v.f.mg[i].radius),
				(v.f.mg[i].pos.Y + v.f.mg[i].radius)))
				);
		}
		//drawing each individual cone. oh my
		for (int i = 0; i < v.f.c.size(); i++) {
			gl::color(1, 1, 1);
			if (i != v.r.c.holding)	gl::draw(v.f.coneTexture, Area(R2S4(
				(v.f.c[i].pos.X - v.f.c[i].radius),
				(v.f.c[i].pos.Y - v.f.c[i].radius),
				(v.f.c[i].pos.X + v.f.c[i].radius),
				(v.f.c[i].pos.Y + v.f.c[i].radius)))
			);
		}
		if (v.r.c.holding != -1 && v.r.c.holding < numCones) { //if actually holding something (the index exists [!= -1])
			float coneX = v.f.c[v.r.c.holding].pos.X;
			float coneY = v.f.c[v.r.c.holding].pos.Y;
			gl::draw(v.f.coneTexture, Area(R2S4(
				(coneX - v.f.c[v.r.c.holding].radius),
				(coneY - v.f.c[v.r.c.holding].radius),
				(coneX + v.f.c[v.r.c.holding].radius),
				(coneY + v.f.c[v.r.c.holding].radius)))
			);
		}
		for (int i = 0; i < v.f.mg.size(); i++) {//drawing how many are stacked
			if (v.f.mg[i].stacked.size() > 0) {
				drawText(v.f.mg[i].stacked.size(), vec3I(
					70 + (int)((v.f.mg[i].pos.X - cRad) * (int)ppi),
					50 + (int)((v.f.f.fieldSizeIn - v.f.mg[i].pos.Y + cRad) * (int)ppi)),
					vec3I(1, 1, 1), 50);
			}
		}
		for (int i = 0; i < v.f.pl.size(); i++) {//drawing how many are stacked
			if (v.f.pl[i].stacked.size() > 0) {
				drawText(v.f.pl[i].stacked.size(), vec3I(
					70 + (int)((v.f.pl[i].pos.X - cRad) * (int)ppi),
					50 + (int)((v.f.f.fieldSizeIn - v.f.pl[i].pos.Y + cRad) * (int)ppi)),
					vec3I(1, 1, 1), 50);
			}
		}
		gl::color(1, 1, 1);
	}
	else drawRobot(&v.r);
	
	gl::color(1, 0, 0);
	gl::drawSolidCircle(R2S2(vec3(v.f.c[CONENUM].pos.X, v.f.c[CONENUM].pos.Y) ), 5);
	if (v.recording) gl::drawString("YES", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));
	else gl::drawString("NO", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));
	//drawText(v.f.f.twentyPoint[0].size(), vec3I(1010, 660), vec3I(1, 1, 1), 30);
	drawText(v.r.db.distance, vec3I(1010, 500), vec3I(1, 1, 1), 30);
	drawText(v.r.db.rotDist, vec3I(1010, 400), vec3I(1, 1, 1), 30);
	drawText(ci::app::getElapsedSeconds(), vec3I(1010, 200), vec3I(1, 1, 1), 30);

	gl::color(1, 1, 1);
	//USER INTERFACE
	buttons();
	gl::drawString("FPS: ", Vec2f(getWindowWidth() - 150, 30), Color(0, 1, 0), Font("Arial", 30));
	drawText(getAverageFps(), vec3I(getWindowWidth() - 90, 30), vec3I(0, 1, 0), 30);
	if(v.debugText) textDraw();//dont run on truspeed sim, unnecessary
}
CINDER_APP_NATIVE(CimulationApp, RendererGl)
