#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"
#include "cinder/Vector.h"
#include "cinder/Text.h"
#include "cinder/Font.h"
#include <ostream>
#include <fstream>
#include <vector>
#include <filesystem>
//my own headers
#include "joystick.h"
#include "field.h"
#include "PID.h"
#include "Custom.h"
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
	std::vector<robot> r;
	tSpeed tS;
	PID pid;
	//customize c;
	field f;
	joystick j;
	vex() : 
		r(3), 
		tS(&r[0]), 
		pid(&r[0]), 
		f(&r){}
	bool debugText = true;
	bool recording = false;
	float scalar;
};
//begin
int tX = 1200;
struct simulation {
	bool mouseClicked = false;
	int hovering = 1;//which button is being hovered over. if any
	enum SimulationType {
		PIDCTRL,
		CUSTOMIZE,
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
	void clicky(int num_buttons, int size);
	void buttons(int size);
	void textDraw();
	//for auto robots
	void goGrab(robot *r, field::element *e, int coneIndex, int roboIndex);
	void stackOn(robot *r, field::element *e);
	void placeIn(robot *r, field::fence::zone *z);
	void reRoute(robot *r, field::element *e, int dir);
	//for customize panel
	void controlPanel(robot *r);
	void callAction(bool increase, int buttonAction);
	bool buttonHover(int x, int y, int x2, int y2, int index, int buttonAction);
	void ctrlButton(int x, int y, int x2, int y2, float scalar, int buttonAction);
	struct customizePanel {
		float size = 18;
		float motorPower = MAXSPEED;//power for the base
	};
	customizePanel cp;
	//for drawing stuff
	Vec2f R2S2(vec3 robot_coord);
	Vec3f R2S3(float robot_coordX, float robot_coordY, float robot_coordZ);
	Rectf R2S4(float p1X, float p1Y, float p2X, float p2Y);
	void robotDebug();
	void drawClaw(robot *r);
	void drawRobot(robot *r);
	void draw();
	vex v;
};
void CimulationApp::setup() {
	srand(time(NULL));//seeds random number generator
	//gl::enableVerticalSync();
	for (int rob = 0; rob < v.r.size(); rob++) {
		v.r[rob].TankBase = gl::Texture(loadImage(loadAsset("Tank Drive.png")));
		v.r[rob].TankBase2 = gl::Texture(loadImage(loadAsset("Tank Drive WheelSpin.png")));
		v.r[rob].CChanel = gl::Texture(loadImage(loadAsset("CChanelSmall.png")));
		v.r[rob].CChanelVERT = gl::Texture(loadImage(loadAsset("CChanelSmallVERT.png")));
	}
	v.f.fieldBare = gl::Texture(loadImage(loadAsset("InTheZoneFieldBare.jpg")));
	v.f.coneTexture = gl::Texture(loadImage(loadAsset("InTheZoneCone.png")));
	v.f.MobileGoal = gl::Texture(loadImage(loadAsset("MoGoWhite.png")));
	setWindowSize(WindowWidth, WindowHeight);
	v.scalar = (float)getWindowWidth() / (float)WindowWidth;
}
//cinder::functions
void CimulationApp::mouseDown(MouseEvent event) {
	if (event.isLeft()) s.mouseClicked = true;
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
				v.tS.activate(&v.r[0], &v.j);
			}
		}
	}
}
void CimulationApp::keyDown(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_UP)	v.r[0].ctrl.ArrowKeyUp = true;
	if (event.getCode() == KeyEvent::KEY_DOWN)	v.r[0].ctrl.ArrowKeyDown = true;
	if (event.getCode() == KeyEvent::KEY_LEFT)	v.r[0].ctrl.RotLeft = true;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r[0].ctrl.RotRight = true;
	if (event.getChar() == 'e' || event.getChar() == 'E') v.r[0].c.grabbing = !v.r[0].c.grabbing;//if want toggling, else look at a while ago
	if (event.getChar() == 'r' || event.getChar() == 'R') v.r[0].mg.grabbing = !v.r[0].mg.grabbing;//if want toggling, else look at a while ago
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r[0].c.liftUp = true;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r[0].c.liftDown = true;//left Z button
	if (event.getChar() == 'p' || event.getChar() == 'P') v.pid.pidVel = !v.pid.pidVel;//right P button
	if (event.getChar() == 'o' || event.getChar() == 'O') v.pid.reset(&v.r[0]);
	if (event.getChar() == 'm' || event.getChar() == 'M') v.debugText = !v.debugText;
	if (event.getChar() == 'n' || event.getChar() == 'N') v.r[0].forwards(100);//works as of rn for ~1" 
	if (event.getChar() == 'B' || event.getChar() == 'b') v.r[0].rotate(-100);//works as of rn as ~1°
	if (event.getChar() == 'c') v.r[0].readScript();
	if (event.getChar() == 'q') { 
		for (int rob = 1; rob < v.r.size(); rob++) {
			v.r[rob].thinking = true;
		}
	}
}
void CimulationApp::keyUp(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_DOWN) v.r[0].ctrl.ArrowKeyDown = false;
	if (event.getCode() == KeyEvent::KEY_UP) v.r[0].ctrl.ArrowKeyUp = false;
	if (event.getCode() == KeyEvent::KEY_RIGHT) v.r[0].ctrl.RotRight = false;
	if (event.getCode() == KeyEvent::KEY_LEFT) v.r[0].ctrl.RotLeft = false;
	if (event.getCode() == KeyEvent::KEY_SPACE) v.r[0].c.liftUp = false;
	if (event.getChar() == 'z' || event.getChar() == 'Z') v.r[0].c.liftDown = false;//left Z button
}
void CimulationApp::update() {
	v.scalar = ((float)getWindowWidth() + (float)getWindowHeight()) / (float)(WindowWidth+WindowHeight);
	int pastRot = v.r[0].p.mRot;
	vec3 pastPos(v.r[0].p.position);
	float pastTime = ci::app::getElapsedSeconds();
	int signVX = getSign(v.r[0].p.velocity.X), signVY = getSign(v.r[0].p.velocity.Y), signRot = getSign(v.r[0].p.rotVel);
	v.j.getAnalog(mousePos);
	for (int rob = 0; rob < v.r.size(); rob++) {
		v.r[rob].update();//calls robot update function
	}
	switch (s.SimRunning) {
	case simulation::CUSTOMIZE:
		v.r[0].d.size = cp.size;
		v.r[0].d.motorSpeed = cp.motorPower;

		tX = 1200;
		v.j.drawX = 300 * v.scalar;
		v.j.drawY = 300 * v.scalar;
		v.j.drawSize = 150 * v.scalar;
		v.pid.isInit = false;
		v.f.isInit = false;
		v.tS.isInit = false;
		v.r[0].moveAround(v.j.analogX, v.j.analogY);
		v.pid.pid.isRunning = false;
		break;
	case simulation::PIDCTRL:
		tX = 1200;
		v.j.drawX = 600 * v.scalar;
		v.j.drawY = 500 * v.scalar;
		v.j.drawSize = 150 * v.scalar;
		v.pid.isInit = true;
		v.f.isInit = false;
		v.tS.isInit = false;
		v.pid.PIDUpdate(&v.r[0]);
		break;
	case simulation::TRUSPEED:
		tX = 1400;
		v.j.drawX = 300*v.scalar;
		v.j.drawY = 300 * v.scalar;
		v.j.drawSize = 300 * v.scalar;
		v.pid.isInit = false;
		v.f.isInit = false;
		v.tS.isInit = true;
		v.tS.TruSpeedUpdate(&v.r[0]);
		v.pid.pid.isRunning = false;
		break;
	case simulation::FIELD:
		tX = 1200;
		v.j.drawX = v.scalar*(ppi*(v.f.f.fieldSizeIn + v.f.f.inFromEnd)+50);
		v.j.drawY = 650 * v.scalar;
		v.j.drawSize = 150 * v.scalar;
		v.pid.isInit = false;
		v.f.isInit = true;
		v.tS.isInit = false;
		v.f.FieldUpdate(&v.r);
		v.pid.pid.isRunning = false;
		v.r[0].moveAround(v.j.analogX, v.j.analogY);
		break;
	}
	for (int rob = 0; rob < v.r.size(); rob++) {//starts at robot 1
		if (v.r[rob].c.holding != v.r[rob].goalCone) {//dynamically refreshes which cone is best in position to be picked up
													  //if(v.r[rob].goalCone == 0)//used to be for waiting for cone, sets definitive target FOREVER, not the best
			int closest = 0;//assumes cone 0 is closest
			for (int i = 0; i < v.f.c.size() - 1; i++) {
				if (v.f.c[i].pos.distance(v.f.pl[0].pos) > v.f.c[i].radius * 4 && v.f.c[i].pos.distance(v.f.pl[1].pos) > v.f.c[i].radius * 4) {
					if (v.r[rob].p.position.distance(v.f.c[i].pos) < v.r[rob].p.position.distance(v.f.c[closest].pos)) {
						/*float d2V[4];//can comment this stuff to get nearest cone. but better to get nearest cone IN FRONT (SLIGHTLY better score wise/time), gets stuck sometimes
						for (int ver = 0; ver < 4; ver++) {
							d2V[ver] = v.f.c[i].pos.distance(v.r[rob].db.vertices[ver]);
						}
						bool inFront = (d2V[0] + d2V[1] < d2V[3] + d2V[3]);//checking if goalCone[rob-1] is closer to the front side
						*/if (v.f.c[i].pos.Z <= cHeight /*&& inFront*/)//so long as not already stacked or in the air
							closest = i;//updates "closest" to whichever cone is closest
					}
				}
			}
			v.r[rob].goalCone = closest;
		}
		if (v.r[rob].mg.holding != v.r[rob].goalMogo) {//dynamically refreshes which cone is best in position to be picked up
			int closest = 0;//assumes cone 0 is closest
			for (int i = 0; i < v.f.mg.size(); i++) {
				if (v.r[rob].p.position.distance(v.f.mg[i].pos) < v.r[rob].p.position.distance(v.f.mg[closest].pos)) {
					if (v.f.mg[i].pos.Z <= cHeight)//so long as not already stacked or in the air
						closest = i;//updates "closest" to whichever cone is closest
				}
			}
			
			v.r[rob].goalMogo = closest;
		}
		if (v.r[rob].thinking) {
			if (v.r[rob].grabMoGo) {//aiming for mogo
				if (v.r[rob].mg.holding != v.r[rob].goalMogo + 100 ) {
					goGrab(&v.r[rob], &v.f.mg[v.r[rob].goalMogo], v.r[rob].goalMogo, rob);
				}
				else {
					placeIn(&v.r[rob], v.f.f.z);
				}
			}
			else {//aiming for cones
				if (v.r[rob].c.holding != v.r[rob].goalCone) {
					goGrab(&v.r[rob], &v.f.c[v.r[rob].goalCone], v.r[rob].goalCone, rob);
				}
				else {
					int poleNum = 0;//assuming robot is closer to pole0 than pole1
					if (v.r[rob].p.position.distance(v.f.pl[0].pos) > v.r[rob].p.position.distance(v.f.pl[1].pos)) {
						poleNum = 1;//robot is closer to pole1 than pole0
					}
					//always stacks on pole 1
					/*int mogoNum = 0;
					for (int i = 0; i < v.f.mg.size(); i++) {
						if (v.f.mg[i].colour == 2) {
							if (v.r[rob].p.position.distance(v.f.mg[mogoNum].pos) > v.r[rob].p.position.distance(v.f.mg[i].pos))
								mogoNum = i;
						}
					}*/
					stackOn(&v.r[rob], &v.f.pl[poleNum]);
				}
			}
		}
		v.r[0].db.distance += getSign(v.r[0].d.basePower)*v.r[0].p.position.distance(pastPos);
		v.r[0].db.rotDist += getSign(v.r[0].p.rotVel)*(v.r[0].p.mRot - pastRot);
		if (v.r[0].readyToRun) {
			enum action {
				ACTION_ROTATE,
				ACTION_FWDS
			};
			if (v.r[0].commands.size() > 0) {//make this more accriate
				float maxSpeed = 127;
				if (v.r[0].commands[0].amnt != 0) {//at least 
					if (v.r[0].commands[0].a == ACTION_ROTATE) {//for rotate
						if (abs(v.r[0].db.rotDist) <= 0.65*abs(v.r[0].commands[0].amnt)) {
							v.r[0].p.amountOfFriction = 10;//decreases uncontrolled rotation
							v.r[0].rotate(getSign(v.r[0].commands[0].amnt) * 127);
						}
						else {
							v.r[0].db.rotDist = RESET;
							v.r[0].p.amountOfFriction = 5;//resets to normal
							v.r[0].commands.erase(v.r[0].commands.begin());//removes first element of vector
						}
					}
					else if (v.r[0].commands[0].a == ACTION_FWDS) {//for fwds
						if (abs(v.r[0].db.distance) <= abs(v.r[0].commands[0].amnt)) {
							v.r[0].forwards(getSign(v.r[0].commands[0].amnt) * 127);
						}
						else {
							v.r[0].db.distance = RESET;
							v.r[0].commands.erase(v.r[0].commands.begin());//removes first element of vector
						}
					}
				}
				else v.r[0].commands.erase(v.r[0].commands.begin());
			}

		}
	}
	if (v.recording) {//macro recording		//less accurate (straight line running)
		if ((v.r[0].p.velocity.X == 0 && v.r[0].p.velocity.Y == 0) ||
			(getSign(v.r[0].p.velocity.X) != signVX &&
				getSign(v.r[0].p.velocity.Y) != signVY)) {//stopped or changed direction
			if (abs(((int)(v.r[0].db.distance * 100)) / 100) >= 0.01) {//but still moved 
				std::stringstream dummyText2;
				std::string distance;
				dummyText2 << (((int)(v.r[0].db.distance * 100)) / 100);
				dummyText2 >> distance;
				scriptFile << "driveFor( " + distance + ");\n";//used for scripting

				v.r[0].db.distance = RESET;//resets change in position after a while
			}
		}
		if ((int)pastRot != (int)v.r[0].p.mRot || getSign(v.r[0].p.rotVel) != signRot) {//rotation changed
			if (((int)(v.r[0].db.distance * 100)) / 100 != 0) {//difference in distance trav
				std::stringstream dummyText2;
				std::string distance;
				dummyText2 << (((int)(v.r[0].db.distance * 100)) / 100);
				dummyText2 >> distance;
				scriptFile << "driveFor( " + distance + ");\n";//used for scripting
				v.r[0].db.distance = RESET;//resets change in position after a while
			}
			if (((int)(v.r[0].db.rotDist * 100)) / 100 != 0) {//difference in rotation
				std::stringstream dummyText;
				std::string newAngle;
				dummyText << getSign(v.r[0].p.rotVel)*((int)(v.r[0].db.rotDist * 100)) / 100;//difference in angle
				dummyText >> newAngle;
				scriptFile << "rotFor( " + newAngle + ");\n";//used for scripting
				v.r[0].db.rotDist = RESET;//resets change in position after a while
			}
		}
	}
}
//for buttons
void CimulationApp::goGrab(robot *r, field::element *e, int index, int roboIndex) {
	float d2V[4];
	for (int ver = 0; ver < 4; ver++) {
		d2V[ver] = e->pos.distance(r->db.vertices[ver]);
	}
	int dir = 1;
	if (!r->reRouting) {
		float speed = limitTo(MAXSPEED, 3*r->p.position.distance(e->pos));
		bool inFront = ((d2V[0] + d2V[1]) < (d2V[2] + d2V[3]));//checking if goalCone[rob-1] is closer to the front side
		bool onRight = ((d2V[1] + d2V[2]) < (d2V[0] + d2V[3]));//checking if goalCone[rob-1] is closer to the right side
		if (onRight) dir = -1;
		if (r->grabMoGo) {
			if (!e->inPossession[roboIndex] && r->p.position.distance(e->pos) >= (r->d.size / 2 +1 + e->radius)) {
				if(r->mg.holding == -1) r->mg.grabbing = true;//pulls out mogo-inator
				r->rotate(0);
				if (!r->directlyInPath(true, r->d.size / 4, e->pos)) {
					r->rotate(dir * speed);
					r->forwards(0);
				}
				else r->forwards(speed);
			}
			else {
				r->mg.holding = index + 100;
				r->mg.grabbing = false;//only closes claw if on same level (height wise)
				r->forwards(0);
				r->rotate(0);
			}
			if (r->p.velocity.X == 0 && r->p.velocity.Y == 0) {//get it to do the same if touching fence
				r->reRouting = true;
			}
		}
		else {
			if (!r->directlyInPath(true, r->d.size / 2, e->pos) || !inFront) {//angle is not pointing towards goal
				r->rotate(dir * speed);
				r->forwards(0);
			}
			else r->rotate(0);
			float offset = 0.5;//dosent update fast enough for small cones, needed little offset heuristic
			if (r->p.position.distance(e->pos) > (r->d.size / 2 + e->radius) + offset && inFront) {//drive fwds towards goalCone[rob-1]
				r->c.grabbing = false;
				r->forwards(speed);
			}
			else {
				if (abs(r->c.liftPos - e->pos.Z) < e->height) r->c.grabbing = true;//only closes claw if on same level (height wise)
				r->forwards(0);
			}
			if (r->c.grabbing && r->c.holding == index) {//holding the cone
				if (r->c.liftPos < v.f.pl[1].height + 4) r->c.liftUp = true;
				else r->c.liftUp = false;
			}
			else {
				r->c.liftUp = false;
				r->rotate(dir * 50);//just do a simple rotation to try and minimize error, gets it closer to the center 
			}
			if (r->c.holding == -1) { // not holding the cone
				if (r->c.liftPos > 0) r->c.liftDown = true;
				else r->c.liftDown = false;
			}
			else {
				r->c.liftDown = false;
			}
		}
		if (r->p.velocity.X == 0 && r->p.velocity.Y == 0) {//get it to do the same if touching fence
			r->reRouting = true;
		}
	}
	else {
		reRoute(r, &v.f.c[r->goalCone], dir);
	}
}
void CimulationApp::reRoute(robot *r, field::element *e, int dir) {
	/*
	float d2V[4];
	for (int ver = 0; ver < 4; ver++) {
		d2V[ver] = (int)e->pos.distance(r->db.vertices[ver]);
	}
	int direct = 1;
	bool inFront = ((d2V[0] + d2V[1]) < (d2V[2] + d2V[3]));//checking if goalCone[rob-1] is closer to the front side
	bool onRight = ((d2V[1] + d2V[2] < 1.2*d2V[0] + d2V[3]) && !inFront);//checking if goalCone[rob-1] is closer to the right side
	if (onRight) direct = -1;
	*/

	int poleNum = 0;//assuming robot is closer to pole0 than pole1
	if (r->p.position.distance(v.f.pl[0].pos) > r->p.position.distance(v.f.pl[1].pos)) {
		poleNum = 1;//robot is closer to pole1 than pole0
	}
	if (r->p.position.distance(v.f.pl[poleNum].pos) < (r->d.size)) {//far enough out of the way
		r->forwards(-127);
	}
	else {//fix this little "heuristic" make it actually detect when theres an obstacle before it and turn around it
		if (r->directlyInPath(true, r->d.size/4, v.f.c[rand() % v.f.c.size()].pos)) r->rotate(dir * MAXSPEED);//moving to horizontal path (turning like 90°)
		else r->reRouting = false;
		r->forwards(0);
		//v.reRouting = false;
	}
}
void CimulationApp::placeIn(robot *r, field::fence::zone *z) {
	/*vec3 zone;
	if(z[1].twentyPoint.size() < 1) zone = vec3(127, 127);
	else if (z[1].tenPoint.size() < 1) zone = vec3(120, 120);
	else zone = vec3(108, 108);*/
	float d2V[4];
	int dir = 1;
	for (int ver = 0; ver < 4; ver++) {
		d2V[ver] = vec3(140, 140).distance(r->db.vertices[ver]);
	}
	bool inFront = ((d2V[0] + d2V[1]) < (d2V[2] + d2V[3]));//checking if goalCone[rob-1] is closer to the front side
	bool onRight = ((d2V[1] + d2V[2]) < (d2V[0] + d2V[3]));//checking if goalCone[rob-1] is closer to the right side
	if (onRight) dir = -1; 
	if (!r->reRouting) {
		float speed = limitTo(MAXSPEED, 3 * r->p.position.distance(vec3(140, 140)));
		if (abs(r->p.position.Y - v.f.f.poleEquation(140.05, 117.5, -1, r->p.position.X))>15) {
			if (!r->directlyInPath(true, r->d.size / 2, vec3(140, 140)) || !inFront) {//angle is not pointing towards goalCone[rob-1]
				r->rotate(dir * speed);
				r->forwards(0);
			}
			else {
				r->rotate(0);
				r->forwards(dir * speed);
			}
		}
		else {
			//r->forwards(0);
			r->mg.grabbing = true;//lets go of mogo
			if (r->mg.protrusion > 7) {//not quite at the ground yet
				r->forwards(-MAXSPEED);//get outta there
			}
		}

	}
	else {//stop moving
		r->forwards(0);
		r->rotate(0);
	}
}
void CimulationApp::stackOn(robot *r, field::element *e) {
	float d2V[4];
	for (int ver = 0; ver < 4; ver++) {
		d2V[ver] = e->pos.distance(r->db.vertices[ver]);
	}
	int dir = 1;
	bool inFront = (d2V[0] + d2V[1] < d2V[2] + d2V[3]);//checking if goalCone[rob-1] is closer to the front side
	bool onRight = (d2V[1] + d2V[2] < d2V[0] + d2V[3]);//checking if goalCone[rob-1] is closer to the right side
	if (onRight) dir = -1;
	if (!r->reRouting) {
		float speed = limitTo(MAXSPEED, 3*r->p.position.distance(e->pos));
		if (!r->directlyInPath(true, r->d.size / 2, e->pos) || !inFront)//angle is not pointing towards goalCone[rob-1]
			r->rotate(dir * speed);
		else r->rotate(0);
		if (r->c.grabbing) {//holding the cone
			if (r->c.liftPos < e->height + e->stacked.size() + 4) r->c.liftUp = true;
			else r->c.liftUp = false;
		}
		else {
			r->c.liftUp = false;
			r->rotate(dir * 50);//just do a simple smaller rotation to try and minimize error, gets it closer to the center 
		}
		if (r->c.liftPos >= e->height + 2 /*+ADD STACKED POS CHANGER HERE*/) {//wait until lift is reasonably high
			if (r->p.position.distance(e->pos) > r->d.size*0.5 + e->radius + 2 && inFront) {//drive fwds towards goalCone[rob-1]
				r->forwards(speed*0.7);//slower since carrying object? eh idk
			}
			else {
				r->forwards(0);
				if (r->p.position.distance(e->pos) <= r->d.size*0.5 + e->radius + 2) {//reall close to the goalCone[rob-1]
					if (r->c.liftPos >= e->height && r->c.grabbing) {
						r->rotate(0);
						r->p.acceleration = vec3(0, 0, 0);
						r->p.velocity = vec3(0, 0, 0);
						r->p.rotAcceleration = 0;
						r->p.rotVel = 0;
						if (r->directlyInPath(true, r->d.size / 4, e->pos)) {
							r->c.grabbing = false;//opens claw
							//r->thinking = false;//basically turns off autonomous thing
							r->goalCone = 0;
						}
						else r->rotate(dir * 127);//just do a simple smaller rotation to try and minimize error, gets it closer to the center 
					}
				}
				//else r->rotate(dir * 127);//just do a simple smaller rotation to try and minimize error, gets it closer to the center 
			}
		}
		else {//stop moving
			r->forwards(0);
			r->rotate(0);
		}
	}
	else {//stop moving
		r->forwards(0);
		r->rotate(0);
	}
}
void CimulationApp::clicky(int AMOUNT_BUTTON, int buttonSize) {//function for clicking the buttons
	for (int i = 0; i < AMOUNT_BUTTON; i++) {//for each button in the array 
		if (mousePos.X > v.scalar * ( buttonSize* (i + 1) - (50) + (25 * i)) &&
			mousePos.X < v.scalar * (buttonSize* (i + 1) + (50) + (25 * i)) &&
			mousePos.Y > v.scalar *25 && mousePos.Y < v.scalar * 75) {//within boundaries for each button based off their index
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
void CimulationApp::buttons(int buttonSize) {//function for drawing the buttons
	int bY = 50, dInBtw = 25;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	struct text {
		string s;
	};
	text t[] = {
		{ "PIDctrl:"},
		{ "CustomR:" },
		{ "TRUSped:" },
		{ "AutoSim:" },
		{ "InitRec:" },
		{ "StopRec:" }
	};
	int i = 1;
	//use str.length() to get number of chars and dynamically push back other things
	for (text& ti : t) {
		if (i-1 == s.SimRunning) { gl::color(0, 1, 0); }//if the button's index is equal to whichever button's index is being hovered over
		else if (i-1 == s.hovering) { gl::color(1, 0, 0); }//if the button's index is equal to whichever button's index is being hovered over
		else { gl::color(1, 1, 1); }
		gl::drawStrokedRoundedRect(Area(v.scalar *(i* buttonSize - 50 + dInBtw*i), v.scalar*(bY - 25), v.scalar*(i*buttonSize + 50 + dInBtw*i), v.scalar*( bY + 25)), 5);//ROUNDED rectangle with corner rad of 7
		gl::color(1, 1, 1);//resets colour 
		gl::drawString(t[i-1].s, Vec2f(v.scalar*(i * buttonSize - t[i-1].s.length()*5 + dInBtw*i), v.scalar*(bY - 12.5)), Color(1, 1, 1), Font("Arial", v.scalar * 25));
		clicky(6, buttonSize);//function for if a button is being hovered of pressed
		i++;
	}
}
void CimulationApp::textDraw() {//function for drawing the buttons 
	//(	WARNING: RESOURCE HOG!!!!!!!!!!!)
	const int dInBtw = 50;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	struct text{
		string s;
		double f;
	};
	text t[] = { 
		{ "Angle:", v.r[0].p.mRot},
		{ "X Pos:", v.r[0].p.position.X},
		{ "Y Pos:", v.r[0].p.position.Y},
		{ "X-Vel:", v.r[0].p.velocity.X},
		{ "Y-Vel:", v.r[0].p.velocity.X},
		{ "X-Acc:", v.r[0].p.acceleration.X},//same as y accel
		{ "Y-Acc:", v.r[0].p.acceleration.Y},//same as X accel
		{ "R-Vel:", v.r[0].p.rotVel},
		{ "R-Acc:", v.r[0].p.rotAcceleration},
		{ "L-Pos:", v.r[0].c.liftPos},
		{ "L-Acc:", v.r[0].p.rotAcceleration}
	};
	int i = 0;
	for (text& ti : t) {
		int tY = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(ti.s, Vec2f(v.scalar*(tX - 70), v.scalar*(tY)), Color(1, 1, 1), Font("Arial", v.scalar * 30));
		drawText(ti.f, vec3I(v.scalar*(tX), v.scalar*(tY)), vec3I(1, 1, 1), v.scalar * 30);
		++i;
	}
}
void CimulationApp::callAction(bool increase, int buttonAction) {
	if (buttonAction == 0) {
		if (increase) cp.size += 0.1;
		else cp.size -= 0.1;
	}
	else if (buttonAction == 1) {
		if (increase) cp.motorPower++;
		else cp.motorPower--;
	}
}
bool CimulationApp::buttonHover(int x, int y, int x2, int y2, int index, int buttonAction) {
	if (mousePos.X > (x) &&
		mousePos.X < (x2) &&
		mousePos.Y > (y) && 
		mousePos.Y < (y2)) {//within boundaries for each button based off their index
		//mouse is hovering
		if (s.mouseClicked == true){
			if(index == 0) {//left button
				callAction(false, buttonAction);
			}
			else if (index == 1) {//right button
				callAction(true, buttonAction);
			}
		}
		return true;
	}
	return false;
}
void CimulationApp::ctrlButton(int x, int y, int x2, int y2, float scalar, int buttonAction) {
	Color(1, 0, 0);
	int sizeOf = (x2 - x);
	x *= scalar; y *= scalar; x2 *= scalar; y2 *= scalar;//gets scaling out of the way
	float dInBtwn = sizeOf*1.15;
	for (int i = 0; i < 2; i++) {
		if (buttonHover(x + i*dInBtwn, y, x2 + i*dInBtwn, y2, i, buttonAction))
			gl::color(1, 0, 0);//if the button's index is equal to whichever button's index is being hovered over
		else  gl::color(1, 1, 1); 
		gl::drawStrokedRoundedRect(Area(x + i*dInBtwn, y, x2 + i*dInBtwn, y2), 5);//ROUNDED rectangle with corner rad of 7
		if (i == 0) gl::drawString("<--", Vec2f(0.5*(x + x2) - 20 + i*dInBtwn, 0.5*(y + y2) - 10), Color(1, 1, 1), Font("Arial", scalar * 30));
		else gl::drawString("-->", Vec2f(0.5*(x + x2) - 20 + i*dInBtwn, 0.5*(y + y2) - 10), Color(1, 1, 1), Font("Arial", scalar * 30));
	}
}
void CimulationApp::controlPanel(robot *r) {//function for drawing the buttons 
	const int dInBtw = 50;//array for #buttons, bY is y position of each btn, dInBtw is distance in bwtween buttons
	struct text {
		string s;
		double f;
	};
	text t[] = {
		{ "Size:", cp.size },
		{ "Power:", cp.motorPower }
	};
	int i = 0;
	//use str.length() to get number of chars and dynamically push back other things
	for (text& ti : t) {
		int tY = (i + 1) * dInBtw;//increment x position for each button based off index
		gl::drawString(ti.s, Vec2f(v.scalar*(tX - ti.s.length()*17), v.scalar*(tY)), Color(1, 1, 1), Font("Arial", v.scalar * 40));
		drawText(ti.f, vec3I(v.scalar*(tX), v.scalar*(tY)), vec3I(1, 1, 1), v.scalar * 40);
		int buttonStart = tX + ti.s.length() * 10;
		ctrlButton(buttonStart, tY, buttonStart + 60, tY + 30, v.scalar, i);
		++i;
	}
}
// Map robot coordinates to screen coordinates.
Vec2f CimulationApp::R2S2(vec3 robot_coord) {
	return Vec2f(ppi * v.scalar * (v.f.f.inFromEnd + robot_coord.X), ppi * v.scalar * (v.f.f.inFromEnd + v.f.f.fieldSizeIn - robot_coord.Y) );
}
Vec3f CimulationApp::R2S3(float robot_coordX, float robot_coordY, float robot_coordZ) {//for 3d coords, usually z is 0 coords
	return Vec3f(ppi * v.scalar * (v.f.f.inFromEnd + robot_coordX), ppi * v.scalar * (v.f.f.inFromEnd + v.f.f.fieldSizeIn - robot_coordY), robot_coordZ);
}
Rectf CimulationApp::R2S4(float p1X, float p1Y, float p2X, float p2Y) {//for rectangular coords
	return Rectf(ppi * v.scalar * (v.f.f.inFromEnd + p1X), ppi * v.scalar * (v.f.f.inFromEnd + v.f.f.fieldSizeIn - p1Y), ppi * v.scalar * (v.f.f.inFromEnd + p2X), ppi * v.scalar * (v.f.f.inFromEnd + v.f.f.fieldSizeIn - p2Y));
}
void CimulationApp::robotDebug() {
	gl::color(1, 0, 0);
	for (int i = 0; i < 4; i++) {//simplified version of drawing the vertices
		gl::drawSolidCircle(R2S2(v.r[0].db.vertices[i]), 5 + i);
		//else gl::drawSolidCircle(Vec2f(ppi*v.scalar * v.r[0].db.vertices[i].X, ppi*v.scalar * v.r[0].db.vertices[i].Y), 5 + i);
	}
		//vertice rectangles
		gl::drawStrokedRect(Area(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.vertices[0].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[0].Y*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.vertices[1].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[1].Y*ppi*v.scalar));
		gl::drawStrokedRect(Area(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.vertices[2].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[2].Y*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar +v.r[0].db.vertices[3].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[3].Y*ppi*v.scalar));
		//vertical lines
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar +(v.r[0].db.vertices[1].X + 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[1].Y + 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[2].X - 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[2].Y - 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		gl::drawLine(cinder::Vec2f( v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[0].X + 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[0].Y + 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[3].X - 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[3].Y - 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		//horizontal lines
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[0].X + 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[0].Y + 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[1].X - 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[1].Y - 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[2].X + 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[2].Y + 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.vertices[3].X - 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.vertices[3].Y - 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		//for mogo
		for (int i = 0; i < 4; i++) {//simplified version of drawing the MGVert
			gl::drawSolidCircle(R2S2(v.r[0].db.MGVert[i]), 5 + i);
			//else gl::drawSolidCircle(Vec2f(ppi*v.scalar * v.r[0].db.MGVert[i].X, ppi*v.scalar * v.r[0].db.MGVert[i].Y), 5 + i);
		}
		//vertice rectangles
		gl::drawStrokedRect(Area(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.MGVert[0].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.MGVert[0].Y*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.MGVert[1].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.MGVert[1].Y*ppi*v.scalar));
		gl::drawStrokedRect(Area(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.MGVert[2].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.MGVert[2].Y*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.MGVert[3].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.MGVert[3].Y*ppi*v.scalar));
		//vertical lines
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[1].X + 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[1].Y + 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[2].X - 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[2].Y - 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[0].X + 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[0].Y + 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[3].X - 300 * cos((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[3].Y - 300 * sin((v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		//horizontal lines
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[0].X + 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[0].Y + 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[1].X - 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[1].Y - 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));
		gl::drawLine(cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[2].X + 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
			v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[2].Y + 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar),
			cinder::Vec2f(v.f.f.inFromEnd*ppi*v.scalar + (v.r[0].db.MGVert[3].X - 300 * sin((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar,
				v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - (v.r[0].db.MGVert[3].Y - 300 * cos((-v.r[0].p.mRot) * PI / 180))*ppi*v.scalar));

		//draw circle
		gl::drawStrokedCircle(Vec2f(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].p.position.X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].p.position.Y*ppi*v.scalar), renderRad * v.r[0].d.size*ppi*v.scalar);
		gl::drawStrokedRect(Area(v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.vertices[0].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[0].Y*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.r[0].db.vertices[2].X*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar - v.r[0].db.vertices[2].Y*ppi*v.scalar));
		
		gl::drawSolidCircle(Vec2f(v.f.f.inFromEnd*ppi*v.scalar+ppi*v.scalar*(v.r[0].p.position.X + (v.r[0].d.size / 2) * cos((-v.r[0].p.mRot) * PI / 180) * sqrt(2)),
			v.f.f.inFromEnd*ppi*v.scalar+v.f.f.fieldSizeIn*ppi*v.scalar - ppi*v.scalar*(v.r[0].p.position.Y - (v.r[0].d.size / 2) * sin((-v.r[0].p.mRot) * PI / 180) * sqrt(2))), 5);
		gl::color(1, 1, 1);//resets colours to regular
}
void CimulationApp::drawClaw(robot *r) {
	gl::draw(v.r[0].CChanel, Area((r->c.clawSize)*ppi*v.scalar, (r->d.size*.5 + r->c.baseSize)*ppi*v.scalar, (-r->c.clawSize)*ppi*v.scalar, (r->d.size*.5)*ppi*v.scalar));
	gl::color(222.0 / 225, 229.0 / 225, 34.0 / 225);
	gl::drawSolidRect(Area(Vec2d((r->c.clawPos + r->c.clawThick)*ppi*v.scalar, (r->d.size*.5 + r->c.clawHeight + r->c.baseSize)*ppi*v.scalar), Vec2d((r->c.clawPos - r->c.clawThick)*ppi*v.scalar, (r->d.size*.5 + r->c.baseSize)*ppi*v.scalar)));
	gl::drawSolidRect(Area(Vec2d((-r->c.clawPos - r->c.clawThick)*ppi*v.scalar, (r->d.size*.5 + r->c.clawHeight + r->c.baseSize)*ppi*v.scalar), Vec2d((-r->c.clawPos + r->c.clawThick)*ppi*v.scalar, (r->d.size*.5 + r->c.baseSize)*ppi*v.scalar)));
	gl::color(1, 1, 1);//reset colour
}
void CimulationApp::drawRobot(robot *r) {
	glPushMatrix();
	///robotDebug(&v, true);
	//had to modify the y because the origin is bottom left hand corner
	gl::translate(Vec3f(R2S3(r->p.position.X, r->p.position.Y, 0.0)));//origin of rotation
	gl::rotate(Vec3f(0, 0, -r->p.mRot - 90));//something for like 3D rotation.... ugh
	gl::color(1, 1, 1);
	if ((r->p.velocity.X != 0 && r->p.velocity.Y != 0) || r->p.rotVel != 0){//changes with small increments of velocity and rotation
		if ((int)(10 * (r->p.velocity.X + 2*r->p.rotVel)) % 2 == 0/* || (int)(10 * r->p.rotVel) % 2 == 0*/)
			gl::draw(r->TankBase, Area((-(r->d.size / 2))*ppi*v.scalar, (-(r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar));
		else
			gl::draw(r->TankBase2, Area((-(r->d.size / 2))*ppi*v.scalar, (-(r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar));
	}
	else gl::draw(r->TankBase, Area((-(r->d.size / 2))*ppi*v.scalar, (-(r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar, ((r->d.size / 2))*ppi*v.scalar));

	//mogo
	gl::draw(v.r[0].CChanelVERT, Area((-r->mg.clawPos - r->mg.clawThick)*ppi*v.scalar, r->mg.protrusion*ppi*v.scalar, (-r->mg.clawPos + r->mg.clawThick)*ppi*v.scalar, (r->mg.protrusion + r->mg.clawHeight)*ppi*v.scalar));
	gl::draw(v.r[0].CChanelVERT, Area((r->mg.clawPos + r->mg.clawThick)*ppi*v.scalar, r->mg.protrusion*ppi*v.scalar, (r->mg.clawPos - r->mg.clawThick)*ppi*v.scalar, (r->mg.protrusion + r->mg.clawHeight)*ppi*v.scalar));
	//draw mogo
	drawClaw(r);
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
	if (s.SimRunning == s.CUSTOMIZE ) {//only for CUSTOMIZE and truspeed sim
		controlPanel(&v.r[0]);
		v.r[0].updateFeatures();
	}
	if (s.SimRunning == s.PIDCTRL) {
		v.pid.textOutput(&v.r[0]);
		v.pid.graphPlot();
	}
	if (s.SimRunning == s.TRUSPEED) {
		drawJoystick(&v.r[0], &v.j);
		v.tS.graphPlot();//draws the graph
		v.tS.textOutput(&v.r[0], &v.j);//draws the text for the graph
	}
	if (s.SimRunning == s.FIELD) {//when field button is pressed
		drawJoystick(&v.r[0], &v.j);
		if (v.recording) {
			int size = 10;//pixels in which to draw the rectangles width
			gl::color(1, 0, 0);
			gl::drawSolidRect(Area(0, 0, size, getWindowHeight()));
			gl::drawSolidRect(Area(0, 0, getWindowWidth(), size));
			gl::drawSolidRect(Area(0, getWindowHeight() - size, getWindowWidth(), getWindowHeight()));
			gl::drawSolidRect(Area(getWindowWidth() - size, 0, getWindowWidth(), getWindowHeight()));
			gl::color(1, 1, 1);//reset to white
		}
		ci::gl::draw(v.f.fieldBare, ci::Area(v.f.f.inFromEnd*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar, v.f.f.inFromEnd*ppi*v.scalar + v.f.f.fieldSizeIn*ppi*v.scalar));
		for (int rob = 0; rob < v.r.size(); rob++) {
			drawRobot(&v.r[rob]);//drawing robot 1
		}
		gl::drawString("Score:", Vec2f(v.scalar * 850, v.scalar * 50), Color(1, 1, 1), Font("Arial", v.scalar * 50));
		drawText(v.f.calculateScore(), vec3I(v.scalar * 1000, v.scalar * 50), vec3I(1, 1, 1), v.scalar * 50);
		gl::drawString("Time(s):", Vec2f(v.scalar * 1350, v.scalar * 100), Color(1, 1, 1), Font("Arial", v.scalar * 40));
		drawText(ci::app::getElapsedSeconds(), vec3I(v.scalar * 1480, v.scalar * 100), vec3I(1, 1, 1), v.scalar*40);

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
		std::vector<field::cone> sortedCones(v.f.c);//created copy of v.f.c to not directly affect anything badly
		std::sort(sortedCones.begin(), sortedCones.end(),//lambda function for sorting new cone vector based off pos.Z position
			[](const field::cone &c1, const field::cone &c2){
			return c1.pos.Z < c2.pos.Z;
		});
		for (int i = 0; i < sortedCones.size(); i++) {//find a way to draw based off z distance
			gl::color(1, 1, 1);
			gl::draw(v.f.coneTexture, Area(R2S4(
				(sortedCones[i].pos.X - sortedCones[i].radius),
				(sortedCones[i].pos.Y - sortedCones[i].radius),
				(sortedCones[i].pos.X + sortedCones[i].radius),
				(sortedCones[i].pos.Y + sortedCones[i].radius)))
			);
		}
		for (int i = 0; i < v.f.mg.size(); i++) {//drawing how many are stacked
			if (v.f.mg[i].stacked.size() > 0) {
				drawText(v.f.mg[i].stacked.size(), vec3I(
					(70 + (int)((v.f.mg[i].pos.X - cRad) * (int)ppi)*v.scalar),
					(50 + (int)((v.f.f.fieldSizeIn - v.f.mg[i].pos.Y + cRad) * (int)ppi)*v.scalar)),
					vec3I(1, 1, 1), v.scalar*50);
			}
		}
		for (int i = 0; i < v.f.pl.size(); i++) {//drawing how many are stacked
			if (v.f.pl[i].stacked.size() > 0) {
				drawText(v.f.pl[i].stacked.size(), vec3I(
					(70 + (int)((v.f.pl[i].pos.X - cRad) * (int)ppi)*v.scalar),
					(50 + (int)((v.f.f.fieldSizeIn - v.f.pl[i].pos.Y + cRad) * (int)ppi)*v.scalar)),
					vec3I(1, 1, 1), v.scalar*50);
			}
		}
		gl::color(1, 1, 1);
	}
	else drawRobot(&v.r[0]);
	gl::color(1, 0, 0);
	//indicator for cone goalCone
	for (int rob = 1; rob < v.r.size(); rob++) {//not counting goalCone of first robot (manual one)
		gl::drawSolidCircle(R2S2(vec3(v.f.c[v.r[rob].goalCone].pos.X, v.f.c[v.r[rob].goalCone].pos.Y)), v.scalar * 4);
		for (int i = 0; i < 3; i++) {
			gl::drawStrokedCircle(R2S2(vec3(v.f.c[v.r[rob].goalCone].pos.X, v.f.c[v.r[rob].goalCone].pos.Y)), v.scalar*v.f.c[v.r[rob].goalCone].radius*ppi + i, 10);
		}
	}
	//debug text
	gl::drawSolidCircle(R2S2(vec3(
		v.r[0].p.position.X + v.r[0].mg.protrusion * cos((v.r[0].p.mRot) * PI / 180)*2, 
		v.r[0].p.position.Y + v.r[0].mg.protrusion * sin((v.r[0].p.mRot) * PI / 180)*2)), v.scalar * 5);

	//gl::drawSolidCircle(R2S2(vec3(v.f.c[30].closestPointMOGO.X, v.f.c[30].closestPointMOGO.Y)), v.scalar * 5);

	if (v.f.mg[5].inPossession[1]) gl::drawString("YES", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));
	else gl::drawString("NO", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));
	drawText(v.r[0].mg.holding, vec3I(1000, 800), vec3I(0, 1, 0), 30);

	//if (v.r[0].mg.grabbing) gl::drawString("YES", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));
	//else gl::drawString("NO", Vec2f(1010, 600), Color(1, 1, 1), Font("Arial", 30));

	drawText(v.r[0].mg.holding, vec3I(1010, 660), vec3I(1, 1, 1), 30);
//	drawText(v.f.pl[0].height, vec3I(1010, 500), vec3I(1, 1, 1), 30);
//	drawText(v.r[0].db.rotDist, vec3I(1010, 400), vec3I(1, 1, 1), 30);

	gl::color(1, 1, 1);
	//USER INTERFACE
	buttons(100);//size in px
	//robotDebug();
	gl::drawString("FPS: ", Vec2f(getWindowWidth() - 150, 30), Color(0, 1, 0), Font("Arial", 30));
	drawText(getAverageFps(), vec3I(getWindowWidth() - 90, 30), vec3I(0, 1, 0), 30);
	if(v.debugText && s.SimRunning != s.CUSTOMIZE) textDraw();//dont run on truspeed sim, unnecessary
}
CINDER_APP_NATIVE(CimulationApp, RendererGl)
