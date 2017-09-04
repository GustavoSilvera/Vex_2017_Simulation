#include "joystick.h"
//here go all the functions for the JOYSTICK
void joystick::getAnalog(vec3 mousePos) {
	if (withinAnalogRange(mousePos)) {
		analogY =  (mousePos.Y - (drawY + drawSize))*(127.0/drawSize);//analog y set to specific parameters of drawY
		analogX = (mousePos.X - (drawX + drawSize))*(127.0 / drawSize);//analog x set to specific parameters of drawX
	}
	else {
		analogX = 0;
		analogY = 0;
	}
}
bool joystick::withinAnalogRange(vec3 mousePos) {
	return (abs(mousePos.Y - (drawY+drawSize)) <= drawSize && abs(mousePos.X - (drawX + drawSize)) <= drawSize);
}