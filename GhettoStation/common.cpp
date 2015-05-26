#include <Arduino.h>

#include "defines.h"
#include "boards.h"
#include "globals.h"

/*##################################### COMMON FUNCTIONS #############################################*/

boolean getBit(byte Reg, byte whichBit)
{
	boolean State;
	State = Reg & (1 << whichBit);
	return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat)
{
	if (stat)
	{
		Reg = Reg | (1 << whichBit);
	}
	else
	{
		Reg = Reg & ~(1 << whichBit);
	}
	return Reg;
}

float toRad(float angle)
{
// convert degrees to radians
	angle = angle * 0.01745329; // (angle/180)*pi
	return angle;
}

float toDeg(float angle)
{
// convert radians to degrees.
	angle = angle * 57.29577951;   // (angle*180)/pi
	return angle;
}

int config_bank[] =
{ 1, 101, 201, 301 }; // 50 bytes reserved per bank.
uint8_t current_bank;

