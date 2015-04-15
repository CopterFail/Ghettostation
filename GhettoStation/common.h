/*
 * common.h
 *
 *  Created on: 12.04.2015
 *      Author: CopterFail
 */

#ifndef COMMON_H_
#define COMMON_H_

boolean getBit(byte Reg, byte whichBit);
byte setBit(byte &Reg, byte whichBit, boolean stat);
float toRad(float angle);
float toDeg(float angle);

#endif /* COMMON_H_ */
