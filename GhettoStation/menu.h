/*
 * Menu.h
 *
 *  Created on: 10.04.2015
 *      Author: CopterFail
 */

#ifndef MENU_H_
#define MENU_H_

/*########################################### MENU ##################################################*/
MenuSystem displaymenu;
Menu rootMenu("FPV STATION");
MenuItem m1i1Item("TRACK");
MenuItem m1i2Item("SET HOME");
Menu m1m3Menu("CONFIG");
Menu m1m3m1Menu("SERVOS");
Menu m1m3m1m1Menu("PAN");
MenuItem m1m3m1m1l1Item("MINPWM");
MenuItem m1m3m1m1l2Item("MAXPWM");
MenuItem m1m3m1m1l3Item("MINANGLE");
MenuItem m1m3m1m1l4Item("MAXANGLE");
Menu m1m3m1m2Menu("TILT");
MenuItem m1m3m1m2l1Item("MINPWM");
MenuItem m1m3m1m2l2Item("MAXPWM");
MenuItem m1m3m1m2l3Item("MINANGLE");
MenuItem m1m3m1m2l4Item("MAXANGLE");
MenuItem m1m3m1i3Item("TEST");
Menu m1m3m2Menu("TELEMETRY");
MenuItem m1m3m2i1Item("PROTOCOL");
MenuItem m1m3m2i2Item("BAUDRATE");
Menu m1m3m3Menu("OTHERS");
MenuItem m1m3m3i1Item("OSD");
MenuItem m1m3m3i2Item("BEARING");
MenuItem m1m3m3i3Item("BATTERY");
MenuItem m1m3m3i4Item("SETTINGS");
Menu m1m4Menu("VIDEO");
MenuItem m1m41i1Item("CHANNEL");
MenuItem m1m41i2Item("RECEIVER");
MenuItem m1m41i3Item("DIVERSITY");

#endif /* MENU_H_ */
