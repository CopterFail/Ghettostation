/*
 * defines.h
 *
 *  Created on: 10.04.2015
 *      Author: CopterFail
 */

#ifndef DEFINES_H_
#define DEFINES_H_

//#define GHETTO_DEBUG

/* ########################################  PROTOCOL ######################################################*/
#define PROTOCOL_UAVTALK                        // OpenPilot / Taulabs protocol
#define PROTOCOL_MSP                            // MSP from Multiwii
#define PROTOCOL_LIGHTTELEMETRY                 // Ghettostation internal protocol.
#define PROTOCOL_MAVLINK                        // Mavlink for Ardupilot / Autoquad / PixHawk / Taulabs (UAVOmavlinkBridge)
#define PROTOCOL_NMEA                           //GPS NMEA ASCII protocol
#define PROTOCOL_UBLOX                          //GPS UBLOX binary protocol
#define PROTOCOL_HOTT							//HoTT USB / BT protocol

/* If you have communication problem at 56K , set this on. ( ie APM 2/2.5/2.6/AIO )
   Most Arduino have a +2.18% error at 57600 bd, Teensyduino has a -0.74% error. Booth cumulated are too much.
   Successfull com between Teensy & arduino requires 58824 bauds for Teensy.*/
//#define BAUDRATE56K 57600
#define BAUDRATE56K 58824

//########### LCD ##################################################################################################
//LCD model
//#define LCDLCM1602 // (adress: 0x27 or 0x20) HobbyKing IIC/I2C/TWI Serial 2004 20x4, LCM1602 IIC A0 A1 A2 & YwRobot Arduino LCM1602 IIC V1
//#define LCDGYLCD  // (adress: 0x20) Arduino-IIC-LCD GY-LCD-V1Arduino-IIC-LCD GY-LCD-V1
//#define LCD03I2C  // (adress: 0x63 or  0xc6) LCD03 / LCD05
//#define GLCDEnable // Graphical LCD - Using system5x7 font so its nearly 20x4 size
//#define OLEDLCD  // Oled 128x64 i2c LCD (address 0x3C or 0x3D)
#define LCDST7735	// 128 x 180 pixel SPI LCD

// I2C LCD Adress
#define I2CADRESS 0x27 // LCD03/05 have 0x63 or 0xc6 ( even if it's written 0xc6 when powering the lcd03, in fact it uses 0x63 so try booth)
                       // LCM1602 uses 0x27 & GY-LCD use 0x20
                       // OLED_LCD use 0x3d or 0x3d

#define CONFIG_VERSION 1004 // Changing this will reset eeprom to default values

//########## BOARD ################################################################################################
//#define TEENSYPLUS2 // Teensy++2 support.
//#define MEGA // Arduino Mega board
#define TEENSY31



//#define COMPASS                                 //Keep it enabled even if unused, but you will need i2c


#define BARO_ALT // Use Baro for Altitude. Comment for using GPS altitude instead.

//Use Mag+imu for heading or GPS heading if not set ( not used for tracker only osd relay )
#define MAGHEADING 1

#define MAGDEC -600  // Your local Magnetic Declination in radian. Get it from here: http://magnetic-declination.com/  then convert it in milliradian: http://www.wolframalpha.com/input/?i=%280%C2%B0+5%27%29+in+radians
                     // only needed if using internal compass.

//Minimum distance in meters where it will stop moving servos.
#define DONTTRACKUNDER  5

// Prevent Ghettostation to send packets to the flightcontroler
// Usefull if you're using OSD or a GCS at the same time.
#define PASSIVEMODE 0

// Default tilt angle used when not tracking.
#define DEFAULTELEVATION  15

//Memory bank name to display on LCD (18 char max)
#define BANK1  "1.2 GHZ"
#define BANK2  "5.8 Ghz"
#define BANK3  "Bank 3"
#define BANK4  "Bank 4"

//GS Battery alarm (2S)
#define MIN_VOLTAGE1 7.0f //10.5f // First battery alarm level. Will emit 2 short tones every 10 sec.
#define MIN_VOLTAGE2 6.6f //10.0f // Second battery alarm level. Will emit 1 short + 1 long tone every 5 sec
#define VOLTAGE_RATIO 1100   // Default multiplier for battery voltage reading * 100. This can eb adjustd later from the menu.
#define VOLTAGE_REF 3.3f
#define DAMPING	0.9f

//########### GROUND OSD TELEMETRY OUTPUT #########################################################################
#define OSD_OUTPUT			// Activate osd output (comment if not needed)
#define OSD_BAUD 57600		//OSD output baudrate


//#################################### SERVOS ENDPOINTS #############################################################
// NO NEED TO EDIT THIS
//. Those are just default values when not configured.
// To prevent burning servo they boot starts at neutral for all values. Adjust them directly from the menu.

#define PAN_MAXPWM 1500     //max pan servo pwm value
#define PAN_MAXANGLE 90     //Max angle clockwise (on the right) relative to PAN_MAXPWM.
#define PAN_MINPWM 1500     //min pan servo pwm valuemin pan servo pwm value
#define PAN_MINANGLE 90	    //Max angle counter clockwise (on the left) relative to PAN_MINPWM.
#define TILT_MAXPWM 1500    //max tilt pwm value
#define TILT_MAXANGLE 90    //max tilt angle considering 0° is facing toward.
#define TILT_MINPWM 1500    //min tilt pwm value
#define TILT_MINANGLE 0     //minimum tilt angle. Considering 0 is facing toward, a -10 value would means we can tilt 10° down.

#endif /* DEFINES_H_ */