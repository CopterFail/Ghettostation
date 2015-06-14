/*
 * Globals.h
 *
 *  Created on: 12.04.2015
 *      Author: CopterFail
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

// type definitions
typedef enum tActivity
{
	ActMenu = 0,
	ActTrack,
	ActSetHome,
	ActPanMinPwm,
	ActPanMinAngle,
	ActPanMaxPwm,
	ActPanMaxAngle,
	ActTiltMinPwm,
	ActTiltMinAngle,
	ActTiltMaxPwm,
	ActTiltMaxAngle,
	ActTestServo,
	ActSetTelemetrie,
	ActSetBaud,
	ActSetBank,
	ActSetOsd,
	ActSetBearing,
	ActSetVoltage,
	ActSetChannel,
	ActSetReceiver,
	ActSetDiversity
};

// declaration of all the global variables used by the station

//Telemetry variables
extern int32_t uav_lat;                        // latitude
extern int32_t uav_lon;                        // longitude
extern float lonScaleDown;                   // longitude scaling
extern uint8_t uav_satellites_visible;         // number of satellites
extern uint8_t uav_fix_type;                  // GPS lock 0-1=no fix, 2=2D, 3=3D
extern int32_t uav_alt;                        // altitude (cm)
extern int32_t rel_alt;                        // relative altitude to home
extern uint16_t uav_groundspeed;                // ground speed in km/h
extern uint8_t uav_groundspeedms;              // ground speed in m/s
extern int16_t uav_pitch;                      // attitude pitch
extern int16_t uav_roll;                       // attitude roll
extern int16_t uav_heading;                    // attitude heading
extern int16_t uav_gpsheading;                 // gps heading
extern uint16_t uav_bat;                        // battery voltage (mv)
extern uint16_t uav_amp;                        // consumed mah.
extern uint16_t uav_current;                    // actual current
extern uint8_t uav_rssi;                       // radio RSSI (%)
extern uint8_t uav_linkquality;                // radio link quality
extern uint8_t uav_airspeed;                   // Airspeed sensor (m/s)
extern uint8_t ltm_armfsmode;
extern uint8_t uav_arm;                        // 0: disarmed, 1: armed
extern uint8_t uav_failsafe;                   // 0: normal,   1: failsafe
extern uint8_t uav_flightmode; // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
// 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
// 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown

extern float voltage_ratio;      // voltage divider ratio for gs battery reading
extern float voltage_actual;             // gs battery voltage in mv

extern char* protocol;
extern long lastpacketreceived;

//home
extern int32_t home_lon;
extern int32_t home_lat;
extern int32_t home_alt;
extern int16_t home_bearing;
extern uint32_t home_dist;
extern uint8_t home_sent;

//tracking
extern int16_t Bearing;
extern int16_t Elevation;
//extern int16_t servoBearing;
//extern int16_t servoElevation;

extern tActivity current_activity;

extern boolean gps_fix;
extern boolean btholdstate;
extern boolean telemetry_ok;
extern boolean home_pos;
extern boolean home_bear;

//servo temp configuration before saving
extern int16_t servoconf_tmp[4];
extern int16_t servoconfprev_tmp[4];
extern uint8_t test_servo_step;
extern uint16_t test_servo_cnt;
//baudrate selection
extern long baudrates[8];

//Configuration stored in EEprom
struct config_t //
{
	int16_t config_crc;
	int16_t pan_minpwm;
	int16_t pan_minangle;
	int16_t pan_maxpwm;
	int16_t pan_maxangle;
	int16_t tilt_minpwm;
	int16_t tilt_minangle;
	int16_t tilt_maxpwm;
	int16_t tilt_maxangle;
	int16_t baudrate;
	int16_t telemetry;
	int16_t bearing;
	uint8_t osd_enabled;
	uint8_t bearing_method;
	uint16_t voltage_ratio;
	uint8_t channel;
	uint8_t receiver;
	uint8_t diversity;
	uint8_t dummy;
};
extern config_t configuration;

extern int config_bank[]; // 50 bytes reserved per bank.
extern uint8_t current_bank;

#endif /* GLOBALS_H_ */
