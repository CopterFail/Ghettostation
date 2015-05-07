/**
 ******************************************************************************
 *
 * @file       GhettoStation.ino
 * @author     Guillaume S
 * @brief      Arduino based antenna tracker & telemetry display for UAV projects.
 * @project    https://code.google.com/p/ghettostation/
 * 
 *             
 *             
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************
*/
 

#include <Arduino.h>

#include "defines.h"
#include "boards.h"
#include "globals.h"
#include "common.h"

/* Arduino needs all includes in the main ino file - even when not used */
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#ifdef GHETTO_DEBUG
//#include <MemoryFree.h>
#endif

#include <Servo.h>
#include <SPI.h>
#include <Wire.h> 

#include <Metro.h>
#include <MenuSystem.h>
#include <Button.h>
#include <EEPROM.h>

#include "menu.h"
#include "buzzer.h"
#include "rx5808.h"
#include "lcd.h"

#ifdef COMPASS //use additional hmc5883L mag breakout
//HMC5883L i2c mag b
#include <HMC5883L.h>
#endif

#include "UAVTalk.h"
#include "MSP.h"
#include "LightTelemetry.h"
#include "Mavlink.h"
#include "GPS_NMEA.h"
#include "GPS_UBLOX.h"
#include "HottBtUsb.h"

/*
 * BOF preprocessor bug prevent
 */
#define nop() __asm volatile ("nop")
#if 1
nop();
#endif
/*
 * EOF preprocessor bug prevent
*/

/**
 * global variable definition, see globals.h
 */
//Telemetry variables
int32_t      uav_lat = 0;                    // latitude
int32_t      uav_lon = 0;                    // longitude
float        lonScaleDown=0.0;               // longitude scaling
uint8_t      uav_satellites_visible = 0;     // number of satellites
uint8_t      uav_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
int32_t      uav_alt = 0;                    // altitude (cm)
int32_t      rel_alt = 0;                    // relative altitude to home
uint16_t     uav_groundspeed = 0;            // ground speed in km/h
uint8_t      uav_groundspeedms = 0;          // ground speed in m/s
int16_t      uav_pitch = 0;                  // attitude pitch
int16_t      uav_roll = 0;                   // attitude roll
int16_t      uav_heading = 0;                // attitude heading
int16_t      uav_gpsheading=0;               // gps heading
uint16_t     uav_bat = 0;                    // battery voltage (mv)
uint16_t     uav_amp = 0;                    // consumed mah.
uint16_t     uav_current = 0;                // actual current
uint8_t      uav_rssi = 0;                   // radio RSSI (%)
uint8_t      uav_linkquality = 0;            // radio link quality
uint8_t      uav_airspeed = 0;               // Airspeed sensor (m/s)
uint8_t      ltm_armfsmode = 0;
uint8_t      uav_arm = 0;                    // 0: disarmed, 1: armed
uint8_t      uav_failsafe = 0;               // 0: normal,   1: failsafe
uint8_t      uav_flightmode = 19;            // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
                                             // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
                                             // 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown

float        voltage_ratio;              // voltage divider ratio for gs battery reading
float        voltage_actual = 0.0;           // gs battery voltage in mv

char* protocol = "";
long lastpacketreceived;
static boolean      enable_frame_request = 0;


//home
int32_t home_lon;
int32_t home_lat;
int32_t home_alt;
int16_t home_bearing = 0;
uint32_t home_dist;
uint8_t home_sent = 0;

//tracking
int16_t Bearing;
int16_t Elevation;
//int16_t servoBearing=0;
//int16_t servoElevation=0;

static uint8_t trackingDisplay = 0U;

tActivity current_activity;
//int current_activity = 0; // Activity status 0: Menu , 1: Track, 2: SET_HOME, 3: PAN_MINPWM, 4: PAN_MINANGLE, 5: PAN_MAXPWM,
                          // 6: PAN_MAXANGLE, 7: TILT_MINPWM, 8: TILT_MINANGLE, 9: TILT_MAXPWM, 10: TILT_MAXANGLE, 11: TEST_SERVO, 12: SET_RATE
boolean gps_fix      = false;
boolean btholdstate  = false;
boolean telemetry_ok = false;
boolean home_pos     = false;
boolean home_bear    = false;

//servo temp configuration before saving
int16_t servoconf_tmp[4];
int16_t servoconfprev_tmp[4];
uint8_t test_servo_step = 1;
uint16_t test_servo_cnt = 360;
//baudrate selection
long baudrates[8]= {1200, 2400, 4800, 9600, 19200, 38400, BAUDRATE56K, 115200};

//Configuration stored in EEprom
config_t configuration;


//##### LOOP RATES
Metro loop1hz = Metro(1000); // 1hz loop
Metro loop10hz = Metro(100); //10hz loop  
Metro loop50hz = Metro(20); // 50hz loop
//##### BUTTONS 
Button buttonUp = Button(UP_BUTTON_PIN,BUTTON_PULLUP_INTERNAL);
Button buttonDown = Button(DOWN_BUTTON_PIN,BUTTON_PULLUP_INTERNAL);
Button buttonEnter = Button(ENTER_BUTTON_PIN,BUTTON_PULLUP_INTERNAL);

//pan/tilt servos
Servo pan_servo;
Servo tilt_servo;

cBuzzer Buzzer;
cRX5808 RX5808;

#if defined(COMPASS)
HMC5883L compass;
#endif

// local function prototypes
void attach_servo(Servo &s, int p, int min, int max);
void detach_servo(Servo &s);
template <class T> int EEPROM_write(int ee, const T& value);
template <class T> int EEPROM_read(int ee, T& value);
void clear_eeprom();

//#################################### SETUP LOOP ####################################################

void setup() {

    //init setup
    init_menu();  
    //retrieve configuration from EEPROM
    current_bank = EEPROM.read(0);
    if (current_bank > 3) {
        current_bank = 0;
        EEPROM.write(0,0);
    }
    EEPROM_read(config_bank[int(current_bank)], configuration);
    // set temp value for servo pwm config
    servoconf_tmp[0] = configuration.pan_minpwm;
    servoconf_tmp[1] = configuration.pan_maxpwm;
    servoconf_tmp[2] = configuration.tilt_minpwm;
    servoconf_tmp[3] = configuration.tilt_maxpwm;
    home_bearing = configuration.bearing; // use last bearing position of previous session.
    voltage_ratio = (float)(configuration.voltage_ratio/100.0);
    delay(20);
    //clear eeprom & write default parameters if config is empty or wrong
    if (configuration.config_crc != CONFIG_VERSION) {
        clear_eeprom();
        delay(20);
    }
    //init LCD
    init_lcdscreen();

    //start serial com
    init_serial();
    
    // attach servos 
    attach_servo(pan_servo, PAN_SERVOPIN, configuration.pan_minpwm, configuration.pan_maxpwm);
    attach_servo(tilt_servo, TILT_SERVOPIN, configuration.tilt_minpwm, configuration.tilt_maxpwm); 

    // move servo to neutral pan & DEFAULTELEVATION tilt at startup 
    vParkServos();

    // setup button callback events
    buttonEnter.releaseHandler(enterButtonReleaseEvents);
    buttonDown.releaseHandler(leftButtonReleaseEvents);
    buttonUp.releaseHandler(rightButtonReleaseEvents);
    buttonEnter.clickHandler(ButtonPressEvents);
    buttonDown.clickHandler(ButtonPressEvents);
    buttonUp.clickHandler(ButtonPressEvents);

#if defined(COMPASS)
    compass = HMC5883L(); // Construct a new HMC5883 compass.
    delay(100);
    compass.SetScale(1.3); // Set the scale of the compass.
    compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
#endif
    delay(2500);  // Wait until osd is initialised
}

//######################################## MAIN LOOP #####################################################################
void loop() 
{
   
   if (loop1hz.check()) 
   {
        read_voltage();
   }
  
    if (loop10hz.check() == 1) 
	{
    	//update buttons internal states
        buttonEnter.isPressed();
        buttonDown.isPressed();
        buttonUp.isPressed();

        #ifdef OSD_OUTPUT
        //pack & send LTM packets to SerialPort2 at 10hz.
        ltm_write();
//DEBUG_SERIAL.write('Y');
while( OSD_SERIAL.available() ){
	DEBUG_SERIAL.write('.');
	DEBUG_SERIAL.write( OSD_SERIAL.read() );
}
        #endif

        //current activity loop
        check_activity();
        //debug output to usb Serial
        #ifdef GHETTO_DEBUG
        debug();
        #endif

        Buzzer.vUpdate();
        RX5808.vDiversity();
    }
	
    if (loop50hz.check() == 1) 
	{
        //update servos
        if (current_activity == ActTrack) 
		{
            if((home_dist / 100) > DONTTRACKUNDER) 
			{
                servoPathfinder(Bearing,Elevation); // refresh servo 
            }
        }
    }
	
    get_telemetry();    
}

//######################################## ACTIVITIES #####################################################################

void check_activity(void) 
{
    if (uav_satellites_visible >= 5) 
	{ 
        gps_fix = true; 
    } 
    else 
	{
        gps_fix = false;
	}
	
    switch (current_activity) 
    {
        case ActMenu:             //MENU
                Bearing = DEFAULTBEARING; 
				Elevation = DEFAULTELEVATION;   
                if (buttonEnter.holdTime() >= 1000 && buttonEnter.held())  //long press 
				{
                    displaymenu.back();
                    vUpdateMenu();
                }
                showMenu();
                break;
        case ActTrack:            //TRACK
                if ((!home_pos) || (!home_bear))  // check if home is set before start tracking
				{
                    Bearing = DEFAULTBEARING; 
					Elevation = DEFAULTELEVATION;       
                    current_activity = ActSetHome;             // set bearing if not set.
                } 
				else if (home_bear) 
				{
                    antenna_tracking();
                    lcddisp_tracking( trackingDisplay );
                    if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press 
					{ 
                        current_activity = ActMenu;
                        vParkServos();
                        //telemetry_off();
                    }
                }
                break;
        case ActSetHome:            //SET HOME
                if (!home_pos)
				{					
                    lcddisp_sethome();
				}
                else
				{
                    if (!home_bear) 
					{ 
                        lcddisp_setbearing();   
                        servoPathfinder(0, DEFAULTELEVATION);
                        delay(200);
                    }
                    else 
					{
                        lcddisp_homeok();
					}
                }
                if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press 
				{ 
                    current_activity = ActMenu;
                }
                break;
        case ActPanMinPwm:             //PAN_MINPWM
                servoconf_tmp[0] = config_servo(1, 1, servoconf_tmp[0] );                
                if (servoconf_tmp[0] != servoconfprev_tmp[0]) 
				{
                    detach_servo(pan_servo);
                    attach_servo(pan_servo, PAN_SERVOPIN, servoconf_tmp[0], configuration.pan_maxpwm);
                } 
                pan_servo.writeMicroseconds(servoconf_tmp[0]);
                //pan_servo.write(0);
                servoconfprev_tmp[0] = servoconf_tmp[0];
                if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press 
				{
                    configuration.pan_minpwm = servoconf_tmp[0];
                    EEPROM_write(config_bank[int(current_bank)], configuration);
                    detach_servo(pan_servo);
                    attach_servo(pan_servo, PAN_SERVOPIN, configuration.pan_minpwm, configuration.pan_maxpwm);
                    //move_servo(pan_servo, 1, 0, configuration.pan_minangle, configuration.pan_maxangle);
                    vParkServos();
                    current_activity=ActMenu;
                }
                break;
        case ActPanMinAngle:             //PAN_MINANGLE
                configuration.pan_minangle = config_servo(1, 2, configuration.pan_minangle);
                pan_servo.writeMicroseconds(configuration.pan_minpwm);
                //pan_servo.write(0);
                if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
				{
                    EEPROM_write(config_bank[int(current_bank)], configuration);
                    //move_servo(pan_servo, 1, 0, configuration.pan_minangle, configuration.pan_maxangle);
                    vParkServos();
                    current_activity=ActMenu;
                }
                break;
        case ActPanMaxPwm:             //PAN_MAXPWM
            servoconf_tmp[1] = config_servo(1, 3, servoconf_tmp[1] );
            if (servoconf_tmp[1] != servoconfprev_tmp[1]) 
			{
                detach_servo(pan_servo);
                attach_servo(pan_servo,PAN_SERVOPIN, configuration.pan_minpwm, servoconf_tmp[1]);
            } 
            pan_servo.writeMicroseconds(servoconf_tmp[1]);
            //pan_servo.write(180);
            servoconfprev_tmp[1] = servoconf_tmp[1];
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                configuration.pan_maxpwm = servoconf_tmp[1];
                EEPROM_write(config_bank[int(current_bank)], configuration);
                detach_servo(pan_servo);
                attach_servo(pan_servo, PAN_SERVOPIN, configuration.pan_minpwm, configuration.pan_maxpwm);
                //move_servo(pan_servo, 1, 0, configuration.pan_minangle, configuration.pan_maxangle);
                vParkServos();
                current_activity=ActMenu;
            }
            break;
            
        case ActPanMaxAngle:             //PAN_MAXANGLE
            configuration.pan_maxangle = config_servo(1, 4, configuration.pan_maxangle );
            pan_servo.writeMicroseconds(configuration.pan_maxpwm);
            //pan_servo.write(180);
             if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			 {   
                EEPROM_write(config_bank[int(current_bank)], configuration);
                //move_servo(pan_servo, 1, 0, configuration.pan_minangle, configuration.pan_maxangle);
                vParkServos();
                current_activity=ActMenu;
            }
            break;
        case ActTiltMinPwm:             //"TILT_MINPWM"
            servoconf_tmp[2] = config_servo(2, 1, servoconf_tmp[2] );
            if (servoconf_tmp[2] != servoconfprev_tmp[2]) 
			{
                detach_servo(tilt_servo);
                attach_servo(tilt_servo, TILT_SERVOPIN, servoconf_tmp[2], configuration.tilt_maxpwm); 
            }
            tilt_servo.writeMicroseconds(servoconf_tmp[2]); 
            //tilt_servo.write(0);
            servoconfprev_tmp[2] = servoconf_tmp[2];
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{    
                configuration.tilt_minpwm = servoconf_tmp[2];
                EEPROM_write(config_bank[int(current_bank)], configuration);
                detach_servo(tilt_servo);
				attach_servo(tilt_servo,TILT_SERVOPIN, configuration.tilt_minpwm, configuration.tilt_maxpwm);
                //move_servo(tilt_servo, 2, 0, configuration.tilt_minangle, configuration.tilt_maxangle);;
				vParkServos();
                current_activity=ActMenu;
            }
            break;
        case ActTiltMinAngle:             //TILT_MINANGLE
            configuration.tilt_minangle = config_servo(2, 2, configuration.tilt_minangle ); 
            tilt_servo.writeMicroseconds(configuration.tilt_minpwm);
            //tilt_servo.write(0);
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                EEPROM_write(config_bank[int(current_bank)], configuration);
                //move_servo(tilt_servo, 2, 0, configuration.tilt_minangle, configuration.tilt_maxangle);
                vParkServos();
                current_activity=ActMenu;
            }
            break;
        case ActTiltMaxPwm:             //"TILT_MAXPWM"
            servoconf_tmp[3] = config_servo(2, 3, servoconf_tmp[3] );
            if (servoconf_tmp[3] != servoconfprev_tmp[3]) 
			{
                detach_servo(tilt_servo);
                attach_servo(tilt_servo, TILT_SERVOPIN, configuration.tilt_minpwm, servoconf_tmp[3]); 
            }
            tilt_servo.writeMicroseconds(servoconf_tmp[3]);
            //tilt_servo.write(180);
            servoconfprev_tmp[3] = servoconf_tmp[3];
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                configuration.tilt_maxpwm = servoconf_tmp[3];
                EEPROM_write(config_bank[int(current_bank)], configuration);
                detach_servo(tilt_servo);
				attach_servo(tilt_servo,TILT_SERVOPIN, configuration.tilt_minpwm, configuration.tilt_maxpwm);
                //move_servo(tilt_servo, 2, 0, configuration.tilt_minangle, configuration.tilt_maxangle);
				vParkServos();
                current_activity=ActMenu;
            }
            break;
        case ActTiltMaxAngle:                //TILT_MAXANGLE
            configuration.tilt_maxangle = config_servo(2, 4, configuration.tilt_maxangle );
            tilt_servo.writeMicroseconds(configuration.tilt_maxpwm);
            //tilt_servo.write(180);
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                EEPROM_write(config_bank[int(current_bank)], configuration);
                //move_servo(tilt_servo, 2, 0, configuration.tilt_minangle, configuration.tilt_maxangle);
                vParkServos();
                current_activity=ActMenu;
            }
            break;
        case ActTestServo:               //TEST_SERVO
            test_servos();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                current_activity=ActMenu;
                test_servo_cnt = 360;
                test_servo_step = 1;
                vParkServos();
            }
            break;
        
        case ActSetTelemetrie:                //Configure Telemetry
            lcddisp_telemetry();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                EEPROM_write(config_bank[int(current_bank)], configuration);
                current_activity=ActMenu;
            }
            break;
        case ActSetBaud:                //Configure Baudrate
            lcddisp_baudrate();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                EEPROM_write(config_bank[int(current_bank)], configuration);
                current_activity=ActMenu;
            }
            break;
        case ActSetBank:                //Change settings bank
            lcddisp_bank();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                EEPROM.write(0,current_bank);
                EEPROM_read(config_bank[int(current_bank)], configuration);
                servoconf_tmp[0] = configuration.pan_minpwm;
                servoconf_tmp[1] = configuration.pan_maxpwm;
                servoconf_tmp[2] = configuration.tilt_minpwm;
                servoconf_tmp[3] = configuration.tilt_maxpwm;
                home_sent = 0;
                current_activity=ActMenu;
            }
            break;
        case ActSetOsd:                //Configure OSD
            lcddisp_osd();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                EEPROM_write(config_bank[int(current_bank)], configuration);
                home_sent = 0; // force resend an OFrame for osd update
                current_activity=ActMenu;
            }
            break;      
        case ActSetBearing:                //Configure bearing method
            lcddisp_bearing_method();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                EEPROM_write(config_bank[int(current_bank)], configuration);
                current_activity=ActMenu;
            }
            break;
        case ActSetVoltage:               //Configure voltage multiplier
            lcddisp_voltage_ratio();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{ 
                configuration.voltage_ratio = (uint16_t)(voltage_ratio * 100.0f);
                EEPROM_write(config_bank[int(current_bank)], configuration);
                current_activity=ActMenu;
            }
            break;
        case ActSetChannel:               //Config video channel
          	vShowSpectrum();
            if (buttonEnter.holdTime() >= 700 && buttonEnter.held()) //long press
			{
                EEPROM_write(config_bank[int(current_bank)], configuration);
                RX5808.vSelectChannel( configuration.channel );
                current_activity=ActMenu;
            }
            break;
    }         
}

//######################################## BUTTONS #####################################################################

void ButtonPressEvents(Button &btn)
{
	Buzzer.vClick();
}

void enterButtonReleaseEvents(Button &btn)
 {
    //DEBUG_SERIAL.println(current_activity);
    if ( buttonEnter.holdTime() < 700 ) // normal press    
	{ 
        if ( current_activity == ActMenu ) //button action depends activity state
		{ 
            displaymenu.select();
            vUpdateMenu();
        }
        else if ( current_activity == ActTrack )
        {
        	trackingDisplay = 1 - trackingDisplay;
        }
        else if ( current_activity == ActSetHome ) 
		{
            if ((gps_fix) && (!home_pos)) 
			{
                //saving home position
                home_lat = uav_lat;
                home_lon = uav_lon;
                home_alt = uav_alt;
                home_pos = true;
                calc_longitude_scaling(home_lat);  // calc lonScaleDown
            }
            else if ((gps_fix) && (home_pos) && (!home_bear)) 
			{            
                //set_bearing();
                switch (configuration.bearing_method) 
				{
                    case 1: 
                        home_bearing += calc_bearing(home_lon, home_lat, uav_lon, uav_lat); // store bearing relative to north
                        home_bear = true;
                        break;
                    case 2:
                    case 3:
                    case 4:
                        home_bear = true;
                        break;
                    default: 
                        configuration.bearing_method = 2; // shouldn't happened, restoring default value.
                        break;
                }
                configuration.bearing = home_bearing;
                EEPROM_write(config_bank[int(current_bank)], configuration); // why store?
                home_sent = 0;  // resend an OFrame to osd
            }
            else if ((gps_fix) && (home_pos) && (home_bear)) 
			{
              // START TRACKING 
              current_activity = ActTrack;
            }
        }
        
    }
}



void leftButtonReleaseEvents(Button &btn)
{
    if ( buttonDown.holdTime() < 700 ) 
	{
        if (current_activity==ActMenu) 
		{
            displaymenu.prev();
            vUpdateMenu();
        }       
        else if ( current_activity != ActMenu && current_activity != ActTrack && current_activity != ActSetHome ) 
		{
              //We're in a setting area: Left button decrase current value.
              switch (current_activity) 
			  {
                  case ActPanMinPwm:   		servoconf_tmp[0]--;            break;
                  case ActPanMinAngle:		configuration.pan_minangle--;  break;
                  case ActPanMaxPwm:		servoconf_tmp[1]--;            break;
                  case ActPanMaxAngle:		configuration.pan_maxangle--;  break;
                  case ActTiltMinPwm:		servoconf_tmp[2]--;            break;
                  case ActTiltMinAngle:		configuration.tilt_minangle--; break;
                  case ActTiltMaxPwm:		servoconf_tmp[3]--;            break;
                  case ActTiltMaxAngle:		configuration.tilt_maxangle--; break;
                  case ActSetTelemetrie:	if (configuration.telemetry > 0) configuration.telemetry -= 1;  break;
                  case ActSetBaud:  		if (configuration.baudrate > 0)  configuration.baudrate -= 1;   break;
                  case ActSetBank:  		if (current_bank > 0) current_bank -= 1; else current_bank = 3; break;
                  case ActSetOsd:  			if (configuration.osd_enabled == 0) configuration.osd_enabled = 1; else configuration.osd_enabled = 0;    break;
                  case ActSetBearing:  		if (configuration.bearing_method > 1) configuration.bearing_method -= 1; else configuration.bearing_method = 4; break;
                  case ActSetVoltage:  		if (voltage_ratio >= 1.0) voltage_ratio -= 0.01; break;
                  case ActSetChannel:      	configuration.channel = (configuration.channel - 1 ) & 0x1f; break;
             }                              
        }
        else if (current_activity==ActSetHome) 
		{
                if (configuration.bearing_method == 2) 
				{      
                    if (home_pos && !home_bear) 
					{
                        home_bearing--;
                        if (home_bearing<0)
						{ 
							home_bearing = 359;
						}
                    }
                }    
                if (gps_fix && home_pos && home_bear) 
				{
                    current_activity = ActMenu;
                }
        }
        else if (current_activity==ActTrack && home_pos && home_bear)
		{
            home_bearing--;
			if (home_bearing<0)
			{ 
				home_bearing = 359;			
			}
		}
    }
}


void rightButtonReleaseEvents(Button &btn)
{
    if ( buttonUp.holdTime() < 700 ) 
	{
        if (current_activity==ActMenu) 
		{
            displaymenu.next();
            vUpdateMenu();
        }
        else if ( current_activity != ActMenu && current_activity != ActTrack && current_activity != ActSetHome ) 
		{
            //We're in a setting area: Right button decrase current value.
            switch (current_activity) 
			{
                case ActPanMinPwm:  servoconf_tmp[0]++;            break;
                case ActPanMinAngle:  configuration.pan_minangle++;  break;
                case ActPanMaxPwm:  servoconf_tmp[1]++;            break;
                case ActPanMaxAngle:  configuration.pan_maxangle++;  break;
                case ActTiltMinPwm:  servoconf_tmp[2]++;            break;
                case ActTiltMinAngle:  configuration.tilt_minangle++; break;
                case ActTiltMaxPwm:  servoconf_tmp[3]++;            break;
                case ActTiltMaxAngle: configuration.tilt_maxangle++; break;
                case ActSetTelemetrie: if (configuration.telemetry < 6) configuration.telemetry += 1;  break;
                case ActSetBaud: if (configuration.baudrate  < 7) configuration.baudrate += 1;   break;
                case ActSetBank: if (current_bank < 3) current_bank += 1; else current_bank = 0; break;
                case ActSetOsd: if (configuration.osd_enabled == 0) configuration.osd_enabled = 1; else configuration.osd_enabled = 0;    break;
                case ActSetBearing: if (configuration.bearing_method < 5) configuration.bearing_method += 1; else configuration.bearing_method = 1; break;
                case ActSetVoltage: voltage_ratio += 0.01; break;
                case ActSetChannel: configuration.channel = (configuration.channel + 1 ) & 0x1f; break;
            }
        }
        else if (current_activity==ActSetHome) 
		{
            if (configuration.bearing_method == 2) 
			{ 
                if (home_pos && !home_bear) {
                    home_bearing++;
                    if(home_bearing>359)
					{
						home_bearing = 0;
					}
                }
            }   
            if (home_pos && home_bear) 
			{
                // reset home pos
                home_pos = false;
                home_bear = false;
                home_sent = 0;
            }
        }
        else if (current_activity==ActTrack && home_pos && home_bear) 
		{
            home_bearing++;
            if(home_bearing>359)
			{
				home_bearing = 0;
			}
        }
    }
}

//########################################################### MENU #######################################################################################

void init_menu() {
    rootMenu.add_item(&m1i1Item, &screen_tracking); //start track
    rootMenu.add_item(&m1i2Item, &screen_sethome); //set home position
    rootMenu.add_menu(&m1m3Menu); //configure
        m1m3Menu.add_menu(&m1m3m1Menu); //config servos
            m1m3m1Menu.add_menu(&m1m3m1m1Menu); //config pan
                m1m3m1m1Menu.add_item(&m1m3m1m1l1Item, &configure_pan_minpwm); // pan min pwm
                m1m3m1m1Menu.add_item(&m1m3m1m1l2Item, &configure_pan_maxpwm); // pan max pwm
                m1m3m1m1Menu.add_item(&m1m3m1m1l3Item, &configure_pan_minangle); // pan min angle
                m1m3m1m1Menu.add_item(&m1m3m1m1l4Item, &configure_pan_maxangle); // pan max angle
            m1m3m1Menu.add_menu(&m1m3m1m2Menu); //config tilt
                m1m3m1m2Menu.add_item(&m1m3m1m2l1Item, &configure_tilt_minpwm); // tilt min pwm
                m1m3m1m2Menu.add_item(&m1m3m1m2l2Item, &configure_tilt_maxpwm); // tilt max pwm
                m1m3m1m2Menu.add_item(&m1m3m1m2l3Item, &configure_tilt_minangle); // tilt min angle
                m1m3m1m2Menu.add_item(&m1m3m1m2l4Item, &configure_tilt_maxangle); // tilt max angle
                m1m3m1Menu.add_item(&m1m3m1i3Item, &configure_test_servo); // servo test
                m1m3m1Menu.add_item(&m1m3m1i4Item, &configure_manual_servo); // servo manual
        m1m3Menu.add_menu(&m1m3m2Menu);  //Telemetry
            m1m3m2Menu.add_item(&m1m3m2i1Item, &configure_telemetry); // select telemetry protocol ( Teensy++2 only ) 
            m1m3m2Menu.add_item(&m1m3m2i2Item, &configure_baudrate); // select telemetry protocol
        m1m3Menu.add_menu(&m1m3m3Menu);  //Others        
#ifdef OSD_OUTPUT
            m1m3m3Menu.add_item(&m1m3m3i1Item, &configure_osd); // enable/disable osd
#endif
            m1m3m3Menu.add_item(&m1m3m3i2Item, &configure_bearing_method); // select tracker bearing reference method
            m1m3m3Menu.add_item(&m1m3m3i3Item, &configure_voltage_ratio);    // set minimum voltage
            m1m3m3Menu.add_item(&m1m3m3i4Item, &screen_bank);    // set eeprom bank
    rootMenu.add_menu(&m1m4Menu); // VIDEO
	m1m4Menu.add_item(&m1m41i1Item, &configure_channel); // CHANNEL
	m1m4Menu.add_item(&m1m41i2Item, &configure_receiver); // RECEIVER
	m1m4Menu.add_item(&m1m41i3Item, &configure_diversity); // DIVERSITY

    displaymenu.set_root_menu(&rootMenu);
}



//menu item callback functions

void screen_tracking(MenuItem* p_menu_item) 
{
    current_activity = ActTrack;
}

void screen_sethome(MenuItem* p_menu_item) 
{
    current_activity = ActSetHome;
}

void configure_pan_minpwm(MenuItem* p_menu_item) 
{
    current_activity = ActPanMinPwm;
}

void configure_pan_minangle(MenuItem* p_menu_item) 
{
    current_activity = ActPanMinAngle;
}

void configure_pan_maxpwm(MenuItem* p_menu_item) 
{
    current_activity = ActPanMaxPwm;
}

void configure_pan_maxangle(MenuItem* p_menu_item) 
{
    current_activity = ActPanMaxAngle;
}

void configure_tilt_minpwm(MenuItem* p_menu_item) 
{
    current_activity = ActTiltMinPwm;
}

void configure_tilt_minangle(MenuItem* p_menu_item) 
{
    current_activity = ActTiltMinAngle;
}

void configure_tilt_maxpwm(MenuItem* p_menu_item) 
{
    current_activity = ActTiltMaxPwm;
}

void configure_tilt_maxangle(MenuItem* p_menu_item) 
{
    current_activity = ActTiltMaxAngle;
}

void configure_test_servo(MenuItem* p_menu_item)
{
    current_activity = ActTestServo;
}

void configure_manual_servo(MenuItem* p_menu_item)
{
    current_activity = ActTestServo;
}

void configure_telemetry(MenuItem* p_menu_item) 
{
    current_activity = ActSetTelemetrie;
}

void configure_baudrate(MenuItem* p_menu_item) 
{
    current_activity = ActSetBaud;
}

void screen_bank(MenuItem* p_menu_item) 
{
    current_activity = ActSetBank;
}

#ifdef OSD_OUTPUT
void configure_osd(MenuItem* p_menu_item) 
{
    current_activity = ActSetOsd;
}
#endif

void configure_bearing_method(MenuItem* p_menu_item) 
{
    current_activity = ActSetBearing;
}

void configure_voltage_ratio(MenuItem* p_menu_item) 
{
    current_activity = ActSetVoltage;
}

void configure_channel( MenuItem* p_menu_item )
{
	current_activity = ActSetChannel;
	vShowSpectrum();
}
void configure_receiver( MenuItem* p_menu_item )
{
	//current_activity = ActSetReceiver;
	if(RX5808.ui8GetReceiver()>0)
	{
		RX5808.vSelectReceiver(0);
		vShowMessage( "Receiver 0", 2, 1000 );
	}
	else
	{
		RX5808.vSelectReceiver(1);
		vShowMessage( "Receiver 1", 2, 1000 );
	}
}
void configure_diversity( MenuItem* p_menu_item )
{
	//current_activity = ActSetDiversity;
	if(RX5808.ui8GetDiversity()>0)
	{
		RX5808.vSelectDiversity(0);
		vShowMessage( "Diversity OFF", 2, 1000 );
	}
	else
	{
		RX5808.vSelectDiversity(1);
		vShowMessage( "Diversity ON", 2, 1000 );
	}
}

//######################################## TELEMETRY FUNCTIONS #############################################
void init_serial(void)
{
	DEBUG_SERIAL.begin(57600);	// always init the port for debug output
#ifdef OSD_OUTPUT
    OSD_SERIAL.begin(OSD_BAUD);
#endif
#ifdef PROTOCOL_HOTT
    vHottInit();
#else
    TELEMETRY_SERIAL.begin(baudrates[configuration.baudrate]);
#endif
#ifdef GHETTO_DEBUG
    DEBUG_SERIAL.println("Serial initialised");
#endif

}

//Preparing adding other protocol
void get_telemetry(void) 
{

   if (millis() - lastpacketreceived > 2000) 
   {
      telemetry_ok = false;     
   }
        
#if defined(PROTOCOL_UAVTALK) // OpenPilot / Taulabs 
   if (configuration.telemetry==0) 
   {
        if (uavtalk_read())
            protocol = "UAVT";
   }
#endif

#if defined(PROTOCOL_MSP) // Multiwii
    if (configuration.telemetry==1) 
	{
        if (!PASSIVEMODE) {
        	msp_read2();
        }
    msp_read(); 
    }
#endif

#if defined(PROTOCOL_LIGHTTELEMETRY) // Ghettostation light protocol. 
   if (configuration.telemetry==2) 
   {
      ltm_read();
   }
#endif

#if defined(PROTOCOL_MAVLINK) // Ardupilot / PixHawk / Taulabs ( mavlink output ) / Other
    if (configuration.telemetry==3) 
	{
        if(enable_frame_request == 1) //Request rate control
		{
            enable_frame_request = 0;
            if (!PASSIVEMODE) {
               request_mavlink_rates();
            }
        }
        read_mavlink(); 
    }
#endif

#if defined (PROTOCOL_NMEA)
   if (configuration.telemetry==4) 
   {
       gps_nmea_read();      
   }
#endif
#if defined (PROTOCOL_UBLOX)
   if (configuration.telemetry==5) 
   {
       gps_ublox_read();      
   }
#endif
#if defined (PROTOCOL_HOTT)
   if (configuration.telemetry==6) 
   {
	   vHottTelemetrie();
   }
#endif
}

//######################################## SERVOS #####################################################################

void attach_servo(Servo &s, int p, int min, int max) {
 // called at setup() or after a servo configuration change in the menu
	if (!s.attached()) 
	{
		s.attach(p,min,max);
    }
}

void detach_servo(Servo &s) 
{
 // called at setup() or after a servo configuration change in the menu
	if (s.attached()) 
	{
	    s.detach();
	}
}


void servoPathfinder(int16_t i16b, int16_t i16a) // ( bearing, elevation, valid interval is: 0 .. 359, min < max )
{   
//find the best way to move pan servo considering 0° reference face toward
	static bool inverse = false;
	int16_t i16PanRange,i16PanRangeInv;
	int16_t microsec;

	// Pan calculation:

	i16b -= home_bearing;				// home_bearing is the direction to N
	i16b -= configuration.pan_minangle;	// now calculations start at 0

	while( i16b < 0 ) 		i16b += 360;
	while( i16b > 359 ) 	i16b -= 360;

	i16PanRange = configuration.pan_maxangle - configuration.pan_minangle;	// i16PanRange is expected to be 0..180
	i16PanRangeInv = i16PanRange + 180;										// i16PanRangeInv is expected to be <360


	if( (i16b >= 0) && (i16b < i16PanRange) )
	{
		// i16b need no change, can be direct accessed by the servo
		inverse = false;
	}
	else if( (i16b >= 180) && (i16b <= i16PanRangeInv) )
	{
		// can be accessed with inverted tilt, i16b need a 180 deg shift
		i16b -= 180;
		inverse = true;
	}
	else if( (i16b >= i16PanRange) && (i16b < 180) )
	{
		// angle cannot be reached, stay at the border with the latest inverse state
		if( inverse ) i16b = 0;
		else i16b = i16PanRange;
	}
	else if( (i16b >= i16PanRangeInv) && (i16b < 360) )
	{
		// angle cannot be reached, stay at the border with the latest inverse state
		if( !inverse ) i16b = 0;
		else i16b = i16PanRange;
	}

	// Tilt calculation:
	if( inverse ) i16a = 180 - i16a;
	if( i16a < configuration.tilt_minangle )	i16a = configuration.tilt_minangle;
	if( i16a > configuration.tilt_maxangle )	i16a = configuration.tilt_maxangle;


    // write the servos:
	//microsec = map(a, 0, i16PanRange, configuration.pan_minpwm, configuration.pan_maxpwm);
    microsec = map(i16b, 0, i16PanRange, configuration.pan_maxpwm, configuration.pan_minpwm);	// pan servo move counter clockwise....
	pan_servo.writeMicroseconds( microsec );

    microsec = map(i16a, configuration.tilt_minangle, configuration.tilt_maxangle, configuration.tilt_minpwm, configuration.tilt_maxpwm);		// tilt servo
	tilt_servo.writeMicroseconds( microsec );
}


void test_servos(void) 
{
    lcddisp_testservo( 0 );
    switch (test_servo_step) 
	{
        case 1:        
            if (test_servo_cnt > 180) 
			{
				Bearing = test_servo_cnt;
				Elevation = (360-test_servo_cnt)/6;
                test_servo_cnt--; 
            }
            else
			{				
                test_servo_step = 2;
			}
            break;
        case 2:
            if (test_servo_cnt < 360) 
			{
				Bearing = test_servo_cnt;
				Elevation = (360-test_servo_cnt)/6;
                test_servo_cnt++;   
            }
            else 
			{
                test_servo_step = 3;
                test_servo_cnt = 0;
            }
            break;
        case 3:
            if (test_servo_cnt < 360) 
			{
				Bearing = test_servo_cnt;
				Elevation = test_servo_cnt/4;
                test_servo_cnt++;
            }
            else 
			{
                test_servo_step = 4;
                test_servo_cnt = 0;
            }
            break;
        case 4:
            if (test_servo_cnt < 360) 
			{
				Bearing = test_servo_cnt;
				Elevation = 90-(test_servo_cnt/4);
                test_servo_cnt++;
            }
            else {
                // finished
                test_servo_step = 1;
                current_activity = ActMenu;
				Bearing = DEFAULTBEARING;
				Elevation = DEFAULTELEVATION;
            }
            break;
    }
	servoPathfinder(Bearing,Elevation);
}

void vParkServos( void )
{
	servoPathfinder( DEFAULTBEARING+home_bearing, DEFAULTELEVATION );
}

//######################################## TRACKING #############################################

void antenna_tracking(void) 
{
// Tracking general function
    //only move servo if gps has a 3D fix, or standby to last known position.
    if (gps_fix && telemetry_ok) 
	{  
        rel_alt = uav_alt - home_alt; // relative altitude to ground in decimeters
        calc_tracking( home_lon, home_lat, uav_lon, uav_lat, rel_alt); //calculate tracking bearing/azimuth
    } 
}

void calc_tracking(int32_t lon1, int32_t lat1, int32_t lon2, int32_t lat2, int32_t alt) 
{
    //calculating Bearing & Elevation  in degree decimal
    Bearing = calc_bearing(lon1,lat1,lon2,lat2);
    Elevation = calc_elevation(alt);
}

int16_t calc_bearing(int32_t lon1, int32_t lat1, int32_t lon2, int32_t lat2) 
{
    float dLat = (lat2 - lat1);
    float dLon = (float)(lon2 - lon1) * lonScaleDown;
    home_dist = sqrt(sq(fabs(dLat)) + sq(fabs(dLon))) * 1.113195; // home dist in cm.
    int16_t b = (int)round( -90 + (atan2(dLat, -dLon) * 57.295775));
    if(b < 0)
	{
		b += 360; 
	}
    return b; 
}

int16_t calc_elevation(int32_t alt) 
{
    float at = atan2(alt, home_dist);
    at = at * 57,2957795;
    int16_t e = (int16_t)round(at);
    return e;
}

void calc_longitude_scaling(int32_t lat) 
{
    float rads = (abs((float)lat) / 10000000.0) * 0.0174532925;
    lonScaleDown = cos(rads);
}



//######################################## COMPASS #############################################

void retrieve_mag() 
{
#if defined(COMPASS)
// Retrieve the raw values from the compass (not scaled).
    MagnetometerRaw raw = compass.ReadRawAxis();
// Retrieved the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = compass.ReadScaledAxis();
//
// Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(scaled.YAxis, scaled.XAxis);

// Once you have your heading, you must then add your ‘Declination Angle’, which is the ‘Error’ of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/

    float declinationAngle = MAGDEC / 1000; 
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if (heading < 0)
	{
		heading += 2*PI;
	}
    
    // Check for wrap due to addition of declination.
    if (heading > 2*PI)
	{
		heading -= 2*PI;
	}
    
    // Convert radians to degrees for readability.
    home_bearing = (int)round(heading * 180/M_PI);
#endif
}

//######################################## BATTERY ALERT#######################################

void read_voltage( void )
{
	static uint8_t s=0;
	uint16_t ui16Volt=0;

	for( uint8_t i=0; i<8; i++ )
	{
		ui16Volt += analogRead(ADC_VOLTAGE);
	}
	ui16Volt >>= 3;
	voltage_actual = (float)(ui16Volt);
	voltage_actual = voltage_actual / 1024.0 * VOLTAGE_REF * voltage_ratio;

	if( s == 0 )	// count lipo cells on first call..
	{
		while( voltage_actual > ( (s+1) * LIPO_MIN_VOLTAGE) ) s++;
	}

    if (voltage_actual <= ( s * MIN_VOLTAGE2) )
    	Buzzer.vsetStatus( BUZZER_ALARM );
    else if (voltage_actual <= ( s * MIN_VOLTAGE1  ))
         Buzzer.vsetStatus( BUZZER_WARN );
    else
    	Buzzer.vsetStatus( BUZZER_IDLE );
}

//######################################## EEPROM #############################################

template <class T> int EEPROM_write(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    cli();
    for (i = 0; i < sizeof(value); i++)
	{
          EEPROM.write(ee++, *p++);
	}
    sei();
    return i;
}

template <class T> int EEPROM_read(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    cli();
    for (i = 0; i < sizeof(value); i++)
	{
          *p++ = EEPROM.read(ee++);
	}
    sei();
    return i;
}




void clear_eeprom( void )
{
    // clearing eeprom
    cli();
    for (int i = 0; i < 1025; i++)
    {
        EEPROM.write(i, 0);
    }

	// eeprom is clear  we can write default config
    //writing 4 setting banks.
    for (int j = 0; j < 4; j++)
    {
	    configuration.config_crc = CONFIG_VERSION;  // config version check
	    configuration.pan_minpwm = PAN_MINPWM;
	    configuration.pan_minangle = PAN_MINANGLE;
	    configuration.pan_maxpwm = PAN_MAXPWM;
	    configuration.pan_maxangle = PAN_MAXANGLE;
	    configuration.tilt_minpwm = TILT_MINPWM;
	    configuration.tilt_minangle = TILT_MINANGLE;
	    configuration.tilt_maxpwm = TILT_MAXPWM;
        configuration.tilt_maxangle = TILT_MAXANGLE;
	    configuration.baudrate = 6;
        configuration.telemetry = 0;
        configuration.bearing = 0;
        configuration.osd_enabled = 0;
        configuration.bearing_method = 1;
        configuration.voltage_ratio = VOLTAGE_RATIO;  // ratio*10
	    EEPROM_write(config_bank[j], configuration);
    }
	sei();
}

//######################################## DEBUG #############################################


#if defined(GHETTO_DEBUG)
void debug(void) 
{
	DEBUG_SERIAL.print("activ:");
	DEBUG_SERIAL.println(current_activity);
	DEBUG_SERIAL.print("conftelem:");
	DEBUG_SERIAL.println(configuration.telemetry);
	DEBUG_SERIAL.print("baud");
	DEBUG_SERIAL.println(configuration.baudrate);
	DEBUG_SERIAL.print("lat=");
	DEBUG_SERIAL.println(uav_lat/10000000.0,7);
	DEBUG_SERIAL.print("lon=");
	DEBUG_SERIAL.println(uav_lon/10000000.0,7);
	DEBUG_SERIAL.print("alt=");
	DEBUG_SERIAL.println(uav_alt);
	DEBUG_SERIAL.print("rel_alt=");
	DEBUG_SERIAL.println(rel_alt);
	DEBUG_SERIAL.print(uav_groundspeed);
	DEBUG_SERIAL.println(uav_groundspeed);
	DEBUG_SERIAL.print("dst=");
	DEBUG_SERIAL.println(home_dist);
	DEBUG_SERIAL.print("El:");
	DEBUG_SERIAL.println(Elevation);
	DEBUG_SERIAL.print("Be:");
	DEBUG_SERIAL.println(Bearing);
	DEBUG_SERIAL.print("H Be:");
	DEBUG_SERIAL.println(home_bearing);
	DEBUG_SERIAL.print("uav_fix_type=");
	DEBUG_SERIAL.println(uav_fix_type);
	DEBUG_SERIAL.print("uav_satellites_visible=");
	DEBUG_SERIAL.println(uav_satellites_visible);
	DEBUG_SERIAL.print("pitch:");
	DEBUG_SERIAL.println(uav_pitch);
	DEBUG_SERIAL.print("roll:");
	DEBUG_SERIAL.println(uav_roll);
	DEBUG_SERIAL.print("yaw:");
	DEBUG_SERIAL.println(uav_heading);
	DEBUG_SERIAL.print("rbat:");
	DEBUG_SERIAL.println(uav_bat);
	DEBUG_SERIAL.print("amp:");
	DEBUG_SERIAL.println(uav_amp);
	DEBUG_SERIAL.print("rssi:");
	DEBUG_SERIAL.println(uav_rssi);
	DEBUG_SERIAL.print("aspeed:");
	DEBUG_SERIAL.println(uav_airspeed);
	DEBUG_SERIAL.print("armed:");
	DEBUG_SERIAL.println(uav_arm);
	DEBUG_SERIAL.print("fs:");
	DEBUG_SERIAL.println(uav_failsafe);
	DEBUG_SERIAL.print("fmode:");
	DEBUG_SERIAL.println(uav_flightmode);
	DEBUG_SERIAL.print("armfsmode");
	DEBUG_SERIAL.println(ltm_armfsmode);
}
#endif





