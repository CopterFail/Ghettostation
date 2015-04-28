
//#include <avr/pgmspace.h>
#include <Arduino.h>

#include "defines.h"
#include "boards.h"
#include "globals.h"
#include "common.h"
#include "buzzer.h"
#include "rx5808.h"



//################################### SETTING OBJECTS ###############################################
// Set the pins on the I2C chip used for LCD connections:
// addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

  Adafruit_ST7735 tft = Adafruit_ST7735(PIN_CS, PIN_DC, PIN_MOSI, PIN_SCLK, PIN_RST);
  //Adafruit_ST7735 tft = Adafruit_ST7735(PIN_CS, PIN_DC, PIN_RST); // hardware spi

#include <MenuSystem.h>
#include <Button.h>

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	K_BLUE  0x0010
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

extern MenuSystem displaymenu;
extern Button buttonUp;
extern Button buttonDown;
extern Button buttonEnter;

extern cBuzzer Buzzer;
extern cRX5808 RX5808;

void retrieve_mag( void );

static bool bMenuUpdate;
static bool bDataUpdate;

void read_voltage( void ); // defined in ghettostation.ino



void vShowSoftkeys( const char *text1, const char *text2, const char *text3 )
{
	const uint8_t w=53;	 // 10 chars?
	int8_t x;
	tft.setTextColor( ST7735_BLACK );
	tft.setTextSize(1);
	tft.fillRect( 0, 118, w, 10, ST7735_BLUE );
	x = ( w - strlen(text1)*6 ) / 2;
	tft.setCursor( x, 118+1 );
	tft.print(text1);
	tft.fillRect( w, 118, w, 10, ST7735_RED );
	x = ( w - strlen(text2)*6 ) / 2;
	tft.setCursor( w+x, 118+1 );
	tft.print(text2);
	tft.fillRect( w+w, 118, w, 10, ST7735_GREEN );
	x = ( w - strlen(text3)*6 ) / 2;
	tft.setCursor( w+w+x, 118+1 );
	tft.print(text3);
}

void vPrepareDataSection( void )
{
	tft.fillRect( 0, 64, 160, 64, ST7735_BLACK );
	tft.setTextColor(ST7735_YELLOW);
	tft.setTextSize(1);
	tft.setCursor( 0, 64+5 );
}

void vShowBar( uint8_t slot, uint8_t value )
{
	tft.setTextColor( ST7735_BLACK );
	tft.setTextSize(1);
	tft.fillRect( 0, 64+10*slot, value, 10, ST7735_GREEN );
	tft.fillRect( value, 64+10*slot, 160-value , 10, ST7735_BLACK );
	tft.setCursor( 10, 64+10*slot+1 );
	tft.print(value);
	tft.print("%");
	//vShowSoftkeys( "","EXIT","" );
}

void vShowGpsStatus( void )
{
	char currentline[21];
	if (telemetry_ok)
	{
		sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type);
	}
	else
	{		
		sprintf(currentline,"P:%s NO TELEMETRY", protocol);
	}
	tft.println(currentline);
}

void vShowGpsData( void )
{
	char currentline[21];
	char bufferl[10];
    char bufferL[10];

	if (gps_fix)
	{
		sprintf(currentline, "3D FIX! Alt:%dm",(int16_t)round(uav_alt/100.0f));  
		tft.println(currentline);			
        sprintf(currentline,"%s %s", dtostrf(uav_lat/10000000.0, 5, 5, bufferl),dtostrf(uav_lon/10000000.0, 5, 5, bufferL));
		tft.println(currentline);	
	}
	else
	{
		tft.println("NO GPS 3D FIX");
	}
}

void vShowBattery( void )
{
  read_voltage();
  tft.print( "Battery[V]:" );
  tft.println( voltage_actual );
}

void vShowRssi( void )
{
  read_voltage();
  tft.print( "RSSI[%]:" );
  tft.println( RX5808.ui8GetRSSI( RX5808.ui8GetReceiver() ));
}

void vShowSpectrum( uint8_t *data, uint8_t channel )
{
	uint8_t y;
	uint16_t color;
	tft.fillRect( 0, 64, 160, 64, ST7735_BLACK );
	for( uint8_t i=1U; i<33U; i++ )
	{
		if( i==channel )
			color = ST7735_RED;
		else
			color = ST7735_GREEN;
		y = *data >> 1;
		tft.fillRect( i<<2, 115-y, 4, y, color );
		data++;
	}
	tft.setTextColor( ST7735_RED );
	tft.setTextSize(2);
	tft.setCursor( 134, 86 );
	tft.print(channel);
	vShowSoftkeys( "PREV","EXIT","NEXT" );
}

void showMenu( void )
{
  if( !bMenuUpdate ) return;
  // Display the menu
  Menu const* cp_menu = displaymenu.get_current_menu();

  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.println(cp_menu->get_name()); //Current menu name
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("");

  MenuComponent const* cp_menu_sel = cp_menu->get_selected();
  for (int i = 0; i < cp_menu->get_num_menu_components(); ++i)
  {
    MenuComponent const* cp_m_comp = cp_menu->get_menu_component(i);

    if (cp_menu_sel == cp_m_comp)
	{
		tft.setTextColor(ST7735_RED);
		tft.setTextSize(2);
	}
    tft.println(cp_m_comp->get_name());
	tft.setTextColor(ST7735_YELLOW);
	tft.setTextSize(1);
  }
  
  vPrepareDataSection();
  vShowBattery();
  vShowRssi();
  
  vShowSoftkeys( "DOWN","SEL/EXIT","UP" );
  bMenuUpdate = false;
  //tft.drawFastHLine(0,63,160,WHITE); // check the limit
}

void init_lcdscreen( void )
{
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(false);
  
  tft.setTextSize(2); // 1=5x7, 2=10x14
  tft.println("FPV STATION");
  tft.setTextSize(1);
  tft.println("Rev 1.0.0-dev");
  
  vShowBattery();
  tft.println("Scan video channels...");
  vShowSoftkeys( "","WAIT","" );

  RX5808.vSelectReceiver(0);
  RX5808.ui8ScanChannels(1);
  
  bMenuUpdate=true;
  bDataUpdate=true;
}

void vUpdateMenu()
{
	bMenuUpdate = true;
}

void vUpdateData( void )
{
	bDataUpdate = true;
}

void lcddisp_sethome( void ) 
{
	vPrepareDataSection();
	vShowGpsStatus();
	vShowGpsData();
	if (!gps_fix)
	{
		tft.println("WAITING");
	}
	vShowSoftkeys( "","QUIT(long)","" );
}

void lcddisp_setbearing( void ) 
{
    switch (configuration.bearing_method) {
        case 2:
            if (buttonUp.holdTime() >= 700 && buttonUp.isPressed() ) {
                home_bearing+=10;
                if (home_bearing > 359) 
                    home_bearing = 0;
                delay(500);
                }
            else if ( buttonDown.holdTime() >= 700 && buttonDown.isPressed() ) {
                home_bearing-=10;
                if (home_bearing < 0) 
                    home_bearing = 359;
                delay(500);   
            }
            break;
        case 3:
            home_bearing = uav_heading;  // use compass data from the uav. 
            break;
        case 4:
            retrieve_mag();
            break;
        default:
            break;
    }
	
	vPrepareDataSection();
	vShowGpsStatus();
	if (configuration.bearing_method != 1) 
	{
		tft.println("Set Heading");
	}
	
	if (configuration.bearing_method == 2)
	{
		vShowSoftkeys( "-","SELECT","+" );
	}
	else
	{
		vShowSoftkeys( "","SELECT","" );
	}
}

void lcddisp_homeok( void )
{
	char currentline[21];
	vPrepareDataSection();
	if (!telemetry_ok)
	{
		tft.println("P:NO TELEMETRY"); 
	}
    else
    {
    	sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type);
    	tft.println(currentline);
    }
    tft.println("HOME IS SET");
    tft.println("Enter:Start Tracking");
    tft.println("<< Menu     Reset >>");
    vShowSoftkeys( "MENU","TRACKING","RESET" );
}

void lcddisp_tracking( void )
{
	char currentline[21];
	vPrepareDataSection();
	vShowGpsStatus();
    sprintf(currentline, "Alt:%dm Spd:%d", (int)round(rel_alt/100.0f), uav_groundspeed);
	tft.println(currentline);
    sprintf(currentline, "Dist:%dm Hdg:%d", (int)round(home_dist/100.0f), uav_heading);
	tft.println(currentline);
	vShowGpsData();
	vShowSoftkeys( "","QUIT(long)","" );
}

void lcddisp_telemetry( void ) 
{
	vPrepareDataSection();
	tft.println("SELECT PROTOCOL:");
    switch (configuration.telemetry) 
	{
    case 0:	tft.println("UAVTalk"); break;
    case 1:	tft.println("MSP"); break;
    case 2:	tft.println("LTM"); break;
    case 3:	tft.println("MavLink"); break;
    case 4:	tft.println("NMEA"); break;
    case 5:	tft.println("UBLOX"); break;
    default:
    case 6:	tft.println("HoTT"); break;
	}
	vShowSoftkeys( "PREV","SELECT","NEXT" );
}

void lcddisp_baudrate( void ) 
{
	vPrepareDataSection();
	tft.println("SELECT BAUDRATE:");
    switch (configuration.baudrate) 
	{
    case 0:	tft.println("1200"); break;
    case 1:	tft.println("2400"); break;
    case 2:	tft.println("4800"); break;
    case 3:	tft.println("9600"); break;
    case 4:	tft.println("19200"); break;
    case 5:	tft.println("38400"); break;
    case 6:	tft.println("57600"); break;
    case 7:	tft.println("115200"); break;
	}
	vShowSoftkeys( "PREV","SELECT","NEXT" );
}

// Settings Bank config
void lcddisp_bank( void ) 
{
	vPrepareDataSection();
	tft.println("SELECT BANK:");
    switch (current_bank) 
	{
    case 0:	tft.println("1.2 GHZ"); break;
    case 1:	tft.println("5.8 GHZ"); break;
    case 2:	tft.println("BANK 3"); break;
    case 3:	tft.println("BANK 4"); break;
	}
	vShowSoftkeys( "PREV","SELECT","NEXT" );
}

void lcddisp_osd( void ) 
{
	vPrepareDataSection();
	tft.println("ENABLE OSD:");
    switch (configuration.osd_enabled) 
	{
    case 0:	tft.println("NO"); break;
    case 1:	tft.println("YES"); break;
	}
	vShowSoftkeys( "PREV","SELECT","NEXT" );
}

void lcddisp_bearing_method( void ) 
{
	vPrepareDataSection();
	tft.println("BEARING METHOD");
    switch (configuration.bearing_method) 
	{
    case 1:	tft.println("MSP"); break;
    case 2:	tft.println("LTM"); break;
    case 3:	tft.println("MavLink"); break;
    case 4:	tft.println("NMEA"); break;
	}
	vShowSoftkeys( "PREV","SELECT","NEXT" );
}


void lcddisp_voltage_ratio( void ) 
{
	char currentline[21];
	char bufferV[6];
	
    read_voltage();
    if (buttonUp.holdTime() >= 700 && buttonUp.isPressed() ) 
	{
		voltage_ratio += 0.1;
        delay(500);
    }
    else if ( buttonDown.holdTime() >= 700 && buttonDown.isPressed() ) 
	{
		voltage_ratio -= 0.1;
		delay(500);
    }
	
	vPrepareDataSection();
	tft.println("ADJUST VOLTAGE RATIO");
	vShowBattery();
	sprintf(currentline,"Ratio:  %s ", dtostrf(voltage_ratio, 3, 2, bufferV));
	tft.println(currentline);

	vShowSoftkeys( "-","QUIT(long)","+" );
}

void lcddisp_testservo( void ) 
{
	vPrepareDataSection();
	tft.println("TESTING SERVOS");
	tft.println("CONFIGURATION");
	vShowSoftkeys( "","QUIT(long)","" );
}

int config_servo(int servotype, int valuetype, int value ) 
{
    char currentline[21];
    char extract[21];
	
    if (buttonUp.holdTime() >= 700 && buttonUp.isPressed() ) 
	{
		value+=20;
        delay(500);
    }
    else if ( buttonDown.holdTime() >= 700 && buttonDown.isPressed() ) 
	{
		value-=20;
        delay(500);
    }
	
	vPrepareDataSection();
	if (servotype==1)
	{
		tft.println("[PAN SERVO]");
	}
	else
	{
		tft.println("[TILT SERVO]");
	}
	    
	switch (valuetype) 
    {           
        case 1: sprintf(currentline, "min endpoint: <%4d>",  value); break;          //minpwm
        case 2: sprintf(currentline, "min angle: <%3d>    ", value); break;         //minangle
        case 3: sprintf(currentline, "max endpoint: <%4d>",  value); break;          //maxpwm
        case 4: sprintf(currentline, "max angle: <%3d>    ", value); break;         //maxangle
    }
	tft.println(currentline);
	vShowSoftkeys( "-","QUIT(long)","+" );
	
    return value;
}
