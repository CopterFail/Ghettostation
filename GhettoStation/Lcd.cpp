
#include <avr/pgmspace.h>
#include <Arduino.h>

#include <Flash.h>

#include "defines.h"
#include "boards.h"
#include "globals.h"
#include "common.h"
#include "text.h"



//################################### SETTING OBJECTS ###############################################
// Set the pins on the I2C chip used for LCD connections:
// addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
#ifdef LCD03I2C
  #include <LCD03.h>
  LCD03 LCD(I2CADRESS);
#endif
#ifdef LCDLCM1602
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C LCD(I2CADRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  //   HobbyKing IIC/I2C/TWI Serial 2004 20x4, LCM1602 IIC A0 A1 A2 & YwRobot Arduino LCM1602 IIC V1
#endif
#ifdef LCDGYLCD
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C LCD(I2CADRESS, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);  //   Arduino-IIC-LCD GY-LCD-V1
#endif
#ifdef GLCDEnable
  #include <glcd.h>
  #include "fonts/SystemFont5x7.h"
#endif

#ifdef OLEDLCD
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define OLED_RESET 4
  Adafruit_SSD1306 display(OLED_RESET);
#endif

#ifdef LCDST7735
  #include <Adafruit_GFX.h>    // Core graphics library
  //#include <Adafruit_mfGFX.h>    // Core graphics library
  #include <Adafruit_ST7735.h> // Hardware-specific library
  #include <SPI.h>

  Adafruit_ST7735 tft = Adafruit_ST7735(PIN_CS, PIN_DC, PIN_MOSI, PIN_SCLK, PIN_RST);
  //Adafruit_ST7735 tft = Adafruit_ST7735(PIN_CS, PIN_DC, PIN_RST); // hardware spi
#endif


#include <MenuSystem.h>
#include <Button.h>

extern MenuSystem displaymenu;
extern Button right_button;
extern Button left_button;
extern Button enter_button;
void retrieve_mag( void );


char lcd_line1[21];
char lcd_line2[21];
char lcd_line3[21];
char lcd_line4[21];
static bool bLcdUpdate;

void read_voltage( void ); // defined in ghettostation.ino

void init_lcdscreen( void )
{
#ifdef GHETTO_DEBUG
    Serial.println("starting lcd"); 
#endif

  bLcdUpdate=false;
  read_voltage();
  char extract[20];

#ifdef OLEDLCD
  display.begin(SSD1306_SWITCHCAPVCC,I2CADRESS);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer
  // init done
#endif


// init LCD
#ifdef OLEDLCD
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(string_load1.copy(extract));
  display.println(string_load2.copy(extract));
  display.println(string_load3.copy(extract));
  char currentline[21];
  char bufferV[6];
  sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
  display.println(currentline);
  display.display();
  delay(2500); //delay to init lcd in time.
  display.clearDisplay();
#endif

#ifdef GLCDEnable
    GLCD.Init(NON_INVERTED);
    GLCD.SelectFont(System5x7);
    LCD.begin(20,4); // No idea why this is still needed for GLCD, but it breaks without
    GLCD.CursorTo(0,0);
    GLCD.println(string_load1.copy(extract));
    GLCD.println(string_load2.copy(extract));
    GLCD.println(string_load3.copy(extract));
    char currentline[21];
    char bufferV[6];
    sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
    GLCD.println(currentline);
    delay(1500); //delay to init lcd in time.
#endif

#ifdef LCDST7735
    tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
    tft.fillScreen(ST7735_BLACK);
    //delay(500);
    tft.setRotation(3);
    tft.setTextSize(1); // 1=5x7, 2=10x14
    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextWrap(false);
    tft.println("01234567890123456789");
    tft.println(string_load1.copy(extract));
    tft.println(string_load2.copy(extract));
    tft.println(string_load3.copy(extract));
    char currentline[21];
    char bufferV[6];
    sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
    tft.println(currentline);
    delay(500);
#endif
#ifdef LCD03I2C
    LCD.begin(20,4);
        delay(20);
    LCD.backlight();
    	delay(250);
    LCD.noBacklight();
    	delay(250);
    LCD.backlight();
        delay(250);
    LCD.setCursor(0,0);
    LCD.print(string_load1.copy(extract));
    LCD.setCursor(0,1);
    LCD.print(string_load2.copy(extract));
    LCD.setCursor(0,2);
    LCD.print(string_load3.copy(extract));
    LCD.setCursor(0,3);
    	char currentline[21];
    	char bufferV[6];
    	sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
    LCD.print(currentline);
    	delay(1500); //delay to init lcd in time.
#endif
#ifdef LCDLCM1602
    LCD.begin(20,4);
        delay(20);
    LCD.backlight();
    	delay(250);
    LCD.noBacklight();
    	delay(250);
    LCD.backlight();
        delay(250);
    LCD.setCursor(0,0);
    LCD.print(string_load1.copy(extract));
    LCD.setCursor(0,1);
    LCD.print(string_load2.copy(extract));
    LCD.setCursor(0,2);
    LCD.print(string_load3.copy(extract));
    LCD.setCursor(0,3);
    	char currentline[21];
    	char bufferV[6];
    	sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
    LCD.print(currentline);
    	delay(1500); //delay to init lcd in time.
#endif
#ifdef LCDGYLCD
    LCD.begin(20,4);
        delay(20);
    LCD.backlight();        
    	delay(250);         
    LCD.noBacklight();      
    	delay(250);
    LCD.backlight(); 
        delay(250);
    LCD.setCursor(0,0);
    LCD.print(string_load1.copy(extract));
    LCD.setCursor(0,1);
    LCD.print(string_load2.copy(extract));
    LCD.setCursor(0,2);
    LCD.print(string_load3.copy(extract));
    LCD.setCursor(0,3);
    	char currentline[21];
    	char bufferV[6];
    	sprintf(currentline,"Battery: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
    LCD.print(currentline);
    	delay(1500); //delay to init lcd in time.
#endif
}

void store_lcdline( int i, char sbuffer[20] ) {
    
    switch (i) {
        case 1: 
        	    if(strcmp(lcd_line1,sbuffer))
        	    {
        	    	bLcdUpdate=true;
        	    }
                strcpy(lcd_line1,sbuffer);
                break;
        case 2: 
    	    if(strcmp(lcd_line2,sbuffer))
    	    {
    	    	bLcdUpdate=true;
    	    }
                strcpy(lcd_line2,sbuffer);
                break;
        case 3: 
    	    if(strcmp(lcd_line3,sbuffer))
    	    {
    	    	bLcdUpdate=true;
    	    }
                strcpy(lcd_line3,sbuffer);
                break;
        case 4: 
    	    if(strcmp(lcd_line4,sbuffer))
    	    {
    	    	bLcdUpdate=true;
    	    }
                strcpy(lcd_line4,sbuffer);
                break;
        default: 
                break;
    }

}

void refresh_lcd() {
// refreshing lcd at defined update.
// update lines

    if(!bLcdUpdate) return;
    bLcdUpdate=false;

#ifdef OLEDLCD
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(lcd_line1);
    	display.println(lcd_line2);
    	display.println(lcd_line3);
        display.println(lcd_line4);
        display.display();
        delay(100);
#endif
#ifdef GLCDEnable
       	GLCD.CursorTo(0,0);
	    GLCD.println(lcd_line1);
    	GLCD.println(lcd_line2);
    	GLCD.println(lcd_line3);
        GLCD.println(lcd_line4);
#endif
#ifdef LCDST7735
        tft.fillScreen(ST7735_BLACK);
        tft.setTextColor(ST7735_WHITE);
        tft.setCursor(0, 0);
        //lcd_line1[18] = 0;	// cut to 18 char per line
        //lcd_line2[18] = 0;
        //lcd_line3[18] = 0;
        //lcd_line4[18] = 0;
        tft.println("01234567890123456789");
        tft.println(lcd_line1);
        tft.println(lcd_line2);
        tft.println(lcd_line3);
        tft.println(lcd_line4);

        // print bar graph for battery and rssi with selected band and channel
        uint16_t x = (int)(voltage_actual / 7.4) * 100;
        uint16_t color = ST7735_GREEN;
        //if( Buzzer.getStatus() == BUZZER_WARN ) color = ST7735_YELLOW;
        //if( Buzzer.getStatus() == BUZZER_ALARM )color = ST7735_RED;
        if(x>100)x=100;
        tft.fillRect( 10, 64 , x, 14, color );
        tft.fillRect( 10+x, 64 , 160-x, 14, ST7735_WHITE );

        tft.setTextColor(ST7735_BLUE);
        tft.setCursor(20, 64+2);
        tft.print("Bat: ");
        tft.print(voltage_actual);

        x = analogRead(ADC_RSSI) / 1024 * 100;
        x = x + 50;
        if(x>100)x=100;
        color = ST7735_GREEN;
        tft.fillRect( 10, 84 , x, 14, color );
        tft.fillRect( 10+x, 84 , 160-x, 14, ST7735_WHITE );
        tft.setCursor(20, 84+2);
        tft.print("RSSI: ");
        tft.print(x);

#else
	LCD.setCursor(0,0);
	LCD.print(lcd_line1);
        LCD.setCursor(0,1);
    	LCD.print(lcd_line2);
        LCD.setCursor(0,2);
    	LCD.print(lcd_line3);
        LCD.setCursor(0,3);
        LCD.print(lcd_line4);
#endif
}

void lcddisp_setupdateflag()
{
	bLcdUpdate = true;
}

void lcddisp_menu() {
    Menu const* displaymenu_current = displaymenu.get_current_menu();
    MenuComponent const* displaymenu_sel = displaymenu_current->get_selected();
    
    uint8_t selected_item;
    uint8_t menu_components_number;
    uint8_t m;
    selected_item = displaymenu_current->get_cur_menu_component_num();
    menu_components_number = displaymenu_current->get_num_menu_components();  
    for (int n = 1; n < 5 ; n++)  {      
        char currentline[21];
        if ( menu_components_number >= n ) {
            if (menu_components_number <= 4)
                m = n; 
            else if (selected_item < (menu_components_number - selected_item - 1))
                m =  selected_item + n ;
            else 
                m =  menu_components_number - (menu_components_number - n - 1);
            MenuComponent const* displaymenu_comp = displaymenu_current->get_menu_component(m - 1);
            sprintf(currentline,displaymenu_comp->get_name());
            for ( int l = strlen(currentline); l<19 ; l++ ) {
                strcat(currentline," ");
            }             
            if (displaymenu_sel == displaymenu_comp) 
                strcat(currentline,"<");
            else 
                strcat(currentline," ");
         }
         else {
             string_load2.copy(currentline);
         }
         store_lcdline(n, currentline);
    }
}


// SET_HOME SCREEN
void lcddisp_sethome() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21] = "";
        char extract[21];
        switch (i) {
            case 1:
                //line1
                if (!telemetry_ok) { 
                    strcpy(currentline, "P:NO TELEMETRY"); }
                else if (telemetry_ok) { 
                    sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type);
                }
                break;
            case 2:
                //line 2
                if (!telemetry_ok) 
                    string_shome1.copy(currentline); // waiting for data
                else 
                {
                    if (!gps_fix)
                        string_shome2.copy(currentline);  // waiting for gps fix
                    else {
                        sprintf(currentline, "%s%dm",string_shome3.copy(extract),(int)round(uav_alt/100.0f));    
                    }
                }
                break;
      
            case 3:
                if (!gps_fix) strcpy(currentline, string_shome4.copy(extract));
                    else {
                        char bufferl[10];
                        char bufferL[10];
                        sprintf(currentline,"%s %s", dtostrf(uav_lat/10000000.0, 5, 5, bufferl),dtostrf(uav_lon/10000000.0, 5, 5, bufferL));
                    }
                break;
            case 4:
                if (!gps_fix) 
                    strcpy(currentline,string_shome5.copy(extract));
                else 
                    string_shome6.copy(currentline);
                break;
            }
 
        for ( int l = strlen(currentline); l<20 ; l++ )
            strcat(currentline," ");
        store_lcdline(i,currentline);
    }  
}

void lcddisp_setbearing() {
    switch (configuration.bearing_method) {
        case 2:
            if (right_button.holdTime() >= 700 && right_button.isPressed() ) {
                home_bearing+=10;
                if (home_bearing > 359) 
                    home_bearing = 0;
                delay(500);
                }
            else if ( left_button.holdTime() >= 700 && left_button.isPressed() ) {
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
    for (int i = 1 ; i<5; i++) {            
        char currentline[21] = "";
        char extract[21];
        switch (i) {
            case 1: 
                if (!telemetry_ok) 
                {
                    strcpy(currentline,"P:NO TELEMETRY"); 
                }
                else if (telemetry_ok)
                    sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type); 
                break;
            case 2:
               if (configuration.bearing_method == 1) 
                   string_load2.copy(currentline);  
               else
                   string_shome7.copy(currentline);
               break;
            case 3:
                if (configuration.bearing_method == 1)
                    string_shome8.copy(currentline);
                else if (configuration.bearing_method == 2)
                    sprintf(currentline, "     << %3d >>", home_bearing);
                else
                    sprintf(currentline, "        %3d   ", home_bearing);
                break;     
            case 4:      
                string_shome9.copy(currentline); break;
            default:
                break;
    
       }
       for ( int l = strlen(currentline); l<20 ; l++ ) {
           strcat(currentline," ");
       }
       store_lcdline(i,currentline);
    }
}

void lcddisp_homeok() {
    for ( int i = 1 ; i<5; i++ ) {
       char currentline[21] = "";
       switch (i) {
            case 1: 
                if (!telemetry_ok) { strcpy(currentline, "P:NO TELEMETRY"); }
                else if (telemetry_ok) sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type);
                break;
            case 2:
                string_shome10.copy(currentline); break;
            case 3:
                string_shome11.copy(currentline); break;                
            case 4:
                string_shome12.copy(currentline); break;
            }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
    store_lcdline(i,currentline);
    }
}

void lcddisp_tracking(){
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        switch (i) {
            case 1: 
                if (!telemetry_ok)
                    strcpy(currentline, "P:NO TELEMETRY");
                else if (telemetry_ok) 
                    sprintf(currentline,"P:%s SATS:%d FIX:%d", protocol, uav_satellites_visible, uav_fix_type);
                break;
           case 2:
                sprintf(currentline, "Alt:%dm Spd:%d", (int)round(rel_alt/100.0f), uav_groundspeed);
                break;
           case 3:
                sprintf(currentline, "Dist:%dm Hdg:%d", (int)round(home_dist/100.0f), uav_heading);
                break;
           case 4:   
                char bufferl[10];
                char bufferL[10];
                sprintf(currentline,"%s %s", dtostrf(uav_lat/10000000.0, 5, 5, bufferl),dtostrf(uav_lon/10000000.0, 5, 5, bufferL));
                break;
        }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
        strcat(currentline," ");
        }
        store_lcdline(i,currentline);
    }
}

void lcddisp_telemetry() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_telemetry1.copy(currentline);  break;
            case 2:
                string_load2.copy(currentline);  break;
            case 3:
                switch (configuration.telemetry) {
                    case 0:
                        // currentline = "UAVTalk"; break;
                        string_telemetry2.copy(currentline); break;
                    case 1:
                         //currentline = "MSP"; break;
                        string_telemetry3.copy(currentline); break;
                    case 2:
                        //currentline = "LTM"; break;
                        string_telemetry4.copy(currentline); break;
                    case 3:
                        //currentline = "MavLink"; break;
                        string_telemetry5.copy(currentline); break;
                    case 4:
                        //currentline = "NMEA"; break;
                        string_telemetry6.copy(currentline); break;
                    case 5:
                        //currentline = "UBLOX"; break;
                        string_telemetry7.copy(currentline); break;
                    default:
                    case 6:
                        //currentline = "HoTT"; break;
                        string_telemetry8.copy(currentline); break;
                     }
                     break;
            case 4:      
                strcpy(currentline, string_shome5.copy(extract)); break;

        }
        for ( int l = strlen(currentline); l<20 ; l++ )
            strcat(currentline," ");
        store_lcdline(i,currentline);
    }
  
}

void lcddisp_baudrate() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_baudrate.copy(currentline);  break;
            case 2:
                strcpy(currentline, string_load2.copy(extract)); break;
            case 3:
                switch (configuration.baudrate) {            
                    case 0:
                        // 1200
                        string_baudrate0.copy(currentline);  break;
                    case 1:
                        //2400
                        string_baudrate1.copy(currentline);  break;
                    case 2:
                        //4800
                        string_baudrate2.copy(currentline); break;
                    case 3:
                        //9600
                        string_baudrate3.copy(currentline);  break;
                    case 4:
                        //19200
                       string_baudrate4.copy(currentline);  break;
                    case 5:
                        //38400
                       string_baudrate5.copy(currentline);  break;
                    case 6:
                        //57600
                       string_baudrate6.copy(currentline);  break;
                    case 7:
                       //115200
                       string_baudrate7.copy(currentline);  break;                                                            
                    }
                    break;
            case 4:      
                    string_shome5.copy(currentline); break;
            }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
        strcat(currentline," ");
        }
    store_lcdline(i,currentline);
    }
}

// Settings Bank config
void lcddisp_bank() {
    for ( int i = 1 ; i<5; i++ ) {
       char currentline[21]="";
       char extract[21];
       switch (i) {
            case 1: 
               string_bank.copy(currentline);  break;
            case 2:
               string_load2.copy(currentline); break;
            case 3:
               switch (current_bank+1) {
                   case 1: sprintf(currentline,"> %s", string_bank1.copy(extract));break;
                   case 2: sprintf(currentline,"> %s", string_bank2.copy(extract));break;
                   case 3: sprintf(currentline,"> %s", string_bank3.copy(extract));break;
                   case 4: sprintf(currentline,"> %s", string_bank4.copy(extract));break;
                }
                break;
            case 4:      
                string_shome5.copy(currentline); break;
        }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
        store_lcdline(i,currentline);
    }
}

void lcddisp_osd() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_osd1.copy(currentline);  break;
            case 2:
                strcpy(currentline, string_load2.copy(extract)); break;
            case 3:
                switch (configuration.osd_enabled) {          
                    case 0:
                        // NO
                        string_osd3.copy(currentline);  break;
                    case 1:
                        //YES
                        string_osd2.copy(currentline);  break;
                }
                break;   
           case 4:      
                    string_shome5.copy(currentline); break;
           }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
       store_lcdline(i,currentline);
    }
}

void lcddisp_bearing_method() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_bearing0.copy(currentline);  break;
            case 2:
                string_load2.copy(currentline);  break;
            case 3:
                switch (configuration.bearing_method) {
                    case 1:
                        //currentline = "MSP"; break;
                        string_bearing1.copy(currentline); break;
                    case 2:
                        //currentline = "LTM"; break;
                        string_bearing2.copy(currentline); break;
                    case 3:
                        //currentline = "MavLink"; break;
                        string_bearing3.copy(currentline); break;
                    case 4:
                        //currentline = "NMEA"; break;
                        string_bearing4.copy(currentline); break;
                }
                break;
            case 4:      
                strcpy(currentline, string_shome5.copy(extract)); break;
        }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
        store_lcdline(i,currentline);
    } 
}


void lcddisp_voltage_ratio() {
    read_voltage();
    if (right_button.holdTime() >= 700 && right_button.isPressed() ) {
              voltage_ratio += 0.1;
              delay(500);
        }
        else if ( left_button.holdTime() >= 700 && left_button.isPressed() ) {
              voltage_ratio -= 0.1;
              delay(500);
        }
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_voltage0.copy(currentline);  break;
            case 2:
                char bufferV[6];
                sprintf(currentline,"Voltage: %s V", dtostrf(voltage_actual, 4, 2, bufferV));
                break;
            case 3:
                char bufferX[5];
                sprintf(currentline,"Ratio:  %s ", dtostrf(voltage_ratio, 3, 2, bufferV));
                break;
            case 4:      
                strcpy(currentline, string_shome5.copy(extract));  break;
        }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
        store_lcdline(i,currentline);
    } 
}

void lcddisp_testservo() {
    for ( int i = 1 ; i<5; i++ ) {
        char currentline[21]="";
        char extract[21];
        switch (i) {
            case 1: 
                string_servos3.copy(currentline);  break;
            case 2:
                string_servos4.copy(currentline); break;
            case 3:
                string_load2.copy(currentline); break;     
            case 4:      
                string_shome5.copy(currentline); break;
        }
        for ( int l = strlen(currentline); l<20 ; l++ ) {
            strcat(currentline," ");
        }
        store_lcdline(i,currentline);
    }
}

// SERVO CONFIGURATION

int config_servo(int servotype, int valuetype, int value ) {
    // servo configuration screen function return configured value
        //check long press left right
        if (right_button.holdTime() >= 700 && right_button.isPressed() ) {
              value+=20;
              delay(500);
        }
        else if ( left_button.holdTime() >= 700 && left_button.isPressed() ) {
              value-=20;
              delay(500);
        }
        char currentline[21];
        char extract[21];
    if (servotype==1) {
        string_servos1.copy(currentline);                              // Pan servo
        store_lcdline(1, currentline);
    }
    else if (servotype==2) {
        string_servos2.copy(currentline);                              // Tilt servo
        store_lcdline(1, currentline);
    }
    string_load2.copy(currentline);
        store_lcdline(2, currentline);
    switch (valuetype) 
    {           
        case 1: sprintf(currentline, "min endpoint: <%4d>",  value); break;          //minpwm
        case 2: sprintf(currentline, "min angle: <%3d>    ", value); break;         //minangle
        case 3: sprintf(currentline, "max endpoint: <%4d>",  value); break;          //maxpwm
        case 4: sprintf(currentline, "max angle: <%3d>    ", value); break;         //maxangle
    }
    store_lcdline(3, currentline);
    string_shome5.copy(currentline);
    store_lcdline(4, currentline); 
    return value;
           
}
