/*
 * lcd.h
 *
 *  Created on: 10.04.2015
 *      Author: CopterFail
 */

#ifndef LCD_H_
#define LCD_H_



void init_lcdscreen( void );
void lcddisp_setupdateflag( void );
void refresh_lcd( void );
void lcddisp_menu( void );
void lcddisp_tracking( void );
void lcddisp_sethome( void );
void lcddisp_setbearing( void );
void lcddisp_homeok( void );
void lcddisp_telemetry( void );
void lcddisp_baudrate( void );
void lcddisp_bank( void );
void lcddisp_osd( void );
void lcddisp_bearing_method( void );
void lcddisp_voltage_ratio( void );
void lcddisp_testservo( void );

int config_servo(int servotype, int valuetype, int value );

#endif /* LCD_H_ */
