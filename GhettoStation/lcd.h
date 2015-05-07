/*
 * lcd.h
 *
 *  Created on: 10.04.2015
 *      Author: CopterFail
 */

#ifndef LCD_H_
#define LCD_H_


void showMenu( void );

void vUpdateMenu( void );
void vUpdateData( void );
void vShowSpectrum( void );
void vShowMessage( char *text, uint8_t size, uint16_t time );

void init_lcdscreen( void );
void lcddisp_menu( void );
void lcddisp_tracking( uint8_t ui8Mode );
void lcddisp_sethome( void );
void lcddisp_setbearing( void );
void lcddisp_homeok( void );
void lcddisp_telemetry( void );
void lcddisp_baudrate( void );
void lcddisp_bank( void );
void lcddisp_osd( void );
void lcddisp_bearing_method( void );
void lcddisp_voltage_ratio( void );
void lcddisp_testservo( uint8_t ui8Mode );
int config_servo(int servotype, int valuetype, int value );

#endif /* LCD_H_ */
