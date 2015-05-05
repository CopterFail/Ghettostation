/*
 * boards.h
 *
 *  Created on: 12.04.2015
 *      Author: CopterFail
 */

#ifndef BOARDS_H_
#define BOARDS_H_

// arduino pin and serial definitions depend an selected board (see defines.h)

//########################################### BOARDS PINOUTS #########################################################
//DON'T EDIT THIS IF YOU DON'T KNOW WHAT YOU'RE DOINGG
//pinout for TEENSY++ 2
#ifdef TEENSYPLUS2
  #include <SoftwareSerial.h>
  #define PAN_SERVOPIN     26    //PWM Pin for pan servo
  #define TILT_SERVOPIN    25 	 //PWM Pin for tilt servo
  #define LEFT_BUTTON_PIN  10    //Any Digital pin
  #define RIGHT_BUTTON_PIN  9    //Any Digital pin
  #define ENTER_BUTTON_PIN  8    //Any Digital pin
  #define SOFTSERIAL_TX    14    //Digital pin used by SoftSerial for sending data to ground osd.
  #define SOFTSERIAL_RX     0    //Digital pin used by SoftSerial for receiving data from ground osd. ( unused yet )
  #define ADC_VOLTAGE      38    //(F0) ADC pin used for voltage reading
  #define BUZZER_PIN       16    //(C6) Any PWM pin (add a 100-150 ohm resistor between buzzer & ground)

// todo: repair the definitions for the serial ports
//  #define OSD_SERIAL 		Serial2  //SoftwareSerial OSD_SERIAL(SOFTSERIAL_RX,SOFTSERIAL_TX);
//  #define TELEMETRY_SERIAL	Serial1
//  #define DEBUG_SERIAL		Serial


#endif

//pinout for Arduino Mega 1280/2560
#ifdef MEGA
  #define PAN_SERVOPIN     11	//PWM Pin for pan servo
  #define TILT_SERVOPIN    12   //PWM Pin for tilt ervo
  #define LEFT_BUTTON_PIN  32   //Any Digital pin
  #define RIGHT_BUTTON_PIN 34   //Any Digital pin
  #define ENTER_BUTTON_PIN 36   //Any Digital pin
  #define ADC_VOLTAGE      41   //(A5) ADC pin used for voltage reading
  #define BUZZER_PIN        8   //(PH5) Any PWM pin ((add a 100-150 ohm resistor between buzzer & ground)

  #define OSD_SERIAL 		Serial2
  #define TELEMETRY_SERIAL	Serial1
  #define DEBUG_SERIAL		Serial

#endif

#ifdef TEENSY31
  #define PAN_SERVOPIN     23	//PWM Pin for pan servo
  #define TILT_SERVOPIN    22   //PWM Pin for tilt ervo
  #define DOWN_BUTTON_PIN   2   //Any Digital pin, blue
  #define UP_BUTTON_PIN  	4   //Any Digital pin, green
  #define ENTER_BUTTON_PIN  3   //Any Digital pin, red
  #define ADC_VOLTAGE      15   //ADC pin used for voltage reading
  #define BUZZER_PIN        6   //(PH5) Any PWM pin ((add a 100-150 ohm resistor between buzzer & ground)

  #define PIN_SCLK         14  // SCLK can also use pin 14 13
  #define PIN_MOSI         11  // MOSI can also use pin 7
  #define PIN_CS           20  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
  #define PIN_DC           21  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
  #define PIN_RST           8  // RST can use any pin
  #define PIN_SDCS          5  // CS for SD card, can use any pin

  #define PIN_BT_STATUS    16  // BT status, any digital pin
  #define PIN_BT_PIN34     17  // BT mode, any digital pin

  #define PIN_RX_SPI_CLK	24 // reserved for RX5808 module with SPI mod, pin is smd pad, CS2
  #define PIN_RX_SPI_DATA	25 // reserved for RX5808 module with SPI mod, pin is smd pad, CS0
  #define PIN_RX_SPI_LE		26 // reserved for RX5808 module with SPI mod, pin is smd pad, CS1
  #define ADC_RSSI_A		27 //A16// reserved for RX5808 module, pin is smd pad (A16 / 27)
  #define ADC_RSSI_B		28 //A17 // reserved for RX5808 module, pin is smd pad (A17 / 28)
  #define PIN_ENABLE_RX_A	32 // reserved for RX5808 module, pin is smd pad
  #define PIN_ENABLE_RX_B	29 //33 did not start ?? // reserved for RX5808 module, pin is smd pad

  #define OSD_SERIAL 		Serial2  // use pins 9(RX2) and 10(TX2)
  #define TELEMETRY_SERIAL	Serial1  // use pins 0(RX1) and 1(TX1)  // make this TELEMETRY_SERIAL for all protocols
  #define DEBUG_SERIAL		Serial   // USB serial for debug information

  // reserved:
  // Serial3:	use pins 7(RX3) and 8(TX3, already used by the display)
  // I2C: 		use 18 and 19, reserved for mag

#endif

//ToDo: needed by LightTelemetry, check this:
#define softserial_delay	(int)round(10000000.0f/(OSD_BAUD)) // time to wait between each byte sent.


#endif /* BOARDS_H_ */
