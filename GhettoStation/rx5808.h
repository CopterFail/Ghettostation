/*
 * rx5808.h
 *
 *  Created on: 16.04.2015
 *      Author: CopterFail
 */

#ifndef RX5808_H_
#define RX5808_H_

class cRX5808
{
private:
	uint8_t ui8ActiveChannel;
	uint8_t ui8ActiveReceiver;

	void vSendBit0( void );
	void vSendBit1( void );
	void vEnableLow( void );
	void vEnableHigh( void );

public:
	cRX5808();
	void vSelectChannel( uint8_t ui8NewChannel );
	void vSelectReceiver( uint8_t ui8Receiver );
	uint16_t fGetRSSI( uint8_t ui8Receiver );
	uint8_t ui8GetChannel( void ){ return ui8ActiveChannel; }
	uint8_t ui8GetReceiver( void ){ return ui8ActiveReceiver; }
	void vDiversity( void );

};



#endif /* RX5808_H_ */
