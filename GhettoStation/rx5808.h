/*
 * rx5808.h
 *
 *  Created on: 16.04.2015
 *      Author: CopterFail
 */

#ifndef RX5808_H_
#define RX5808_H_

//#define RX5808_DEBUG
#define CHANNELCOUNT 32
class cRX5808
{
private:
	uint8_t ui8ActiveChannel;
	uint8_t ui8ActiveReceiver;
	uint8_t ui8ActiveDiversity;
	uint8_t ui8RssiMode; // 0 normal receiving, 1 find min/max
	uint16_t ui16MaxRssi, ui16MinRssi;
	uint16_t aui16Rssi[CHANNELCOUNT];

	void vSendBit0( void );
	void vSendBit1( void );
	void vEnableLow( void );
	void vEnableHigh( void );

public:
	cRX5808();
	void vSelectChannel( uint8_t ui8NewChannel );
	void vSelectReceiver( uint8_t ui8Receiver );
	void vSelectDiversity( uint8_t ui8Diversity );
	uint16_t ui16GetRssi( uint8_t ui8Receiver );
	uint16_t ui16GetMaxRssi( void ){ return ui16MaxRssi; }
	uint16_t ui16GetMinRssi( void ){ return ui16MinRssi; }
	uint8_t ui8GetChannel( void ){ return ui8ActiveChannel; }
	uint8_t ui8GetReceiver( void ){ return ui8ActiveReceiver; }
	uint8_t ui8GetDiversity( void ){ return ui8ActiveDiversity; }
	uint8_t ui8ScanChannels( uint8_t ui8Set );
	void vCalibrateRssi( uint8_t ui8Mode );
	uint16_t *pui16GetAllRSSI( void ){ return aui16Rssi; }
	void vDiversity( void );
};



#endif /* RX5808_H_ */
