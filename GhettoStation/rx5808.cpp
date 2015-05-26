/*
 * rx5808.cpp
 *
 *  Created on: 16.04.2015
 *      Author: CopterFail
 */

#include <Arduino.h>

#include "defines.h"
#include "boards.h"
#include "rx5808.h"

#define DIVERSITY_HYSTERESIS	15	// 30 = 0.1V

// Channels to sent to the SPI registers
const uint16_t channelTable[]
{
// Channel 1 - 8
		0x2A05, 0x299B, 0x2991, 0x2987, 0x291D, 0x2913, 0x2909, 0x289F, // Band A
		0x2903, 0x290C, 0x2916, 0x291F, 0x2989, 0x2992, 0x299C, 0x2A05, // Band B
		0x2895, 0x288B, 0x2881, 0x2817, 0x2A0F, 0x2A19, 0x2A83, 0x2A8D, // Band E
		0x2906, 0x2910, 0x291A, 0x2984, 0x298E, 0x2998, 0x2A02, 0x2A0C // Band F / Airwave
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[]
{
// Channel 1 - 8
		5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
		5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
		5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
		5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880  // Band F / Airwave
};

cRX5808::cRX5808()
{
	uint8_t ui8Channel;
	pinMode( PIN_RX_SPI_LE, OUTPUT);
	pinMode( PIN_RX_SPI_CLK, OUTPUT);
	pinMode( PIN_RX_SPI_DATA, OUTPUT);

	pinMode( PIN_ENABLE_RX_A, OUTPUT);
	pinMode( PIN_ENABLE_RX_B, OUTPUT);

	digitalWrite(PIN_RX_SPI_LE, LOW);
	digitalWrite(PIN_RX_SPI_CLK, LOW);
	digitalWrite(PIN_RX_SPI_DATA, LOW);

	ui8ActiveChannel = 255;		// none
	ui8ActiveReceiver = 255;	// none
	ui8ActiveDiversity = 0;		// off
	ui16MaxRssi = 310; // 1V with 3V3 Referenz: 1 * 1024 / 3,3 = 310
	ui16MinRssi = 160; // 0.5V ???

	for (ui8Channel = 0; ui8Channel < CHANNELCOUNT; ui8Channel++)
	{
		aui16Rssi[ui8Channel] = ui16MinRssi;
	}

	vSelectReceiver(0);
	vSelectChannel(0);

}

void cRX5808::vSelectReceiver(uint8_t ui8Receiver)
{
	if (ui8Receiver == ui8ActiveReceiver)
		return; // nothing to do

	if (ui8Receiver)
	{
		digitalWrite( PIN_ENABLE_RX_B, HIGH);
		digitalWrite( PIN_ENABLE_RX_A, LOW);
		ui8ActiveReceiver = 1;
	}
	else
	{
		digitalWrite( PIN_ENABLE_RX_B, LOW);
		digitalWrite( PIN_ENABLE_RX_A, HIGH);
		ui8ActiveReceiver = 0;
	}
}

void cRX5808::vSelectChannel(uint8_t ui8NewChannel)
{
	uint16_t channelData;
	uint8_t i;

	if (ui8NewChannel == ui8ActiveChannel)
		return; // nothing to do

	ui8NewChannel &= 0x1f;
	channelData = channelTable[ui8NewChannel];
	vEnableHigh();
	delayMicroseconds(1);
	vEnableLow();
	// bit bash out 25 bits of data
	// Order: A0-3, !R/W, D0-D19
	// A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
	vSendBit0();
	vSendBit0();
	vSendBit0();
	vSendBit1();
	vSendBit0();
	for (i = 20; i > 0; i--)
		vSendBit0();

	vEnableHigh();
	delayMicroseconds(1);
	vEnableLow();

	// Second is the channel data from the lookup table
	// 20 bytes of register data are sent, but the MSB 4 bits are zeros
	// register address = 0x1, write, data0-15=channelData data15-19=0x0
	vEnableHigh();
	// ??? delayMicroseconds(1);
	vEnableLow();

	// Register 0x1
	vSendBit1();
	vSendBit0();
	vSendBit0();
	vSendBit0();

	// Write to register
	vSendBit1();

	// D0-D15
	for (i = 16; i > 0; i--)
	{
		if (channelData & 0x1)
		{
			vSendBit1();
		}
		else
		{
			vSendBit0();
		}

		// Shift bits along to check the next one
		channelData >>= 1;
	}

	// Remaining D16-D19
	for (i = 4; i > 0; i--)
		vSendBit0();

	// Finished clocking data in
	vEnableHigh();
	delayMicroseconds(1);

	digitalWrite(PIN_RX_SPI_LE, LOW);
	digitalWrite(PIN_RX_SPI_CLK, LOW);
	digitalWrite(PIN_RX_SPI_DATA, LOW);

	ui8ActiveChannel = ui8NewChannel;
}

void cRX5808::vSelectDiversity(uint8_t ui8Diversity)
{
	if (ui8Diversity)
	{
		ui8ActiveDiversity = 1;
	}
	else
	{
		ui8ActiveDiversity = 0;
	}

}
uint16_t cRX5808::ui16GetRssi(uint8_t ui8Receiver)
{
#define RSSI_COUNT 9
	uint16_t rssi = 0;
	uint16_t buffer[RSSI_COUNT];
	uint8_t i;
	bool sorted = false;

	if (ui8Receiver == 255)
		ui8Receiver = ui8ActiveReceiver;

	for (i = 0; i < RSSI_COUNT; i++)
	{
		buffer[i] = analogRead(ui8Receiver ? ADC_RSSI_B : ADC_RSSI_A);
		delay(1);
	}

	while (!sorted)	// median with primitive sort
	{
		sorted = true;
		for (i = 0; i < RSSI_COUNT - 1; i++)
		{
			if (buffer[i] > buffer[i + 1])
			{
				rssi = buffer[i + 1];
				buffer[i + 1] = buffer[i];
				buffer[i] = rssi;
				sorted = false;
			}
		}
	}
	rssi = buffer[RSSI_COUNT >> 1];

	if (ui8RssiMode == 1)
	{
		if (rssi > ui16MaxRssi)
			ui16MaxRssi = rssi;
		else if (rssi < ui16MinRssi)
			ui16MinRssi = rssi;
	}
	aui16Rssi[ui8ActiveChannel & 0x1f] = rssi;
	return rssi;
}

uint8_t cRX5808::ui8ScanChannels(uint8_t ui8Set)
{
	uint8_t ui8Best = 0, ui8Rssi, ui8BestRssi = 0;
	uint8_t ui8Channel;

	for (ui8Channel = 0; ui8Channel < CHANNELCOUNT; ui8Channel++)
	{
		vSelectChannel(ui8Channel);
		delay(150);
		ui8Rssi = ui16GetRssi(ui8ActiveReceiver);
		aui16Rssi[ui8Channel] = ui8Rssi;
		if (ui8Rssi >= ui8BestRssi)
		{
			ui8BestRssi = ui8Rssi;
			ui8Best = ui8Channel;
		}
	}
	if (ui8Set > 0U)
	{
		vSelectChannel(ui8Best);
	}
	return ui8Best;
}

void cRX5808::vCalibrateRssi(uint8_t ui8Mode)
{
	if (ui8Mode == 0)
	{
		ui8RssiMode = 0;
	}
	else
	{
		ui8RssiMode = 1;
		ui16MaxRssi = 1;
		ui16MinRssi = 1023;
	}
}

void cRX5808::vDiversity(void)
{
	int16_t i16RssiA;
	int16_t i16RssiB;
	int16_t idiff;

	if (0 == ui8ActiveDiversity)
		return;

	i16RssiA += (int16_t) ui16GetRssi(0);
	i16RssiB += (int16_t) ui16GetRssi(1);
	i16RssiA >>= 1;
	i16RssiB >>= 1;
	idiff = i16RssiA - i16RssiB;

	if ((idiff > DIVERSITY_HYSTERESIS) && (ui8ActiveReceiver == 1))
	{
		vSelectReceiver(0);
	}
	else if ((idiff < -DIVERSITY_HYSTERESIS) && (ui8ActiveReceiver == 0))
	{
		vSelectReceiver(1);
	}
}

void cRX5808::vSendBit1(void)
{
	digitalWrite(PIN_RX_SPI_CLK, LOW);
	delayMicroseconds(1);

	digitalWrite(PIN_RX_SPI_DATA, HIGH);
	delayMicroseconds(1);
	digitalWrite(PIN_RX_SPI_CLK, HIGH);
	delayMicroseconds(1);

	digitalWrite(PIN_RX_SPI_CLK, LOW);
	delayMicroseconds(1);
}

void cRX5808::vSendBit0(void)
{
	digitalWrite(PIN_RX_SPI_CLK, LOW);
	delayMicroseconds(1);

	digitalWrite(PIN_RX_SPI_DATA, LOW);
	delayMicroseconds(1);
	digitalWrite(PIN_RX_SPI_CLK, HIGH);
	delayMicroseconds(1);

	digitalWrite(PIN_RX_SPI_CLK, LOW);
	delayMicroseconds(1);
}

void cRX5808::vEnableLow(void)
{
	delayMicroseconds(1);
	digitalWrite(PIN_RX_SPI_LE, LOW);
	delayMicroseconds(1);
}

void cRX5808::vEnableHigh(void)
{
	delayMicroseconds(1);
	digitalWrite(PIN_RX_SPI_LE, HIGH);
	delayMicroseconds(1);
}
