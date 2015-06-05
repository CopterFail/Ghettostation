/* HoTT USB/BT Telemetry , 2015 by CopterFail */

#include <Arduino.h>

#include "defines.h"
#include "boards.h"
#include "globals.h"
#include "HottBtUsb.h"

#ifdef PROTOCOL_HOTT

/* Configuration */
#define HOTT_WAIT_TIME 100U		// time [ms] to wait for response or next request

/* local defines */
#define HOTT_REQUEST_GPS	1
#define HOTT_WAIT_GPS		2
#define HOTT_REQUEST_EAM	3
#define HOTT_WAIT_EAM		4
#define HOTT_REQUEST_RX		5
#define HOTT_WAIT_RX		6
#define HOTT_IDLE			10

/* local functions */
static void vHottFormatGpsString(char *buffer, uint16_t high, uint16_t low);
static float fHottGetGpsDegree(uint16_t high, uint16_t low);
static uint32_t ui32HottGetGpsDegree(uint16_t high, uint16_t low);
static void vHottSendGpsRequest(void);
static bool bHottReadGpsResponse(void);
static void vHottSendEamRequest(void);
static bool bHottReadEamResponse(void);
static void vHottSendReceiverRequest(void);
static bool bHottReadReceiverResponse(void);
static void vHottClean(void);
static void vUpdateGlobalData(void);
static bool bIsBtConnected(void);

/* local data */
static uint16_t ui16GpsOk = 0;
static uint16_t ui16GpsFail = 0;
static uint16_t ui16EamOk = 0;
static uint16_t ui16EamFail = 0;
static uint16_t ui16RxOk = 0;
static uint16_t ui16RxFail = 0;

/* local structures */
struct
	__attribute__((__packed__))
	{
		// Telegram Header:
		uint8_t ui8Start; // 0x00
		uint16_t ui16Dummy;
		uint8_t ui8Header1; // 0x18
		uint8_t ui8Header2; // 0x00
		uint8_t ui8Header3; // 0x04
		uint8_t ui8Header4; // 0x01
		// GPS Data:
		uint16_t ui16DummyH;
		uint16_t ui16Speed; //10+11
		uint16_t ui16DistanceToHome; //12+13
		uint16_t ui16Altitude; //14+15
		uint16_t ui16LatitudeHigh; //16+17
		uint16_t ui16LatitudeLow; //18+19
		uint16_t ui16LongitudeHigh; //20+21
		uint16_t ui16LongitudeLow; //22+23
		uint16_t ui16Direction; //24+25
		uint8_t ui8NS; //
		uint8_t ui8EW; //
		uint16_t ui16Alt1s;
		uint8_t ui8Alt3s;
		uint8_t ui8Dbm;
		uint8_t ui8Sat;
		uint8_t ui8FixChar;
		uint16_t ui16HomeDirection;
		uint8_t ui8AngleX;
		uint8_t ui8AngleY;
		uint8_t ui8AngleZ;
		uint16_t ui16GyroX;
		uint16_t ui16GyroY;
		uint16_t ui16GyroZ;
		uint8_t ui8Vibration;
		uint8_t ui8FreeChar1;
		uint8_t ui8FreeChar2;
		uint8_t ui8FreeChar3;
		uint8_t ui8VerNo;
		uint16_t ui16DummyD;
	} GPSData;

	struct
		__attribute__((__packed__))
		{
			// Telegram Header:
			uint8_t ui8Start; // 0x00
			uint16_t ui16Dummy;
			uint8_t ui8Header1; // 0x0b
			uint8_t ui8Header2; // 0x00
			uint8_t ui8Header3; // 0x04
			uint8_t ui8Header4; // 0x01
			// Receiver Data:
			uint16_t ui16DummyH;
			uint16_t ui16Temp;
			uint16_t ui16LossPack;
			uint8_t ui8Strength;
			uint8_t ui8Volt;
			uint8_t ui8Dbm;
			uint8_t ui8Quality;
			uint8_t ui8LowVolt;
			uint16_t ui16DummyD;
		} ReceiverData;

		struct
			__attribute__((__packed__))
			{
				// Telegram Header:
				uint8_t ui8Start; // 0x00
				uint16_t ui16Dummy;
				uint8_t ui8Header1; // 0x30
				uint8_t ui8Header2; // 0x00
				uint8_t ui8Header3; // 0x04
				uint8_t ui8Header4; // 0x01
				// EAM Data:
				uint16_t ui16DummyH;
				uint16_t ui16CellV1;
				uint16_t ui16CellV2;
				uint16_t ui16CellV3;
				uint16_t ui16CellV4;
				uint16_t ui16CellV5;
				uint16_t ui16CellV6;
				uint16_t ui16CellV7;
				uint16_t ui16CellV8;
				uint16_t ui16CellV9;
				uint16_t ui16CellV10;
				uint16_t ui16CellV11;
				uint16_t ui16CellV12;
				uint16_t ui16CellV13;
				uint16_t ui16CellV14;
				uint16_t ui16Batt1;
				uint16_t ui16Batt2;
				uint16_t ui16Temp1;
				uint16_t ui16Temp2;
				uint16_t ui16Alt;
				uint16_t ui16PowerCurrent;
				uint16_t ui16PowerVolt;
				uint16_t ui16PowerCapacity;
				uint16_t ui16Alt1s;
				uint8_t ui8Alt3s;
				uint8_t ui8DBM;
				uint16_t ui16RPM;
				uint8_t ui8Min;
				uint8_t ui8Sec;
				uint8_t ui8Speed;
				uint8_t ui8Version;
//				uint16_t ui16DummyD;
			} EamData;

			static float fHottGetGpsDegree(uint16_t high, uint16_t low)
			{
				float fResult;
				fResult = (float) (high % 100);
				fResult += ((float) low / 10000.0);
				fResult /= 60.0;
				fResult += (float) ((uint16_t)(high / 100));
				return fResult;
			}

			static uint32_t ui32HottGetGpsDegree(uint16_t high, uint16_t low)
			{
				uint32_t ui32Result;
				ui32Result = ((uint32_t)(high % 100)) * 10000000;
				ui32Result += (uint32_t) low * 1000;
				ui32Result /= 60;
				ui32Result += (uint32_t)(high / 100) * 10000000;
				return ui32Result;
			}

			void vHottInit(void)
			{
				pinMode(PIN_BT_PIN34, OUTPUT);
				pinMode(PIN_BT_STATUS, INPUT);
				digitalWrite(PIN_BT_PIN34, LOW);
				TELEMETRY_SERIAL.begin(115200);
#ifdef HOTT_DEBUG
				DEBUG_SERIAL.println("vHottInit");
#endif
			}

			void vHottTelemetrie(void)
			{
				static uint8_t ui8State = HOTT_IDLE;
				static uint8_t ui8GpsCnt = 0;
				static uint32_t ui32RequestTime = 0;
				static uint8_t lastState = 0;

				uint32_t ui32Timeout;

				ui32Timeout = millis() - ui32RequestTime;

#ifdef HOTT_DEBUG

				if (lastState != ui8State)
				{
					DEBUG_SERIAL.println(ui8State);
					lastState = ui8State;
				}

#endif

				switch (ui8State)
				{
				case HOTT_REQUEST_GPS:
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendGpsRequest();
						ui8State = HOTT_WAIT_GPS;
					}
					break;
				case HOTT_WAIT_GPS:
					if ( TELEMETRY_SERIAL.available() >= sizeof(GPSData))
					{
						if (bHottReadGpsResponse())
						{
							ui16GpsOk++;
							vUpdateGlobalData();
							ui8State = HOTT_REQUEST_EAM;
						}
						else
						{
							ui16GpsFail++;
							ui8State = HOTT_REQUEST_EAM;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16GpsFail++;
						ui8State = HOTT_REQUEST_EAM;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.println("HOTT_GPS_TIMEOUT");
#endif
					}
					/*
					 if( ui8GpsCnt > 0)
					 {
					 ui8GpsCnt--;
					 ui8State = HOTT_REQUEST_GPS;
					 }
					 else
					 {
					 ui8GpsCnt = 0;
					 ui8State = HOTT_REQUEST_RX;
					 }
					 */
					break;
				case HOTT_REQUEST_EAM:
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendEamRequest();
						ui8State = HOTT_WAIT_EAM;
					}
					break;
				case HOTT_WAIT_EAM:
					if ( TELEMETRY_SERIAL.available() >= sizeof(EamData))
					{
						if (bHottReadEamResponse())
						{
							ui16EamOk++;
							vUpdateGlobalData();
							ui8State = HOTT_REQUEST_RX;
						}
						else
						{
							ui16EamFail++;
							ui8State = HOTT_REQUEST_RX;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16EamFail++;
						ui8State = HOTT_REQUEST_RX;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.print("HOTT_EAM_TIMEOUT - ");
						DEBUG_SERIAL.print(TELEMETRY_SERIAL.available());
						DEBUG_SERIAL.print(" ");
						DEBUG_SERIAL.println(sizeof(EamData));
#endif
					}

					break;
				case HOTT_REQUEST_RX:
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendReceiverRequest();
						ui8State = HOTT_WAIT_RX;
					}
					break;
				case HOTT_WAIT_RX:
					if ( TELEMETRY_SERIAL.available() >= sizeof(ReceiverData))
					{
						if (bHottReadReceiverResponse())
						{
							ui16RxOk++;
							vUpdateGlobalData();
							ui8State = HOTT_IDLE;
						}
						else
						{
							ui16RxFail++;
							ui8State = HOTT_IDLE;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16RxFail++;
						ui8State = HOTT_IDLE;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.println("HOTT_RX_TIMEOUT");
#endif
					}

					break;
				case HOTT_IDLE:
				default:
					if (bIsBtConnected())
					{
						ui8State = HOTT_REQUEST_GPS;
					}
					else
					{
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.println("HOTT no BT");
#endif

					}
					break;
				}
			}

			static void vHottSendGpsRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x38);
				TELEMETRY_SERIAL.write(0x9f);
				TELEMETRY_SERIAL.write(0x7b);
			}

			static bool bHottReadGpsResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((uint8_t *) &GPSData,
						sizeof(GPSData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(GPSData));
			}

			static void vHottSendReceiverRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x34);
				TELEMETRY_SERIAL.write(0x13);
				TELEMETRY_SERIAL.write(0xba);
			}

			static bool bHottReadReceiverResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((uint8_t *) &ReceiverData,
						sizeof(ReceiverData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(ReceiverData));
			}

			static void vHottSendEamRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x36);
				TELEMETRY_SERIAL.write(0x51);
				TELEMETRY_SERIAL.write(0x9a);
			}

			static bool bHottReadEamResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((uint8_t *) &EamData,
						sizeof(EamData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(EamData));
			}

			static void vHottClean(void)
			{
				while ( TELEMETRY_SERIAL.available() > 0)
					TELEMETRY_SERIAL.read();
			}

			static bool bIsBtConnected(void)
			{
				return digitalRead(PIN_BT_STATUS);
			}

			static void vUpdateGlobalData(void)
			{
				/*
				 GPSData.ui16DistanceToHome
				 */

				uav_lat = ui32HottGetGpsDegree(GPSData.ui16LatitudeHigh,
						GPSData.ui16LatitudeLow); 	// latitude (deg * 1e7)
				uav_lon = ui32HottGetGpsDegree(GPSData.ui16LongitudeHigh,
						GPSData.ui16LongitudeLow);	// longitude
				uav_satellites_visible = GPSData.ui8Sat;// number of satellites
				uav_fix_type = GPSData.ui8FixChar;// GPS lock 0-1=no fix, 2=2D, 3=3D
				uav_alt = (int32_t) GPSData.ui16Altitude * 10;  // altitude (cm)
				uav_groundspeed = GPSData.ui16Speed;     // ground speed in km/h
				uav_groundspeedms = GPSData.ui16Speed / 3.6f; // ground speed in m/s
				uav_pitch = GPSData.ui8AngleX;                 // attitude pitch
				uav_pitch = uav_pitch * 2;
				if( uav_pitch > 180 ) uav_pitch -= 360;
				uav_roll = GPSData.ui8AngleY;                   // attitude roll
				uav_roll = uav_roll * 2;
				if( uav_roll > 180 ) uav_roll -= 360;
				uav_heading = GPSData.ui8AngleZ;             // attitude heading
				uav_gpsheading = GPSData.ui16Direction;           // gps heading
				//uav_bat = ReceiverData.ui8Volt*100U;            // battery voltage (mv)
				uav_bat =
						(EamData.ui16Batt1 > EamData.ui16Batt2) ?
								EamData.ui16Batt1 : EamData.ui16Batt2;
				uav_bat *= 100;                                 //??
				uav_current = 0;                			// actual current
				uav_rssi = ReceiverData.ui8Strength;		// radio RSSI (%)
				uav_linkquality = ReceiverData.ui8Quality;// radio link quality
				uav_airspeed = 0;               		// Airspeed sensor (m/s)
				uav_arm = 0;                    		// 0: disarmed, 1: armed
				uav_failsafe = 0;               	// 0: normal,   1: failsafe
				uav_flightmode = 19; // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
									 // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
									 // 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
				protocol = "HoTT";
				telemetry_ok = true;
				lastpacketreceived = millis();

#ifdef HOTT_DEBUG
				static uint32_t ui32FirstPacketTime = millis();
				DEBUG_SERIAL.print("HoTT active for ");
				DEBUG_SERIAL.print(lastpacketreceived - ui32FirstPacketTime);
				DEBUG_SERIAL.println(" ms.");
				DEBUG_SERIAL.print("GPS (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16GpsOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16GpsFail);
				DEBUG_SERIAL.print("EAM (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16EamOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16EamFail);
				DEBUG_SERIAL.print("Rx (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16RxOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16RxFail);

				DEBUG_SERIAL.print("Roll/Pitch/Height: ");
				DEBUG_SERIAL.print(uav_roll);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.print(uav_pitch);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(uav_alt);

#endif

#ifdef HOTT_SIMULATION_DEBUG
				static float w=0;
				static int16_t k=0;
				static int8_t d=1;
				if(w>5){
					uav_satellites_visible = 6;
					uav_fix_type = '3';
				}

				// The home pos:
				uav_lat = 1e7 * 51.3678;
				uav_lon = 1e7 * 6.78756;
				if( home_pos )
				{
					// fly circles (r=1km) around the home pos...
					uav_lat += 1e7 * 0.01113195 * sin(w);// north
					uav_lon += 1e7 * 0.01113195 * cos(w);// east

					// 0 .. 1000m height depend on pan angle...
					uav_alt = (int32_t)(50000.0 * ( 1 + sin(w) ) );
				}
				w+=0.1;

				uav_pitch = 0;
				uav_roll = k;
				k+=d;
				if( k >= 30 ) d=-1;
				if( k <= -30 ) d=1;
#endif

			}

#endif

