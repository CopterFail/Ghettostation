/* HoTT USB/BT Telemetry , 2015 by CopterFail */

#ifdef PROTOCOL_HOTT

/* Configuration */
#define HOTT_WAIT_TIME 800U		// time to wait for response
#define HOTT_DELAY_TIME 1000U	// time to wait between requests
//#define HOTT_DEBUG

/* local defines */
#define HOTT_REQUEST_GPS	1
#define HOTT_WAIT_GPS		2
#define HOTT_REQUEST_RX		3
#define HOTT_WAIT_RX		4
#define HOTT_UPDATE_GHETTO	5
#define HOTT_IDLE			6

/* local functions */
static void vHottFormatGpsString( char *buffer, uint16_t high, uint16_t low );
static float fHottGetGpsDegree( uint16_t high, uint16_t low );
static uint32_t ui32HottGetGpsDegree( uint16_t high, uint16_t low );
static void vHottSendGpsRequest( void );
static bool bHottReadGpsResponse( void );
static void vHottSendReceiverRequest( void );
static bool bHottReadReceiverResponse( void );
static bool vHottClean( void );
static void vUpdateGhettoData( void );
static bool bIsBtConnected( void );

/* local data */

/* local structures */
struct  __attribute__((__packed__)) {
    // Telegram Header:
  uint8_t  ui8Start; // 0x00
  uint16_t ui16Dummy;
  uint8_t  ui8Header1; // 0x18
  uint8_t  ui8Header2; // 0x00
  uint8_t  ui8Header3; // 0x04
  uint8_t  ui8Header4; // 0x01
  // GPS Data:
  uint16_t ui16DummyH; 
  uint16_t ui16Speed; //10+11
  uint16_t ui16DistanceToHome; //12+13
  uint16_t ui16Altitude; //14+15
  uint16_t ui16LatitudeHigh; //16+17
  uint16_t ui16LatitudeLow; //18+19
  uint16_t ui16LongitudeHigh;//20+21
  uint16_t ui16LongitudeLow;//22+23
  uint16_t ui16Direction; //24+25
  uint8_t  ui8NS; //
  uint8_t  ui8EW; //
  uint16_t ui16Alt1s;
  uint8_t  ui8Alt3s;
  uint8_t  ui8Dbm;
  uint8_t  ui8Sat;
  uint8_t  ui8FixChar;
  uint16_t ui16HomeDirection;
  uint8_t  ui8AngleX; 
  uint8_t  ui8AngleY; 
  uint8_t  ui8AngleZ; 
  uint16_t ui16GyroX; 
  uint16_t ui16GyroY; 
  uint16_t ui16GyroZ; 
  uint8_t  ui8Vibration; 
  uint8_t  ui8FreeChar1; 
  uint8_t  ui8FreeChar2; 
  uint8_t  ui8FreeChar3; 
  uint8_t  ui8VerNo;
  uint16_t ui16DummyD;
} GPSData;


struct  __attribute__((__packed__)) {
  // Telegram Header:
  uint8_t  ui8Start; // 0x00
  uint16_t ui16Dummy;
  uint8_t  ui8Header1; // 0x0b
  uint8_t  ui8Header2; // 0x00
  uint8_t  ui8Header3; // 0x04
  uint8_t  ui8Header4; // 0x01
  // Receiver Data:
  uint16_t ui16DummyH; 
  uint16_t ui16Temp;
  uint16_t ui16LossPack;
  uint8_t  ui8Strength;
  uint8_t  ui8Volt;
  uint8_t  ui8Dbm;
  uint8_t  ui8Quality;
  uint8_t  ui8LowVolt;
  uint16_t ui16DummyD;
} ReceiverData;

static float fHottGetGpsDegree( uint16_t high, uint16_t low )
{
  float fResult;
  fResult = (float)(high % 100);
  fResult += ((float)low / 10000.0);
  fResult /= 60.0;
  fResult += (float)((uint16_t)(high / 100));
  return fResult;
}

static uint32_t ui32HottGetGpsDegree( uint16_t high, uint16_t low )
{
  uint32_t ui32Result;
  ui32Result = ((uint32_t)(high % 100)) * 10000000;
  ui32Result += (uint32_t)low * 1000;
  ui32Result /= 60;
  ui32Result += (uint32_t)(high/100) * 10000000;
  return ui32Result;
}

void vHottInit( void )
{  
  pinMode(PIN_BT_PIN34, OUTPUT);  
  pinMode(PIN_BT_STATUS, INPUT);
  digitalWrite(PIN_BT_PIN34, LOW);
  HOTT_BT_SERIAL.begin(115200);  
#ifdef HOTT_DEBUG
  Serial.println("vHottInit");
#endif
}

void vHottTelemetrie( void )
{
	static uint8_t ui8State = HOTT_IDLE;
	static uint32_t ui32RequestTime = millis();
	static bool bUpdate=false;
	uint32_t ui32Timeout = millis() - ui32RequestTime;

#ifdef HOTT_DEBUG
 	//Serial.print(ui8State);
	//Serial.print(" ");
#endif

	switch( ui8State )
	{
		case HOTT_REQUEST_GPS:
			if( ui32Timeout > HOTT_WAIT_TIME )
			{
				ui32RequestTime = millis();
				vHottSendGpsRequest();
				ui8State = HOTT_WAIT_GPS;
			}
			break;
		case HOTT_WAIT_GPS:
			if( HOTT_BT_SERIAL.available() >= sizeof(GPSData) )
			{
				if( bHottReadGpsResponse() ) bUpdate = true;
				ui8State = HOTT_REQUEST_RX;
			}
			else if( ui32Timeout > HOTT_WAIT_TIME )
			{
#ifdef HOTT_DEBUG
				Serial.println("HOTT_GPS_TIMEOUT");
#endif
				ui8State = HOTT_REQUEST_RX;
			}
		    break;
		case HOTT_REQUEST_RX:
			if( ui32Timeout > HOTT_WAIT_TIME )
			{
				ui32RequestTime = millis();
				vHottSendReceiverRequest();
				ui8State = HOTT_WAIT_RX;
			}
			break;
		case HOTT_WAIT_RX:
			if( HOTT_BT_SERIAL.available() >= sizeof(ReceiverData) )
			{
				if(bHottReadReceiverResponse()) bUpdate = true;
				ui8State = HOTT_UPDATE_GHETTO;
			}
			else if( ui32Timeout > HOTT_WAIT_TIME )
			{
#ifdef HOTT_DEBUG
				Serial.println("HOTT_RX_TIMEOUT");
#endif
				ui8State = HOTT_UPDATE_GHETTO; //HOTT_REQUEST_RX;
			}
		    break;
		case HOTT_UPDATE_GHETTO:
			if( bUpdate )
			{
				vUpdateGhettoData();
#ifdef HOTT_DEBUG
  Serial.println("HOTT_UPDATE_GHETTO");
#endif
			}
			bUpdate = false;
			ui8State = HOTT_IDLE;
			break;
		case HOTT_IDLE:
		default:
			if( bIsBtConnected() )
			{	
				ui8State = HOTT_REQUEST_GPS;
			}
			break;
	}
}

static void vHottSendGpsRequest( void )
{
    vHottClean();
	HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x03 );
    HOTT_BT_SERIAL.write( 0xfc );
    HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x04 );
    HOTT_BT_SERIAL.write( 0x38 ); 
    HOTT_BT_SERIAL.write( 0x9f );
    HOTT_BT_SERIAL.write( 0x7b );
}

static bool bHottReadGpsResponse( void )
{
	uint8_t ui8Cnt;
	ui8Cnt = HOTT_BT_SERIAL.readBytes( (uint8_t *)&GPSData, sizeof(GPSData) );
#ifdef HOTT_DEBUG
				Serial.println(ui8Cnt);
#endif
	return ( ui8Cnt == sizeof(GPSData) );
}

static void vHottSendReceiverRequest( void )
{
    vHottClean();
	HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x03 );
    HOTT_BT_SERIAL.write( 0xfc );
    HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x00 );
    HOTT_BT_SERIAL.write( 0x04 );
    HOTT_BT_SERIAL.write( 0x34 ); 
    HOTT_BT_SERIAL.write( 0x13 );
    HOTT_BT_SERIAL.write( 0xba );
}

static bool bHottReadReceiverResponse( void )
{
	uint8_t ui8Cnt;
	ui8Cnt = HOTT_BT_SERIAL.readBytes( (uint8_t *)&ReceiverData, sizeof(ReceiverData) );
#ifdef HOTT_DEBUG
				Serial.println(ui8Cnt);
#endif
	return ( ui8Cnt == sizeof(ReceiverData) );
}

static bool vHottClean( void )
{
	while( HOTT_BT_SERIAL.available() > 0 ) HOTT_BT_SERIAL.read();
}

static bool bIsBtConnected( void )
{
	return digitalRead(PIN_BT_STATUS);
}

static void vUpdateGhettoData( void )
{
	/*
    GPSData.ui16DistanceToHome 
	*/
	
	uav_lat = ui32HottGetGpsDegree( GPSData.ui16LatitudeHigh, GPSData.ui16LatitudeLow); 	// latitude (deg * 1e7)
	uav_lon = ui32HottGetGpsDegree( GPSData.ui16LongitudeHigh, GPSData.ui16LongitudeLow);	// longitude
	uav_satellites_visible = GPSData.ui8Sat;		// number of satellites
	uav_fix_type = GPSData.ui8FixChar;				// GPS lock 0-1=no fix, 2=2D, 3=3D
	uav_alt = GPSData.ui16Altitude/10;   			// altitude (dm)
	rel_alt = GPSData.ui16Altitude/10; 				// relative altitude to home
	uav_groundspeed = GPSData.ui16Speed;            // ground speed in km/h
	uav_groundspeedms = GPSData.ui16Speed;          // ground speed in m/s
	uav_pitch = GPSData.ui8AngleX;                  // attitude pitch
	uav_roll = GPSData.ui8AngleY;                   // attitude roll
	uav_heading = GPSData.ui8AngleZ;                // attitude heading
	uav_gpsheading=GPSData.ui16Direction;           // gps heading
	uav_bat = ReceiverData.ui8Volt*100U;            // battery voltage (mv)
	uav_current = 0;                				// actual current
	uav_rssi = ReceiverData.ui8Strength;			// radio RSSI (%)
	uav_linkquality = ReceiverData.ui8Quality;		// radio link quality
	uav_airspeed = 0;               				// Airspeed sensor (m/s)
	uav_arm = 0;                    				// 0: disarmed, 1: armed
	uav_failsafe = 0;               				// 0: normal,   1: failsafe 
	uav_flightmode = 19;            				// Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
													// 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND, 
													// 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
	protocol = "HoTT";
	telemetry_ok = true;
	lastpacketreceived = millis();

#ifdef HOTT_DEBUG
	uav_satellites_visible = 6;
	uav_fix_type = 3;
#endif


}

#endif

