

void ltm_read( void );
void ltm_write( void );
void ltm_check( void );

#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
#define LIGHTTELEMETRY_OFRAMELENGTH 18
	
void telemetry_off(void);
