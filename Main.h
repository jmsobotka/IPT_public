
/////////////////////////////////////////////////////////////
// Prototypes
void Init_Device(void);
void InitPressure(void);
void ReadPressure(void);
void InitTemperature(void);
void ReadTemperature(void);
void CompPressure(void);
void CompTemperature(void);
float CompPressureUnAdjusted(long p);
//float TemperatureC( word r );
//float TemperatureF( word r );
void ResetPressureADC(void);
//void SPIOWriteBuf( byte * b, byte len );
void SPIOWrite( byte b );
byte SPIORead( void );
void WriteNVRam( void );
void ReloadWatchdog(void);

////////////////////////////////////////////////////////////////////////////
// defines for data locations within the IPT EEPROM block
// per datasheet ADS-14152 Rev B page 14
#define EE_SIZE 0xB6
#define COF_BYTE_SIZE 152 // 38 floats
#define COF_float_SIZE COF_BYTE_SIZE / 4
#define EE_PRANGE 148
#define EE_PMIN 156
#define EE_PMAX 160
#define EE_TMIN 164
#define EE_TMAX 166
#define EE_TYPE 168
#define EE_MONTH 169
#define EE_DAY 170
#define EE_YEAR 171
#define EE_SN 172
#define EE_PN 176
#define EE_CK 182

// EEPROM Commands
#define EE_READ 3 // Read data from memory array beginning at selected address   
#define EE_WRIT 2 // Write data to memory array beginning at selected address    
#define EE_WRDI 4 // Reset the write enable latch (disable write operations)     
#define EE_WREN 6 // Set the write enable latch (enable write operations)        
#define EE_RDSR 5 // Read STATUS register                                        
#define EE_WRSR 1 // Write STATUS register   

/////////////////////////////////////////////////////////////////
// gray code scheduler import
// task flags          bit   ms         hz
extern bit Tsf0; //    0      7.812   128 
extern bit Tsf1; //    1     15.625   64    
extern bit Tsf2; //    2     31.250   32  
extern bit Tsf3; //    3     62.500   16  
extern bit Tsf4; //    4    125.000   8   
extern bit Tsf5; //    5    250.000   4   
extern bit Tsf6; //    6    500.000   2   
extern bit Tsf7; //    7   1000.000   1   
extern bdata byte Tsf;
extern byte t4o, t4m;

/////////////////////////////////////////////////////////////////
// Honeywell ADS-140562 PPT Command support
#define HC_A_ MAKEWORD('A', '=')
#define HC_BP MAKEWORD('B', 'P')
#define HC_CK MAKEWORD('C', 'K')
#define HC_DU MAKEWORD('D', 'U')
#define HC_ID MAKEWORD('I', 'D')
#define HC_M_ MAKEWORD('M', '=')
#define HC_P_ MAKEWORD('P', '=')
#define HC_P1 MAKEWORD('P', '1')
#define HC_RS MAKEWORD('R', 'S')
#define HC_S_ MAKEWORD('S', '=')
#define HC_SP MAKEWORD('S', 'P')
#define HC_T1 MAKEWORD('T', '1')
#define HC_T3 MAKEWORD('T', '3')
#define HC_V_ MAKEWORD('V', '=')
#define HC_WE MAKEWORD('W', 'E')
#define HC_X_ MAKEWORD('X', '=')
#define HC_Z_ MAKEWORD('Z', '=')

/////////////////////////////////////////////////////////////////
// DATA exports

extern data int scale;
 
extern data float psi;

extern int offset, span;

// 32 bit multiply
extern data long n1; 
extern data long n2; 
extern data char sgn;

// PDATA exports
extern long cof[];

// structures for converting eeprom data
extern WordByteUnion TMin, TMax;

// EEProm byte array
extern xdata byte ee[];

