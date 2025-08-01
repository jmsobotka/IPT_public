/*///////////////////////////////////////////////////////////////////////////
//  Header for Gen 8051 Library
//  Date:    July 12, 2008
//  Version: 0.01
//
//  SelectCode                 for                  
//  Bruce Hansen               DFW Instruments      
//  5013 Peninsula Way         2544 Tarpley Rd # 116
//  Grand Prairie TX           Carrollton, TX 75006 
//  Copyright 2008      
//  
//  Change History
//  Version	Date	    Eng   Description  
//
//  See 8051Lib.C
//
//////////////////////////////////////////////////////////////////////////////*/
//------------------------------------------------------------------------------------
// data type definitions
#define dword unsigned long
#define word unsigned int
#define byte unsigned char
#define boolean bit

//////////////////////////////////////////////////////////////////////////////
// Status string for the hnywell PPT RS= command
// error codes into a 4 char string where each char is an error class (RStatusTypes)
// per enumerations and SetRStatus()
// see honeywell ppt manual page 66 for further info

void SetRStatus( byte type, char error );

#define RS_OK '0'

extern char RStatus[];

enum RStatusTypes {
RSP,
RSQ,
RSR,
RSS
};

// EEPROM errors (why its called p I do not know...)
enum RSP_Type {
RSP_CHECKSUM = '1',
};

// general errors (no definition in the datasheet)
enum RSQ_Type {
RSQ_NVRAM = '1',
RSQ_COFOVFLW = 'O',
};

// Serial errors
enum RSR_Type {
RSR_OVERFLOW = '1',
};

// Sensor errors and processor
enum RSS_Type {
RSS_OVERTEMP =  '>',
RSS_UNDERTEMP = '<',
RSS_OVERPRS =   '+',
RSS_UNDERPRS =  '-',
RSS_WATCHDOG =  'W',
RSS_JTAG =  	'J',
RSS_RSTPIN =  	'R',
RSS_MISSCLOCK =	'M',
RSS_SOFTW =  	'S',
RSS_CNVRSEF =  	'C',
RSS_C0RSEF =  	'0',
RSS_ADCINIT =   'A',
RSS_ADCERROR =  'E',
};


//////////////////////////////////////////////////////////////////////////////
// Packet Message Type 

//////////////////////////////////////////////////////////////////////////////*/
// return codes for the Packet Parser
enum PacketParseReturnEnums {
PACKETFOUND = 0, 	 // 0
BUFFER_EMPTY};   // Must be last, this is the start of data Naks
// end of this table is start of data naks
// this allows this library to directly return naks

// State machine for the host packet parser
enum PacketParseStateEnums {
HP_IDLE,  				// 0
HP_ADDRESS1,
HP_ADDRESS2,
HP_COMMAND1,
HP_COMMAND2,
HP_GETPARM,
HP_CRETURN,
};

// CRC Settings
#define CRC_SEED 0
#define BITS_PER_BYTE 8
#define CRC_POLY 0x18

// RSTSRC SFR 0xEF Reset Source
// R       R/W     R/W     R/W     R       R       R/W     R
// ------  CNVRSEF C0RSEF  SWRSEF  WDTRSF  MCDRSF  PORSF   PINRSF
// Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define CNVRSEF 0x40
#define C0RSEF  0x20
#define SWRSF   0x10
#define WDTRSF  0x08
#define MCDRSF  0x04
#define PORSF   0x02
#define PINRSF  0x01

//////////////////////////////////////////////////////////////////////////////
// Port Pins
sbit TX0=P0^0;       // 485,EX,232 TX0
sbit RX0=P0^1;       // 485,EX,232 RX0
sbit SCK=P0^2;       // SPIO CLOCK
sbit MISO=P0^3;      // SPIO IN 
sbit MOSI=P0^4;      // SPIO OUT
sbit NSS=P0^5;       // SPIO SELECT 


//////////////////////////////////////////////////////////////////////////////
// Library Data Exports
//////////////////////////////////////////////////////////////////////////////
// Port 3
// defined in 8051Lib_I.asm and exported here to allow no inconsistancy
// MAXIM 3162 232 and 485 RX and TX Enables on UART1, Shared with asm code
extern bit TE232; // BIT	P3.0  
extern bit RE232; // BIT	P3.1
extern bit DE485; // BIT    P3.2  // driver enable	
extern bit RE485; // BIT	P3.3			; 
// 4 and 5 are NC
// ADM489AR 485 RX and TX Enable on UART0, Shared with asm code
extern bit DR485; // BIT    P3.6

extern byte xdata Rb[];
extern byte xdata cp[];                      // maint buffer return double buffer 2

// Enumeration
enum {
b0000,
b0001,
b0010,
b0011,
b0100,
b0101,
b0110,
b0111,
b1000,
b1001,
b1010,
b1011,
b1100,
b1101,
b1110,
b1111,
};

// Constants
#define RI1 1;
#define TI1 2;
#define REN1 0x10;

// utility
#define FALSE 0
#define TRUE -1
#define ON 1
#define OFF 0
#define SET 1
#define CLR 0

// debug defines
#define PRINTOUT	1    
#define QUIET		0

// Constants
#define PAYLOADMAXLEN 50       // size of the inpacket data buffer
#define PACKET_IN_START_CHAR '<'
#define PACKET_OUT_START_CHAR '>'

#define XTAL 22118400L

// uart circle buffer size
// do not change without altering code, byte index is used
#define IBSIZE 0x100   
#define MBSIZE 0x100  
// serial text buffer size
#define SERBUFSIZE 50

// bit Control Flags for menu 
#define MNU_POLL 0x1
#define MNU_HIDE 0x2

// maint menu item
typedef struct {
	char * trig;
	char * desc;
	byte cntl;
} MenuItemType;

//////////////////////////////////////////////////////////////////////////
// Function prototypes
char * GetCommand( byte c );
void ParseCommand(char * ib);
int CheckCommand( char * ib, MenuItemType mnu[] );

void RealTimeClock( void );

// onboard serial
int print0   (char *, char s, ...);
int print1   (const char *, ...);
byte ProcessUART1( void );
int GetHostPacket(void);
void InitSerial();
void TransmitToHost(byte * b, byte len);
void TransmitToMonitor(char *b, byte len);
byte Transmit_UART0(byte c);
byte Transmit_UART1(byte c);
char putchar(char c); 
byte AbsDiff( byte in, byte out );
unsigned char Crc8(byte *DataPtr, unsigned char Count);
byte RunCRC8( byte b );
byte RunCRC8_Calc( byte b );

// onboard led drivers
void WriteDac( byte add, int dta );

// onboard switches
#define SWBUFFSIZE 8
byte INT_BitPos8( byte b );

// util
word BCD2Int(char *s, byte len);
byte Bin2Hex8 (byte i);
byte BitPos8(byte b);
byte BitCnt8(byte b);
byte BitRev8(byte b);
void Strup( char * s );
void MSDelay( word t);
void USDelay( word t);
word GetLastADC( byte c );
float FConvert( float c );
float CpuTemperature(void);
float sign( float f );
void M32_16();
void M32_24();

// flash
void Flash_Write( byte * b, byte len );
void Flash_Read( byte * b, byte len );
void Flash_ByteWrite (word addr, byte b, bit SFLE);
unsigned char Flash_ByteRead (word addr, bit SFLE);
void Flash_PageErase (word addr, bit SFLE);
byte MakeCheckSum( byte *p, int len);
bit CheckSum( byte *p, int len);

// error reporting
void WriteErrorQue(byte Source, byte Error);
void InitEQue(void);
void ErrLookup( byte src, byte err );
void ErrTable( void );
void ErrDump(void);

// utility
void mul32bit();

// byte access to typed variables
typedef union  {
float f;
byte  c[4];
} FloatByteUnion; 

typedef union {
unsigned long l;
byte c[4];
} ULongByteUnion;

typedef union {
long l;
byte c[4];
} SLongByteUnion;

typedef union {
byte c[2];
word w;
}WordByteUnion;

#define MAKEWORD(high, low) ((word)((((word)(high)) << 8) | ((byte)(low))))

// serial data
extern data byte NetAdd;	 // Address switch &0F
extern data byte HPNetAdd;
extern data WordByteUnion HPCmd;
extern data byte ParmIndx;
extern byte Parm[];

//////////////////////////////////////////////////////////////////////////////*/
// Error Handling

extern xdata byte TaskOverRun; // make the task overrun public

// size of the error queue
#define EQUSIZE 16
extern byte xdata eqi;       // error queue index, make available for debug
extern byte xdata EQue[EQUSIZE]; // error queue

//////////////////////////////////////////////////////////////////////////////*/
// Local to the Library
// ANSI Color codes ESCm
#define BLACK    30 
#define RED      31 
#define GREEN    32 
#define YELLOW   33 
#define BLUE     34 
#define MAGENTA  35 
#define CYAN     36 
#define WHITE    37
 
// ANSI Attribute codes ESCm
#define NORMAL         0
#define BOLD           1
#define UNDERLINE      4
#define BLINK          5
#define REVERSE        7
#define NONDISPLAYED   8


extern char code Line[];
extern char code Head[]; 
void A_Color( byte a, byte c );
void A_Pos( byte x, byte y );
void MenuTask( byte task );

void Display_MEMSCAP_EEPROM_Info(void);
