/* ///////////////////////////////////////////////////////////////////////////
  8051 IPT Interface
  Date:    May 15 2009
                             
  SelectCode                 for                  
  Bruce Hansen               DFW Instruments      
  5013 Peninsula Way         2544 Tarpley Rd # 116
  Garland TX           		  Carrollton, TX 75006 
  Copyright 2009            

  Description
  This code is specific to IPT Interface board Rev A and Honeywell IPT
  See schematic and related docs for specifics  

  Change History
  Version	Date  		Auth 	Description
  0.00		5/15/09		BH		Initial
  1.00		8/25/09		BH		Release
  1.10		11/16/09		BH		RevB Board, added 5v enable
  										moved temperature read from 2hz to 1hz to get rid of stutter
										Added an ignore for the Honeywell SP=ALL command to allow better calibration compatibility
  1.11		1/27/2010	BH		Added support for 9200 addresses as follows
											Switch = E : address is set to 11
											Switch = F : address is set to 21
  1.12		8/18/10		BH		Decreased span granularity from 00003 to 00001
  1.14		6/22/12				Increased calibration to (+-)30000 range
  										span X highalt 1/3 foot resolution
  										offset Z lowalt 1/4 foot resolution 
  
////////////////////////////////////////////////////////////////////////////*/
/* TBD

	Kick up baud rate, figure a way to make it backward compatible

*/
////////////////////////////////////////////////////////////////////////////
// custom includes
#include "C8051F020_Kdefs.h"
#include "8051LIB.H"
#include "Main.H"
#include "float.H"
#include "LibSet.h"

// compiler includes
#include "stdio.H"
#include "stdlib.H"
#include "string.H"

#define NAME "IPTI_B"
#define PART ""
#define VERSION "1.1.4"

#define PSI_TO_MBAR 68.94759

// integer scale for the temperature conversion routines
// balances accuracy with the quad integer size
// if too large, the quad multiply overflows and the 
// conversion fails, if too small, accuracy suffers
#define SCALE 10000
data int scale;

////////////////////////////////////////////////////////////////////////////
// constant code data

// identify this file to the library
char code EXNAME[] = NAME;
char code EXVERSION[] = VERSION;
char code EXPART[] = PART;
char code EXFILE[] = __FILE__;
char code EXTIME[] = __DATE__ " " __TIME__;

////////////////////////////////////////////////////////////////////////////
// bit variables

// EEPROM Commands
sbit CS_EE=P1^2; // Chip Select EEPROM
sbit CS_TP=P1^1; // Chip Select Temperature
sbit CS_PR=P1^0; // Chip Select Pressure
sbit CS_5V=P2^7; // 5 Volt Enable

sbit LED=P2^4;   // LED

////////////////////////////////////////////////////////////////////////////
// type defs

// NVRam write/read buffer
typedef struct  {
byte displayunits;
int offset;
int span;
byte CalDate[10];
byte NetAdd;
byte CRC;
} NVR_BufType; 

////////////////////////////////////////////////////////////////////////
// DATA
////////////////////////////////////////////////////////////////////////

// union uses compiler to convert eeprom byte data to float
data FloatByteUnion tf;

// raw pressure and temperature from ADC 
data ULongByteUnion pr;
data WordByteUnion tp;

// integration accumulator
data unsigned long acc = 0;

// 32 bit multiply	: registered interface to assembly
data long n1 			_at_ 0x30; 
data long n2 			_at_ 0x34; 
data ULongByteUnion Hr 	_at_ 0x38; 
data ULongByteUnion Lr 	_at_ 0x3C; 
data char sgn;

// temperature factors
data long f[6];

// result
data float psi = 0;

// fast ram calibration, with shadows in NVRam
// byte displayunits = 0; // 0 psi, 1 mbar
int offset = 0; 
int span = 0;

// EEProm Fletchers checksum
data byte ck1, ck2;

////////////////////////////////////////////////////////////////////////
// PDATA
////////////////////////////////////////////////////////////////////////

// nvram buffer
NVR_BufType NVR_Buf;

// temperature compensation coefficients from IPT eeprom as longs
long cof[COF_float_SIZE];

// structures for converting eeprom data
ULongByteUnion PMin, PMax, IPTSerialNo;
WordByteUnion TMin, TMax;

byte PrsRange = 0;  // pressure range  coefficient 37 from IPT eeprom
					// used for calibration

////////////////////////////////////////////////////////////////////////
// XDATA
////////////////////////////////////////////////////////////////////////

// EEProm byte array
xdata byte ee[EE_SIZE]; // not required, but its a one time event


/* ///////////////////////////////////////////////////////////////////////////
	Start of C 
////////////////////////////////////////////////////////////////////////// */
main() {
byte a;
data char RetStr[10]; // return packet header (not used for pressure)

	CS_5V = 1; // 5 Volt enable

	// turn off chip selects - active low
	
	CS_EE=1; 
	CS_TP=1;
	CS_PR=1;

	Init_Device();
	SPI0CFG |= 0x7;  // must set SPI to 8 bits manually, Config2 does not set SPISFRS

	// if restart is not powerup
	if( !(RSTSRC & PORSF) ) {

		// convert Start Reset Source Enable and Flag
		if( (RSTSRC & CNVRSEF) ) {          
			SetRStatus( RSS, RSS_CNVRSEF );
		}
		// comparator0 Reset Enable and Flag
		if( (RSTSRC & C0RSEF) ) {          
			SetRStatus( RSS, RSS_C0RSEF );
		}
		// software Reset Force and Flag
		if( (RSTSRC & SWRSF) ) {          
			SetRStatus( RSS, RSS_SOFTW );
		}
		// watchdog reset source
		if( (RSTSRC & WDTRSF) ) {          
			SetRStatus( RSS, RSS_WATCHDOG );
		}
		// Missing Clock Detector Flag
		if( (RSTSRC & MCDRSF) ) {          
			SetRStatus( RSS, RSS_MISSCLOCK );
		}
		// HW Pin Reset Flag
		if( (RSTSRC & PINRSF) ) {          
			SetRStatus( RSS, RSS_RSTPIN );
		}
 	}

	InitEQue();
	InitSerial();

	NetAdd = ~P2;
	P3 = 0;
	NetAdd &= 0xF;
	if(NetAdd == 14) NetAdd = 11; // legacy address support
	if(NetAdd == 15) NetAdd = 21; // legacy address support

	// scheduler
	t4o = 0;    // task overrun
	t4m = 0xFF; // task mask

	LED = 0; 	// led off causes led to flash at watchdog

	// read parameters from nvram
	Flash_Read( (byte *)&NVR_Buf, sizeof(NVR_Buf) );
	a = Crc8((byte *)&NVR_Buf, sizeof(NVR_Buf)-1 ); // calculate checksum for nvram
	if( NVR_Buf.CRC != a ) {
		offset = 0;
		span = 0;
		NVR_Buf.displayunits = 0; // psi
		memset(NVR_Buf.CalDate, 0, sizeof(NVR_Buf.CalDate));
		NVR_Buf.NetAdd = 0;
		SetRStatus( RSQ, RSQ_NVRAM );
	} else {
		offset = NVR_Buf.offset;
		span = NVR_Buf.span;
	}

	// if the switch is set to zero, use nvram setting
	// else use the switch setting
	if( NetAdd == 0 ) {
		NetAdd = NVR_Buf.NetAdd;
	}

	// read the sensor eeprom
	a = 10;
	while( (!ReadEEProm()) && (--a)){
		} // will watchdog reset here if sensor not present
		  // just what we want, retries the sensor and 
		  // causes LED to flash at high rate

	if(!a) SetRStatus( RSP, RSP_CHECKSUM ); 

	// intialize IPT devices
	InitPressure();	
	InitTemperature();												  
	ReadTemperature(); // set up temperature compensation

////////////////////////////////////////////////////////////
	EA = 1;  // interrupt enable

//	printf("%s IPT %s %bd\r\r", Head, RStatus, NetAdd);

	while( TRUE ) {

		WDTCN = WDRLD;// reload the watchdog

		// grey code scheduler running off timer interrupt
		// each of these tasks is called at the rate indicated
		// goto's are used so that the compiler will create
		// a low overhead jump table
		if(Tsf0){ Tsf0=CLR; goto task0;}
		if(Tsf1){ Tsf1=CLR; goto task1;}
		if(Tsf2){ Tsf2=CLR; goto task2;}
		if(Tsf3){ Tsf3=CLR; goto task3;}
		if(Tsf4){ Tsf4=CLR; goto task4;}
		if(Tsf5){ Tsf5=CLR; goto task5;}
		if(Tsf6){ Tsf6=CLR; goto task6;}
 		if(Tsf7){ Tsf7=CLR; goto task7;}
		goto idle;

		task7: // 1000ms 1hz
			ReadTemperature();
			goto idle;
	
		task6: // 500ms 2hz
			
			goto idle;
	
		task5: // 250ms 4hz
			// Blink LED
			LED = ~LED;	
			goto idle;

		task4: // 125ms 8hz
			goto idle;

		task3: // 62.5ms 16hz
			goto idle;

		task2: // 31.25ms  32hz 
			goto idle;

		task1: // 15.625ms   64hz
			ReadPressure();	
			goto idle;

		task0: // 7.8125ms   128hz
			goto idle;

		idle:											 
			if( GetHostPacket() == PACKETFOUND ) {

			// build response header, support Honeywell silliness
			// two letter commands add an equal sign, one letter commands do not
			if( HPCmd.c[1] == '=' )
				sprintf( RetStr, "#%02bd%c%c", NetAdd, HPCmd.c[0], HPCmd.c[1] );
			else
				sprintf( RetStr, "#%02bd%c%c=", NetAdd, HPCmd.c[0], HPCmd.c[1] );

// parse command by word hash
 			switch( HPCmd.w ) {						

// cal date
			 	case HC_A_:
					if( ParmIndx ) {  // if data set parameter
						strcpy( &NVR_Buf.CalDate, &Parm ); 
						WriteNVRam();
					} else {		  // else return setting
						printf("%s%s\r", RetStr, NVR_Buf.CalDate );
					}
					break;

// set baud rate
				case HC_BP:
					if( ParmIndx ) {  // if data set parameter
						//Parm[0] = 0x20;
						a = (byte)(atoi(Parm+2));
						switch(a) {
							case 96 :
							case 9 :
								TH1 = 0xB8;
								break;	
							case 14 :
								TH1 = 0xD0;
								break;	
							case 19 :
								TH1 = 0xDC;
								break;	
							case 28 :
								TH1 = 0xE8;
								break;	
							case 57 :
								TH1 = 0xF4;
								break;	
							case 11 :
								TH1 = 0xFA;
								break;	
						};
					} else {		  // else return setting
						printf("%s%s\r", RetStr, "N" ); // return parity only....nice
					}
					break;

// check eeprom
			 	case HC_CK:
					printf("%s%s\r", RetStr , ReadEEProm() ? "OK" : "ERR"  );
				   	break;

// display units
			 	case HC_DU:
					if( ParmIndx ) {  // if data set parameter
						if( !strcmp(Parm, "=PSI" )) {			
							NVR_Buf.displayunits = 0;
							WriteNVRam();
						}
						if( !strcmp(Parm, "=MBAR" )) {			
							NVR_Buf.displayunits = 1;
							WriteNVRam();
						}
					} else {		  // else return setting
						printf("%s%s\r", RetStr, NVR_Buf.displayunits ? "MBAR" : "PSI" );
					}
					break;

// net address and identification
			 	case HC_ID:
					if(	ParmIndx ) {  // if data set parameter
						NetAdd = (byte)atoi((Parm+1)); // +1 removes the = sign
						NVR_Buf.NetAdd = NetAdd;
						WriteNVRam();
					} else {		  // else return NAME
						printf("%s%s\r", RetStr , NAME  );
					}
				   	break;

// pressure range
			 	case HC_M_:
					printf("%s%bd(PSI)\r", RetStr, PrsRange);
					// this value is ballpark only because it is temperature dependant
//					printf("%s%7.3f\r", RetStr , CompPressureUnAdjusted(PMax.l) );
				   	break;

// production date
			 	case HC_P_:		
					printf("%s%02bd/%02bd/%02bd\r", RetStr, ee[EE_MONTH], ee[EE_DAY], ee[EE_YEAR] );
					break;
					
// pressure				   note: this command DOES NOT return the command string...sigh
			 	case HC_P1:
					switch (NVR_Buf.displayunits) {
						case 0:
							printf("#%02bdCP=%7.4f\r", NetAdd, psi );
							break;
						case 1:
							printf("#%02bdCP=%7.3f\r", NetAdd, (psi * PSI_TO_MBAR) );
							break;
					}
					break;
// serial number
			 	case HC_S_:	
					IPTSerialNo.c[0] = ee[EE_SN];	
					IPTSerialNo.c[1] = ee[EE_SN+1];	
					IPTSerialNo.c[2] = ee[EE_SN+2];	
					IPTSerialNo.c[3] = ee[EE_SN+3];	
					printf("%s%ld\r", RetStr, IPTSerialNo.l );
					break;
					
// temperature C
			 	case HC_T1:
					// curve fit temperature
					printf("%s%4.1f\r", RetStr , TemperatureC(tp.w));
				   	break;

// temperature F
			 	case HC_T3:
					// curve fit temperature and convert to F
					printf("%s%4.1f\r", RetStr , TemperatureF(tp.w));
				   	break;

// version number
			 	case HC_V_:
					printf("%s%s\r", RetStr , VERSION  );
				   	break;

// write enable
			 	case HC_WE:	 // no return
					break;

// eeprom enable
			 	case HC_SP:	 // no return
					break;

// span
			 	case HC_X_:
					if( ParmIndx ) {  // if data set parameter
						span = atoi(Parm);
						WriteNVRam();
					} else // no return if setting
						printf("%s%d\r",RetStr, span  ); 
					break;

// offset
				case HC_Z_:
					if( ParmIndx ) {  // if data set parameter
						offset = atoi(Parm);
						WriteNVRam();
					} else // no return if setting
						printf("%s%d\r", RetStr, offset  );
					break;

// status
				case HC_RS:
					printf("%s%s\r", RetStr, RStatus );
					break;

// returns empty string to unsupported commands
				default:
					printf("%s\r", RetStr );
					

			}
		}
	}
}

/* ///////////////////////////////////////////////////////////////////////////
	InitTemperature
		TBD 	: Read back check
	Arguments	:
	Returns		:
////////////////////////////////////////////////////////////////////////// */
code byte TMode[] = {0x10, 0x80 };		// Mode Register - 20 hz
void InitTemperature() {
code byte TFilter[] = {0x20, 0x03 };	// Filter Register - buffer and gain per data sheet

	CS_TP=0;	  	  // chip select
	SPIOWriteBuf( TFilter, sizeof(TFilter));
	SPIOWriteBuf( TMode, sizeof(TMode));
	CS_TP=1;	  	  // chip select
}

/* ///////////////////////////////////////////////////////////////////////////
	ReadTemperature
		TBD 	:  
	Arguments	:
	Returns		: tp 		
////////////////////////////////////////////////////////////////////////// */
void ReadTemperature() {
byte a;
byte b;

	CS_TP=0;	  	  	// chip select

	b = 1;
	while( MISO && b ) {		// wait for conversion
		b++;	
	};

	SPIOWrite( 0x38 );	//  Send single read
	b = 1;
	while( MISO && b) {		// wait for conversion
		b++;	
	};

	for( a = 0; a < 2; a++ ) {
		tp.c[a] = SPIORead();
  	}

	SPIOWriteBuf( TMode, sizeof(TMode));

	CS_TP=1;	  	  // chip select

	CompTemperature();

	// compare against eeprom max/min : set status for temperature
	if(tp.w > TMax.w) SetRStatus( RSS, RSS_OVERTEMP ); 
	if(tp.w < TMin.w) SetRStatus( RSS, RSS_UNDERTEMP ); 

}

/* ///////////////////////////////////////////////////////////////////////////
	InitPressure
	Arguments	:
	Returns    	:
////////////////////////////////////////////////////////////////////////// */
code byte PConfig[] = {0x10, 0x10, 0x20 };	    // Configuration Register - buffer and gain per data sheet
code byte PMode[] = {0x08, 0x30, 0x01 };	    // Mode Register - power on, 500 hz
void InitPressure() {
byte a, b;

	ResetPressureADC();

	CS_PR=0;	  	  // chip select

	SPIOWriteBuf( PConfig, sizeof(PConfig));		// write
	SPIOWrite( 0x50 ); 								// read back
	a = SPIORead();
	b = SPIORead();
 	if( a != PConfig[1] || b != PConfig[2]) SetRStatus( RSS, RSS_ADCINIT );

	SPIOWriteBuf( PMode, sizeof(PMode));			// write
	SPIOWrite( 0x48 ); 								// read back
	a = SPIORead();
	b = SPIORead();
	if ( a != PMode[1] || b != PMode[2] ) SetRStatus( RSS, RSS_ADCINIT );

	SPIOWrite( 0x40 ); // read status register  (normal return is 0x88)
	a = SPIORead();
	if( a != 0x88 ) SetRStatus( RSS, RSS_ADCERROR );
	//printf("IP Status %bX\r", a );

	CS_PR=1;	  	  // chip select
}

/* ///////////////////////////////////////////////////////////////////////////
	ReadPressure
		over and under errors
	Arguments	:		
	Returns 	: pr
/////////////////////////////////////////////////////////////////////////// */
void ReadPressure() {
byte a;
byte b;
static byte c;

	CS_PR=0;	  	  	// chip select

	pr.c[0] = 0;		// 24 bit, zero unused byte
	SPIOWrite( 0x58 );	// read command to Communications register 

	b = 1;
	while( MISO && b ) {// wait for conversion
		b++;	
	};
							  
	// read 3 bytes
	for( a = 1; a < 4; a++ ) {
		pr.c[a] = SPIORead();
  	}

	SPIOWriteBuf( PMode, sizeof(PMode)); 	// start next read

	CS_PR=1;	  	  						// chip select

	// set status for pressure
	if(pr.l > PMax.l) SetRStatus( RSS, RSS_OVERPRS ); 
	if(pr.l < PMin.l) SetRStatus( RSS, RSS_UNDERPRS ); 

//#define NOINTEGRATION
#ifdef NOINTEGRATION
	acc = pr.l;
	CompPressure();
#else
//	integration
//	2 4
//	3 8
//	4 16
	acc += pr.l;
	c++;
	if( c > 3 ) {
		acc >>= 2;
		CompPressure();
		c = 0;
		acc = 0;
	}
#endif													   
}

/* ///////////////////////////////////////////////////////////////////////////
	ReadEEProm
	Arguments	:	
	Returns 	: cof and success flag
/////////////////////////////////////////////////////////////////////////// */
byte ReadEEProm() {
byte a;
bit Overflow;
code byte EEPRead[] = {EE_READ, 0}; 

	CS_EE=0;  // chip select

	// command
	SPIOWriteBuf( EEPRead, sizeof(EEPRead));

	// read data, with checksum, address auto increments
	ck1 = ck2 = 0;

	// read entire for checksum calculation
	for( a = 0; a < EE_SIZE; a++ ) {

		ee[a] = SPIORead(); // store as byte
	
		// Fletchers Checksum
		ck1 += ee[a];  // 0
		ck2 += ck1;    // 0
	}
	
	CS_EE=1;  // end of read, chip deselect

	// checksum ok?
	if((ck1 == 0) && (ck2 == 0)) {

		
		scale = SCALE;
		do {
			Overflow = 0;
			// convert coeficient bytes to long integers
			for( a = 0; a < COF_float_SIZE; a++ ) {
				tf.c[0] = ee[a*4];
				tf.c[1] = ee[a*4+1];
				tf.c[2] = ee[a*4+2];
				tf.c[3] = ee[a*4+3];
				cof[a] = (long)(tf.f * scale);
				if( !SignCheck( cof[a], tf.f )) {
					
					// SetRStatus( RSQ, RSQ_COFOVFLW );
					scale -= 1000;
					Overflow = 1;
					break;
				}
			}
			WDTCN = WDRLD;// reload the watchdog
		} while( Overflow );

		// pressure range
		PrsRange = (byte)(cof[37] / scale);

		// max pressure
		PMax.c[0] = ee[EE_PMAX]; 
		PMax.c[1] = ee[EE_PMAX+1]; 
		PMax.c[2] = ee[EE_PMAX+2]; 
		PMax.c[3] = ee[EE_PMAX+3]; 
		// min pressure
		PMin.c[0] = ee[EE_PMIN]; 
		PMin.c[1] = ee[EE_PMIN+1]; 
		PMin.c[2] = ee[EE_PMIN+2]; 
		PMin.c[3] = ee[EE_PMIN+3]; 

		// max temperature
		TMax.c[0] = ee[EE_TMAX];
		TMax.c[1] = ee[EE_TMAX+1];
		// min temperature
		TMin.c[0] = ee[EE_TMIN];
		TMin.c[1] = ee[EE_TMIN+1];

	} else {
		return FALSE;
	}
	return TRUE;
}

bit SignCheck( long a, float b ) {
bit ab = 0; 
bit bb = 0;

	ab = (a < 0) ? 0 : 1;	
	bb = (b < 0) ? 0 : 1;	
	return( ab == bb );

}

/* ///////////////////////////////////////////////////////////////////////////
	SPIOWriteBuf
	TBD 		: 
	Arguments	: byte buffer ptr, # bytes to write	
	Returns 	: 
/////////////////////////////////////////////////////////////////////////// */
void SPIOWriteBuf( byte * b, byte len ) {
	byte a;
	for( a = 0; a < len; a++ ) {
	   SPIOWrite( b[a] );
	}
}

/* ///////////////////////////////////////////////////////////////////////////
	SPIOWrite
	TBD 		: timeout
	Arguments	: 1 byte to write
	Returns 	: 
/////////////////////////////////////////////////////////////////////////// */
void SPIOWrite( byte b ) {
byte a = 1;
	SPIF = 0;
	SPI0DAT = b;	
	while( !SPIF && a++ )
		;
	SPIF = 0;
}

/* ///////////////////////////////////////////////////////////////////////////
	SPIORead
	TBD 		: timeout
	Arguments	: 
	Returns 	: 1 byte read
/////////////////////////////////////////////////////////////////////////// */
byte SPIORead( void ) {
byte a = 1;
	SPIF = 0;
	SPI0DAT = 0;
	while( !SPIF && a++ )
		;
	SPIF = 0;
	return SPI0DAT; 
}

/* ///////////////////////////////////////////////////////////////////////////
	CompPressure
		See IPT See IPT ADS-14152 Rev B 8_08.pdf and Compensation spreadsheet

	Y = A + x(F1 + x(F2 + x(F3 +x(F4 + x(F5 + x(F6)))))) 

	Arguments	: f factors
	Returns 	: psi
/////////////////////////////////////////////////////////////////////////// */
void CompPressure() {

	//p = pr.l;
	//p = acc;
	n1 = acc;
	n2 = f[5];
	M32_24();

	n2 += f[4];
	M32_24();

	n2 += f[3];
	M32_24();

	n2 += f[2];
	M32_24();

	n2 += f[1];
	M32_24();

	n2 += f[0];
	M32_24();

/*
	psi = ((float)(n2 + cof[0])) / SCALE;  // convert back to float
	// apply calibration
	psi += (float)(offset * .0001);
	psi += (float)(15.5-psi)*(span * .00001);
*/

	n2 = n2 + cof[0];
	// apply calibration
	n2 += (int)offset;	// Z lowalt

  	psi = (float)n2 / scale; // convert integer pressure back to a psi float
	psi += ((float)PrsRange-psi)*((float)span * .000001); // X highalt 1/3 foot resolution

}

/* ///////////////////////////////////////////////////////////////////////////
	CompPressureUnAdjusted
		See IPT See IPT ADS-14152 Rev B 8_08.pdf and Compensation spreadsheet
	NOTE		USED ONLY BY MAINTENANCE, NOT BY ANY HONEWELL COMMANDS
	 			: for converting eeprom min and max
	Arguments	: f factors
	Returns 	: psi
/////////////////////////////////////////////////////////////////////////// */
float CompPressureUnAdjusted(long p) {

	n1 = p;
	n2 = f[5];
	M32_24();

	n2 += f[4];
	M32_24();

	n2 += f[3];
	M32_24();

	n2 += f[2];
	M32_24();

	n2 += f[1];
	M32_24();

	n2 += f[0];
	M32_24();

	return ((float)(n2 + cof[0])) / scale;
}


/* ///////////////////////////////////////////////////////////////////////////
	CompTemperature
		See IPT ADS-14152 Rev B 8_08.pdf and compensation spreadsheet

	F1 = a1 + x(b1 + x(c1 + x(d1 + x(e1 + x(f1))))) 
	F2 = a2 + x(b2 + x(c2 + x(d2 + x(e2 + x(f2))))) 
	F3 = a3 + x(b3 + x(c3 + x(d3 + x(e3 + x(f3))))) 
	F4 = a4 + x(b4 + x(c4 + x(d4 + x(e4 + x(f4))))) 
	F5 = a5 + x(b5 + x(c5 + x(d5 + x(e5 + x(f5))))) 
	F6 = a6 + x(b6 + x(c6 + x(d6 + x(e6 + x(f6))))) 

	Arguments	: cof factors
	Returns 	: f factors
/////////////////////////////////////////////////////////////////////////// */
void CompTemperature() {
byte i = 0;

	// temperature factors
	for( i = 0; i < 6; i++ ) {
		n1 = (long)tp.w;;
		n2 = cof[31+i];
		M32_16();

		n2 += cof[25+i];
		M32_16();

		n2 += cof[19+i];
		M32_16();

		n2 += cof[13+i];
		M32_16();

		n2 += cof[7+i];
		M32_16();

		f[i] = n2 + cof[1+i]; 
	
	}	
}

/* ///////////////////////////////////////////////////////////////////////////
	WriteNVRam
	TBD 		: 
	Arguments	: offset, span and CalDate
	Returns 	: 
/////////////////////////////////////////////////////////////////////////// */
void WriteNVRam( void ) {
	NVR_Buf.offset = offset;
	NVR_Buf.span = span;
	NVR_Buf.CRC = Crc8((byte *)&NVR_Buf, sizeof(NVR_Buf)-1 );	// NVRAM checksum
	Flash_Write( (byte *)&NVR_Buf, sizeof(NVR_Buf) );      		// write it
}

/* ///////////////////////////////////////////////////////////////////////////
	Convert temperature reading to celsius linear scaled (calibrated) per 
			Temperature Max and Min from IPT EEPROM set to be 
			Min -40, Max 85 Range 125 degrees C
			Note: This solution is an educated guess, temperature is not
			well documented in the honeywell IPT datasheet
	Arguments	: raw temp reading
	Returns 	: approximate temperature in cel
/////////////////////////////////////////////////////////////////////////// */
float TemperatureC( word r ) {

	return(((float)(r - TMin.w)* (float)125/(TMax.w-TMin.w))) - 40;
}

/* ///////////////////////////////////////////////////////////////////////////
	Convert temperature reading to Fahrenheit
	Arguments	: raw temp reading
	Returns 	: Fahrenheit
/////////////////////////////////////////////////////////////////////////// */
float TemperatureF( word r ) {
	return((TemperatureC(r)*9)/5)+32;
}


void ResetPressureADC() {
byte a, b= 0xFF;
	SPI0CN    = 0x02; // spio off
	MOSI = 1;
	CS_PR= 0;
	for( a = 0; a < 65; a ++ ) {
		SCK	=1;
		while( b-- );
		SCK	=0;
		while( b-- );
	}
	CS_PR= 1;
	SPI0CN    = 0x03; // spio on
}
