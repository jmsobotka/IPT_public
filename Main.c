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
#include "TP1200.h" // <<< CHANGE 1: INCLUDE NEW HEADER

// compiler includes
#include "stdio.H"
#include "stdlib.H"
#include "string.H"

#define NAME "IPTI_B"
#define PART ""
#define VERSION "1.2.0"

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

////////////////////////////////////////////////////////////////////////
// XDATA
////////////////////////////////////////////////////////////////////////

/* ///////////////////////////////////////////////////////////////////////////
	Start of C 
////////////////////////////////////////////////////////////////////////// */
void main() {
    byte a;
    data char RetStr[10]; // return packet header (not used for pressure)
    unsigned int raw_temp_adc, raw_press_adc;
    float compensated_temp_c;
    float compensated_press;

	WDTCN = 0xDE;
	WDTCN = 0xAD;
    
	CS_5V = 1; // 5 Volt enable

	// turn off chip selects - active low
	
	CS_EE=1; 
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

    Initialize_Sensor(); // <<< CALL TO NEW SENSOR INITIALIZER

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
			// ReadTemperature(); // This call is removed as temperature is read with pressure.
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
            Get_Raw_Sensor_Readings(&raw_press_adc, &raw_temp_adc);
            g_raw_temp_adc = raw_temp_adc; // Update global for maintenance function
            compensated_temp_c = Calculate_Compensated_Temperature(raw_temp_adc);
            compensated_press = Calculate_Compensated_Pressure(raw_press_adc, raw_temp_adc);
            psi = Apply_System_Calibration(compensated_press, offset, span);
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
					printf("%s%bd(PSI)\r", RetStr, (Max_P - Min_P));
					// this value is ballpark only because it is temperature dependant
					//printf("%s%7.3f\r", RetStr , CompPressureUnAdjusted(PMax.l) );
				   	break;

// production date
			 	case HC_P_:		
					printf("%s%02bu/%02bu/%04u\r", RetStr, g_ProductInfo.cal_month, g_ProductInfo.cal_day, g_ProductInfo.cal_year );
					break;
					
// pressure				   note: this command DOES NOT return the command string...sigh
			 	case HC_P1:
					if (NVR_Buf.displayunits == 0) { // PSI
					    printf("#%02bdCP=%7.4f\r", NetAdd, psi);
					} else { // MBAR
					    printf("#%02bdCP=%7.3f\r", NetAdd, psi * PSI_TO_MBAR);
					}
					break;
// serial number
			 	case HC_S_:	
					printf("%s%lu\r", RetStr, g_ProductInfo.serial_number );
					break;
					
// temperature C
			 	case HC_T1:
					// curve fit temperature
					printf("%s%4.1f\r", RetStr, compensated_temp_c);
				   	break;

// temperature F
			 	case HC_T3:
					// curve fit temperature and convert to F
					printf("%s%4.1f\r", RetStr, (compensated_temp_c * 9.0f / 5.0f) + 32.0f);
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
	SPIOWriteBuf
	TBD 		: 
	Arguments	: byte buffer ptr, # bytes to write	
	Returns 	: 
/////////////////////////////////////////////////////////////////////////// */
/*
void SPIOWriteBuf( byte * b, byte len ) {
	byte a;
	for( a = 0; a < len; a++ ) {
	   SPIOWrite( b[a] );
	}
}
*/

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
/*
float TemperatureC( word r ) {

	return(((float)(r - TMin.w)* (float)125/(TMax.w-TMin.w))) - 40;
}
*/

/* ///////////////////////////////////////////////////////////////////////////
	Convert temperature reading to Fahrenheit
	Arguments	: raw temp reading
	Returns 	: Fahrenheit
/////////////////////////////////////////////////////////////////////////// */
/*
float TemperatureF( word r ) {
	return((TemperatureC(r)*9)/5)+32;
}
*/

void ReloadWatchdog(void)
{
	WDTCN = WDRLD;
}
