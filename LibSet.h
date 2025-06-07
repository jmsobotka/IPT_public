/*//////////////////////////////////////////////////////////////////////////
//  Library Settings for 8051Lib 
//  Date:    July 12 2008
//
//  SelectCode                 for                  
//  Bruce Hansen               DFW Instruments      
//  5013 Peninsula Way         2544 Tarpley Rd # 116
//  Grand Prairie TX           Carrollton, TX 75006 
//  Copyright 2008      
//  
//  Ths Archimedes Compiler is picky about uncalled code due to its stack 
//  setup. This file implements ifdef defines to allow quickly adding and  
//  removing library routines from a project. 
//
//  Change History
//  Version		Date	Eng	Description  
//
///////////////////////////////////////////////////////////////////////////// */
// Build flags
//#define DEVKIT 			// code specific to the F020 Developer board
//#define DEBUGPRINT 		// activate printf debugs

/*/////////////////////////////////////////////////////////////////////////// */

#define ENABLE_WATCHDOG_RESET  0xA5
#define DISABLE_WATCHDOG_RESET 0xAD

// set processor watchdog reset reload
#define WDRLD ENABLE_WATCHDOG_RESET

// Library Control Flags will remove these sections of code if not used
#define SET_SERIAL
#ifdef SET_SERIAL

//////////////////////////////////////////////////////////////////////////////
// network address
	#define SET_INITSERIAL
//	#define SET_GETHOSTPACKET        	// mutually exclusive
	#define SET_GETHOSTPACKETNOCRC		// mutually exclusive
	#define SET_GETCOMMAND
//	#define SET_TRANSMITTOHOST
//	#define SET_TRANSMITTOMONITOR
	#define SET_TRANSMIT_UART0
	#define SET_TRANSMIT_UART1
	#define SET_PUTCHAR
	#define SET_ABSDIFF	
//	#define SET_CRC8TEST
//	#define SET_PROCESSUART1
#endif

//#define SET_ADC0_ISR
#ifdef SET_ADC0_ISR
	#define ADCFREQ 0x8000; 			// overides init, TIMER2 duty cycle
	#define GET_LASTADC
	#define SET_TEMPERATURE
#endif

#define SET_TIMER3_ISR

//#define SET_SPIO_ISR

#define SET_UTIL
#ifdef SET_UTIL
//#define SET_USDELAY
#define SET_STRUP
//#define SET_BIN2HEX8
//#define SET_BCD2INT
//#define SET_BITSTR
//#define SET_BITCNT8
//#define SET_BITREV8
//#define SET_BITPOS8
//#define SET_REALTIMECLOCK
#endif

#define SET_NVFLASH
#ifdef SET_NVFLASH
	#define SET_FLASH_WRITE
	#define SET_FLASH_HREAD
	#define SET_FLASH_BYTEWRITE
	#define SET_FLASH_BYTEREAD
	#define SET_FLASH_PAGEERASE
#endif

#define SET_ERROR
#ifdef SET_ERROR
	#define SET_INITEQUE
	#define SET_WRITEERRORQUE
	#define SET_ERRLOOKUP
	#define SET_ERRTABLE
	#define SET_ERRDUMP
#endif



