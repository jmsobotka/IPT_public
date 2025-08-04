/* ///////////////////////////////////////////////////////////////////////////
	 8051 IPTI Library
	 Date:	 June 8 2009
	 Version: 1.0
										 
	 SelectCode						 for						 
	 Bruce Hansen					 DFW Instruments		 
	 5013	Peninsula Way			 2544	Tarpley Rd # 116
	 Garland	TX					 Carrollton, TX 75006 
	 Copyright 2008				 

	 Description
	 This	code is specific to 8051 Kit
	 See schematic	and related	docs for	specifics  

	 Change History
	 Version	Date		 Auth	 Description

		Peripheral Map
		-----------------------------------------------------------------------
		Peripheral | Uses							| Interrupt	 |	For
		-----------------------------------------------------------------------
		UART0		  | TIMER1 (baud)				| REC/TRN	 |	485 Host
		UART1		  | TIMER4 (baud)				| REC/TRN	 |	485 2 Honeywell Sensors
		ADC0		  | TIMER2,	channels	0-2+9	| CONV DONE	 |	Read Pots, cpu	temp
		TIMER0	  |								| NONE		 |	Timing Honeywell inputs
		TIMER1	  |								| NONE		 |	Baudrate	Gen
		TIMER2	  |								| NONE		 |	Start	ADC Conversion
		TIMER3	  |    (see RCT3 below)		| OVERFLOW	 |	Utility timer
		TIMER4	  |               			| NONE   	 |	Baudrate	Gen

	Real Time Clock Timer 3	set at 22118400: Div	12	: E3E0 Reload (7200)
	Timer	semaphores cycle at the	following rates
		bit		 ms	hz
		0		 7.812	128 
		1		15.625	64		
		2		31.250	32	 
		3		62.500	16	 
		4	  125.000	8	 
		5	  250.000	4	 
		6	  500.000	2	 
		7	 1000.000	1	 

/////////////////////////////////////////////////////////////////////////// */

#include	"C8051F020_Kdefs.h"							//	Register	definition file
#include	"LibSet.h"
#include	"Error.H" 
#include	"8051LIB.H"
#include	"Main.H"

#include	<string.h>
#include	<stdio.h>
#include	<ctype.h>
#include	<stdarg.h>
#include	<intrins.h>		//	for inline NOP

#include	"CRCTable.H"

// compiler has problems with this export
// export is from Main.c 
extern data ULongByteUnion Hr; 
extern data ULongByteUnion Lr; 

//	allow	linking file to identify itself
extern char	code EXNAME[];
extern char	code EXVERSION[];
extern char	code EXPART[];
extern char	code EXFILE[];
extern char	code EXTIME[];

//	Identify	this file to any that link	to	it
char code LIBNAME[] = "8051 Library";
char code LIBVERSION[] = "1.00";
char code LIBFILE[] = __FILE__;
char code LIBTIME[] = __DATE__ "	" __TIME__;

//	Real Time Clock
//xdata byte sec=0,	min=0, hrs=0;

xdata byte TaskOverRun = 0;

bdata byte Tsf;		//	Scheduler Timer flags
sbit Tsf0 =	Tsf ^	0;
sbit Tsf1 =	Tsf ^	1;
sbit Tsf2 =	Tsf ^	2;
sbit Tsf3 =	Tsf ^	3;
sbit Tsf4 =	Tsf ^	4;
sbit Tsf5 =	Tsf ^	5;
sbit Tsf6 =	Tsf ^	6;
sbit Tsf7 =	Tsf ^	7;

//	Misc variable
byte data LError;												//	last error,	debug	tool
char code Line[] = "----------------------------------------------\r";
char code Head[] = {27,'[','2','J',27,'[','1',';','1','H', 0};	//	ansi clear screen	and home

#ifdef SET_ERROR
//	Error	Handling
byte xdata EQue[EQUSIZE]; // error queue
byte xdata eqi;			  // error queue index
#endif

#ifdef SET_SERIAL

//	485 Com variables

data byte NetAdd;	 // Address switch
data byte HPNetAdd;
data WordByteUnion HPCmd;
data byte ParmIndx;
byte Parm[0x10];

data byte RState;
data byte crc;

byte xdata Rb[IBSIZE] _at_	0x100;			//	100 for compact model fixup UART0 RX BUFFER
byte xdata Tb[IBSIZE] _at_	0x200;			//	asm optimization, mirror in ASM 
byte data OutRp, InRp;						//	pointers (index) to the io circle buffers
byte data OutTp, InTp;						//	used in asm routines, don't change type

//	export to asm serial	interrupt routines
bit TX_Int_Restart_Flag	= 0;				//	NZ if TI=1 is required
bit TX_Int_Restart_Flag1 =	0;				//	NZ if TI=1 is required

//	Maintenance	command variables
xdata	byte xdata cs[SERBUFSIZE];			//	command string
xdata	byte xdata cp[SERBUFSIZE];			//	data string
xdata	byte xdata cb[SERBUFSIZE];			//	data buffer
bit Echo1;										//	flag to echo u0 to u1
bit Echo2;										//	flag to echo u1 to u0
xdata	word RawCPUTemperature;				//	raw cpu temperature (adc 8)
xdata	byte MenuPoll = CLR;					//	menu task under poll
#endif

#ifdef SET_BITSTR
byte xdata bs[9];
#endif

#ifdef SET_TIMER3_ISR
byte t4c, t4o,	t4m; // timer 4 counter
#endif

/*	///////////////////////////////////////////////////////////////////////////
	232 and 485	Serial Routines

InitSerial
Description:
	Init global	Packet parse variables,	RS485	and Maint port	(UART1 and 0)
	Also inits the	ASM transmit bit flags
Arguments: 
Returns:	
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_INITSERIAL
void InitSerial() {

	//	set serial hardware enables
	DE485	= CLR;		//	tx1 disabled
	RE485	= CLR;		//	_rx1 enabled (active	low)

	//	circle buffer pointers UART0 receive and transmit
	OutRp	= 0;
	InRp 	= 0;
	OutTp	= 0;
	InTp	= 0;

	RState 	= 0;	//	state	of	485 packet parser
	
	//	high level uart flags
	Echo2	= 0;		
	Echo1	= 0;

	//	low level uart	(set if data in transmit circle buffer)
	TX_Int_Restart_Flag 	= 1;
	TX_Int_Restart_Flag1	= 1;

	LError 	= 0;

}
#endif

#ifdef SET_GETHOSTPACKETNOCRC
/*	///////////////////////////////////////////////////////////////////////////
GetHostPacketNoCRC
Description: Name overload to	disable CRC	check
	see GetHostPacket
/////////////////////////////////////////////////////////////////////////// */
int GetHostPacket() {
byte InChar;

	while(OutRp	!=	InRp)	{ // process data	incoming
		InChar=Rb[OutRp++];

		switch(RState){						  // packet	parse	state	machine
		case HP_IDLE:						//	look for	packet start char
			if(InChar == '*'){
				RState = HP_ADDRESS1;
			} else {
				GetCommand(	InChar );  // process menu, as long	as	no	command contains '<'
			}
			break;

		case HP_ADDRESS1:						// check for digit
			if( isdigit(InChar)) {
				HPNetAdd = (InChar - '0') * 10;
				RState = HP_ADDRESS2;
			} else {
				RState = HP_IDLE;
			}
			break;

		case HP_ADDRESS2:					   
			if( isdigit(InChar)) {				// check for digit
				HPNetAdd += (InChar - '0');
				if( (HPNetAdd == NetAdd) || (HPNetAdd == 99)) {		// check network address (99 is global)
					RState = HP_COMMAND1;
				} else {
					RState = HP_IDLE;
				}
			} else {
				RState = HP_IDLE;
			}
			break;
  
		case HP_COMMAND1:					  	// set command byte 0 
			HPCmd.c[0] = InChar;
			RState = HP_COMMAND2;
			break;
 
		case HP_COMMAND2:						// set command byte 1 
			HPCmd.c[1] = InChar;				   
			ParmIndx = 0;
			RState = HP_GETPARM;
			break;

		case HP_GETPARM:						// get parameter
			if( InChar == 0xD ) {
				Parm[ParmIndx] = 0; 			// null terminate
				RState = HP_IDLE;
				return(PACKETFOUND);
			} else {
				Parm[ParmIndx] = InChar;
				ParmIndx++;
				ParmIndx &= 0xF;				// protect buffer
			}
			break;

		case HP_CRETURN:					  	// 
			RState = HP_IDLE;
			return(PACKETFOUND);

		} // switch
	} // while
	return(BUFFER_EMPTY);
}

#endif

/*	///////////////////////////////////////////////////////////////////////////
Crc8
	Description:
		Calculate a	crc for a byte	buffer
	Arguments:
		byte buffer
		# of bytes to crc
	Returns:
		crc
/////////////////////////////////////////////////////////////////////////// */
unsigned char Crc8(byte	*DataPtr, unsigned char	len) { 
	crc = CRC_SEED; 
  
	for (; len > 0; len--) { 
	RunCRC8( * DataPtr	);
	(DataPtr)++; 
	} 
	return crc;	 
} 

#ifdef SET_CRC8TEST
/*	///////////////////////////////////////////////////////////////////////////
RunCrc8_Calc
	Description:
		Calculate a	running crc, slow, but saves codes space and	used to check table
	Arguments:
		byte 
	Returns:
		crc
/////////////////////////////////////////////////////////////////////////// */
byte RunCRC8_Calc( byte	b ) {
  byte i	; 
	for (i =	0 ; i	< BITS_PER_BYTE ;	i++) { 
	  if ((b	^ crc	) & 0x01) {	
		 // shift and subtract poly 
		 crc = (( crc ^ CRC_POLY )	>>	1 ) |	0x80 ; 
	  } else	{ 
		 // transparent shift 
		 crc >>=	1 ; 
	  } 
	  b >>= 1 ;	
	} 
	return crc ;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
RunCrc8
	Description:
		Look Up a running	crc
	Arguments:
		byte 
	Returns:
		crc
/////////////////////////////////////////////////////////////////////////// */
byte RunCRC8( byte b	) {
	crc = CRCTable[(b ^ crc)];
	return crc;
}


/*	///////////////////////////////////////////////////////////////////////////
Transmit_UART0
Description:
	Output one character to com port
Arguments:
	character to output
Returns:
	character output
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_TRANSMIT_UART0
//#pragma disable	//	instruct compiler to disable interrupts during this call
byte Transmit_UART0(byte c) {

	//	Add the data to the transmit buffer
	Tb[InTp++] = c;
	if(InTp == OutTp)	{
		WriteErrorQue(UART00, TXOVFLW);
		OutTp++;	//	allow current byte, remove older data
	}

	//	transmit interrupt is disabled, then enable it.
	if(TX_Int_Restart_Flag)	{
	TX_Int_Restart_Flag = 0;
		TI0=1;			 // generate transmit interrupt
	}
	return (c);
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
putchar
	Description:
		Make printf	work with the comport either uart. Slow transmit at 200 characters
	  in the	transmit	buffer, but	don't	wait forever.
	Arguments:
		character to output
	Returns:
		character output	(ansi)
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_PUTCHAR
char putchar( char c) {
int to =	-1;

	while(AbsDiff(	InTp,	OutTp	) > 200 && to--) { // don't overflow transmit buffer
		WDTCN	= WDRLD;// reload the watchdog
	}
	Transmit_UART0(c);
	return(c);
}
#endif

////////////////////////////////////////////////////////////////////////////
//	General Utility

#ifdef SET_TEMPERATURE
///////////////////////////////////////////////////////////////////////////
//	Celsius to Fahrenheit conversion
float FConvert( float c	) {
	c *= 9;
	c /= 5;
	c += 32;
	return c;
}

///////////////////////////////////////////////////////////////////////////
//	floating	point, 2.4 volt adc to celsius
float CpuTemperature(void)	{
float f;
	f = RawCPUTemperature *	5.9326;
	f = (f -	7750)	/ 27.5;
	return f;

/*	optimized 2.4 volt to integer	celsius	(works nicely,	patent pending	Bruce	Hansen)
				w = RawCPUTemperature;
		  w *= 16;
				w /= 27;
				w -= 775;
		  w *= 4;
				w /= 11;
				return w;
*/
}
#endif

/////////////////////////////////////////////////////////////////////////////
//	External	Interrupt 7
//	Description:
//	This code is called on the	rising edge	of	Port 3 bit 7
//	
// UNUSED 
void Ext7_ISR(	void ) interrupt 19 {
	P3IF &= ~0x80;				  // clear int	source
}

#ifdef SET_SPIO_ISR
/* ///////////////////////////////////////////////////////////////////////////
	Serial Peripheral Input Output Interrupt
		Half Duplex Hardware Transmit of clocked serial 
	Arguments:
		SPIO_Buf[SPIO_DATA_CNT] byte array containing bit data to turn SPIO_Buf on and off
	Returns:

/////////////////////////////////////////////////////////////////////////// */
// UNUSED 
void SPIO_ISR (void) interrupt 6 using 1 {
byte to;

	SPIF = 0;								    // clear interrupt
	if( SPIIndx < SPIO_DATA_CNT ) {      // last byte or spurious? 
		to = SPI0_Buf.b[SPIIndx];         // set data				;
		SPI0DAT = to;
		SPIIndx++;
	} else {
		ADDPORT = 0xF;					// all chip selects high
		SPIO_Interrupt_Flag = CLR; // flag done
	}
	// clear any duplex errors, one way communication only
	WCOL = CLR;      // write collision
	MODF = CLR;      // nss pulled low
	RXOVRN = CLR;    // byte overrun
}


#endif

/*	///////////////////////////////////////////////////////////////////////////
Timer3_ISR
Description:
	General purpose timer
Returns:
	Graycode	in	Tsf
	Task overrun in LError
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_TIMER3_ISR
void Timer3_ISR( void )	interrupt INTERRUPT_TIMER3	{
byte ShiftMask; // 

	TMR3CN &= ~(0x80);	  // clear Interrupt	source TF3
	t4c++;					  // task counter
	ShiftMask =	1;	  
	while( (!(ShiftMask & t4c)) && ShiftMask ) // create the	code
		ShiftMask <<= 1;
	ShiftMask &= t4m;		  // apply task mask
	t4o =	Tsf &	ShiftMask; // setting bit not	cleared by foreground is overrun
	if( t4o )				  // save and report	error
		TaskOverRun = t4o;		
	Tsf |= ShiftMask;		  // set	task bit
}	
#endif

/*	///////////////////////////////////////////////////////////////////////////
//	MSDelay(	word t)
Description:driven by Timer4 if enabled, else count millisec timer
Arguments:
Returns:
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_MSDELAY
void MSDelay( word t) {

	if(TMR4CN &	0x04)	{ // timer 4 enabled
		SetTimerCounter( 1, 25 * t	);
		while( t4[1].status == COUNTING ) {
			WDTCN	= WDRLD;// reload	the watchdog
		}
		t4[1].status =	DISABLED;

	} else {
		while(t--) {
			USDelay(1000);
			WDTCN	= WDRLD;// reload	the watchdog
		}
	}
}

#endif

/*	///////////////////////////////////////////////////////////////////////////
  void Strup( char *	s )
	 Description:
		 uppercase a string
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_STRUP
void Strup(	char * s	) {
int i;
	i = 0;
	while(s[i])	{
		s[i] = toupper(s[i]);
		i++;
	}
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
  USDelay( word t)
	 Description:
		 5.4 microseconds	: the	time taken by light to travel	one mile	in	a vacuum
	 Arguments:
	 Returns:
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_USDELAY
void USDelay( word t) {
	while(t--)
		;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
  FlashWrite
	Description:
		Writes a	block	of	data to NV Shadow	Flash.
	  NOTE:
			Interrupts are	disabled	through much of this	write	to	prevent handlers from
			accidently writing to Flash (MOVX is redirected)
			This area is NOT erased	when code space is reprogrammed via	JTAG
			Data is stored	from address 0	to	len only
			This call erases the	entire sector (128 bytes) prior to write.
			The sector is only 128 bytes long
			This data cannot be locked
  Arguments:
			buffer b	of	len length
  Returns:
			zip
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_FLASH_WRITE
void Flash_Write(	byte * b, byte	len )	{
word addr =	0;

	if(len >	128) {
		len =	128;
		WriteErrorQue(NVRAM, BUFOVER);
	}

	Flash_PageErase (0, 1);
	while(len--) {
		Flash_ByteWrite(addr++,	*(b++), 1);
	}
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
  Flash_Read
	Description:
		Read a block of len bytes from non-volatile shadow	flash
		Data is read from	address 0 to len

			Interrupts are	disabled	through much of this	read to prevent handlers from
			accidently writing to FLASH (MOVX is redirected)

  Arguments:
			buffer b	of	len length
  Returns:
			buffer filled with data	from flash
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_FLASH_HREAD
void Flash_Read( byte *	b,	byte len	) {
word addr =	0;

	if(len >	128) {
		len =	128;
		WriteErrorQue(NVRAM, BUFOVER);
	}

	while(len--) {
		*(b++) =	Flash_ByteRead(addr++, 1);
	}
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
Flash_ByteWrite
	Description:
		Write	one byte	to	flash	at	address
			NOTE:
			Interrupts are	disabled	through much of this	write	to	prevent handlers from
			accidently writing to FLASH (MOVX is redirected)
			Does NOT	erase	prior	to	write
			SFLE set	= non-volatile	shadow, clear is CODE FLASH

	Arguments:
			buffer b	of	len length
	Returns:
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_FLASH_BYTEWRITE
void Flash_ByteWrite	(word	addr,	byte c, bit	SFLE)
{
	bit EA_SAVE	= EA;							//	preserve	EA
	char xdata * data	pwrite;				//	FLASH	write	pointer

	EA	= 0;										//	disable interrupts

	pwrite =	(char	xdata	*)	addr;

	FLSCL	|=	0x01;								//	enable FLASH writes/erases
	PSCTL	|=	0x01;								//	PSWE = 1

	if	(SFLE) {
		PSCTL	|=	0x04;							//	set SFLE
	}

	*pwrite = c;								//	write	the byte

	if	(SFLE) {
		PSCTL	&=	~0x04;						//	clear	SFLE
	}

	PSCTL	&=	~0x01;							//	PSWE = 0
	FLSCL	&=	~0x01;							//	disable FLASH writes/erases

	EA	= EA_SAVE;								//	restore interrupts
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
FLASH_ByteRead
	Description:
	Arguments:
		address of flash to read, shadow	flag
	Returns:
		byte at flash address
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_FLASH_BYTEREAD
unsigned	char Flash_ByteRead (word addr, bit	SFLE)
{
	bit EA_SAVE	= EA;							//	preserve	EA
	char code *	data pread;					//	FLASH	read pointer
	byte c;

	EA	= 0;										//	disable interrupts

	pread	= (char code *) addr;

	if	(SFLE) {
		PSCTL	|=	0x04;							//	set SFLE
	}

	c = *pread;									//	read the	byte

	if	(SFLE) {
		PSCTL	&=	~0x04;						//	clear	SFLE
	}

	EA	= EA_SAVE;								//	restore interrupts

	return c;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
FLASH_PageErase
	Description:
		This routine erases the	FLASH	512 byte	page containing the address
	Arguments:
		address of sector, shadow flag
	Returns:
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_FLASH_PAGEERASE
void Flash_PageErase	(word	addr,	bit SFLE)
{
	bit EA_SAVE	= EA;							//	preserve	EA
	char xdata * data	pwrite;				//	FLASH	write	pointer

	EA	= 0;										//	disable interrupts

	pwrite =	(char	xdata	*)	addr;

	FLSCL	|=	0x01;								//	enable FLASH writes/erases
	PSCTL	|=	0x03;								//	PSWE = 1; PSEE	= 1

	if	(SFLE) {
		PSCTL	|=	0x04;							//	set SFLE
	}

	*pwrite = 0;								//	initiate	page erase

	if	(SFLE) {
		PSCTL	&=	~0x04;						//	clear	SFLE
	}

	PSCTL	&=	~0x03;							//	PSWE = 0; PSEE	= 0
	FLSCL	&=	~0x01;							//	disable FLASH writes/erases

	EA	= EA_SAVE;								//	restore interrupts
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
MakeCheckSum
	Generic byte checksum for a block of len bytes	
	Arguments: generic pointer	to	start	of	bytes, sizeof compatible len		
	Returns:	byte checksum		
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_MAKECHECKSUM
byte MakeCheckSum( byte	*p, int len) {
int i;
byte c;
	c = 0;
	for(i	= 0; i <	len-1; i++)
		c += p[i];	
	return c;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
CheckSum
	Validates a	checksum	created by MakeCheckSum	where	checksum	is	in	the last	
	byte of the	block
	Arguments: generic pointer	to	start	of	bytes, sizeof compatible len		
	Returns:	bit 1	for match else	0		
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_CHECKSUM
bit CheckSum( byte *p, int	len) {
int i;
byte c;
	c = 0;
	for(i	= 0; i <	len-1; i++)
		c += p[i];	
	return (c == p[len-1]) ? 1	: 0;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
AbsDiff
Description:
	Utility to return	buffer size
Arguments:
	com circle buffer	indexes
Returns:
	absolute	difference
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_ABSDIFF
byte AbsDiff( byte in, byte out ) {

//	return(in-out);

	if( in != out ) {
		if( out > in )
			return(out-in);
		else
			return(IBSIZE-(in-out));
	}
	return 0;

}
#endif

//	tested, unused	for now
/*	///////////////////////////////////////////////////////////////////////////
BtoH
	Description: binary to hex
	Arguments:	takes	a value 0 -	15
	Returns:		returns a hex digit
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_BIN2HEX8
byte Bin2Hex8 (byte i) {
byte ret;
	if(i < 10) {				//	digit
		ret =	i + 0x30;
	} else {							//	letter
		ret =	(i	- 10)	+ 0x41;
	}
	if(isxdigit(ret))			//	check	results
		return ret;
	else {
		WriteErrorQue(LIBUTL, TRNSLAT);
		return 'F';			  // default
	}
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
BCD2Int
Description:Translate a	BCD string into an integer	value
Arguments:	byte pointer containing	len BCD hex	digits and # of digits
Returns:		integer value
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_BCD2INT
word BCD2Int(char	*s, byte	len){
word r =	0;
word shft=1;
byte i=len;

	while(i--){					//	all digits
		if(isxdigit(s[i])) {	//	check	if	hex valid
			r += (toint(s[i])	* shft);	//
			shft*=16;
		}	else {
			WriteErrorQue(P3LIBG, TRNSLAT);
		}
	}
	return(r);
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
BitStr
	Description: Debug util
	Arguments:	 byte
	Returns:		 string with binary representation of byte
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_BITSTR
char * BitStr(	byte b )	{
byte i;
	i = 0;
	while(i<8) {
		if((0x80>>i) &	b)
			bs[i]	= '1';
		else
			bs[i]	= '0';
		i++;
	}
	bs[8]=0;
	return bs;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
BitCnt8
	Description: Returns	# of bits set in a byte
	Arguments:	 byte
	Returns:		 count of bits	set 0-8
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_BITCNT8
byte BitCnt8(byte	b)	{
  b =	((b &	0xAA)	>>	1)	+ (b & 0x55);
  b =	((b &	0xCC)	>>	2)	+ (b & 0x33);
	b = ((b & 0xF0) >> 4) +	(b	& 0x0F);
	return b;
}
#endif

/*	Implemeted in 8051Lib_I.ASM to take	advantage of the asm	nibble SWAP
	Compiler	failed to use it and	there	is	no	intrinsic 
byte BitRev8(byte	b)	{
  b =	(((b & 0xAA) >> 1) |	((b &	0x55)	<<	1));
  b =	(((b & 0xCC) >> 2) |	((b &	0x33)	<<	2));
	b = (((b	& 0xF0) >> 4) | ((b & 0x0F) << 4));
	return b;
}
*/

//	Returns position of first rightmost	set bit in a byte	as
//	..8421
//	..3210
#ifdef SET_BITPOS8
byte BitPos8( byte b	) {
byte m;
	m = 0;
	while( b	) {
		if( b	& 1)
			return m;
		m++;
		b >>=	1;
	}
	return 0xFF;
	
}
#endif

/////////////////////////////////////////////////////////////////////////////
//	Error	Handling

/*	///////////////////////////////////////////////////////////////////////////
InitEQue
	Description:
	  Set	up	the Error queue
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_INITEQUE
void InitEQue(void) {
	memset(EQue,0,sizeof(EQue));
	eqi =	0;
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
WriteErrorQue
	Description
	  Write 1 error to the error queue:
	Arguments:
		Source of error (where it happenened) and	error	code,	defines in 8051lib.h
	Returns:
	  Writes	byte into the error queue and	and increments	the pointer
/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_WRITEERRORQUE
void WriteErrorQue(byte	Source, byte Error) {

	// don't allow consequtive to spam the que
	if((Source == EQue[eqi-2]) && (Error == EQue[eqi-1]))
		return;

	if(eqi >= sizeof(EQue))	{
		eqi =	0;
		EQue[eqi++]	= ERRQUE;	  // flag error que wrap
		EQue[eqi++]	= QUEWRAP;
	}
	EQue[eqi++]	= Source;
	EQue[eqi++]	= Error;
}
#endif

#ifdef SET_ERRDUMP
/*	///////////////////////////////////////////////////////////////////////////
ErrDump
	Description
	  If any	errors, output	to	maintenance	port
	Arguments:
	Returns:
		output to maintenance port
/////////////////////////////////////////////////////////////////////////// */
void ErrDump(void) {
byte a;

	if( eqi ) {
		printf("Errors: (%bd)\r", (eqi/2));
		eqi=0;
		a = t4m; // clear the task mask to prevent timer over run error
		t4m = 0;

		while(EQue[eqi]) {
			if(EQue[eqi] == DIAGBY){
				printf("%s %0bX\r", sc[EQue[eqi++]].desc,	EQue[eqi++]	);
			} else {
				ErrLookup( EQue[eqi++],	EQue[eqi++]	);	
			}
			WDTCN	= WDRLD;// reload	the watchdog
		}
		t4m = a;
	}
}
#endif

#ifdef SET_ERRTABLE
/*	///////////////////////////////////////////////////////////////////////////
ErrTable
	Description
	  This is a	debug	to	printf the entire	Error	source/code	table
	Arguments:
	  none
	Returns:
	  The	description	text of src	+ err	
/////////////////////////////////////////////////////////////////////////// */
void ErrTable(	void ) {
byte i, a;

	i = 1;
	a = t4m; // clear the task mask to prevent timer over run error
	t4m = 0;
	printf("Source\r%s",	Line);
	while( i	< STABSIZE ) {
		printf("%02bX:%-24s\r",	 i, sc[i].desc);
		i++;
		WDTCN	= WDRLD;// reload	the watchdog
	}
	i = 1;
	printf("\rError\r%s", Line);
	while( i	< ETABSIZE ) {
		printf("%02bX:%-24s\r",	 i, ec[i].desc);
		i++;
		WDTCN	= WDRLD;// reload	the watchdog
	}
	t4m = a;
}
#endif

#ifdef SET_ERRLOOKUP
/*	///////////////////////////////////////////////////////////////////////////
ErrLookup
	Description
	  Printf	the descriptions of a src + err pair
	Arguments:
	  Error source	and Error code	
	Returns:
	  The	description	text of src	+ err	
/////////////////////////////////////////////////////////////////////////// */
void ErrLookup( byte	src, byte err ) {

	if( src < STABSIZE )	{
		printf("%02bX:%-24s",  src, sc[src].desc);				
	} else {
		printf("%02bX%-24s",	src, "Bad Error Source Code");
	}
	printf("	: ");

	if( err < ETABSIZE )	{
		printf("%02bX:%-24s",  err, ec[err].desc);				
	} else {
		printf("%02bX %-24s", err,	"Bad Error Code");
	}
	printf("\r");
}
#endif

#ifdef SET_REALTIMECLOCK
/*	///////////////////////////////////////////////////////////////////////////
RealTimeClock
	Prints a	HH:MM:SS	clock	at	 Maintenance port.
  Arguments:
		runs off	the Timer scheduler
  Returns:
		hour:minute:second
/////////////////////////////////////////////////////////////////////////// */
void RealTimeClock( void )	{
	printf("%c[s",	27);
	A_Pos(2,2);	A_Color(BOLD,YELLOW);
	printf("%s : %02bd:%02bd:%02bd",	EXNAME, hrs, min,	sec);
	printf("%c[u",	27);

	//	rippler
	sec++;
	if( sec >59	) {
		sec =	0;
		min++;
		if( min >59	) {
			min =	0;
			hrs++;
			if( hrs >12	) {
				hrs =	1;						
			}
		}
	}
}
#endif

/*	///////////////////////////////////////////////////////////////////////////
GetCommand
	Takes	data from the Maintenance port, intended to be polled
		Simple terminal CR is process, backspace key	is	edit
		Sample command:
		  ^MENU<CR>
  Arguments:
	  none
  Returns:
		byte cmd	and parameters, space delimited in a string to be parsed	by	sscanf

/////////////////////////////////////////////////////////////////////////// */
#ifdef SET_GETCOMMAND
char * GetCommand( byte	c ) {
static char	i;

//	while(OutRp	!=	InRp)	{ // process data	coming in
//		cb[i]=Rb[OutRp++];
		cb[i]	= c;

		if( cb[i] == '^' ) {		//	reset	buffer on caret
			i = 0; cb[i] =	'^';
		}

		//	backspace
		if( cb[i] == '\b'	) {
			i-=2;
			if(i<0) i =	0;
		} else {

			//	command in on cr
			if( cb[i] == '\r'	) {
				if( cb[0] == '^' ) {		//	if	long enough	for a	command
					cb[i]	= 0;		//	null term the command replacing cr
					memset(cs,0,sizeof(cs));
					memset(cp,0,sizeof(cp));
					sscanf( cb,	"^%s %s", cs, cp );
					Strup(cs);
					ParseCommand(cs);
				}
				i = 0xFF;				//	reset	input	buffer on any cr
			}
		}
		if(i++ >	48)	i = 0; // don't allow input overflow
//	}

	//	utilize poll to do polled tasks
	if(MenuPoll)
		MenuTask(MenuPoll);

	return(cs);
}

/////////////////////////////////////////////////////////////////////////// 
//	Return index of command	from the	Maintenance	Port
int CheckCommand(	char * ib, MenuItemType	mnu[]	) {
int i;

	i = 0;
	while(mnu[i].trig[0]) {
		if( memcmp(mnu[i].trig,	ib, sizeof(mnu[i].trig)) == 0) {
			break;
		}
		i++;
	}
	return i;
}

MenuItemType code	mnu[]	= {
"MEN",	  "^Maint Menu",			0,
"ID",     "Code Identification",  	0,
"EQ",	  "Display Error Queue",  	0,
"ETAB",	  "Display Error Table",  	0,
"TEM",	  "CPU Temperature",		MNU_POLL,
"ZCB",	  "Zero Comm Bufs",		  	0,
"EIN",	  "Echo Rx Data to U1",	  	0,
"EOUT",	  "Echo Tx Data to U1",	  	0,
"SCL",	  "Scale",				  	0,
"TSK",	  "Task Mask bX",			0,
"REE",	  "Read Eeprom", 			0,
"IPR",	  "Init Pressure",	  		0,
"RPR",	  "Read Pressure",		  	0,
"ITP",	  "Init Temperature",		0,
"RTP",	  "Read Temperature",		0,
"OFS",	  "Offset %d",	  			0,
"SPN",	  "Span %d",	  			0,
"M24",	  "32 bit mult shift 24",	0,
"M16",	  "32 bit mult shift 16", 	0,

"",		  "",							  MNU_HIDE
};

/////////////////////////////////////////////////////////////////////////// 
//	Parse	and execute	commands	from the	Maintenance	Port
void ParseCommand(char * ib) {
word i;
	//	Process commands from the Maintenance Port
	if(ib[0]) {
		i = CheckCommand(	ib, mnu );

		if(mnu[i].desc[0]) {
			if(mnu[i].cntl	==	MNU_POLL) {
				MenuPoll	= i;
			} else {
				MenuPoll	= 0;
			};

//			A_Color(NORMAL,40);	//	background black
//			printf("%s", Head	);	//	clear	home
			MenuTask( i	);
		}
	}
}
/*	/////////////////////////////////////////////////////////////////////////// 
MenuTask
	Description: Performs a	Maintenance	command task, either polled or single
	Arguments: Task to do per mnu	Structure
	Returns:
/////////////////////////////////////////////////////////////////////////// */
void MenuTask(	byte task )	{
word i;
byte a; 
// byte b, c;														    
//			A_Pos(2,4);				//	pos down	4 from title
//			A_Color(BOLD,CYAN);	
//			printf("%s\r",	mnu[task].desc);
//			A_Color(NORMAL,CYAN);
//			printf("%s\r",	Line);
//			A_Color(NORMAL,WHITE);

	switch(task) {
		case 0:
			i = 0;
			a = t4m; // clear the task mask to prevent timer over run error
			t4m = 0;
			while(mnu[i].trig[0]) {
				if(!(mnu[i].cntl & MNU_HIDE))	{  // output all menu items
					printf("%-8s: %s\r",	mnu[i].trig, mnu[i].desc);
					WDTCN = WDRLD;// reload	the watchdog
				}
				i++;
			}
			t4m = a; // restore task mask
			break;

		case 1:
			a = t4m; // clear the task mask to prevent timer over run error
			t4m = 0;
			printf("%-16.0s%s\r","Name:", EXNAME);
			printf("%-16.0s%s\r","Version:", EXVERSION);
			printf("%-16.0s%s\r", "Part#:", EXPART);
			printf("%-16.0s%s\r", "File:", EXFILE);
			WDTCN	= WDRLD;// reload	the watchdog
			printf("%-16.0s%s\r","Lib Name:", LIBNAME);
			printf("%-16.0s%s\r","Lib Version:", LIBVERSION);
			printf("%-16.0s%s\r","Lib File:", LIBFILE);
			printf("%-16.0s%s\r", "Compile Time:",	LIBTIME);
			printf("%-16.0s",	"Model:");
			switch(__MODEL__)	{
				case 0:
					printf("Small\r");
					break;
				case 1:
					printf("Compact\r");
					break;
				case 2:
					printf("Large\r");
					break;
			}
			t4m = a; // restore task mask
			break;

//	Dump the	Error	Queue
		case 2:
			ErrDump();
			break;

//	Dump the	Error	Table
		case 3:
			ErrTable();	
			break;

//	CPU Temperature Dump
		case 4:
			//i = CpuTemperature();
			//printf( "Temperature	cpu:%3d (%3d) \r", i, FConvert(i));
			break;

//	Zero Buffers
		case 5:
			memset(Rb, 0, 0x400);
			printf("\rBuffers	Zero'd" );
			break;

//	Echo
		case 6:
			Echo1=!Echo1;
			printf("Echo1=%s ", (Echo1)?"On":"Off");
			break;

		case 7:
			Echo2=!Echo2;
			printf("Echo2=%s ", (Echo2)?"On":"Off");
			break;

//	Return scale
		case 8:
			printf("%d \r", scale);
			break;

//	Task mask
		case 9:
			sscanf( cp,	"%bX", &t4m);
			printf("%bX	",	t4m);
			break;

//	Read EEprom
		case 10:
			// The old logic is removed and replaced with a single call
            // to the new display function, which handles all the necessary
            // EEPROM reads and printing for the MEMSCAP sensor.
			printf("%s",Head);	
			Display_MEMSCAP_EEPROM_Info();
            break;

//	
		case 11:
			printf("Init Pressure Not Supported\r");
			break;
//	
		case 12:
			//printf( "Read: %lu %u %f\r", pr.l, tp.w, psi );
			printf("Not supported\r");
			break;
//
		case 13:
			printf("Init Temperature Not Supported\r");
			break;
//
		case 14:
			printf( "Read Temperature Not Supported\r");
			break;

//	offset
		case 15:
			sscanf( cp,	"%bd", &offset);
			printf("Offset: %bd %f \r",	offset, (offset * .0001));
			break;

//	span
		case 16:
			sscanf( cp,	"%bd", &span);
			printf("Span: %bd %f \r", span, (span * .0001));
			break;
				
//	32 bit mult with 24 bit shift right
		case 17:
			//sscanf( cp,	"%ld:%ld", &n1, &n2);
			//M32_24();
			//printf("\r=  %lu %ld  \r", Hr.l, n2);
			printf("Not supported\r");

			break;	

//	32 bit mult with 16 bit shift right
		case 18:
			//sscanf( cp,	"%ld:%ld", &n1, &n2);
			//M32_16();
			//printf("\rM32_16 = %ld\r", n2);
			printf("Not supported\r");
			break;	

	} // parse switch
}

#endif

/* ///////////////////////////////////////////////////////////////////////////
	SetRStatus
		TBD 	: stack errors 
	Arguments	: type and error, where type is index, error is character
					both per enumerations
	Returns		: RStatus string set per args 		
////////////////////////////////////////////////////////////////////////// */
//                 p      q      r      s
char RStatus[5] = {RS_OK, RS_OK, RS_OK, RS_OK, 0 };
void SetRStatus( byte type, char error ) {
 	RStatus[(type & 3)] = error;
}

/*
//	ANSI Position Worker
//
void A_Pos(	byte x, byte y	) {
	printf("%c[%bd;%bdH", 27, y, x);	
}


/*	///////////////////////////////////////////////////////////////////////////
	ANSI Escape	code support
 /////////////////////////////////////////////////////////////////////////// 
attributes							 foreground	colors	background colors
0 normal	display					 30 black				40	black
1 bold								 31 red					41	red
4 underline	(mono	only)			 32 green				42	green
5 blink on							 33 yellow				43	yellow
7 reverse video on				 34 blue					44	blue
8 nondisplayed	(invisible)		 35 magenta				45	magenta
										 36 cyan					46	cyan
										 37 white				47	white
/////////////////////////////////////////////////////////////////////////// 
void A_Color( byte a, byte	c ) {
	printf("%c[%bd;%bdm", 27, a, c);	
}
*/
/*	/////////////////////////////////////////////////////////////////////////// 
	Description:
	Arguments:
	Returns:
/////////////////////////////////////////////////////////////////////////// */

