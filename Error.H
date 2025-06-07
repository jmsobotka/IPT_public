

typedef struct {
	unsigned char ecode;
	char * desc;
} errtype;

// Error code source and error definitions
#define ENDOFLIST  0

// Error Source Definitions
enum STab {
NONE,
ERRQUE, 
DEVRST,
UART00,
UART01,
LIBUTL,
ADCCAL,
ADCRED,
TIMER0,
TIMER1,
TIMER2,
TIMER3,
TIMER4,
NVRAM, 
EXD001,
EXD002,
USER01,
DIAGBY,
EX7ISR,
SWITCH,
SPIONT,
HNYSNS,
VSICTRL,
EVACCTRL,
EVACVENT,
EPRSCTRL,
EPRSVENT,
CTRLSEQ,
SPDCTRL,
PUMPPRS,

// always last
STABSIZE,
};

// Error Definitions        
enum EDef {
_NOERROR,

// RSTSRC SFR 0xEF Reset Source
// R       R/W     R/W     R/W     R       R       R/W     R
// ------  CNVRSEF C0RSEF  SWRSEF  WDTRSF  MCDRSF  PORSF   PINRSF
// Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
_PINRSF,
_PORSF,
_MCDRSF,
_WDTRSF,
_SWRSEF,
_C0RSEF,
_CNVRSEF,

QUEWRAP,
TXOVFLW,
RXOVFLW,
BUFOVER,
TRNSLAT,
INVDVAL,
INVDMIN,
INVDMAX,
INVDHED,
INVDCHK,
OVRTEMP,
FANFLT,
ACSENBL,
INVSET,
TIMESLO,
KEYREAD,
TIMEOUT,
NOISE,
OVRRUN,
INVSTATE,
RATEMAX,

// always last
ETABSIZE,
};

#ifdef WINDOZE
extern const errtype sc[STABSIZE+1];
extern const errtype ec[ETABSIZE+1];
#else
extern code errtype sc[STABSIZE+1];
extern code errtype ec[ETABSIZE+1];
#endif
