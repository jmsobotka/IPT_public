// Error Code String Lookups 
// This is for reference only
// Not Required if code space is needed   
// The defines are in the header for global access
#include "Error.H"
// Error Source string table
#ifdef WINDOZE
const errtype sc[STABSIZE+1] = {
#else
code errtype sc[STABSIZE+1] = {
#endif

_NOERROR,    "No Error",
ERRQUE,     "Library Error Queue",
DEVRST,     "8051 Reset Check",
UART00,     "Serial 8051 UART0",
UART01,     "Serial 8051 UART1",
LIBUTL,     "Library Utility Routines",
ADCCAL,     "ADC Calibration",
ADCRED,     "ADC Read",
TIMER0,     "Timer 0",
TIMER1,     "Timer 1",
TIMER2,     "Timer 2",
TIMER3,     "Timer 3",
TIMER4,     "Timer 4",
NVRAM,      "Non Volatile Ram",
EXD001,     "External Device 01",
EXD002,     "External Device 02",
USER01,     "User Error",
DIAGBY,     "Diag:",
EX7ISR,     "Key Interrupt",
SWITCH,     "Switch Default",
SPIONT,     "SPIO Interrupt",
HNYSNS,     "Honeywell Sensors",
VSICTRL,    "VSI Control",
EVACCTRL,   "Vac Control",
EVACVENT,   "Vac Vent",
EPRSCTRL,   "Prs Control",
EPRSVENT,   "Prs Vent",
CTRLSEQ,    "Control Sequencer",
SPDCTRL,    "Knots Control",
PUMPPRS,    "Pump Prs/Vac",


ENDOFLIST, ENDOFLIST
};

// Error String Table
#ifdef WINDOZE
const errtype ec[ETABSIZE+1] = {
#else
code errtype ec[ETABSIZE+1] = {
#endif

_NOERROR,   "No Error",

_PINRSF,   "HW pin (jtag)",  // keep in order, bit flags
_PORSF,    "Power-on reset",
_MCDRSF,   "Missing clock detector",
_WDTRSF,   "Watchdog timer",
_SWRSEF,   "Software reset",
_C0RSEF,   "Comparator0 reset",
_CNVRSEF,  "Convert start reset source",

QUEWRAP,  "In pointer equals out pointer",
TXOVFLW,  "Transmit over flow",
RXOVFLW,  "Receiver over flow",
BUFOVER,  "Buffer over flow",
TRNSLAT,  "Invalid conversion attempted",
INVDVAL,  "Invalid value",
INVDMIN,  "Value lower than minimum ",
INVDMAX,  "Value higher than maximum",
INVDHED,  "Invalid header",
INVDCHK,  "Invalid checksum",
OVRTEMP,  "Temperature sensor over threshold",
FANFLT,   "Fan under threshold speed",
ACSENBL,  "Attempt to access while not enabled",
INVSET,   "Invalid Setting",
TIMESLO,  "Task was not completed within its timeslot",
KEYREAD,  "Transmit Time Out",
TIMEOUT,  "Time out",
NOISE,    "Excessive spurious data",
OVRRUN,   "Gray code timer over run",
INVSTATE, "Invalid State",
RATEMAX,  "Rate too high",

ENDOFLIST, ENDOFLIST,
};
