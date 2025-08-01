//-----------------------------------------------------------------------------
// C8051F020_defs.h
//-----------------------------------------------------------------------------
// Copyright 2007, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Register/bit definitions for the C8051F02x family.
// **Important Note**: The compiler_defs.h header file should be included 
// before including this header file.
//
// Target:         C8051F020, 'F021, 'F022, 'F023
// Tool chain:     Generic
// Command Line:   None
//
// Release 1.3 - 07 AUG 2007 (PKC)
//    -Removed #include <compiler_defs.h>. The C source file should include it.
// Release 1.2 - 09 JUL 2007 (PKC)
//    -Reformatted header file to enable portable SFR definitions

//-----------------------------------------------------------------------------
// Header File Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef C8051F020_DEFS_H
#define C8051F020_DEFS_H

//-----------------------------------------------------------------------------
// Byte Registers
//-----------------------------------------------------------------------------

sfr P0 = 0x80;                        // Port 0 Latch
sfr SP = 0x81;                        // Stack Pointer
sfr DPL = 0x82;                       // Data Pointer Low
sfr DPH = 0x83;                       // Data Pointer High
sfr P4 = 0x84;                        // Port 4 Latch
sfr P5 = 0x85;                        // Port 5 Latch
sfr P6 = 0x86;                        // Port 6 Latch
sfr PCON = 0x87;                      // Power Control
sfr TCON = 0x88;                      // Timer/Counter Control
sfr TMOD = 0x89;                      // Timer/Counter Mode
sfr TL0 = 0x8A;                       // Timer/Counter 0 Low
sfr TL1 = 0x8B;                       // Timer/Counter 1 Low
sfr TH0 = 0x8C;                       // Timer/Counter 0 High
sfr TH1 = 0x8D;                       // Timer/Counter 1 High
sfr CKCON = 0x8E;                     // Clock Control
sfr PSCTL = 0x8F;                     // Program Store R/W Control
sfr P1 = 0x90;                        // Port 1 Latch
sfr TMR3CN = 0x91;                    // Timer/Counter 3 Control
sfr TMR3RLL = 0x92;                   // Timer/Counter 3 Reload Low
sfr TMR3RLH = 0x93;                   // Timer/Counter 3 Reload High
sfr TMR3L = 0x94;                     // Timer/Counter 3 Low
sfr TMR3H = 0x95;                     // Timer/Counter 3 High
sfr P7 = 0x96;                        // Port 7 Latch
sfr SCON0 = 0x98;                     // Serial Port UART0 Control
sfr SBUF0 = 0x99;                     // Serial Port UART0 Data Buffer
sfr SPI0CFG = 0x9A;                   // SPI0 Configuration
sfr SPI0DAT = 0x9B;                   // SPI0 Data
sfr ADC1 = 0x9C;                      // ADC1 Data
sfr SPI0CKR = 0x9D;                   // SPI0 Clock Rate Control
sfr CPT0CN = 0x9E;                    // Comparator 0 Control
sfr CPT1CN = 0x9F;                    // Comparator 1 Control
sfr P2 = 0xA0;                        // Port 2 Latch
sfr EMI0TC = 0xA1;                    // EMIF Timing Control
sfr EMI0CF = 0xA3;                    // EMIF Configuration
sfr P0MDOUT = 0xA4;                   // Port 0 Output Mode Configuration
sfr P1MDOUT = 0xA5;                   // Port 1 Output Mode Configuration
sfr P2MDOUT = 0xA6;                   // Port 2 Output Mode Configuration
sfr P3MDOUT = 0xA7;                   // Port 3 Output Mode Configuration
sfr IE = 0xA8;                        // Interrupt Enable
sfr SADDR0 = 0xA9;                    // Serial Port UART0 Slave Address
sfr ADC1CN = 0xAA;                    // ADC1 Control
sfr ADC1CF = 0xAB;                    // ADC1 Analog Mux Configuration
sfr AMX1SL = 0xAC;                    // ADC1 Analog Mux Channel Select
sfr P3IF = 0xAD;                      // Port 3 External Interrupt Flags
sfr SADEN1 = 0xAE;                    // Serial Port UART1 Slave Address Mask
sfr EMI0CN = 0xAF;                    // EMIF Control
sfr P3 = 0xB0;                        // Port 3 Latch
sfr OSCXCN = 0xB1;                    // External Oscillator Control
sfr OSCICN = 0xB2;                    // Internal Oscillator Control
sfr P74OUT = 0xB5;                    // Ports 4 - 7 Output Mode
sfr FLSCL = 0xB6;                     // Flash Memory Timing Prescaler
sfr FLACL = 0xB7;                     // Flash Acess Limit
sfr IP = 0xB8;                        // Interrupt Priority
sfr SADEN0 = 0xB9;                    // Serial Port UART0 Slave Address Mask
sfr AMX0CF = 0xBA;                    // ADC0 Mux Configuration
sfr AMX0SL = 0xBB;                    // ADC0 Mux Channel Selection
sfr ADC0CF = 0xBC;                    // ADC0 Configuration
sfr P1MDIN = 0xBD;                    // Port 1 Input Mode
sfr ADC0L = 0xBE;                     // ADC0 Data Low
sfr ADC0H = 0xBF;                     // ADC0 Data High
sfr SMB0CN = 0xC0;                    // SMBus0 Control
sfr SMB0STA = 0xC1;                   // SMBus0 Status
sfr SMB0DAT = 0xC2;                   // SMBus0 Data
sfr SMB0ADR = 0xC3;                   // SMBus0 Slave Address
sfr ADC0GTL = 0xC4;                   // ADC0 Greater-Than Register Low
sfr ADC0GTH = 0xC5;                   // ADC0 Greater-Than Register High
sfr ADC0LTL = 0xC6;                   // ADC0 Less-Than Register Low
sfr ADC0LTH = 0xC7;                   // ADC0 Less-Than Register High
sfr T2CON = 0xC8;                     // Timer/Counter 2 Control
sfr T4CON = 0xC9;                     // Timer/Counter 4 Control
sfr RCAP2L = 0xCA;                    // Timer/Counter 2 Capture Low
sfr RCAP2H = 0xCB;                    // Timer/Counter 2 Capture High
sfr TL2 = 0xCC;                       // Timer/Counter 2 Low
sfr TH2 = 0xCD;                       // Timer/Counter 2 High
sfr SMB0CR = 0xCF;                    // SMBus0 Clock Rate
sfr PSW = 0xD0;                       // Program Status Word
sfr REF0CN = 0xD1;                    // Voltage Reference 0 Control
sfr DAC0L = 0xD2;                     // DAC0 Register Low
sfr DAC0H = 0xD3;                     // DAC0 Register High
sfr DAC0CN = 0xD4;                    // DAC0 Control
sfr DAC1L = 0xD5;                     // DAC1 Register Low
sfr DAC1H = 0xD6;                     // DAC1 Register High
sfr DAC1CN = 0xD7;                    // DAC1 Control
sfr PCA0CN = 0xD8;                    // PCA0 Control
sfr PCA0MD = 0xD9;                    // PCA0 Mode
sfr PCA0CPM0 = 0xDA;                  // PCA0 Module 0 Mode Register
sfr PCA0CPM1 = 0xDB;                  // PCA0 Module 1 Mode Register
sfr PCA0CPM2 = 0xDC;                  // PCA0 Module 2 Mode Register
sfr PCA0CPM3 = 0xDD;                  // PCA0 Module 3 Mode Register
sfr PCA0CPM4 = 0xDE;                  // PCA0 Module 4 Mode Register
sfr ACC = 0xE0;                       // Accumulator
sfr XBR0 = 0xE1;                      // Port I/O Crossbar Control 0
sfr XBR1 = 0xE2;                      // Port I/O Crossbar Control 1
sfr XBR2 = 0xE3;                      // Port I/O Crossbar Control 2
sfr RCAP4L = 0xE4;                    // Timer 4 Capture Register Low
sfr RCAP4H = 0xE5;                    // Timer 4 Capture Register High
sfr EIE1 = 0xE6;                      // External Interrupt Enable 1
sfr EIE2 = 0xE7;                      // External Interrupt Enable 2
sfr ADC0CN = 0xE8;                    // ADC0 Control
sfr PCA0L = 0xE9;                     // PCA0 Counter Low
sfr PCA0CPL0 = 0xEA;                  // PCA0 Capture 0 Low
sfr PCA0CPL1 = 0xEB;                  // PCA0 Capture 1 Low
sfr PCA0CPL2 = 0xEC;                  // PCA0 Capture 2 Low
sfr PCA0CPL3 = 0xED;                  // PCA0 Capture 3 Low
sfr PCA0CPL4 = 0xEE;                  // PCA0 Capture 4 Low
sfr RSTSRC = 0xEF;                    // Reset Source Configuration/Status
sfr B = 0xF0;                         // B Register
sfr SCON1 = 0xF1;                     // Serial Port UART1 Control
sfr SBUF1 = 0xF2;                     // Serail Port UART1 Data
sfr SADDR1 = 0xF3;                    // Serail Port UART1 Slave Address
sfr TL4 = 0xF4;                       // Timer/Counter 4 Low
sfr TH4 = 0xF5;                       // Timer/Counter 4 High
sfr EIP1 = 0xF6;                      // External Interrupt Priority 1
sfr EIP2 = 0xF7;                      // External Interrupt Priority 2
sfr SPI0CN = 0xF8;                    // SPI0 Control
sfr PCA0H = 0xF9;                     // PCA0 Counter High
sfr PCA0CPH0 = 0xFA;                  // PCA0 Capture 0 High
sfr PCA0CPH1 = 0xFB;                  // PCA0 Capture 1 High
sfr PCA0CPH2 = 0xFC;                  // PCA0 Capture 2 High
sfr PCA0CPH3 = 0xFD;                  // PCA0 Capture 3 High
sfr PCA0CPH4 = 0xFE;                  // PCA0 Capture 4 High
sfr WDTCN = 0xFF;                     // Watchdog Timer Control

//-----------------------------------------------------------------------------
// 16-bit Register Definitions might not be supported by all compilers
//-----------------------------------------------------------------------------

sfr16 DP = 0x82;                      // Data Pointer
sfr16 TMR3RL = 0x92;                  // Timer3 Reload Value
sfr16 TMR3 = 0x94;                    // Timer3 Counter
sfr16 ADC0 = 0xBE;                    // ADC0 Data
sfr16 ADC0GT = 0xC4;                  // ADC0 Greater Than Window
sfr16 ADC0LT = 0xC6;                  // ADC0 Less Than Window
sfr16 RCAP2 = 0xCA;                   // Timer2 Capture/Reload
sfr16 T2 = 0xCC;                      // Timer2 Counter
sfr16 TMR2RL = 0xCA;                  // Timer2 Capture/Reload
sfr16 TMR2 = 0xCC;                    // Timer2 Counter
sfr16 RCAP4 = 0xE4;                   // Timer4 Capture/Reload
sfr16 T4 = 0xF4;                      // Timer4 Counter
sfr16 TMR4RL = 0xE4;                  // Timer4 Capture/Reload
sfr16 TMR4 = 0xF4;                    // Timer4 Counter
sfr16 DAC0 = 0xD2;                    // DAC0 Data
sfr16 DAC1 = 0xD5;                    // DAC1 Data

//-----------------------------------------------------------------------------
// Address Definitions for bit-addressable SFRs
//-----------------------------------------------------------------------------

#define SFR_P0       0x80
#define SFR_TCON     0x88
#define SFR_P1       0x90
#define SFR_SCON0    0x98
#define SFR_P2       0xA0
#define SFR_IE       0xA8
#define SFR_P3       0xB0
#define SFR_IP       0xB8
#define SFR_SMB0CN   0xC0
#define SFR_T2CON    0xC8
#define SFR_PSW      0xD0
#define SFR_PCA0CN   0xD8
#define SFR_ACC      0xE0
#define SFR_ADC0CN   0xE8
#define SFR_B        0xF0
#define SFR_SPI0CN   0xF8

//-----------------------------------------------------------------------------
// Bit Definitions
//-----------------------------------------------------------------------------

// TCON 0x88
sbit TF1 = SFR_TCON^7;               // Timer 1 Overflow Flag
sbit TR1 = SFR_TCON^6;               // Timer 1 On/Off Control
sbit TF0 = SFR_TCON^5;               // Timer 0 Overflow Flag
sbit TR0 = SFR_TCON^4;               // Timer 0 On/Off Control
sbit IE1 = SFR_TCON^3;               // Ext. Interrupt 1 Edge Flag
sbit IT1 = SFR_TCON^2;               // Ext. Interrupt 1 Type
sbit IE0 = SFR_TCON^1;               // Ext. Interrupt 0 Edge Flag
sbit IT0 = SFR_TCON^0;               // Ext. Interrupt 0 Type

// SCON0 0x98
sbit SM00 = SFR_SCON0^7;             // Serial Mode Control Bit 0
sbit SM10 = SFR_SCON0^6;             // Serial Mode Control Bit 1
sbit SM20 = SFR_SCON0^5;             // Multiprocessor Communication Enable
sbit REN0 = SFR_SCON0^4;             // Receive Enable
sbit TB80 = SFR_SCON0^3;             // Transmit Bit 8
sbit RB80 = SFR_SCON0^2;             // Receive Bit 8
sbit TI0 = SFR_SCON0^1;              // Transmit Interrupt Flag
sbit RI0 = SFR_SCON0^0;              // Receive Interrupt Flag

// IE 0xA8
sbit EA = SFR_IE^7;                  // Global Interrupt Enable
sbit IEGF0 = SFR_IE^6;               // General Purpose Flag 0
sbit ET2 = SFR_IE^5;                 // Timer 2 Interrupt Enable
sbit ES0 = SFR_IE^4;                 // Uart0 Interrupt Enable
sbit ET1 = SFR_IE^3;                 // Timer 1 Interrupt Enable
sbit EX1 = SFR_IE^2;                 // External Interrupt 1 Enable
sbit ET0 = SFR_IE^1;                 // Timer 0 Interrupt Enable
sbit EX0 = SFR_IE^0;                 // External Interrupt 0 Enable

// IP 0xB8
                                       // Bit7 UNUSED
                                       // Bit6 UNUSED
sbit PT2 = SFR_IP^5;                 // Timer 2 Priority
sbit PS = SFR_IP^4;                  // Serial Port Priority
sbit PT1 = SFR_IP^3;                 // Timer 1 Priority
sbit PX1 = SFR_IP^2;                 // External Interrupt 1 Priority
sbit PT0 = SFR_IP^1;                 // Timer 0 Priority
sbit PX0 = SFR_IP^0;                 // External Interrupt 0 Priority

// SMB0CN 0xC0
sbit BUSY = SFR_SMB0CN^7;            // SMBus 0 Busy
sbit ENSMB = SFR_SMB0CN^6;           // SMBus 0 Enable
sbit STA = SFR_SMB0CN^5;             // SMBus 0 Start Flag
sbit STO = SFR_SMB0CN^4;             // SMBus 0 Stop Flag
sbit SI = SFR_SMB0CN^3;              // SMBus 0 Interrupt Pending Flag
sbit AA = SFR_SMB0CN^2;              // SMBus 0 Assert/Acknowledge Flag
sbit SMBFTE = SFR_SMB0CN^1;          // SMBus 0 Free Timer Enable
sbit SMBTOE = SFR_SMB0CN^0;          // SMBus 0 Timeout Enable

// T2CON 0xC8
sbit TF2 = SFR_T2CON^7;              // Timer 2 Overflow Flag
sbit EXF2 = SFR_T2CON^6;             // External Flag
sbit RCLK0 = SFR_T2CON^5;            // Uart0 Rx Clock Source
sbit TCLK0 = SFR_T2CON^4;            // Uart0 Tx Clock Source
sbit EXEN2 = SFR_T2CON^3;            // Timer 2 External Enable Flag
sbit TR2 = SFR_T2CON^2;              // Timer 2 On/Off Control
sbit CT2 = SFR_T2CON^1;              // Timer Or Counter Select
sbit CPRL2 = SFR_T2CON^0;            // Capture Or Reload Select

//  PSW 0xD0
sbit CY = SFR_PSW^7;                 // Carry Flag
sbit AC = SFR_PSW^6;                 // Auxiliary Carry Flag
sbit F0 = SFR_PSW^5;                 // User Flag 0
sbit RS1 = SFR_PSW^4;                // Register Bank Select 1
sbit RS0 = SFR_PSW^3;                // Register Bank Select 0
sbit OV = SFR_PSW^2;                 // Overflow Flag
sbit F1 = SFR_PSW^1;                 // User Flag 1
sbit P = SFR_PSW^0;                  // Accumulator Parity Flag

// PCA0CN 0xD8
sbit CF = SFR_PCA0CN^7;              // PCA 0 Counter Overflow Flag
sbit CR = SFR_PCA0CN^6;              // PCA 0 Counter Run Control Bit
                                       // Bit5 UNUSED
sbit CCF4 = SFR_PCA0CN^4;            // PCA 0 Module 4 Interrupt Flag
sbit CCF3 = SFR_PCA0CN^3;            // PCA 0 Module 3 Interrupt Flag
sbit CCF2 = SFR_PCA0CN^2;            // PCA 0 Module 2 Interrupt Flag
sbit CCF1 = SFR_PCA0CN^1;            // PCA 0 Module 1 Interrupt Flag
sbit CCF0 = SFR_PCA0CN^0;            // PCA 0 Module 0 Interrupt Flag

// ADC0CN 0xE8
sbit AD0EN = SFR_ADC0CN^7;           // ADC 0 Enable
sbit AD0TM = SFR_ADC0CN^6;           // ADC 0 Track Mode
sbit AD0INT = SFR_ADC0CN^5;          // ADC 0 Converision Complete Interrupt Flag
sbit AD0BUSY = SFR_ADC0CN^4;         // ADC 0 Busy Flag
sbit AD0CM1 = SFR_ADC0CN^3;          // ADC 0 Start Of Conversion Mode Bit 1
sbit AD0CM0 = SFR_ADC0CN^2;          // ADC 0 Start Of Conversion Mode Bit 0
sbit AD0WINT = SFR_ADC0CN^1;         // ADC 0 Window Compare Interrupt Flag
sbit AD0LJST = SFR_ADC0CN^0;         // ADC 0 Right Justify Data Bit

// SPI0CN 0xF8
sbit SPIF = SFR_SPI0CN^7;            // SPI 0 Interrupt Flag
sbit WCOL = SFR_SPI0CN^6;            // SPI 0 Write Collision Flag
sbit MODF = SFR_SPI0CN^5;            // SPI 0 Mode Fault Flag
sbit RXOVRN = SFR_SPI0CN^4;          // SPI 0 Rx Overrun Flag
sbit TXBSY = SFR_SPI0CN^3;           // SPI 0 Tx Busy Flag
sbit SLVSEL = SFR_SPI0CN^2;          // SPI 0 Slave Select
sbit MSTEN = SFR_SPI0CN^1;           // SPI 0 Master Enable

sbit CS_EE=P1^2; // Chip Select EEPROM
sbit CS_PR=P1^0; // Chip Select Pressure
sbit CS_5V=P2^7; // 5 Volt Enable
sbit LED=P2^4;   // LED

sbit SPIEN = SFR_SPI0CN^0;           // SPI 0 SPI Enable

//-----------------------------------------------------------------------------
// Interrupt Priorities
//-----------------------------------------------------------------------------

#define INTERRUPT_INT0           0     // External Interrupt 0
#define INTERRUPT_TIMER0         1     // Timer0 Overflow
#define INTERRUPT_INT1           2     // External Interrupt 1
#define INTERRUPT_TIMER1         3     // Timer1 Overflow
#define INTERRUPT_UART0          4     // Serial Port UART0
#define INTERRUPT_TIMER2         5     // Timer2 Overflow
#define INTERRUPT_SPI0           6     // SPI0 Interface
#define INTERRUPT_SMBUS0         7     // SMBus0 Interface
#define INTERRUPT_ADC0_WINDOW    8     // ADC0 Window Comparison
#define INTERRUPT_PCA0           9     // PCA0 Peripheral
#define INTERRUPT_COMPARATOR0F   10    // Comparator0 Falling Edge
#define INTERRUPT_COMPARATOR0R   11    // Comparator0 Rising Edge
#define INTERRUPT_COMPARATOR1F   12    // Comparator1 Falling Edge
#define INTERRUPT_COMPARATOR1R   13    // Comparator1 Rising Edge
#define INTERRUPT_TIMER3         14    // Timer3 Overflow
#define INTERRUPT_ADC0_EOC       15    // ADC0 End Of Conversion
#define INTERRUPT_TIMER4         16    // Timer4 Overflow
#define INTERRUPT_ADC1_EOC       17    // ADC1 End Of Conversion
#define INTERRUPT_INT6           18    // External Interrupt 6
#define INTERRUPT_INT7           19    // External Interrupt 7
#define INTERRUPT_UART1          20    // Serial Port UART1
#define INTERRUPT_XTAL_READY     21    // External Crystal Oscillator Ready

//-----------------------------------------------------------------------------
// Header File PreProcessor Directive
//-----------------------------------------------------------------------------

#endif                                 // #define C8051F020_DEFS_H

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
