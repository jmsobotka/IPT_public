/**
 * @file TP1200.h
 * @brief Header file for the TP1200 pressure sensor driver.
 *
 * This file contains the function prototypes, global variable declarations,
 * and constant definitions needed to interface with the TP1200 sensor.
 */

#ifndef TP1200_H
#define TP1200_H

#include "c8051f020_kdefs.h" // For sbit definition

//=============================================================================
// Constant Definitions
//=============================================================================

// Defines the size of the pressure look-up table (LUT_P) after
// decimation and range limiting have been applied.
#define LUT_P_SIZE 1178

// Constants derived from the SWD11587 EEPROM Layout document.
#define POINTS_P 30           // Number of pressure points in the LUT.
#define POINTS_T 140          // Full number of temperature points in the LUT.
#define POINTS_LIN_T 12       // Number of points in the temp linearization table.

// EEPROM Base Addresses
#define SENSOR_SN_BASE_ADDRESS 8
#define CAL_DATE_BASE_ADDRESS 20
#define ADC_MODE_BASE_ADDRESS 24
#define ADC_CONFIG_P_BASE_ADDRESS 32
#define ADC_CONFIG_T_BASE_ADDRESS 68
#define ADC_MAX_P_BASE_ADDRESS 44
#define ADC_MIN_P_BASE_ADDRESS 48
#define MAX_P_BASE_ADDRESS 52
#define MIN_P_BASE_ADDRESS 54
#define LUT_P_BASE_ADDRESS 788
#define LUT_T_ADC_BASE_ADDRESS 228
#define LUT_P_ADC_BASE_ADDRESS 108
#define LIN_DATA_T_BASE_ADDRESS 17592

//=============================================================================
// Hardware Pin Definitions
//=============================================================================

//=============================================================================
// Type Definitions
//=============================================================================

// Defines a structure to hold a point from the temperature linearization table,
// which contains a temperature in Celsius and its corresponding raw ADC value.
typedef struct {
    float temperature_C;
    unsigned int adc_value;
} Lin_Data_Point;

// New struct to hold product information read from EEPROM
typedef struct {
    long serial_number;
    unsigned int cal_year;
    unsigned char cal_month;
    unsigned char cal_day;
} ProductInfoType;

//=============================================================================
// Global Variable Declarations
//=============================================================================

extern unsigned int xdata LUT_P[LUT_P_SIZE];
extern Lin_Data_Point xdata Lin_data_T[POINTS_LIN_T];
extern unsigned int xdata LUT_T_ADC[POINTS_T];
extern unsigned int xdata LUT_P_ADC[POINTS_P];
extern unsigned int xdata ADC_config_P;
extern unsigned int xdata ADC_config_T;
extern unsigned int xdata ADC_mode;
extern int xdata Max_P;
extern int xdata Min_P;
extern unsigned int xdata g_raw_temp_adc;
extern ProductInfoType xdata g_ProductInfo; // New global for product info


//=============================================================================
// Function Prototypes
//=============================================================================

void Initialize_Sensor(void);
void Get_Raw_Sensor_Readings(unsigned int* raw_press_adc, unsigned int* raw_temp_adc);
float Calculate_Compensated_Temperature(unsigned int raw_temp_adc);
float Calculate_Compensated_Pressure(unsigned int raw_press_adc, unsigned int comp_temp_adc);
float Apply_System_Calibration(float compensated_press, int offset, int span);
float CompPressureUnadjusted(long p);
void Load_Pressure_LUT(void);
unsigned int Calculate_LUT_Start_Address(void);
void Load_Lin_Data_T(void);
void Load_LUT_T_ADC(void);
void Load_LUT_P_ADC(void);
void Load_ADC_Config(void);
void Load_Product_Info(void); // New function prototype
void Read_EEPROM_Bytes(unsigned char* buffer, unsigned int start_address, unsigned int count);
void Display_MEMSCAP_EEPROM_Info(void);
void Print_Float(float f, unsigned char precision);
byte ReadEEProm(void);
unsigned long Calculate_CRC32(unsigned char* d, unsigned int count);

#endif // TP1200_H
