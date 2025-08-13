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
// User-Configurable Temperature Range & Decimation
//=============================================================================

// Define the operational temperature range in Fahrenheit.
// The driver will only load LUT data for this range.
#define TEMP_LOW_F 32.0f
#define TEMP_HIGH_F 114.0f

// Define the decimation factor for the temperature axis of the LUT.
// A factor of 2 means every other temperature point will be loaded.
#define TEMP_DECIMATION_FACTOR 2

//=============================================================================
// Compile-Time LUT Size Calculation
//=============================================================================
// These macros calculate the required array sizes based on the settings above.
// Do not modify these directly.

// 1. Convert user-defined Fahrenheit range to Celsius
#define TEMP_LOW_C ((TEMP_LOW_F - 32.0f) * 5.0f / 9.0f)
#define TEMP_HIGH_C ((TEMP_HIGH_F - 32.0f) * 5.0f / 9.0f)

// 2. Define EEPROM's full temperature scale characteristics
#define EEPROM_TEMP_MIN_C -55.0f
#define EEPROM_TEMP_MAX_C 85.0f
#define EEPROM_TEMP_POINTS 140.0f
#define EEPROM_TEMP_SPAN (EEPROM_TEMP_MAX_C - EEPROM_TEMP_MIN_C)
#define EEPROM_POINTS_PER_C (EEPROM_TEMP_POINTS / EEPROM_TEMP_SPAN)

// 3. Calculate the start and end indices in the full EEPROM LUT
#define TEMP_RANGE_START_INDEX (unsigned int)((TEMP_LOW_C - EEPROM_TEMP_MIN_C) * EEPROM_POINTS_PER_C)
#define TEMP_RANGE_END_INDEX   (unsigned int)((TEMP_HIGH_C - EEPROM_TEMP_MIN_C) * EEPROM_POINTS_PER_C)

// 4. Calculate the number of points in the selected range and after decimation
#define TEMP_POINTS_IN_RANGE (TEMP_RANGE_END_INDEX - TEMP_RANGE_START_INDEX + 1)
#define POINTS_T_DECIMATED ((TEMP_POINTS_IN_RANGE + TEMP_DECIMATION_FACTOR - 1) / TEMP_DECIMATION_FACTOR)

// 5. Define final array sizes
#define POINTS_P 30 // Number of pressure points is constant
#define LUT_P_SIZE_DECIMATED (POINTS_T_DECIMATED * POINTS_P)

//=============================================================================
// Constant Definitions
//=============================================================================

// Constants derived from the SWD11587 EEPROM Layout document.
#define POINTS_T_FULL 140     // Full number of temperature points in the EEPROM LUT.
#define POINTS_LIN_T 12       // Number of points in the temp linearization table.

// EEPROM Base Addresses
#define SENSOR_SN_BASE_ADDRESS      8
#define CAL_DATE_BASE_ADDRESS      20
#define ADC_MODE_BASE_ADDRESS      24
#define ADC_GPOCON_BASE_ADDRESS    28
#define ADC_CONFIG_P_BASE_ADDRESS  32
#define ADC_OFFSET_P_BASE_ADDRESS  36
#define ADC_GAIN_P_BASE_ADDRESS    40
#define ADC_MAX_P_BASE_ADDRESS     44
#define ADC_MIN_P_BASE_ADDRESS     48
#define MAX_P_BASE_ADDRESS         52
#define MIN_P_BASE_ADDRESS         54
#define ADC_CONFIG_T_BASE_ADDRESS  68
#define ADC_OFFSET_T_BASE_ADDRESS  72
#define ADC_GAIN_T_BASE_ADDRESS    76
#define LUT_P_ADC_BASE_ADDRESS    108
#define LUT_T_ADC_BASE_ADDRESS    228
#define LUT_P_BASE_ADDRESS        788
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
    float adc_value;
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

extern float xdata LUT_P[LUT_P_SIZE_DECIMATED];
extern float xdata LUT_T_ADC[POINTS_T_DECIMATED];
extern float xdata LUT_P_ADC[POINTS_P];

extern Lin_Data_Point xdata Lin_data_T[POINTS_LIN_T];
extern unsigned int xdata ADC_config_P;
extern unsigned int xdata ADC_config_T;
extern unsigned int xdata ADC_mode;
extern int xdata Max_P;
extern int xdata Min_P;
extern unsigned long xdata g_raw_temp_adc;
extern ProductInfoType xdata g_ProductInfo;

extern unsigned long xdata ADC_offset_P;
extern unsigned long xdata ADC_gain_P;
extern unsigned long xdata ADC_offset_T;
extern unsigned long xdata ADC_gain_T;
extern unsigned char xdata ADC_gpocon;

//=============================================================================
// Function Prototypes
//=============================================================================

void Initialize_Sensor(void);
void Get_Raw_Sensor_Readings(unsigned long* raw_press_adc, unsigned long* raw_temp_adc);
float Calculate_Compensated_Temperature(unsigned long raw_temp_adc);
float Calculate_Compensated_Pressure(unsigned long raw_press_adc, unsigned long comp_temp_adc);
float Apply_System_Calibration(float compensated_press, int offset, int span);
float CompPressureUnadjusted(long p);
void Load_Pressure_LUT(void);
void Load_Lin_Data_T(void);
void Load_LUT_T_ADC(void);
void Load_LUT_P_ADC(void);
void Load_ADC_Config(void);
void Load_Product_Info(void);
void Read_EEPROM_Bytes(unsigned char* buffer, unsigned int start_address, unsigned int count);
void Read_EEPROM_Floats(float* buffer, unsigned int start_address, unsigned int count);
void Display_MEMSCAP_EEPROM_Info(void);
byte ReadEEProm(void);
unsigned long Calculate_CRC32(unsigned char* d, unsigned int count);
void Initialize_ADC(void);
void USDelay( word t);

#endif // TP1200_H
