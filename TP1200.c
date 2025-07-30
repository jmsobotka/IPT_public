/**
 * @file TP1200.c
 * @brief Driver for handling the MEMSCAP TP1200 pressure sensor.
 *
 * This file contains the full pipeline for interfacing with the TP1200:
 * 1. Initialization of sensor by loading configuration and LUTs from EEPROM.
 * 2. Acquiring raw ADC readings for temperature and pressure.
 * 3. Calculating final compensated temperature and pressure values.
 */

#include <math.h> // For fabs()
#include "TP1200.h"
#include "Main.h" // For access to SPI functions like SPIOWrite and SPIORead
#include "c8051f020_kdefs.h" // For sbit definitions like CS_EE

//=============================================================================
// Type Definitions
//=============================================================================

// Union to facilitate the conversion of four bytes from the EEPROM
// into a 32-bit float value, respecting the little-endian format.
typedef union {
    float f;
    unsigned char c[4];
} FloatByteUnion;


//=============================================================================
// Global Variables
//=============================================================================

// This global array will store the pressure look-up table.
unsigned int xdata LUT_P[LUT_P_SIZE];

// Global array for the temperature linearization table (ADC vs. Temp C).
Lin_Data_Point xdata Lin_data_T[POINTS_LIN_T];

// Global array for the temperature ADC values look-up table.
unsigned int xdata LUT_T_ADC[POINTS_T];

// Global array for the pressure ADC values look-up table.
unsigned int xdata LUT_P_ADC[POINTS_P];

// Global variables to hold ADC configuration settings loaded from EEPROM.
unsigned int xdata ADC_config_P;
unsigned int xdata ADC_config_T;
unsigned int xdata ADC_mode;


//=============================================================================
// Function Definitions
//=============================================================================

/**
 * @brief Top-level function to initialize the sensor by loading all LUTs.
 */
void Initialize_Sensor(void)
{
    // Load all necessary configuration and look-up tables from EEPROM.
    Load_ADC_Config();
    Load_Lin_Data_T();
    Load_LUT_T_ADC();
    Load_LUT_P_ADC();
    Load_Pressure_LUT();
}

/**
 * @brief Calculates the final compensated temperature.
 *
 * This function performs a linear interpolation on the Lin_data_T table
 * to convert a raw temperature ADC reading into a compensated temperature
 * value in degrees Celsius.
 *
 * @param raw_temp_adc The raw ADC value from the temperature sensor.
 * @return The compensated temperature as a float.
 */
float Calculate_Compensated_Temperature(unsigned int raw_temp_adc)
{
    unsigned int i;
    float y1, y2; // Bracketing temperatures from Lin_data_T
    unsigned int x1, x2; // Corresponding ADC values from Lin_data_T

    // Find the two points in the linearization table that bracket the raw ADC value.
    for (i = 0; i < POINTS_LIN_T - 1; i++)
    {
        if (raw_temp_adc >= Lin_data_T[i].adc_value && raw_temp_adc <= Lin_data_T[i+1].adc_value)
        {
            x1 = Lin_data_T[i].adc_value;
            y1 = Lin_data_T[i].temperature_C;
            x2 = Lin_data_T[i+1].adc_value;
            y2 = Lin_data_T[i+1].temperature_C;
            
            // Perform linear interpolation
            return y1 + (y2 - y1) * ((float)raw_temp_adc - (float)x1) / ((float)x2 - (float)x1);
        }
    }
    // If the value is out of range, return the temperature from the closest endpoint.
    return (raw_temp_adc < Lin_data_T[0].adc_value) ? Lin_data_T[0].temperature_C : Lin_data_T[POINTS_LIN_T - 1].temperature_C;
}


/**
 * @brief Calculates the final compensated pressure.
 *
 * This function performs a bilinear interpolation using the loaded look-up
 * tables to convert raw pressure and temperature ADC values into a final,
 * compensated pressure reading.
 *
 * @param raw_press_adc The raw ADC value from the pressure sensor.
 * @param comp_temp_adc The compensated temperature ADC value (used for indexing).
 * @return The compensated pressure as a float.
 */
float Calculate_Compensated_Pressure(unsigned int raw_press_adc, unsigned int comp_temp_adc)
{
    unsigned int i;
    unsigned int x1_idx = 0, x2_idx = 0; // Pressure indices
    unsigned int y1_idx = 0, y2_idx = 0; // Temperature indices

    // Find bracketing indices for pressure ADC value in LUT_P_ADC
    for (i = 0; i < POINTS_P - 1; i++) {
        if (raw_press_adc >= LUT_P_ADC[i] && raw_press_adc <= LUT_P_ADC[i+1]) {
            x1_idx = i;
            x2_idx = i + 1;
            break;
        }
    }

    // Find bracketing indices for temperature ADC value in LUT_T_ADC
    for (i = 0; i < POINTS_T - 1; i++) {
        if (comp_temp_adc >= LUT_T_ADC[i] && comp_temp_adc <= LUT_T_ADC[i+1]) {
            y1_idx = i;
            y2_idx = i + 1;
            break;
        }
    }

    // Get the four corner ADC values for interpolation
    float x1 = (float)LUT_P_ADC[x1_idx];
    float x2 = (float)LUT_P_ADC[x2_idx];
    float y1 = (float)LUT_T_ADC[y1_idx];
    float y2 = (float)LUT_T_ADC[y2_idx];

    // Get the four corner pressure values from the main LUT_P table
    // The table is flattened, so index is calculated as [row * num_columns + column]
    float q11 = (float)LUT_P[y1_idx * POINTS_P + x1_idx];
    float q12 = (float)LUT_P[y2_idx * POINTS_P + x1_idx];
    float q21 = (float)LUT_P[y1_idx * POINTS_P + x2_idx];
    float q22 = (float)LUT_P[y2_idx * POINTS_P + x2_idx];

    // Perform bilinear interpolation
    float x = (float)raw_press_adc;
    float y = (float)comp_temp_adc;

    float p1 = q11 * (x2 - x) * (y2 - y);
    float p2 = q21 * (x - x1) * (y2 - y);
    float p3 = q12 * (x2 - x) * (y - y1);
    float p4 = q22 * (x - x1) * (y - y1);

    return (p1 + p2 + p3 + p4) / ((x2 - x1) * (y2 - y1));
}


/**
 * @brief Gets raw ADC readings for temperature and pressure from the TP1200.
 */
void Get_Raw_Sensor_Readings(unsigned int* raw_press_adc, unsigned int* raw_temp_adc)
{
    unsigned char msb, mid, lsb;
    unsigned int single_conversion_mode;

    single_conversion_mode = (ADC_mode & 0x1FFF) | 0x2000;

    CS_ADC = 0;

    // --- Get Pressure Reading ---
    SPIOWrite(0x10);
    SPIOWrite((ADC_config_P >> 8) & 0xFF);
    SPIOWrite(ADC_config_P & 0xFF);
    SPIOWrite(0x08);
    SPIOWrite((single_conversion_mode >> 8) & 0xFF);
    SPIOWrite(single_conversion_mode & 0xFF);
    while(MISO == 1);
    SPIOWrite(0x58);
    msb = SPIORead();
    mid = SPIORead();
    lsb = SPIORead();
    *raw_press_adc = ((unsigned int)msb << 8) | mid;

    // --- Get Temperature Reading ---
    SPIOWrite(0x10);
    SPIOWrite((ADC_config_T >> 8) & 0xFF);
    SPIOWrite(ADC_config_T & 0xFF);
    SPIOWrite(0x08);
    SPIOWrite((single_conversion_mode >> 8) & 0xFF);
    SPIOWrite(single_conversion_mode & 0xFF);
    while(MISO == 1);
    SPIOWrite(0x58);
    msb = SPIORead();
    mid = SPIORead();
    lsb = SPIORead();
    *raw_temp_adc = ((unsigned int)msb << 8) | mid;

    CS_ADC = 1;
}

/**
 * @brief Loads ADC configuration settings from the EEPROM.
 */
void Load_ADC_Config(void)
{
    unsigned char lsb, msb;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((ADC_MODE_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(ADC_MODE_BASE_ADDRESS & 0xFF);
    lsb = SPIORead();
    msb = SPIORead();
    ADC_mode = ((unsigned int)msb << 8) | lsb;
    SPIORead(); SPIORead();
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((ADC_CONFIG_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(ADC_CONFIG_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead();
    msb = SPIORead();
    ADC_config_P = ((unsigned int)msb << 8) | lsb;
    SPIORead(); SPIORead();
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((ADC_CONFIG_T_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(ADC_CONFIG_T_BASE_ADDRESS & 0xFF);
    lsb = SPIORead();
    msb = SPIORead();
    ADC_config_T = ((unsigned int)msb << 8) | lsb;
    SPIORead(); SPIORead();
    CS_EE = 1;
}

/**
 * @brief Loads the temperature linearization table (Lin_data_T) from EEPROM.
 */
void Load_Lin_Data_T(void)
{
    unsigned int i;
    unsigned char lsb, msb;
    FloatByteUnion temp_converter;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((LIN_DATA_T_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(LIN_DATA_T_BASE_ADDRESS & 0xFF);
    for (i = 0; i < POINTS_LIN_T; i++)
    {
        temp_converter.c[0] = SPIORead();
        temp_converter.c[1] = SPIORead();
        temp_converter.c[2] = SPIORead();
        temp_converter.c[3] = SPIORead();
        Lin_data_T[i].temperature_C = temp_converter.f;
        lsb = SPIORead();
        msb = SPIORead();
        Lin_data_T[i].adc_value = ((unsigned int)msb << 8) | lsb;
    }
    CS_EE = 1;
}

/**
 * @brief Loads the temperature ADC look-up table (LUT_T_ADC) from EEPROM.
 */
void Load_LUT_T_ADC(void)
{
    unsigned int i;
    unsigned char lsb, msb;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((LUT_T_ADC_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(LUT_T_ADC_BASE_ADDRESS & 0xFF);
    for (i = 0; i < POINTS_T; i++)
    {
        lsb = SPIORead();
        msb = SPIORead();
        LUT_T_ADC[i] = ((unsigned int)msb << 8) | lsb;
    }
    CS_EE = 1;
}

/**
 * @brief Loads the pressure ADC values look-up table (LUT_P_ADC) from EEPROM.
 */
void Load_LUT_P_ADC(void)
{
    unsigned int i;
    unsigned char lsb, msb;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((LUT_P_ADC_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(LUT_P_ADC_BASE_ADDRESS & 0xFF);
    for (i = 0; i < POINTS_P; i++)
    {
        lsb = SPIORead();
        msb = SPIORead();
        LUT_P_ADC[i] = ((unsigned int)msb << 8) | lsb;
    }
    CS_EE = 1;
}


/**
 * @brief Calculates the starting memory address for the LUT_P table read.
 */
unsigned int Calculate_LUT_Start_Address(void)
{
    const float target_temp_F = 36.0f;
    const float target_temp_C = (target_temp_F - 32.0f) * 5.0f / 9.0f;
    unsigned int i;
    float y1, y2;
    unsigned int x1, x2;
    for (i = 0; i < POINTS_LIN_T - 1; i++)
    {
        if (target_temp_C >= Lin_data_T[i].temperature_C && target_temp_C <= Lin_data_T[i+1].temperature_C)
        {
            y1 = Lin_data_T[i].temperature_C;
            x1 = Lin_data_T[i].adc_value;
            y2 = Lin_data_T[i+1].temperature_C;
            x2 = Lin_data_T[i+1].adc_value;
            break;
        }
    }
    float target_adc_f = (float)x1 + ((float)x2 - (float)x1) * (target_temp_C - y1) / (y2 - y1);
    unsigned int target_adc = (unsigned int)target_adc_f;
    unsigned int closest_index = 0;
    float min_diff = -1.0f;
    for (i = 0; i < POINTS_T; i++)
    {
        float diff = fabs((float)LUT_T_ADC[i] - (float)target_adc);
        if (min_diff < 0 || diff < min_diff)
        {
            min_diff = diff;
            closest_index = i;
        }
    }
    unsigned int address_offset = closest_index * POINTS_P * 2;
    return LUT_P_BASE_ADDRESS + address_offset;
}

/**
 * @brief Loads the pressure look-up table (LUT_P) from the TP1200 EEPROM.
 */
void Load_Pressure_LUT(void)
{
    unsigned int start_address = Calculate_LUT_Start_Address();
    unsigned int i;
    unsigned char lsb, msb;
    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((start_address >> 8) & 0xFF);
    SPIOWrite(start_address & 0xFF);
    for (i = 0; i < LUT_P_SIZE; i++)
    {
        lsb = SPIORead();
        msb = SPIORead();
        LUT_P[i] = ((unsigned int)msb << 8) | lsb;
    }
    CS_EE = 1;
}
