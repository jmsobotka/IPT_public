/**
 * @file TP1200.c
 * @brief Driver for handling the MEMSCAP TP1200 pressure sensor.
 *
 * This file contains the full pipeline for interfacing with the TP1200:
 * 1. Initialization of sensor by loading configuration and LUTs from EEPROM.
 * 2. Acquiring raw ADC readings for temperature and pressure.
 * 3. Calculating final compensated temperature and pressure values.
 */

#include "c8051f020_kdefs.h" // For sbit definitions like CS_EE
#include "8051LIB.H"
#include "Main.h" // For access to SPI functions like SPIOWrite and SPIORead
#include "TP1200.h"

#include <math.h> // For fabs()
#include "stdio.H"

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

// Variables to hold calibration range data from EEPROM
int xdata Max_P;
int xdata Min_P;

unsigned int xdata g_raw_temp_adc; // Global for latest temperature reading

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
 * @brief Applies the final system-level offset and span calibration.
 *
 * This function takes the compensated pressure value from the sensor and
 * applies the user-adjustable two-point calibration to account for
 * system-level linear errors.
 *
 * @param compensated_press The pressure value after factory compensation.
 * @param offset The user-defined zero-point adjustment value.
 * @param span The user-defined gain adjustment value.
 * @return The final, system-calibrated pressure as a float.
 */
float Apply_System_Calibration(float compensated_press, int offset, int span)
{
    float final_pressure = compensated_press;
    float pressure_range = (float)(Max_P - Min_P);

    // 1. Apply Offset (Zero-Point Adjustment)
    // The scaling factor of 0.0001 is derived from the original implementation's
    // "1/4 foot resolution" comment and may need tuning.
    final_pressure += (float)offset * 0.0001f;

    // 2. Apply Span (Gain Adjustment)
    // This formula scales the reading relative to the full pressure range.
    // The scaling factor of 0.000001 is derived from the original implementation.
    final_pressure += (pressure_range - final_pressure) * ((float)span * 0.000001f);

    return final_pressure;
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
    float x1, x2, y1, y2, q11, q12, q21, q22, x, y, p1, p2, p3, p4;

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
    x1 = (float)LUT_P_ADC[x1_idx];
    x2 = (float)LUT_P_ADC[x2_idx];
    y1 = (float)LUT_T_ADC[y1_idx];
    y2 = (float)LUT_T_ADC[y2_idx];

    // Get the four corner pressure values from the main LUT_P table
    // The table is flattened, so index is calculated as [row * num_columns + column]
    q11 = (float)LUT_P[y1_idx * POINTS_P + x1_idx];
    q12 = (float)LUT_P[y2_idx * POINTS_P + x1_idx];
    q21 = (float)LUT_P[y1_idx * POINTS_P + x2_idx];
    q22 = (float)LUT_P[y2_idx * POINTS_P + x2_idx];

    // Perform bilinear interpolation
    x = (float)raw_press_adc;
    y = (float)comp_temp_adc;

    p1 = q11 * (x2 - x) * (y2 - y);
    p2 = q21 * (x - x1) * (y2 - y);
    p3 = q12 * (x2 - x) * (y - y1);
    p4 = q22 * (x - x1) * (y - y1);

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

    CS_PR = 0;

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

    CS_PR = 1;
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

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((MAX_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(MAX_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead(); msb = SPIORead();
    Max_P = ((unsigned int)msb << 8) | lsb;
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((MIN_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(MIN_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead(); msb = SPIORead();
    Min_P = ((unsigned int)msb << 8) | lsb;
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
    float target_adc_f;
    unsigned int target_adc;
    unsigned int closest_index;
    float min_diff;
    unsigned int address_offset;

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

    target_adc_f = (float)x1 + ((float)x2 - (float)x1) * (target_temp_C - y1) / (y2 - y1);
    target_adc = (unsigned int)target_adc_f;
    closest_index = 0;
    min_diff = -1.0f;

    for (i = 0; i < POINTS_T; i++)
    {
        float diff = fabs((float)LUT_T_ADC[i] - (float)target_adc);
        if (min_diff < 0 || diff < min_diff)
        {
            min_diff = diff;
            closest_index = i;
        }
    }

    address_offset = closest_index * POINTS_P * 2;

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

/**
 * @brief Helper function to read a block of bytes from a specific EEPROM address.
 */
void Read_EEPROM_Bytes(unsigned char* buffer, unsigned int start_address, unsigned int count)
{
    unsigned int i;
    CS_EE = 0;
    SPIOWrite(0x03); // Read command
    SPIOWrite((start_address >> 8) & 0xFF);
    SPIOWrite(start_address & 0xFF);
    for (i = 0; i < count; i++)
    {
        buffer[i] = SPIORead();
    }
    CS_EE = 1;
}

/**
 * @brief Reads and displays key info from the MEMSCAP EEPROM for the maintenance menu.
 *
 * This function replaces the old ReadEEProm logic in the maintenance menu. It reads
 * specific values like Serial Number, Calibration Date, and Min/Max ranges directly
 * from the EEPROM and prints them, then uses the already-loaded LUT data to
 * display compensated values.
 */
void Display_MEMSCAP_EEPROM_Info(void)
{
    unsigned char temp_buffer[20]; // Small buffer for EEPROM reads
    unsigned long serial_number;
    float temp_f;

    // --- Display Min/Max values (already loaded during initialization) ---
    // Note: PMax/PMin are now the compensated values for the ADC limits.
    // We need to read the ADC min/max from EEPROM to calculate this.
    unsigned char p_min_adc_bytes[4], p_max_adc_bytes[4];
    unsigned long p_min_adc, p_max_adc;

    // --- Read and Display Serial Number (Bytes 8-11) ---
    Read_EEPROM_Bytes(temp_buffer, 8, 4);
    serial_number = *((unsigned long*)temp_buffer);
    printf("Sensor S/N: %lu\r", serial_number);

    // --- Read and Display Calibration Date (Bytes 20-23) ---
    Read_EEPROM_Bytes(temp_buffer, 20, 4);
    printf("Cal Date: %u/%u/%u\r", (unsigned int)temp_buffer[1], (unsigned int)temp_buffer[0], *((unsigned int*)&temp_buffer[2]));

    Read_EEPROM_Bytes(p_min_adc_bytes, ADC_MIN_P_BASE_ADDRESS, 4);
    p_min_adc = *((unsigned long*)p_min_adc_bytes);

    Read_EEPROM_Bytes(p_max_adc_bytes, ADC_MAX_P_BASE_ADDRESS, 4);
    p_max_adc = *((unsigned long*)p_max_adc_bytes);

    printf("Pressure Max: %ld %f\r", p_max_adc, CompPressureUnAdjusted(p_max_adc));
    printf("Pressure Min: %ld %f\r", p_min_adc, CompPressureUnAdjusted(p_min_adc));

    // For temperature, we can use the loaded Lin_data_T table
    temp_f = (Lin_data_T[POINTS_LIN_T - 1].temperature_C * 9.0f / 5.0f) + 32.0f;
    printf("Temperature Max: %u %f\r", (unsigned int)Lin_data_T[POINTS_LIN_T - 1].adc_value, temp_f);

    temp_f = (Lin_data_T[0].temperature_C * 9.0f / 5.0f) + 32.0f;
    printf("Temperature Min: %u %f\r", (unsigned int)Lin_data_T[0].adc_value, temp_f);
}
