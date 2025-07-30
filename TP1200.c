/**
 * @file TP1200.c
 * @brief Driver for handling the MEMSCAP TP1200 pressure sensor.
 *
 * This file contains the initialization sequence for the TP1200 sensor,
 * which involves loading all necessary look-up tables from the EEPROM.
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
// It is placed in XDATA memory to accommodate its size.
unsigned int xdata LUT_P[LUT_P_SIZE];

// Global array for the temperature linearization table (ADC vs. Temp C).
// This must be loaded from the EEPROM before calculations can be performed.
Lin_Data_Point xdata Lin_data_T[POINTS_LIN_T];

// Global array for the temperature ADC values look-up table.
// This must be loaded from the EEPROM before calculations can be performed.
unsigned int xdata LUT_T_ADC[POINTS_T];


//=============================================================================
// Function Definitions
//=============================================================================

/**
 * @brief Top-level function to initialize the sensor by loading all LUTs.
 *
 * This function orchestrates the loading of all necessary look-up tables
 * from the EEPROM into the microcontroller's memory. It calls the loading
 * functions in the correct dependency order.
 */
void Initialize_Sensor(void)
{
    // 1. Load the temperature linearization data first, as it's needed to
    //    calculate the start address for the pressure LUT.
    Load_Lin_Data_T();

    // 2. Load the full temperature ADC table, also needed for the calculation.
    Load_LUT_T_ADC();

    // 3. Finally, load the decimated and range-limited pressure LUT.
    //    This function internally calls Calculate_LUT_Start_Address().
    Load_Pressure_LUT();
}


/**
 * @brief Loads the temperature linearization table (Lin_data_T) from EEPROM.
 *
 * This function reads the 12 data points for the temperature linearization
 * table. Each point consists of a temperature (float, 4 bytes) and a
 * corresponding ADC value (unsigned int, 2 bytes).
 */
void Load_Lin_Data_T(void)
{
    unsigned int i;
    unsigned char lsb, msb;
    FloatByteUnion temp_converter;

    CS_EE = 0; // Assert EEPROM chip select

    // Send READ command
    SPIOWrite(0x03);

    // Send the 16-bit memory address for Lin_data_T
    SPIOWrite((LIN_DATA_T_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(LIN_DATA_T_BASE_ADDRESS & 0xFF);

    // Read the 12 data points
    for (i = 0; i < POINTS_LIN_T; i++)
    {
        // Read 4 bytes for the temperature value (float)
        temp_converter.c[0] = SPIORead();
        temp_converter.c[1] = SPIORead();
        temp_converter.c[2] = SPIORead();
        temp_converter.c[3] = SPIORead();
        Lin_data_T[i].temperature_C = temp_converter.f;

        // Read 2 bytes for the ADC value (unsigned int)
        lsb = SPIORead();
        msb = SPIORead();
        Lin_data_T[i].adc_value = ((unsigned int)msb << 8) | lsb;
    }

    CS_EE = 1; // De-assert EEPROM chip select
}


/**
 * @brief Loads the temperature ADC look-up table (LUT_T_ADC) from EEPROM.
 *
 * This function reads the 140 data points for the temperature ADC look-up
 * table from its fixed address in the EEPROM. Each point is a 16-bit
 * unsigned integer, totaling 280 bytes.
 */
void Load_LUT_T_ADC(void)
{
    unsigned int i;
    unsigned char lsb, msb;

    CS_EE = 0; // Assert EEPROM chip select

    // Send READ command
    SPIOWrite(0x03);

    // Send the 16-bit memory address for LUT_T_ADC
    SPIOWrite((LUT_T_ADC_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(LUT_T_ADC_BASE_ADDRESS & 0xFF);

    // Read the 140 data points
    for (i = 0; i < POINTS_T; i++)
    {
        // Data is little-endian (LSB first)
        lsb = SPIORead();
        msb = SPIORead();
        LUT_T_ADC[i] = ((unsigned int)msb << 8) | lsb;
    }

    CS_EE = 1; // De-assert EEPROM chip select
}


/**
 * @brief Calculates the starting memory address for the LUT_P table read.
 *
 * This function determines the precise starting address within the EEPROM's
 * LUT_P table that corresponds to the minimum required temperature of 36°F.
 * It performs a linear interpolation on the Lin_data_T table to find the
 * target ADC value for 36°F, then finds the closest matching index in the
 * LUT_T_ADC table. This index is used to calculate the final byte address.
 *
 * @return The calculated starting address for the LUT_P read operation.
 */
unsigned int Calculate_LUT_Start_Address(void)
{
    // --- Step 1: Find Target ADC Value via Linear Interpolation ---

    // Target temperature in Fahrenheit.
    const float target_temp_F = 36.0f;
    // Convert target temperature to Celsius for calculation.
    const float target_temp_C = (target_temp_F - 32.0f) * 5.0f / 9.0f;

    unsigned int i;
    float y1, y2; // Bracketing temperatures from Lin_data_T
    unsigned int x1, x2; // Corresponding ADC values from Lin_data_T

    // Find the two points in the linearization table that bracket our target temperature.
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

    // Perform linear interpolation to find the ADC value for our target temperature.
    // Formula: target_adc = x1 + (x2 - x1) * (target_y - y1) / (y2 - y1)
    float target_adc_f = (float)x1 + ((float)x2 - (float)x1) * (target_temp_C - y1) / (y2 - y1);
    unsigned int target_adc = (unsigned int)target_adc_f;


    // --- Step 2: Find Closest Index in LUT_T_ADC ---

    unsigned int closest_index = 0;
    float min_diff = -1.0f; // Use -1 to indicate first run

    for (i = 0; i < POINTS_T; i++)
    {
        // Calculate the absolute difference between the current ADC value and our target.
        float diff = fabs((float)LUT_T_ADC[i] - (float)target_adc);

        // If this is the first iteration or if the new difference is smaller,
        // update the minimum difference and the closest index.
        if (min_diff < 0 || diff < min_diff)
        {
            min_diff = diff;
            closest_index = i;
        }
    }


    // --- Step 3: Calculate Final Memory Address ---

    // The LUT_P is a 2D table flattened in memory, organized by [Temp][Pressure].
    // To find the start of our desired temperature row, we multiply the
    // temperature index by the number of pressure points.
    // Each point is 2 bytes, so we multiply by 2 for the final byte offset.
    unsigned int address_offset = closest_index * POINTS_P * 2;

    // Add the offset to the base address of the LUT_P table.
    return LUT_P_BASE_ADDRESS + address_offset;
}


/**
 * @brief Loads the pressure look-up table (LUT_P) from the TP1200 EEPROM.
 *
 * This function reads a specific subset of the LUT_P table from the sensor's
 * EEPROM. It reads 1,178 16-bit unsigned integer data points (totaling
 * 2,356 bytes) and stores them in the global LUT_P array.
 *
 * The starting address and the number of points to read are based on the
 * memory optimization plan (decimation and temperature range limiting).
 *
 * @return void
 */
void Load_Pressure_LUT(void)
{
    // Calculate the dynamic start address before reading.
    unsigned int start_address = Calculate_LUT_Start_Address();

    unsigned int i;
    unsigned char lsb; // To store the least significant byte
    unsigned char msb; // To store the most significant byte

    // --- Begin SPI Communication ---

    // 1. Assert the EEPROM chip select to begin communication.
    CS_EE = 0;

    // 2. Send the READ command to the EEPROM.
    SPIOWrite(0x03);

    // 3. Send the 16-bit memory address to start reading from.
    // The address is sent MSB first, then LSB.
    SPIOWrite((start_address >> 8) & 0xFF); // Send high byte of address
    SPIOWrite(start_address & 0xFF);      // Send low byte of address

    // 4. Loop to read 1,178 data points (2,356 bytes).
    for (i = 0; i < LUT_P_SIZE; i++)
    {
        // The EEPROM data is in little-endian format (LSB first).
        lsb = SPIORead(); // Read the least significant byte
        msb = SPIORead(); // Read the most significant byte

        // Combine the two bytes into a 16-bit unsigned integer and
        // store it in the global LUT_P array.
        LUT_P[i] = ((unsigned int)msb << 8) | lsb;
    }

    // 5. De-assert the EEPROM chip select to end the communication.
    CS_EE = 1;
}
