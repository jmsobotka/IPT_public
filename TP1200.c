/**
 * @file TP1200.c
 * @brief Driver for handling the MEMSCAP TP1200 pressure sensor.
 *
 * This file contains the function to load the decimated and range-limited
 * pressure look-up table (LUT_P) from the sensor's EEPROM into the
 * microcontroller's XDATA memory.
 */

#include "TP1200.h"
#include "Main.h" // For access to SPI functions like SPIOWrite and SPIORead
#include "c8051f020_kdefs.h" // For sbit definitions like CS_EE

//=============================================================================
// Global Variables
//=============================================================================

// This global array will store the pressure look-up table.
// It is placed in XDATA memory to accommodate its size.
unsigned int xdata LUT_P[LUT_P_SIZE];

//=============================================================================
// Function Definitions
//=============================================================================

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
    // The starting address of the full LUT_P table in the EEPROM is 788.
    // This address needs to be adjusted based on the specific start point
    // of the desired temperature range. For this example, we will start
    // reading from a calculated offset.
    // NOTE: The exact start_address will need to be determined by analyzing
    // the LUT_T_ADC table to find the index corresponding to 36°F.
    // For now, a placeholder address is used.
    unsigned int start_address = 788; // Placeholder: Replace with calculated start address

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
