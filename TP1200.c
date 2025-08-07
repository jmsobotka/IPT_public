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
// CRC32 Look-up Table
//=============================================================================
// This table is used for efficient CRC32 calculation. It is placed in code
// memory as it is constant and relatively large.
code unsigned long crc_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643684,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
    0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
    0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
    0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693,
    0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

//=============================================================================
// Global Variables
//=============================================================================

// This global array will store the decimated pressure look-up table.
float xdata LUT_P[LUT_P_SIZE_DECIMATED];

// Global array for the temperature linearization table (ADC vs. Temp C).
Lin_Data_Point xdata Lin_data_T[POINTS_LIN_T];

// Global array for the decimated temperature ADC values look-up table.
float xdata LUT_T_ADC[POINTS_T_DECIMATED];

// Global array for the pressure ADC values look-up table.
float xdata LUT_P_ADC[POINTS_P];

// Global variables to hold ADC configuration settings loaded from EEPROM.
unsigned int xdata ADC_config_P;
unsigned int xdata ADC_config_T;
unsigned int xdata ADC_mode;

// Variables to hold calibration range data from EEPROM
int xdata Max_P;
int xdata Min_P;

unsigned int xdata g_raw_temp_adc; // Global for latest temperature reading

ProductInfoType xdata g_ProductInfo;

//=============================================================================
// Function Definitions
//=============================================================================

/**
 * @brief Top-level function to initialize the sensor by loading all LUTs.
 */
void Initialize_Sensor(void)
{
    // Load all necessary configuration and look-up tables from EEPROM.
    Load_Product_Info();
    Load_ADC_Config();
    Load_Lin_Data_T();
    Load_LUT_P_ADC(); // Note: Must be loaded before other LUTs
    Load_LUT_T_ADC();
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
            
            if ((x2 - x1) == 0) return y1; // Avoid division by zero
            return y1 + (y2 - y1) * ((float)raw_temp_adc - (float)x1) / ((float)x2 - (float)x1);
        }
    }

    // If the value is out of range, return the temperature from the closest endpoint.
    return (raw_temp_adc < Lin_data_T[0].adc_value) ? Lin_data_T[0].temperature_C : Lin_data_T[POINTS_LIN_T - 1].temperature_C;
}


/**
 * @brief Calculates the final compensated pressure.
 * @note This function is UPDATED to work with the new decimated LUTs.
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
    unsigned int y1_idx = 0, y2_idx = 0; // Temperature indices (for decimated LUT)
    float x1, x2, y1, y2, q11, q12, q21, q22, x, y, p1, p2, p3, p4;

    // Find bracketing indices for pressure ADC value in LUT_P_ADC
    // This LUT is not decimated.
    for (i = 0; i < POINTS_P - 1; i++) {
        if (raw_press_adc >= LUT_P_ADC[i] && raw_press_adc <= LUT_P_ADC[i+1]) {
            x1_idx = i;
            x2_idx = i + 1;
            break;
        }
    }

    // Find bracketing indices for temperature ADC value in the DECIMATED LUT_T_ADC
    for (i = 0; i < POINTS_T_DECIMATED - 1; i++) {
        if (comp_temp_adc >= LUT_T_ADC[i] && comp_temp_adc <= LUT_T_ADC[i+1]) {
            y1_idx = i;
            y2_idx = i + 1;
            break;
        }
    }

    // Get the four corner ADC values for interpolation
    x1 = LUT_P_ADC[x1_idx];
    x2 = LUT_P_ADC[x2_idx];
    y1 = LUT_T_ADC[y1_idx];
    y2 = LUT_T_ADC[y2_idx];

    // Get the four corner pressure values from the main decimated LUT_P table
    // The table is flattened, so index is calculated as [row * num_columns + column].
    // The number of columns is always POINTS_P.
    q11 = LUT_P[y1_idx * POINTS_P + x1_idx];
    q12 = LUT_P[y2_idx * POINTS_P + x1_idx];
    q21 = LUT_P[y1_idx * POINTS_P + x2_idx];
    q22 = LUT_P[y2_idx * POINTS_P + x2_idx];

    // Perform bilinear interpolation
    x = (float)raw_press_adc;
    y = (float)comp_temp_adc;

    // Check for division by zero, which can happen if ADC values are identical
    if ((x2 - x1) == 0 || (y2 - y1) == 0) {
        return q11; // Return one of the corners if the interpolation area is zero
    }

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
    ReloadWatchdog();
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
    ReloadWatchdog();
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
    SPIORead(); SPIORead(); // Consume the rest of the 32-bit field
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((ADC_CONFIG_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(ADC_CONFIG_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead();
    msb = SPIORead();
    ADC_config_P = ((unsigned int)msb << 8) | lsb;
    SPIORead(); SPIORead(); // Consume the rest of the 32-bit field
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((ADC_CONFIG_T_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(ADC_CONFIG_T_BASE_ADDRESS & 0xFF);
    lsb = SPIORead();
    msb = SPIORead();
    ADC_config_T = ((unsigned int)msb << 8) | lsb;
    SPIORead(); SPIORead(); // Consume the rest of the 32-bit field
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((MAX_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(MAX_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead(); msb = SPIORead();
    Max_P = ((int)msb << 8) | lsb;
    CS_EE = 1;

    CS_EE = 0;
    SPIOWrite(0x03);
    SPIOWrite((MIN_P_BASE_ADDRESS >> 8) & 0xFF);
    SPIOWrite(MIN_P_BASE_ADDRESS & 0xFF);
    lsb = SPIORead(); msb = SPIORead();
    Min_P = ((int)msb << 8) | lsb;
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
 * @note This function is UPDATED to read a range-limited and decimated subset
 * of the full table from EEPROM to save XDATA space.
 */
void Load_LUT_T_ADC(void)
{
    unsigned int i;
    unsigned int eeprom_read_addr;
    FloatByteUnion temp_converter;

    for (i = 0; i < POINTS_T_DECIMATED; i++)
    {
        // Calculate the index in the full EEPROM table, applying decimation
        unsigned int full_table_index = TEMP_RANGE_START_INDEX + (i * TEMP_DECIMATION_FACTOR);

        // Calculate the byte address in EEPROM for this float value
        eeprom_read_addr = LUT_T_ADC_BASE_ADDRESS + (full_table_index * 4);

        // Read the 4 bytes for the float value
        Read_EEPROM_Bytes(temp_converter.c, eeprom_read_addr, 4);
        
        // Store the float value in our decimated XDATA table
        LUT_T_ADC[i] = temp_converter.f;
    }
}

/**
 * @brief Loads the pressure ADC values look-up table (LUT_P_ADC) from EEPROM.
 * @note This function is UPDATED to read 4-byte floats instead of 2-byte ints.
 * This table is NOT decimated.
 */
void Load_LUT_P_ADC(void)
{
    // The entire LUT_P_ADC table is read, as it is small and not decimated.
    Read_EEPROM_Floats(LUT_P_ADC, LUT_P_ADC_BASE_ADDRESS, POINTS_P);
}

/**
 * @brief Loads the pressure look-up table (LUT_P) from the TP1200 EEPROM.
 * @note This function is UPDATED to read a range-limited and decimated subset
 * of the full table from EEPROM to save XDATA space.
 */
void Load_Pressure_LUT(void)
{
    unsigned int i;
    unsigned int eeprom_row_addr;
    float* xdata_row_ptr;

    for (i = 0; i < POINTS_T_DECIMATED; i++)
    {
        // Calculate the index of the row to read from the full EEPROM table
        unsigned int full_table_row_index = TEMP_RANGE_START_INDEX + (i * TEMP_DECIMATION_FACTOR);

        // Calculate the starting byte address of that row in the EEPROM
        // Each row has POINTS_P (30) floats, and each float is 4 bytes.
        eeprom_row_addr = LUT_P_BASE_ADDRESS + (full_table_row_index * POINTS_P * 4);

        // Calculate the pointer to the destination row in our XDATA table
        xdata_row_ptr = &LUT_P[i * POINTS_P];

        // Read the entire row (30 floats) from EEPROM into our XDATA table
        Read_EEPROM_Floats(xdata_row_ptr, eeprom_row_addr, POINTS_P);
    }
}

/**
 * @brief Helper function to read a block of floats from a specific EEPROM address.
 *
 * @param buffer Pointer to the destination float array in XDATA.
 * @param start_address The starting byte address in EEPROM.
 * @param float_count The number of floats to read.
 */
void Read_EEPROM_Floats(float* buffer, unsigned int start_address, unsigned int float_count)
{
    unsigned int i;
    FloatByteUnion temp_converter;

    CS_EE = 0;
    SPIOWrite(0x03); // Read command
    SPIOWrite((start_address >> 8) & 0xFF);
    SPIOWrite(start_address & 0xFF);
    
    for (i = 0; i < float_count; i++)
    {
        // Read 4 bytes for one float
        temp_converter.c[0] = SPIORead();
        temp_converter.c[1] = SPIORead();
        temp_converter.c[2] = SPIORead();
        temp_converter.c[3] = SPIORead();
        
        // Assign the converted float to the buffer
        buffer[i] = temp_converter.f;
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
    FloatByteUnion p_min_adc, p_max_adc;

    // --- Read and Display Serial Number (Bytes 8-11) ---
    Read_EEPROM_Bytes(temp_buffer, SENSOR_SN_BASE_ADDRESS, 4);
    serial_number = *((unsigned long*)temp_buffer);
    printf("Sensor S/N: %lu\r", serial_number);

    // --- Read and Display Calibration Date (Bytes 20-23) ---
    Read_EEPROM_Bytes(temp_buffer, CAL_DATE_BASE_ADDRESS, 4);
    printf("Cal Date: %u/%u/%u\r", (unsigned int)temp_buffer[1], (unsigned int)temp_buffer[0], *((unsigned int*)&temp_buffer[2]));

    Read_EEPROM_Bytes(p_min_adc.c, ADC_MIN_P_BASE_ADDRESS, 4);
    Read_EEPROM_Bytes(p_max_adc.c, ADC_MAX_P_BASE_ADDRESS, 4);

    printf("Pressure Max "); Print_Float(p_max_adc.f, 0); printf(" ");
    Print_Float(CompPressureUnAdjusted((long)p_max_adc.f), 4);
    printf("\r");

    printf("Pressure Min "); Print_Float(p_min_adc.f, 0); printf(" ");
    Print_Float(CompPressureUnAdjusted((long)p_min_adc.f), 4);
    printf("\r");

    // For temperature, we can use the loaded Lin_data_T table
    temp_f = (Lin_data_T[POINTS_LIN_T - 1].temperature_C * 9.0f / 5.0f) + 32.0f;
    printf("Temperature Max %6u ", (unsigned int)Lin_data_T[POINTS_LIN_T - 1].adc_value);
    Print_Float(temp_f, 4);
    printf("\r");

    temp_f = (Lin_data_T[0].temperature_C * 9.0f / 5.0f) + 32.0f;
    printf("Temperature Min %6u ", (unsigned int)Lin_data_T[0].adc_value);
    Print_Float(temp_f, 4);
    printf("\r");
}

//=============================================================================
// Function Definitions
//=============================================================================

/**
 * @brief Calculates the CRC32 checksum for a block of data.
 *
 * @param data Pointer to the data buffer.
 * @param count The number of bytes to process.
 * @return The calculated 32-bit CRC.
 */
unsigned long Calculate_CRC32(unsigned char* d, unsigned int count)
{
    unsigned long crc = 0xFFFFFFFF;
    while (count--)
    {
        crc = crc_table[(crc ^ *d++) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}


/**
 * @brief Verifies the integrity of the MEMSCAP EEPROM's configuration block.
 *
 * This function replaces the original ReadEEProm() for the HC_CK host command.
 * It reads the first 108 bytes (104 bytes of data + 4 bytes of CRC) of the
 * EEPROM, calculates the CRC32 on the data, and compares it to the stored CRC.
 *
 * @return 1 (TRUE) if the checksum is valid, 0 (FALSE) otherwise.
 */
byte ReadEEProm(void)
{
    unsigned char buffer[108];
    unsigned long stored_crc;
    unsigned long calculated_crc;

    Read_EEPROM_Bytes(buffer, 0, 108);
    stored_crc = ((long)buffer[107] << 24) | ((long)buffer[106] << 16) | ((long)buffer[105] << 8) | buffer[104];
    calculated_crc = Calculate_CRC32(buffer, 104);

    return (calculated_crc == stored_crc);
}

/**
 * @brief Loads product information (S/N, Cal Date) from the EEPROM.
 */
void Load_Product_Info(void)
{
    unsigned char buffer[4];

    Read_EEPROM_Bytes(buffer, SENSOR_SN_BASE_ADDRESS, 4);
    g_ProductInfo.serial_number = ((long)buffer[3] << 24) | ((long)buffer[2] << 16) | ((long)buffer[1] << 8) | buffer[0];

    Read_EEPROM_Bytes(buffer, CAL_DATE_BASE_ADDRESS, 4);
    g_ProductInfo.cal_day = buffer[0];
    g_ProductInfo.cal_month = buffer[1];
    g_ProductInfo.cal_year = ((unsigned int)buffer[3] << 8) | buffer[2];
}

/**
 * @brief Calculates unadjusted, factory-compensated pressure. (For Maintenance)
 *
 * This function provides the factory-compensated pressure value before any
 * system-level offset and span calibration is applied. It uses the most
 * recent temperature reading for its calculation. The prototype is matched
 * to the original function for compatibility with the maintenance library.
 *
 * @param p The raw pressure ADC value to be compensated.
 * @return The factory-compensated pressure as a float.
 */
float CompPressureUnAdjusted(long p)
{
    // Cast the input 'long' to 'unsigned int' to match the new function's
    // requirements and use the last known temperature reading.
    return Calculate_Compensated_Pressure((unsigned int)p, g_raw_temp_adc);
}
