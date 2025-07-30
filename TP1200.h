// TP1200.h

#ifndef TP1200_H
#define TP1200_H

// --- Constants / Macros ---
// These values would need to be defined based on the EEPROM layout
#define POINTS_P 30
#define POINTS_T 140
#define POINTS_LIN_T 12
#define LUT_P_SIZE 1178 // Our calculated reduced size

#define LUT_P_BASE_ADDRESS 0x0800  // Example, get from EEPROM doc
// ... other base addresses

// --- Type Definitions ---
typedef struct {
    float temperature_C;
    unsigned int adc_value;
} Lin_Data_Point;

// --- Function Prototypes ---
void Initialize_Sensor(void);
void Get_Raw_Sensor_Readings(unsigned int* raw_press_adc, unsigned int* raw_temp_adc);
float Calculate_Compensated_Temperature(unsigned int raw_temp_adc);
float Calculate_Compensated_Pressure(unsigned int raw_press_adc, unsigned int comp_temp_adc);

// These are "private" to the C file but could be here if needed elsewhere
void Load_ADC_Config(void);
void Load_Lin_Data_T(void);
void Load_LUT_T_ADC(void);
void Load_LUT_P_ADC(void);
void Load_Pressure_LUT(void);
unsigned int Calculate_LUT_Start_Address(void);

#endif // TP1200_H
