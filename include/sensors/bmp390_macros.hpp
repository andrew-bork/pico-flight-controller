#pragma once

#define BMP390_ADDR 0x77

// SETTINGS
//                    00010011
#define BMP390_MODE 0b00110011 //NORMAL, PRESSURE ENABLED, TEMPERATURE ENABLED
#define BMP390_RESOLUTION // Standard
#define BMP390_OSRS 0b00000011 //x8 PRESSURE, x1 TEMPERATURE
#define BMP390_CONFIG 0b00000010 //2 IIR FILTER

#define BMP390_SOFT_RESET 0xB6
#define BMP390_FIFO_FLUSH 0xB0

// SETTINGS REG
#define BMP390_REG_PWR_CTRL 0x1B
#define BMP390_REG_OSR 0x1C
#define BMP390_REG_CONFIG 0x1F
#define BMP390_REG_PRESS_7_0 0x04
#define BMP390_REG_PRESS_15_8 0x05
#define BMP390_REG_PRESS_23_16 0x06
#define BMP390_REG_TEMP_7_0 0x07
#define BMP390_REG_TEMP_15_8 0x08
#define BMP390_REG_TEMP_23_16 0x09
#define BMP390_REG_CMD 0x7E // Commands
#define BMP390_REG_OSR 0x1C // Oversampling Settings
#define BMP390_REG_ODR 0x1D // Output Data Rate

//TRIMMING MEM
#define NVM_PAR_P11 0x45
#define NVM_PAR_P10 0x44
#define NVM_PAR_P9_H 0x43
#define NVM_PAR_P9_L 0x42
#define NVM_PAR_P8 0x41
#define NVM_PAR_P7 0x40
#define NVM_PAR_P6_H 0x3F
#define NVM_PAR_P6_L 0x3E
#define NVM_PAR_P5_H 0x3D
#define NVM_PAR_P5_L 0x3C
#define NVM_PAR_P4 0x3B
#define NVM_PAR_P3 0x3A
#define NVM_PAR_P2_H 0x39
#define NVM_PAR_P2_L 0x38
#define NVM_PAR_P1_H 0x37
#define NVM_PAR_P1_L 0x36
#define NVM_PAR_T3 0x35
#define NVM_PAR_T2_H 0x34
#define NVM_PAR_T2_L 0x33
#define NVM_PAR_T1_H 0x32
#define NVM_PAR_T1_L 0x31

#define BMP390_PRES_ENABLE 0b1
#define BMP390_TEMP_ENABLE 0b10
#define BMP390_NORM_MODE 0x30
#define BMP390_FORCE_MODE 0x10
#define BMP390_SLEEP_MODE 0x00

#define BMP390_FIFO_CONFIG_2 0x18
#define BMP390_FIFO_CONFIG_1 0x17
#define BMP390_FIFO_WATERMARK_1 0x16
#define BMP390_FIFO_WATERMARK_0 0x15
#define BMP390_FIFO_DATA 0x14
#define BMP390_FIFO_LENGTH_1 0x13
#define BMP390_FIFO_LENGTH_0 0x12