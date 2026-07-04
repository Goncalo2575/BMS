/*
 * 	pl455.h
 *
 *  Created on: July, 2023
 *  Author: TLMoto
 *  Adapted from https://github.com/shyamanthrh/passive_cell_balancing/blob/c4b509a561ca9be845ad43cb3c21c7b519d1b6b7/STM32%20Embedded%20code/MDK-ARM/pl455.c
 *
 *
 */

#ifndef PL455_H_
#define PL455_H_

#include "main.h"

//typedefs
typedef unsigned char BYTE;
typedef unsigned int BOOL;
typedef signed int HANDLE;

// User defines for REQ_TYPE
#define FRMWRT_SGL_R	0x00 // single device write with response
#define FRMWRT_SGL_NR	0x10 // single device write without response
#define FRMWRT_GRP_R	0x20 // group broadcast with response
#define FRMWRT_GRP_NR	0x30 // group broadcast without response
#define FRMWRT_ALL_R	0x60 // general broadcast with response
#define FRMWRT_ALL_NR	0x70 // general broadcast without response

// Data size for slaves
#define NOC_13      15  // Number of cells, Slave 1 and 3
#define NOC_24      14  // Number of cells, Slave 2 and 4
#define DATA_13     33  // Size of data, Slave 1 and 3
#define DATA_24     31  // Size of data, Slave 2 and 4

// Function Prototypes

// Function to Power Down BQ76PL455A
void powerDown(void);

// Function to Wake Up BQ76PL455A
void wakeUp(void);

// WriteReg(Slave ID, Address, Address data, size of data, REQ_TYPE)
// size of data: cada 8bits, aumenta 1 (0x00 = 1, 0x0000 = 2, ...)
int  WriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType);
int  WriteFrame(BYTE bID, uint16_t wAddr, BYTE * pData, BYTE bLen, BYTE bWriteType);


//new functions
void setupSlave(void);
void balancing_setup(void);
void dataSlave(const uint8_t *data);
float* dataSlave_auxiliar(const uint8_t *data, uint8_t slaveId);
void balancing(void);

// Convert hexadecimal value to float
float hexToFloat(uint16_t value);
// Read and print cell voltages
float* Voltages(const uint8_t *data, uint8_t length, uint8_t slaveID);
void cellVoltages(const uint8_t *data, uint8_t length);
void cellVoltages1(const uint8_t *data, uint8_t length);	//idea
void cellVoltages2(const uint8_t *data, uint8_t length);	//idea
void cellVoltages3(const uint8_t *data, uint8_t length);	//idea
void cellVoltages4(const uint8_t *data, uint8_t length);	//idea

uint16_t CRC16(const uint8_t *pBuf, size_t nLen);
uint16_t CRC16_CCITT_FALSE(const uint8_t *data, size_t length);


#endif /* PL455_H_ */
