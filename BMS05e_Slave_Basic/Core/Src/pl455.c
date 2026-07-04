/*
 * 	pl455.c
 *
 *  Created on: July, 2023
 *  Author: TLMoto
 *
 */

#include "pl455.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

//Declare the defines
#define number_cells 30
#define number_aux 8
#define number_slaves 2
#define number_all_data 82 //82 bytes that I will receive from slaves when I ask to the data, 76 from data, 2 from 2 headers (1 per slave), 4 from CRC (2 per slave)
#define number_data 41 //41 bytes from data of each slave
#define SlaveRx_Buf_size 128

// Declare the external variables defined in the main file
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

extern char msg_debug[512];
extern size_t msg_debug_len;

extern uint8_t msg_slave[SlaveRx_Buf_size];
extern uint16_t msg_slave_len;

//CAN variables

// Function to Wake Up BQ76PL455A
// This will reset and the wake the BQ76PL455A
void wakeUp(void)
{
    // Toggle wake signal
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
}

// Function to Power Down BQ76PL455A
void powerDown(void)
{
	WriteReg(0 , 12, 0x40, 1, FRMWRT_ALL_NR); 		// Power Down all devices in the stack
}

int WriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType)
{
    int bRes = 0;
    BYTE bBuf[8] = {0};

    for (int i = 0; i < bLen; i++) {
        bBuf[i] = (dwData >> (8 * (bLen - i - 1))) & 0xFF;
    }

    bRes = WriteFrame(bID, wAddr, bBuf, bLen, bWriteType);
    return bRes;
}

int WriteFrame(BYTE bID, uint16_t wAddr, BYTE *pData, BYTE bLen, BYTE bWriteType)
{
    int    	bPktLen = 0;		// Frame Length
    BYTE   	pFrame[32];			// Frame data with a size of 32 bytes.
    BYTE 	*pBuf = pFrame;		// Pointer to the start of pFrame array
    uint16_t   wCRC;			// Calculated CRC value

    //if (bLen == 7 || bLen > 8)	// sera que é necessario?
        //return 0;

    memset(pFrame, 0x7F, sizeof(pFrame));
    //Choose 16-bit (wAddr > 255) or 8-bit (else) address mode
    if (wAddr > 255)
    {
        *pBuf++ = 0x88 | bWriteType | bLen; // use 16-bit address, sets bit 7 and 3
        if (bWriteType == FRMWRT_SGL_R || bWriteType == FRMWRT_SGL_NR || bWriteType == FRMWRT_GRP_R || bWriteType == FRMWRT_GRP_NR)//(bWriteType != FRMWRT_ALL_NR)// || (bWriteType != FRMWRT_ALL_R))
        {
            *pBuf++ = (bID & 0x00FF);		// stores the device ID
        }
        *pBuf++ = (wAddr & 0xFF00) >> 8;
        *pBuf++ =  wAddr & 0x00FF;
    }
    else
    {
        *pBuf++ = 0x80 | bWriteType | bLen; // use 8-bit address, sets bit 7
        if (bWriteType == FRMWRT_SGL_R || bWriteType == FRMWRT_SGL_NR || bWriteType == FRMWRT_GRP_R || bWriteType == FRMWRT_GRP_NR)
        {
            *pBuf++ = (bID & 0x00FF);
        }
        *pBuf++ = wAddr & 0x00FF;
    }

    // Copy data from pData to pBuf
    while(bLen--)
        *pBuf++ = *pData++;

    bPktLen = pBuf - pFrame;

    wCRC = CRC16(pFrame, bPktLen);
    *pBuf++ = wCRC & 0x00FF;
    *pBuf++ = (wCRC & 0xFF00) >> 8;
    bPktLen += 2;

	for (uint16_t i = 0; i < bPktLen; i++)
    {
        printf("%02X ", pFrame[i]);
    }
    printf("\n");

    /*
     * pFrame[n] array:
     * pFrame[0] 	= Frame Header
     * pFrame[1] 	= Device ID
     * pFrame[2]	= Register Address MSB (if 16-bit address)
     * pFrame[3]	= Register Address LSB
     * pFrame[4] 	= Start of Data
     * ...
     * pFrame[n-2] 	= End of Data
     * pFrame[n-1] 	= CRC
     * pFrame[n] 	= CRC
     */

    //Test if I can use DMA to transmit to
 	HAL_UART_Transmit(&huart4, pFrame, (uint16_t)bPktLen, 2);

    return bPktLen;
}

// CRC16 for PL455
// ITU_T polynomial: x^16 + x^15 + x^2 + 1
const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t CRC16(const uint8_t *pBuf, size_t nLen)
{
    uint16_t wCRC = 0;
    int i;

    for (i = 0; i < nLen; i++)
    {
        wCRC ^= (*pBuf++) & 0x00FF;
        wCRC = crc16_table[wCRC & 0x00FF] ^ (wCRC >> 8);
    }
    return wCRC;
}

uint16_t CRC16_CCITT_FALSE(const uint8_t *data, size_t length)
{
    uint16_t crc = 0x0000;  // Initial value

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


//Send the initial messages to the BQ;
void setupSlave(void){

	WriteReg(0, 16, 0x10E0, 2, FRMWRT_ALL_NR); //1.2.1

	WriteReg(0, 14, 0x10, 1, FRMWRT_ALL_NR); //1.2.2
	WriteReg(0, 12, 0x08, 1, FRMWRT_ALL_NR); //1.2.2

	WriteReg(0, 10, 0x00, 1, FRMWRT_ALL_NR); //1.2.3 - Slave 1, Address 0
	WriteReg(0, 10, 0x01, 1, FRMWRT_ALL_NR); //1.2.3 - Slave 2, Address 1

	//This can be commented after verified that is all working correctly
	WriteReg(0, 10, 0x00, 1, FRMWRT_SGL_R);		//1.2.4 - Slave 1, Address 0 (Master Slave)

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 1.2.4 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (MS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(1, 10, 0x00, 1, FRMWRT_SGL_R);		//1.2.4 - Slave 2, Address 1 (Slave Slave)

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 1.2.4 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(1, 16, 0x1020, 2, FRMWRT_SGL_NR); //1.2.5
	WriteReg(0, 16, 0x10C0, 2, FRMWRT_SGL_NR); //1.2.6

	WriteReg(1, 82, 0xFFC0, 2, FRMWRT_SGL_NR);	//1.2.7 - Slave 2
	WriteReg(0, 82, 0xFFC0, 2, FRMWRT_SGL_NR);	//1.2.7 - Slave 1

	/***********************************************************************************************************************************/
	//USER_CKSUM Erro ler registo depois tirar
	//Slave 2
	WriteReg(1, 240, 0x03, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro Fault User_cksum SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS) erro user_cksum: %02X%02X%02X%02X\n", msg_slave[1], msg_slave[2], msg_slave[3], msg_slave[4]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Slave 1
	WriteReg(0, 240, 0x03, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro Fault User_cksum MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (mS) erro user_cksum: %02X%02X%02X%02X\n", msg_slave[1], msg_slave[2], msg_slave[3], msg_slave[4]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);
	//

	//USER_CKSUM_RD Erro ler registo depois tirar
	//Slave 2
	//
	WriteReg(1, 244, 0x03, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro Fault User_cksum_RD SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS) erro user_cksum_RD: %02X%02X%02X%02X\n", msg_slave[1], msg_slave[2], msg_slave[3], msg_slave[4]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Slave 1
	WriteReg(0, 244, 0x03, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro Fault User_cksum_RD MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (mS) erro user_cksum_RD: %02X%02X%02X%02X%\n", msg_slave[1], msg_slave[2], msg_slave[3], msg_slave[4]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);
	/************************************************************************************************************************************************************************/
	//Ajustar estes valores conforme o que ler acima

	//escrever para o slave 2 o CKSUM_RSLT para o CKSUM
	WriteReg(1, 240, 0xA07192FC, 4, FRMWRT_SGL_NR);

	//escrever para o slave 1 o CKSUM_RSLT para o CKSUM
	WriteReg(0, 240, 0xA11191FC, 4, FRMWRT_SGL_NR);
	/************************************************************************************************************************************************************************/

	// Configuring AFE from top to bottom
	//Slave 2
	WriteReg(1, 61, 0x00, 1, FRMWRT_SGL_NR); //2.2.1
	WriteReg(1, 62, 0xBC, 1, FRMWRT_SGL_NR); //2.2.2
	WriteReg(1, 7, 0x00, 1, FRMWRT_SGL_NR); //2.2.3
	WriteReg(1, 81, 0x38, 1, FRMWRT_SGL_NR); //2.2.4
	WriteReg(1, 82, 0xFFC0, 2, FRMWRT_SGL_NR); //2.2.4

	//Verify if the faults was clear, comment after tested
	WriteReg(1, 81, 0x00, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.1 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(1, 82, 0x01, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.2 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS): %02X%02X \n", msg_slave[1], msg_slave[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Slave 1
	WriteReg(0, 61, 0x00, 1, FRMWRT_SGL_NR); //2.2.1
	WriteReg(0, 62, 0xBC, 1, FRMWRT_SGL_NR); //2.2.2		//it was 0x04
	WriteReg(0, 7, 0x00, 1, FRMWRT_SGL_NR); //2.2.3
	WriteReg(0, 81, 0x38, 1, FRMWRT_SGL_NR); //2.2.4
	WriteReg(0, 82, 0xFFC0, 2, FRMWRT_SGL_NR); //2.2.4

	//Verify if the faults was clear, comment after tested
	WriteReg(0, 81, 0x00, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.1 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (MS): %02X\n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(0, 82, 0x01, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.2 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (MS): %02X%02X\n", msg_slave[1], msg_slave[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);


	// 2.2.5 Select number of cells and channels to sample
	WriteReg(0, 13, 0x0F, 1, FRMWRT_ALL_NR); // set number of cells to 15
	WriteReg(0, 3, 0x7FFF0F00, 4, FRMWRT_ALL_NR); // select all things, in the future, select only the things that matter
	//HAL_Delay(1000);

	// 2.2.6 Set cell over-voltage and cell under-voltage thresholds on all boards
	WriteReg(0, 144, 0xD70A, 2, FRMWRT_ALL_NR); // set OV threshold = 4.2000V
	WriteReg(0, 142, 0x8001, 2, FRMWRT_ALL_NR); // set UV threshold = 2.5000V
	//HAL_Delay(1000);

	// Set on the internal pull up resistors from AUX channels on all boards
	WriteReg(0, 37, 0x07, 1, FRMWRT_ALL_NR);	 // set pull up resistors from AUX 0 to 2

	//I need to verify if the BQ was well configured!!!!

	return;
}

//Função para escrever para a EEPROM as configurações já alteradas, só usar quando tiver a certeza que tudo está bem configurado
void setup_EEPROM(void){
	//Se mantiver os erros iguais, vou reescrever para a EEPROM
	//Escrever para todos
	WriteReg(0, 130, 0x8C2DB194, 4, FRMWRT_ALL_NR); //Write Magic Number 1 in all devices
	WriteReg(0, 252, 0xA375E60F, 4, FRMWRT_ALL_NR); //Write Magic Number 2 in all decices

	WriteReg(0, 12, 0x10, 1, FRMWRT_ALL_NR); //Write to EEPROM in all devices
	HAL_Delay(1000); //Write to EEPROM takes 200ms

	//Slave 1 ver se está tudo okay
	WriteReg(0, 12, 0x00, 1, FRMWRT_SGL_R); //Read the value in Write_EEPROM

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro EEPROM Slave 1 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 EEPROM (MS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Slave 2 ver se está tudo okay
	WriteReg(1, 12, 0x00, 1, FRMWRT_SGL_R); //Read the value in Write_EEPROM

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro EEPROM Slave 2 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 EEPROM (SS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Testar para ver se o fault desapareceu
	msg_debug_len = sprintf(msg_debug, "Teste Faults 3 - Depois de ter escrito na EEPROM\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//Verify if the faults was clear, comment after tested Slave 2
	WriteReg(1, 81, 0x00, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.1 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS): %02X \n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(1, 82, 0x01, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.2 SS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 2 (SS): %02X%02X \n", msg_slave[1], msg_slave[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	//Verify if the faults was clear, comment after tested Slave 1
	WriteReg(0, 81, 0x00, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.1 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (MS): %02X\n", msg_slave[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);

	WriteReg(0, 82, 0x01, 1, FRMWRT_SGL_R);

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle(&huart4, msg_slave, SlaveRx_Buf_size, &msg_slave_len, 100) ){
		msg_debug_len = sprintf(msg_debug, "Erro 2.2.4.2 MS\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
		//HAL_Delay(100);
	}

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (MS): %02X%02X\n", msg_slave[1], msg_slave[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
	//HAL_Delay(100);
}

//Function that process the data from slaves
void dataSlave(const uint8_t *data){
	//All of this will be in format of higher to minor, vsense15 to vsense0, aux3 to aux0
	float* slave1_voltages = (float*) malloc(15 * sizeof(float));
	float* slave1_aux = (float*) malloc(4 * sizeof(float));
	float* slave2_voltages = (float*) malloc(15 * sizeof(float));
	float* slave2_aux = (float*) malloc(4 * sizeof(float));

	//Verify if I have received all data
	if(msg_slave_len == number_all_data){

		//Slave 2
		float* slave2_all = dataSlave_auxiliar(data, 2);

		if (slave2_all != NULL) {
			// Copy first 15 values to slave2_voltages
			memcpy(slave2_voltages, slave2_all, 15 * sizeof(float));

			// Copy next 4 values to slave2_aux
			memcpy(slave2_aux, slave2_all + 15, 4 * sizeof(float));

			free(slave2_all);  // Free allocated memory after copying
		}

		//Check if any over/under voltage occur - Implementar com faults


		//Slave 1
		float* slave1_all = dataSlave_auxiliar(data, 1);

		if (slave1_all != NULL) {
			// Copy first 15 values to slave2_voltages
			memcpy(slave1_voltages, slave1_all, 15 * sizeof(float));

			// Copy next 4 values to slave2_aux
			memcpy(slave1_aux, slave1_all + 15, 4 * sizeof(float));

			free(slave1_all);  // Free allocated memory after copying
		}

		//Check if any over/under voltage occur - Implementar com faults

		//Print for debug
		msg_debug_len = sprintf(msg_debug,
				"Slave 1 no formato 15, 14, ...;\n Slave1 = %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f\n",
				slave1_voltages[0], slave1_voltages[1], slave1_voltages[2], slave1_voltages[3], slave1_voltages[4], slave1_voltages[5],
				slave1_voltages[6], slave1_voltages[7], slave1_voltages[8], slave1_voltages[9], slave1_voltages[10], slave1_voltages[11],
				slave1_voltages[12], slave1_voltages[13], slave1_voltages[14]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, HAL_MAX_DELAY);

		msg_debug_len = sprintf(msg_debug,
				"Slave 1 aux no formato 4, 3, ...;\n Slave1_AUX = %.03f, %.03f, %.03f, %.03f\n",
				slave1_aux[0], slave1_aux[1], slave1_aux[2], slave1_aux[3]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, HAL_MAX_DELAY);


		msg_debug_len = sprintf(msg_debug,
				"Slave 2 no formato 15, 14, ...;\n Slave2 = %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f\n",
				slave2_voltages[0], slave2_voltages[1], slave2_voltages[2], slave2_voltages[3], slave2_voltages[4], slave2_voltages[5],
				slave2_voltages[6], slave2_voltages[7], slave2_voltages[8], slave2_voltages[9], slave2_voltages[10], slave2_voltages[11],
				slave2_voltages[12], slave2_voltages[13], slave2_voltages[14]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, HAL_MAX_DELAY);

		msg_debug_len = sprintf(msg_debug,
				"Slave 2 aux no formato 4, 3, ...;\n Slave2_AUX = %.03f, %.03f, %.03f, %.03f\n",
				slave2_aux[0], slave2_aux[1], slave2_aux[2], slave2_aux[3]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, HAL_MAX_DELAY);

		msg_slave_len = 0;
		data = NULL; //cleaning

		//clear the buffer
		memset(msg_slave, 0, SlaveRx_Buf_size);
	}

	// Free allocated memory
	free(slave1_voltages);
	free(slave1_aux);
	free(slave2_voltages);
	free(slave2_aux);

	//provavelmente vou ter que colocar estes slave2 e slave1 como globais para depois poder mandar por can na principal




}

float* dataSlave_auxiliar(const uint8_t *data, uint8_t slaveId){
	uint8_t numBytes = (number_data - 3) / 2; //Number of bytes with relevant data (exclude 2 CRC and 1 header bytes)

	uint8_t startIndex;

	// Arrays for hexadecimal and float values
	//uint16_t hexadecimalValues[numBytes];
	float* decimalValues = (float*) malloc(numBytes * sizeof(float));

	uint16_t mergedBytes;

	// Group bytes in groups of 2 and convert to float
	if(slaveId == 2){
		startIndex = 1; //Ignores the first byte (header)

		for (uint8_t i = 0; i < numBytes; i++) {
			mergedBytes = (data[startIndex] << 8) | data[startIndex + 1];
			//hexadecimalValues[i] = mergedBytes;

			float decimalValue = hexToFloat(mergedBytes);
			decimalValues[i] = decimalValue;
			startIndex += 2;
		}
	}
	else{
		startIndex = number_data + 1; //Ignore the data from slave 1 and the header
		for (uint8_t i = 0; i < numBytes; i++) {
			mergedBytes = (data[startIndex] << 8) | data[startIndex + 1];
			//hexadecimalValues[i] = mergedBytes;

			float decimalValue = hexToFloat(mergedBytes);
			decimalValues[i] = decimalValue;
			startIndex += 2;
		}
	}

	return decimalValues;
}




float hexToFloat(uint16_t value) {
	//float factor = 13107.38791552861;
	float factor_inverse = 7.62928515158445919e-5;
    return (float)value * factor_inverse;
}




