/*
 * 	pl455.c
 *
 *  Created on: July, 2023
 *  Author: TLMoto
 *
 */

#include "pl455.h"
#include "math.h"

//Declare the defines
#define number_cells 30
#define number_aux 8
#define number_slaves 2
#define number_all_data 82 //82 bytes that I will receive from slaves when I ask to the data, 76 from data, 2 from 2 headers (1 per slave), 4 from CRC (2 per slave)
#define number_data 41 //41 bytes from data of each slave
#define SlaveRx_Buf_size 128

#define NUM_CELLS_SLAVE 15
#define NUM_TEMPERATURES 6

#define TARGET_VOLT 3.3

// Declare the external variables defined in the main file
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

extern char msg_debug[512];
extern size_t msg_debug_len;

extern uint8_t msg_slave[SlaveRx_Buf_size];
extern uint16_t msg_slave_len;

extern float v_slave1[NUM_CELLS_SLAVE];
extern float v_slave2[NUM_CELLS_SLAVE];
extern float temperatures[NUM_TEMPERATURES];

extern float B_plus;
extern float B_plus_fuse;

extern float HV_plus;

/* BALANCING */
uint16_t balancing_slave1;
uint16_t balancing_slave2;

//CAN variables


// Function to Power Down BQ76PL455A
void powerDown(void)
{
	HAL_Delay(1000); //Test if this is realy necessary
	WriteReg(0 , 12, 0x40, 1, FRMWRT_ALL_NR); 		// Power Down all devices in the stack
}

// Function to Wake Up BQ76PL455A
// This will reset and the wake the BQ76PL455A
void wakeUp(void)
{
    // Toggle wake signal
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
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

	msg_debug_len = sprintf(msg_debug, "Response Slave 1 (mS) erro user_cksum_RD: %02X%02X%02X%02X\n", msg_slave[1], msg_slave[2], msg_slave[3], msg_slave[4]);
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
	WriteReg(0, 37, 0x07, 1, FRMWRT_SGL_NR);	 // set pull up resistors from AUX 0 to 2
	WriteReg(1, 37, 0x07, 1, FRMWRT_SGL_NR);	 // set pull up resistors from AUX 0 to 2

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

	//Verify if I have received all data
	if(msg_slave_len == number_all_data){

		HV_plus = 0;

		//Slave 2
		float* slave2_all = dataSlave_auxiliar(data, 2);

		if (slave2_all != NULL) {
			// Copy first 15 values to v_slave2
			memcpy(v_slave2, slave2_all, 15 * sizeof(float));

			// Copy aux3 to B_plus_fuse;
			//memcpy(&B_plus_fuse, slave2_all + 15, 1 * sizeof(float));
			//B_plus_fuse *= 27.109660; // Transform from 0-5V to 0-126V;

			// Copy aux2, aux1, aux0 values to temperatures
			memcpy(temperatures + 3, slave2_all + 16, 3 * sizeof(float));

			free(slave2_all);  // Free allocated memory after copying
		}

		//Check if any over/under voltage occur - Implementar com faults


		//Slave 1
		float* slave1_all = dataSlave_auxiliar(data, 1);

		if (slave1_all != NULL) {
			// Copy first 15 values to v_slave1
			memcpy(v_slave1, slave1_all, 15 * sizeof(float));
			//v_slave1[0] = 3.69;

			// Copy aux3 to B_plus_fuse;
			memcpy(&B_plus, slave1_all + 15, 1 * sizeof(float));
			B_plus *= 27.109660; // Transform from 0-5V to 0-126V
			//B_plus *= 2; //Teste na bancada LVS, para a mota tirar isto

			// Copy next 4 values to temperatures
			memcpy(temperatures, slave1_all + 16, 3 * sizeof(float));

			free(slave1_all);  // Free allocated memory after copying
		}

		//Check if any over/under voltage occur - Implementar com faults

		//Calculate Temperature from Voltage in vector temperatures
		//FIXME: Código com a curva dos NTCS (espero que esteja bem também)
		for(int i = 0; i < 6; i++){
			temperatures[i] = (((27000 * temperatures[i])/5.3) - 1000)/(1 - (temperatures[i]/5.3));
			temperatures[i] =1/(-0.001853578215 + 0.0005413328992*log(temperatures[i]) - 0.0000006499196894*pow(log(temperatures[i]),3)) - 273.15;
		}

		//Print for debug

		msg_debug_len = sprintf(msg_debug,
				"-------------------------------------------------------------------------\n\n"
				"[CELL TEMPERATURES]\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"\t[SLAVE 1]\n"
				"\t[1, 2, 3]\n"
				"\t%.03f %.03f %.03f\n\n",
				temperatures[2], temperatures[1], temperatures[0]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"\t[SLAVE 2]\n"
				"\t[1, 2, 3]\n"
				"\t%.03f %.03f %.03f\n\n",
				temperatures[5], temperatures[4], temperatures[3]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"-------------------------------------------------------------------------\n\n"
				"[CELL VOLTAGES]\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"\t[SlAVE 1]\n"
				"\t[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]\n"
				"\t%.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f\n\n",
				v_slave1[14], v_slave1[13], v_slave1[12], v_slave1[11], v_slave1[10], v_slave1[9],
				v_slave1[8], v_slave1[7], v_slave1[6], v_slave1[5], v_slave1[4], v_slave1[3],
				v_slave1[2], v_slave1[1], v_slave1[0]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"\t[SlAVE 2]\n"
				"\t[16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]\n"
				"\t%.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f\n\n",
				v_slave2[14], v_slave2[13], v_slave2[12], v_slave2[11], v_slave2[10], v_slave2[9],
				v_slave2[8], v_slave2[7], v_slave2[6], v_slave2[5], v_slave2[4], v_slave2[3],
				v_slave2[2], v_slave2[1], v_slave2[0]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"-------------------------------------------------------------------------\n\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		msg_debug_len = sprintf(msg_debug,
				"\t[B+ VOLTAGE]\n"
				"\t%.03f\n\n",
				B_plus);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

		//data = NULL; //cleaning, isto vai fazer eu não conseguir depois dar o memset do msg_slave porque perco o ponteiro do msg_slave, não é muito boa pratica fazer isto acho eu

		//clear the buffer
		memset(msg_slave, 0, SlaveRx_Buf_size);
		msg_slave_len = 0;
	}

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

			/* Compute the total value of the pack in tension*/
			if(i < 15){
				HV_plus += decimalValue;
			}

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

			/* Compute the total value of the pack in tension*/
			if(i < 15){
				HV_plus += decimalValue;
			}

			startIndex += 2;
		}
	}

	return decimalValues;
}

void balancing_setup() {

	//FIXME: Alterei isto, mudei para 0 no primeiro argumento
	WriteReg(0, 0x13, 0x10, 1, FRMWRT_ALL_NR); // max 1s of balancing before resetting

}

//Turn on the balancing circuit
void balancing(){

	//Falta muito mais do que isto, é preciso fazer todas as comparações
	/*
	WriteReg(0, 19, 0x0008, 1, FRMWRT_ALL_NR);		// configuring cell balancing (0000 1000)
	WriteReg(0, 30, 0x0000, 1, FRMWRT_ALL_NR);  	// disabling squeeze function TSTConfig to enable cell balancing  (0000 0000 0000 0000)
	WriteReg(0, 3, 0x00, 2, FRMWRT_ALL_NR);  		//channel select to enable MODULESEL (not needed...set above)
	WriteReg(0, 20, 0x7FFF, 2, FRMWRT_ALL_NR);

	WriteReg(3, 14, 0x00, 1, FRMWRT_SGL_NR);
	*/

	//TODO: porque tem duas variaveis iguais? dw_data e balancing?
	//FIXME: tem duas variaveis iguais, não percebo porque?
	uint16_t dw_data_slave_1 = 0x0000;
	uint16_t dw_data_slave_2 = 0x0000;

	for (uint8_t i = 0; i < NUM_CELLS_SLAVE; i++) {

	  if ((balancing_slave1 & (0x0001 << i)) == 0x0000) {
		  // if not balancing check v is above TARGET_VOLT
		  if (v_slave1[i] > TARGET_VOLT) {
			  dw_data_slave_1 |= 0x0001 << i;
			  balancing_slave1 |= 0x0001 << i;
		  }
	  }
	  else {
		  // if balancing check if v is already equal or below the TARGET_VOLT
		  if (v_slave1[i] <= TARGET_VOLT) {
			  dw_data_slave_1 &= ~(0x0001 << i);
			  balancing_slave1 &= ~(0x0001 << i);
		  }
	  }

	  if ((balancing_slave2 & (0x0001 << i)) == 0x0000) {
		  // if not balancing check v is above TARGET_VOLT
		  if (v_slave2[i] > TARGET_VOLT) {
			  dw_data_slave_2 |= 0x0001 << i;
			  balancing_slave2 |= 0x0001 << i;
		  }
	  }
	  else {
		  // if balancing check if v is already equal or below the TARGET_VOLT
		  if (v_slave2[i] <= TARGET_VOLT) {
			  dw_data_slave_2 &= ~(0x0001 << i);
			  balancing_slave2 &= ~(0x0001 << i);
		  }
	  }

	}

	msg_debug_len = sprintf(msg_debug,
			"-------------------------------------------------------------------------\n\n"
			"[BALANCING]\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

	msg_debug_len = sprintf(msg_debug,
			"\t[SLAVE 1]\n"
			"\t[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]\n"
			"\t %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n\n",
			(dw_data_slave_1 >> 14) & 0x1, (dw_data_slave_1 >> 13) & 0x1, (dw_data_slave_1 >> 12) & 0x1, (dw_data_slave_1 >> 11) & 0x1, (dw_data_slave_1 >> 10) & 0x1,
			(dw_data_slave_1 >> 9) & 0x1, (dw_data_slave_1 >> 8) & 0x1, (dw_data_slave_1 >> 7) & 0x1, (dw_data_slave_1 >> 6) & 0x1, (dw_data_slave_1 >> 5) & 0x1,
			(dw_data_slave_1 >> 4) & 0x1, (dw_data_slave_1 >> 3) & 0x1, (dw_data_slave_1 >> 2) & 0x1, (dw_data_slave_1 >> 1) & 0x1, (dw_data_slave_1 >> 0) & 0x1);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);

	msg_debug_len = sprintf(msg_debug,
			"\t[SLAVE 2]\n"
			"\t[16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]\n"
			"\t %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n\n",
			(dw_data_slave_2 >> 14) & 0x1, (dw_data_slave_2 >> 13) & 0x1, (dw_data_slave_2 >> 12) & 0x1, (dw_data_slave_2 >> 11) & 0x1, (dw_data_slave_2 >> 10) & 0x1,
			(dw_data_slave_2 >> 9) & 0x1, (dw_data_slave_2 >> 8) & 0x1, (dw_data_slave_2 >> 7) & 0x1, (dw_data_slave_2 >> 6) & 0x1, (dw_data_slave_2 >> 5) & 0x1,
			(dw_data_slave_2 >> 4) & 0x1, (dw_data_slave_2 >> 3) & 0x1, (dw_data_slave_2 >> 2) & 0x1, (dw_data_slave_2 >> 1) & 0x1, (dw_data_slave_2 >> 0) & 0x1);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);
	// Slave 1
	WriteReg(0, 0x14, dw_data_slave_1, 2, FRMWRT_SGL_NR);

	// Slave 2
	WriteReg(1, 0x14, dw_data_slave_2, 2, FRMWRT_SGL_NR);

}


float hexToFloat(uint16_t value) {
	//float factor = 13107.38791552861;
	float factor_inverse = 7.62928515158445919e-5;
    return (float)value * factor_inverse;
}


