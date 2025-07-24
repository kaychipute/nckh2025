#ifndef __MC522__H__
#define __MC522__H__
#include "main.h"
#include <string.h>

#define MFRC522_CS_EN   HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_RESET)
#define MFRC522_CS_DIS  HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_SET)

#define MAXRLEN           18  
#define CARD_FOUND        1
#define CARD_NOT_FOUND    2
#define ERROR             3 

static void SW_SPI_init(void);
unsigned char spi(unsigned char byte);
unsigned char spi_transmit(unsigned char data);
void RFID_write(unsigned char reg, unsigned char data);
unsigned char RFID_read(unsigned char reg);
void SetBitMask(unsigned char reg,unsigned char mask);
void ClearBitMask(unsigned char reg,unsigned char mask);
unsigned char MFRC522_ComMF522(unsigned char  Command, 
                               unsigned char  *pInData, 
                               unsigned char  InLenByte,
                               unsigned char  *pOutData, 
                               unsigned int *pOutLenBit);
unsigned char MFRC522_Request(unsigned char req_code,unsigned char *pTagType);
unsigned char MFRC522_Anticoll(unsigned char *pSnr);
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);
unsigned char MFRC522_Halt(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);															 
unsigned char MFRC522_init(
GPIO_TypeDef *_MO_PORT,uint16_t _MO_pin,  
GPIO_TypeDef *_MI_PORT,uint16_t _MI_pin,
GPIO_TypeDef *_SCK_PORT,uint16_t _SCK_pin,
GPIO_TypeDef *_CS_PORT ,uint16_t _CS_pin);
unsigned char isCard(void);
unsigned char readCardSerial(uint8_t *id);
															 
#endif
//end file
