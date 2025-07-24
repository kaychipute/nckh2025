#include "mc522.h"

GPIO_TypeDef *MOSI_PORT;
uint16_t 	    MOSI_PIN;
GPIO_TypeDef *MISO_PORT;
uint16_t      MISO_PIN;
GPIO_TypeDef *SCK_PORT;
uint16_t 	    SCK_PIN;
GPIO_TypeDef *CS_PORT;
uint16_t      CS_PIN;


static void SW_SPI_init(void) //khoi tao vao ra cho GPIO
{
	GPIO_InitTypeDef GPIO_InitStruct;            // cho MOSI la out
        GPIO_InitStruct.Pin = MOSI_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(MOSI_PORT, &GPIO_InitStruct);
	
        GPIO_InitStruct.Pin = CS_PIN;              // cho CS la out
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(CS_PORT, &GPIO_InitStruct);
	
        GPIO_InitStruct.Pin = SCK_PIN;              // cho SCK la out
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SCK_PORT, &GPIO_InitStruct);
                
        GPIO_InitStruct.Pin = MISO_PIN;           //cho MISO la in
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(MISO_PORT, &GPIO_InitStruct);
}
unsigned char spi(unsigned char byte)
{
	unsigned char rv=0;
	for(int i=0;i<8;i++)
	{
		if((byte & (0x80 >>i)) != 0)HAL_GPIO_WritePin(MOSI_PORT,MOSI_PIN,GPIO_PIN_SET); 
		else HAL_GPIO_WritePin(MOSI_PORT,MOSI_PIN,GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(SCK_PORT,SCK_PIN,GPIO_PIN_SET);      
		rv<<=1;
		if(HAL_GPIO_ReadPin(MISO_PORT,MISO_PIN) != 0)rv|=0x01;
		HAL_GPIO_WritePin(SCK_PORT,SCK_PIN,GPIO_PIN_RESET);      
	}
	return rv;
}
unsigned char spi_transmit(unsigned char data)
{
	uint8_t datRV[1];
	datRV[0] = spi(data);
	return datRV[0];
}
///////////////////////////////////////////////////////////////////////////////
void RFID_write(unsigned char reg, unsigned char data)
{ 
	MFRC522_CS_EN;
	spi_transmit((reg<<1)&0x7E);
	spi_transmit(data); 
	MFRC522_CS_DIS;
}
///////////////////////////////////////////////////////////////////////////////
unsigned char RFID_read(unsigned char reg)
        {
        unsigned char data; 
        MFRC522_CS_EN;
        spi_transmit(((reg<<1)&0x7E)|0x80);
        data = spi_transmit(0x00); 
        MFRC522_CS_DIS;
        return data;
        } 
///////////////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)  
        {
        unsigned char tmp = 0x0;
        tmp = RFID_read(reg);
        RFID_write(reg,tmp | mask);  
        }
///////////////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask)  
        {
        unsigned char tmp = 0x0;
        tmp = RFID_read(reg);
        RFID_write(reg, tmp & ~mask);  
        } 
///////////////////////////////////////////////////////////////////////////////
unsigned char MFRC522_ComMF522(unsigned char  Command, 
                               unsigned char  *pInData, 
                               unsigned char  InLenByte,
                               unsigned char  *pOutData, 
                               unsigned int *pOutLenBit)
        {
        unsigned char status = 2;
        unsigned char irqEn   = 0x00;
        unsigned char waitFor = 0x00;
        unsigned char lastBits;
        unsigned char n;
        unsigned int i;
        switch (Command)
                {
                case 0x0E: irqEn   = 0x12; waitFor = 0x10; break;
                case 0x0C: irqEn   = 0x77; waitFor = 0x30; break;
                default:                                   break;
                } 
        RFID_write(0x02,irqEn|0x80);
        ClearBitMask(0x04,0x80);
        RFID_write(0x01,0x00);
        SetBitMask(0x0A,0x80); 
        for (i=0; i<InLenByte; i++)
                {   
                RFID_write(0x09, pInData[i]);
                }
        RFID_write(0x01, Command);
        if (Command == 0x0C)
                {    
                SetBitMask(0x0D,0x80);  
                }
        i = 600; 
        do 
                {
                n = RFID_read(0x04);
                i--;
                }
        while ((i!=0) && !(n&0x01) && !(n&waitFor));
        ClearBitMask(0x0D,0x80); 
        if (i!=0)
                {    
                if(!(RFID_read(0x06)&0x1B))
                        {
                        status = 0;
                        if (n & irqEn & 0x01)
                                {   
                                status = 1;   
                                }
                        if (Command == 0x0C)
                                {
                                n = RFID_read(0x0A);
                                lastBits = RFID_read(0x0C) & 0x07;
                                if (lastBits)
                                        {   
                                        *pOutLenBit = (n-1)*8 + lastBits;  
                                        }
                                else
                                        {   
                                        *pOutLenBit = n*8;   
                                        }
                                if (n == 0)
                                        {   
                                        n = 1;    
                                        }
                                        if (n > MAXRLEN)
                                        {   
                                        n = MAXRLEN;   
                                        }
                                for (i=0; i<n; i++)
                                        {   
                                        pOutData[i] = RFID_read(0x09);    
                                        }
                                }
                        }
                else
                        {   
                        status = 2;   
                        }      
                }
        SetBitMask(0x0C,0x80);      
        RFID_write(0x01,0x00); 
        return status;
        }
///////////////////////////////////////////////////////////////////////////////
unsigned char MFRC522_Request(unsigned char req_code,unsigned char *pTagType)
        {
        unsigned char status=2;  
        unsigned int  Length;
        unsigned char Buffer[MAXRLEN];  
        ClearBitMask(0x08,0x08);
        RFID_write(0x0D,0x07);
        SetBitMask(0x14,0x03); 
        Buffer[0] = req_code;
        status = MFRC522_ComMF522(0x0C,Buffer,1,Buffer,&Length);
        if ((status == 0) && (Length == 0x10))
                {    
                *pTagType     = Buffer[0];
                *(pTagType+1) = Buffer[1];
                }
        else
                {   
                status = 2;   
                }
        return status;
        }  
/////////////////////////////////////////////////////////////////////////////// 
unsigned char MFRC522_Anticoll(unsigned char *pSnr)
        {
        unsigned char status;
        unsigned char i,snr_check=0;
        unsigned int  Length;
        unsigned char Buffer[MAXRLEN]; 
        ClearBitMask(0x08,0x08);
        RFID_write(0x0D,0x00);
        ClearBitMask(0x0E,0x80); 
        Buffer[0] = 0x93;
        Buffer[1] = 0x20; 
        status = MFRC522_ComMF522(0x0C,Buffer,2,Buffer,&Length); 
        if (status == 0)
                {
                for (i=0; i<5; i++)
                        {   
                        *(pSnr+i)  = Buffer[i];
                        snr_check ^= Buffer[i];
                        }
                if (snr_check != Buffer[i])
                        {   
                        status = 2;    
                        }
                } 
        SetBitMask(0x0E,0x80);
        return status;
        }
///////////////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
        {
        unsigned char i,n;
        ClearBitMask(0x05,0x04);
        RFID_write(0x01,0x00);
        SetBitMask(0x0A,0x80);
        for (i=0; i<len; i++)
                {   
                RFID_write(0x09, *(pIndata+i));   
                }
        RFID_write(0x01, 0x03);
        i = 0xFF;
        do 
                {
                n = RFID_read(0x05);
                i--;
                }
        while ((i!=0) && !(n&0x04));
        pOutData[0] = RFID_read(0x22);
        pOutData[1] = RFID_read(0x21);
        }
///////////////////////////////////////////////////////////////////////////////
unsigned char MFRC522_Halt(void)
        { 
        unsigned char Buffer[MAXRLEN];  
        Buffer[0] = 0x50;
        Buffer[1] = 0;
        CalulateCRC(Buffer,2,&Buffer[2]);  
        return 0; 
        } 
///////////////////////////////////////////////////////////////////////////////
void MFRC522_AntennaOn(void)
        {
        unsigned char i;
        i = RFID_read(0x14);
        if (!(i & 0x03))
                {
                SetBitMask(0x14, 0x03);
                }
        }
///////////////////////////////////////////////////////////////////////////////
void MFRC522_AntennaOff(void)
        {
        ClearBitMask(0x14, 0x03);
        } 
void MFRC522_Reset(void)
        { 
//        MFRC522_RST_DIS;
//        HAL_Delay (1);
//        MFRC522_RST_EN;
//        HAL_Delay (1);
//        MFRC522_RST_DIS;
//        HAL_Delay (1);
        RFID_write( 0x01, 0x0F ); 
        HAL_Delay (1);
        } 				
unsigned char MFRC522_init(
GPIO_TypeDef *_MO_PORT,uint16_t _MO_pin,   //nap vao 4 chan dieu khien
GPIO_TypeDef *_MI_PORT,uint16_t _MI_pin,
GPIO_TypeDef *_SCK_PORT,uint16_t _SCK_pin,
GPIO_TypeDef *_CS_PORT ,uint16_t _CS_pin)
{       
	      MOSI_PORT=_MO_PORT;
				MISO_PORT=_MI_PORT;
				SCK_PORT=_SCK_PORT;
				CS_PORT=_CS_PORT;
	   
	      MOSI_PIN = _MO_pin;
	      MISO_PIN = _MI_pin;
	      SCK_PIN = _SCK_pin;
	      CS_PIN = _CS_pin;
	
        SW_SPI_init();					
        MFRC522_CS_DIS;
				MFRC522_Reset(); 
        HAL_Delay(1); 
        RFID_write(0x11,0x3D);        
        RFID_write(0x2D,30);           
        RFID_write(0x2C,0);
        RFID_write(0x2A,0x8D);
        RFID_write(0x2B,0x3E);
        RFID_write(0x15,0x40); 
        MFRC522_AntennaOff(); 
        MFRC522_AntennaOn();  
        HAL_Delay(200); 
        return 0;
}
///////////////////////////////////////////////////////////////////////////////
unsigned char isCard(void) 
        {
        unsigned char status;
        unsigned char id[16];
        status = MFRC522_Request(0x52, id); 
        if(status == 0) 
                {
                return 1;
                } 
        else  
                { 
                return 0; 
                }
        }  
///////////////////////////////////////////////////////////////////////////////
unsigned char readCardSerial(uint8_t *id)
{ 
	unsigned char status;
	unsigned char str[16]; 
	status = MFRC522_Anticoll(str);
	memcpy(id, str, 5); 
	if (status == 0) 
					{ 
						return 1; 
					} 
	else 
					{ 
						return 0; 
					}
} 

//end file
