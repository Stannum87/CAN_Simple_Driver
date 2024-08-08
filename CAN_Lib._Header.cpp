
#ifndef CAN_LIB_HEADERS_H_
#define CAN_LIB_HEADERS_H_

#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>

typedef struct
{
	uint16_t   id;
	uint8_t    rtr;
	uint8_t    length;
	uint8_t    data [8];
} CANMessage;


void SPI_transmit(int data);

int SPI_Receive();

void SPI_CAN_Write(int Addr, int Data, uint8_t Device);

int SPI_CAN_Read(int Addr, uint8_t Device);

void SPI_CAN_Bit_Modify(int Addr, int Mask, int Data, uint8_t Device);

void Init_SPI();

void CAN_Init0();

void CAN_Init1();

uint8_t CAN_RX_Status();

void CAN_TX_Load(CANMessage *P_message0_TX);

void CAN_TX_Load_D1(CANMessage *P_message0_TX_D1);

uint8_t CAN_RX_Unload(CANMessage *P_message0_RX);

uint8_t CAN_RX_Unload_D0(CANMessage *P_message0_RX_D0);

#endif /* CAN_LIB_HEADERS_H_ */