#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "CAN_Lib_Headers.h"



#define SPI_Reset 0xC0
#define	SPI_BIT_MODIFY 0x05
#define	SPI_RTS 0x80
#define	SPI_RX_STATUS 0xB0
#define	SPI_READ_RX 0x90

#define RTR 6

#define INSTRUCTION_Read  0b00000011
#define INSTRUCTION_Write 0b00000010

#define ADDRESS_RXB0CTRL 0x60
#define ADDRESS_RXB1CTRL 0x70

#define ADDRESS_RXM0SIDH 0x20
#define ADDRESS_RXM0SIDL 0x21
#define ADDRESS_RXM0EID8 0x22
#define ADDRESS_RXM0EID0 0x23
#define ADDRESS_RXM1SIDH 0x24
#define ADDRESS_RXM1SIDL 0x25
#define ADDRESS_RXM1EID8 0x26
#define ADDRESS_RXM1EID0 0x27
#define ADDRESS_BFPCTRL 0xC0
#define ADDRESS_TXRTSCTRL 0x0D
#define ADDRESS_RXB0SIDL 0x62
#define ADDRESS_RXB0Dm 0x66

#define ADDRESS_TXB0CTRL 0x30
#define ADDRESS_TXB0SIDH 0x31
#define ADDRESS_TXB0SIDL 0x32
#define ADDRESS_TXB0DLC 0x35
#define ADDRESS_TXB0D0 0x36
#define ADDRESS_TXB0D1 0x31

#define ADDRESS_CNF1 0x2A
#define ADDRESS_CNF2 0x29
#define ADDRESS_CNF3 0x28

#define ADDRESS_CANCTRL 0x0F
#define ADDRESS_CANINTE 0x2B
#define ADDRESS_CANINTF 0x2C




	


void SPI_transmit(int data)
{
	// load data into register
	SPDR = data;

	// Wait for transmission complete
	while(!(SPSR & (1 << SPIF)));
}

int SPI_Receive()
{
	// transmit dummy byte
	SPDR = 0xFF;

	// Wait for reception complete
	while(!(SPSR & (1 << SPIF)));

	// return Data Register
	return SPDR;
}

void SPI_CAN_Write(int Addr, int Data, uint8_t Device)
{
	if (Device == 0)
	{
		PORTB &= ~(1<<PINB4);
		SPI_transmit(INSTRUCTION_Write);
		SPI_transmit(Addr);
		SPI_transmit(Data);
		PORTB |= 1<<PINB4;
	}
	
	else if (Device == 1)
	{
		PORTB &= ~(1<<PINB3);
		SPI_transmit(INSTRUCTION_Write);
		SPI_transmit(Addr);
		SPI_transmit(Data);
		PORTB |= 1<<PINB3;
	}
}

int SPI_CAN_Read(int Addr, uint8_t Device)
{
	int SPI_res = 0;
	
	if (Device == 0)
	{
		PORTB &= ~(1<<PINB4);
		SPI_transmit(INSTRUCTION_Read);
		SPI_transmit(Addr);
		SPI_res = SPI_Receive();
		PORTB |= 1<<PINB4;
		
	//	return SPI_res;
	}
	
	else if (Device == 1)
	{
		PORTB &= ~(1<<PINB3);
		SPI_transmit(INSTRUCTION_Read);
		SPI_transmit(Addr);
		SPI_res = SPI_Receive();
		PORTB |= 1<<PINB3;
		
	//	return SPI_res;
	}
	return SPI_res;
}

void SPI_CAN_Bit_Modify(int Addr, int Mask, int Data, uint8_t Device)
{
		if (Device == 0)
		{
			PORTB &= ~(1<<PINB4);
			SPI_transmit(SPI_BIT_MODIFY);
			SPI_transmit(Addr);
			SPI_transmit(Mask);
			SPI_transmit(Data);
			PORTB |= 1<<PINB4;
		}
		
		else if (Device == 1)
		{
			PORTB &= ~(1<<PINB3);
			SPI_transmit(SPI_BIT_MODIFY);
			SPI_transmit(Addr);
			SPI_transmit(Mask);
			SPI_transmit(Data);
			PORTB |= 1<<PINB3;
		}
}

void Init_SPI()
{
	DDRB = (1<<PINB7) | (1<<PINB5) | (1<< PINB4) | (1<<PINB3);
	DDRB &= ~(1<<PINB6);
	DDRD = 0x00;
	//	DDRA = 0x00;

	PORTB |= 1<<PINB4;
	PORTB |= 1<<PINB3;
	
	SPCR |= (1<<MSTR) | 1<<SPR0 | 1<<SPR1;
	
	SPCR |= 1<<SPE;
}

void CAN_Init0()
{
	// SPI RESET
	PORTB &= ~(1<<PINB4);
	SPI_transmit(SPI_Reset);
	_delay_ms(1);
	PORTB |= 1<<PINB4;
	_delay_ms(10);
	    
	// bit rate config.
	SPI_CAN_Write(ADDRESS_CNF1, 0x03, 0);
	SPI_CAN_Write(ADDRESS_CNF2, 0x91, 0);
	SPI_CAN_Write(ADDRESS_CNF3, 0x01, 0);
	
	//Enable receiver interrupt 
	SPI_CAN_Write(ADDRESS_CANINTE, 0x03, 0);
	
	// set filter, no filters
	SPI_CAN_Write(ADDRESS_RXB0CTRL, 0x60, 0);
	SPI_CAN_Write(ADDRESS_RXB1CTRL, 0x60, 0);
	
	// Receive Mask - clearing all bits
			// receive buffer 0
	SPI_CAN_Write(ADDRESS_RXM0SIDH,0,0);
	SPI_CAN_Write(ADDRESS_RXM0SIDL,0,0);
	SPI_CAN_Write(ADDRESS_RXM0EID8,0,0);
	SPI_CAN_Write(ADDRESS_RXM0EID0,0,0);
			// receive buffer 1
	SPI_CAN_Write(ADDRESS_RXM1SIDH,0,0);
	SPI_CAN_Write(ADDRESS_RXM1SIDL,0,0);
	SPI_CAN_Write(ADDRESS_RXM1EID8,0,0);
	SPI_CAN_Write(ADDRESS_RXM1EID0,0,0);
	
	//buffer control and status register - setting all 0
	SPI_CAN_Write(ADDRESS_BFPCTRL,0,0);
	
	// setting input pins - disabled
	SPI_CAN_Write(ADDRESS_TXRTSCTRL,0,0);
	
	//setting operation mode- normal
	SPI_CAN_Bit_Modify(ADDRESS_CANCTRL,0xE0,0,0);
}

void CAN_Init1()
{
	// SPI RESET
	PORTB &= ~(1<<PINB3);
	SPI_transmit(SPI_Reset);
	_delay_ms(1);
	PORTB |= 1<<PINB3;
	_delay_ms(10);
	
	// bit rate config.
	SPI_CAN_Write(ADDRESS_CNF1, 0x03, 1);
	SPI_CAN_Write(ADDRESS_CNF2, 0x91, 1);
	SPI_CAN_Write(ADDRESS_CNF3, 0x01, 1);
	
	//Enable receiver interrupt
	SPI_CAN_Write(ADDRESS_CANINTE, 0x03, 1);
	
	// set filter, no filters
	SPI_CAN_Write(ADDRESS_RXB0CTRL, 0x60, 1);
	SPI_CAN_Write(ADDRESS_RXB1CTRL, 0x60, 1);
	
	// Receive Mask - clearing all bits
		// receive buffer 0
	SPI_CAN_Write(ADDRESS_RXM0SIDH,0,1);
	SPI_CAN_Write(ADDRESS_RXM0SIDL,0,1);
	SPI_CAN_Write(ADDRESS_RXM0EID8,0,1);
	SPI_CAN_Write(ADDRESS_RXM0EID0,0,1);
		// receive buffer 1
	SPI_CAN_Write(ADDRESS_RXM1SIDH,0,1);
	SPI_CAN_Write(ADDRESS_RXM1SIDL,0,1);
	SPI_CAN_Write(ADDRESS_RXM1EID8,0,1);
	SPI_CAN_Write(ADDRESS_RXM1EID0,0,1);
	
	//buffer control and status register - setting all 0
	SPI_CAN_Write(ADDRESS_BFPCTRL,0,1);
	
	// setting input pins as disabled
	SPI_CAN_Write(ADDRESS_TXRTSCTRL,0,1);
	
	//setting operation mode- normal
	SPI_CAN_Bit_Modify(ADDRESS_CANCTRL,0xE0,0,1);
}

//int D0 = 0xaa;

// void CAN_TX_send()
// {
// 	// for TX
// 		// Create new TX message
// 
// 	
// 	SPI_CAN_Bit_Modify(ADDRESS_TXB0CTRL, 0x03, 0x03, 0); // setting highest priority
// 	
// 	SPI_CAN_Write(ADDRESS_TXB0SIDH, ( uint8_t ) ( id >> 3 ), 0); // setting id H byte
// 	SPI_CAN_Write(ADDRESS_TXB0SIDL, ( uint8_t ) ( id << 5 ), 0); // setting id L byte
// 	
// 	SPI_CAN_Write(ADDRESS_TXB0DLC, 2, 0); // data length
// 	
// 	SPI_CAN_Write(ADDRESS_TXB0D0, D0, 0); // data H byte
// 	SPI_CAN_Write(ADDRESS_TXB0D1, 0xA5, 0); // data L byte
// 	    
// 	// enable send CAN data
// 	PORTB &= ~(1<<PINB4);
// 	SPI_transmit(SPI_RTS | 0x01);
// 	PORTB |= 1<<PINB4;
// }

uint8_t CAN_RX_Status()
{
	// reading receive status to check if received 
	uint8_t RX_Data = 0;
	
	PORTB &= ~(1<<PINB3);
	SPI_transmit(SPI_RX_STATUS);
	RX_Data = SPI_Receive();
	SPI_Receive();
	PORTB |= 1<<PINB3;
	
	return RX_Data;
}


void CAN_TX_Load(CANMessage *P_message0_TX) // this function is used for sending single packet(8 bytes) mode and its more covenient than sending multiple packets at a time 
{	

			
//	uint16_t id = 0x01;
	
	uint8_t length = (uint8_t) P_message0_TX -> length;
	
	SPI_CAN_Bit_Modify(ADDRESS_TXB0CTRL, 0x03, 0x03, 0); // setting highest priority
	
	SPI_CAN_Write(ADDRESS_TXB0SIDH, ( uint8_t ) (P_message0_TX -> id >> 3 ), 0); // setting id H byte
	SPI_CAN_Write(ADDRESS_TXB0SIDL, ( uint8_t ) (P_message0_TX -> id << 5 ), 0); // setting id L byte
	
	// the message is a "Remote Transmit Request" - meaning telling other CAN modules in bus to send or requesting others to send
    if ( P_message0_TX->rtr )
    {
        /* An RTR message has a length,
           but no data */
       
        // Set message length + RTR - basically sending RTR without data
        SPI_CAN_Write(ADDRESS_TXB0DLC, ( 1 <<RTR ) | length, 0);
    }
	
    else
    {
        // Set message length, data is sent without any kind of RTR in the packet
        SPI_CAN_Write(ADDRESS_TXB0DLC, length, 0);
	
	//P_message0_TX->data [0]++;
	/*	SPI_CAN_Write(ADDRESS_TXB0DLC, ( uint8_t ) (P_message0_TX -> length), 0); // data length*/
		for (int i = 0; i<length; i++)
		{
		//	SPI_transmit(P_message0_TX -> data[i]);
			SPI_CAN_Write(ADDRESS_TXB0D0 + i, ( uint8_t ) P_message0_TX -> data[i], 0); // sending data in a for loop by incrementing address and data byte 
		}
	
//	SPI_CAN_Write(ADDRESS_TXB0D0, ( uint8_t ) P_message0_TX -> data[0], 0);
		
// 	SPI_CAN_Write(ADDRESS_TXB0D0, ( uint8_t ) P_message0_TX -> data[0], 0); // data0 byte
// 	SPI_CAN_Write(ADDRESS_TXB0D1, ( uint8_t ) P_message0_TX -> data[1], 0); // data1 byte
	}
	
	// enable send CAN data
	PORTB &= ~(1<<PINB4);
	SPI_transmit(SPI_RTS | 0x01);
	PORTB |= 1<<PINB4;
	
	
}

// used for second slave devive in spi
void CAN_TX_Load_D1(CANMessage *P_message0_TX_D1) // this function is used for sending single packet(8 bytes) mode and its more covenient than sending multiple packets at a time 
{	

			
//	uint16_t id = 0x01;
	
	uint8_t length = (uint8_t) P_message0_TX_D1 -> length;
	
	SPI_CAN_Bit_Modify(ADDRESS_TXB0CTRL, 0x03, 0x03, 1); // setting highest priority
	
	SPI_CAN_Write(ADDRESS_TXB0SIDH, ( uint8_t ) (P_message0_TX_D1 -> id >> 3 ), 1); // setting id H byte
	SPI_CAN_Write(ADDRESS_TXB0SIDL, ( uint8_t ) (P_message0_TX_D1 -> id << 5 ), 1); // setting id L byte
	
	// the message is a "Remote Transmit Request" - meaning telling other CAN modules in bus to send or requesting others to send
    if ( P_message0_TX_D1->rtr )
    {
        /* An RTR message has a length,
           but no data */
       
        // Set message length + RTR - basically sending RTR without data
        SPI_CAN_Write(ADDRESS_TXB0DLC, ( 1 <<RTR ) | length, 1);
    }
	
    else
    {
        // Set message length, data is sent without any kind of RTR in the packet
        SPI_CAN_Write(ADDRESS_TXB0DLC, length, 0);
	
	//P_message0_TX->data [0]++;
	/*	SPI_CAN_Write(ADDRESS_TXB0DLC, ( uint8_t ) (P_message0_TX -> length), 0); // data length*/
		for (int i = 0; i<length; i++)
		{
		//	SPI_transmit(P_message0_TX -> data[i]);
			SPI_CAN_Write(ADDRESS_TXB0D0 + i, ( uint8_t ) P_message0_TX_D1 -> data[i], 1); // sending data in a for loop by incrementing address and data byte 
		}
	
//	SPI_CAN_Write(ADDRESS_TXB0D0, ( uint8_t ) P_message0_TX -> data[0], 0);
		
// 	SPI_CAN_Write(ADDRESS_TXB0D0, ( uint8_t ) P_message0_TX -> data[0], 0); // data0 byte
// 	SPI_CAN_Write(ADDRESS_TXB0D1, ( uint8_t ) P_message0_TX -> data[1], 0); // data1 byte
	}
	
	// enable send CAN data
	PORTB &= ~(1<<PINB3);
	SPI_transmit(SPI_RTS | 0x01);
	PORTB |= 1<<PINB3;
	
	
}


uint8_t CAN_RX_Unload(CANMessage *P_message0_RX)
{

	
	uint8_t status = 0;
	if (bit_is_clear(PINB,2))
	{
		//	PORTA |= 1<<PINB0;
		status = CAN_RX_Status();
		if(bit_is_set(status,6)) // for rx buffer 0
		{
			PORTB &= ~(1<<PINB3);
			SPI_transmit(SPI_READ_RX);
		}
		
		else if(bit_is_clear(status,7)) // for rx buffer 1
		{
			PORTB &= ~(1<<PINB3);
			SPI_transmit(SPI_READ_RX | 0x04);
		}
		else
		{
			return 0xff; // nothing received
		}
		
		// Read standard ID
		P_message0_RX -> id =  ( uint16_t ) SPI_Receive() << 3; // high byte
		P_message0_RX -> id |= ( uint16_t ) SPI_Receive() >> 5; // Low byte, because they are 11 bits wide and remaining are discarded by shifting appropriatly
		
		SPI_transmit(0xff); // dummy
		SPI_transmit(0xff); // dummy, because we are not using extended CAN or CAN FD bit fields
		
		P_message0_RX -> length = ( uint8_t ) SPI_Receive() & 0x0f; // received length (DLC)
		
		P_message0_RX -> data[0] = ( uint8_t ) SPI_Receive(); // received Data byte 0
		P_message0_RX -> data[1] = ( uint8_t ) SPI_Receive(); // received Data byte 1
		P_message0_RX -> data[2] = ( uint8_t ) SPI_Receive();
		P_message0_RX -> data[3] = ( uint8_t ) SPI_Receive();
		P_message0_RX -> data[4] = ( uint8_t ) SPI_Receive();
		P_message0_RX -> data[5] = ( uint8_t ) SPI_Receive();
		P_message0_RX -> data[6] = ( uint8_t ) SPI_Receive();
		P_message0_RX -> data[7] = ( uint8_t ) SPI_Receive();
		
		PORTB |= 1<<PINB3;
		
		// to check if RTR is sent by other CAN transmitter
		if ( bit_is_set ( status, 3 ) )
		{
			P_message0_RX -> rtr = 1 ;
		}
		else
		{
			P_message0_RX -> rtr = 0 ;
		}
		
		// Clear interrupt flags
		if (bit_is_set (status, 6))
		{
			SPI_CAN_Bit_Modify(ADDRESS_CANINTF, 0x01, 0, 1); //
		}
		else
		{
			SPI_CAN_Bit_Modify(ADDRESS_CANINTF, 0x02, 0, 1); //
		}
	}
	return ( status & 0x07 );
}

// used for second slave devive in spi
uint8_t CAN_RX_Unload_D0(CANMessage *P_message0_RX_D0)
{

	
	uint8_t status = 0;
	if (bit_is_clear(PINB,1))
	{
		//	PORTA |= 1<<PINB0;
		status = CAN_RX_Status();
		if(bit_is_set(status,6)) // for rx buffer 0
		{
			PORTB &= ~(1<<PINB4);
			SPI_transmit(SPI_READ_RX);
		}
		
		else if(bit_is_clear(status,7)) // for rx buffer 1
		{
			PORTB &= ~(1<<PINB4);
			SPI_transmit(SPI_READ_RX | 0x04);
		}
		else
		{
			return 0xff; // nothing received
		}
		
		// Read standard ID
		P_message0_RX_D0 -> id =  ( uint16_t ) SPI_Receive() << 3; // high byte
		P_message0_RX_D0 -> id |= ( uint16_t ) SPI_Receive() >> 5; // Low byte, because they are 11 bits wide and remaining are discarded by shifting appropriatly
		
		SPI_transmit(0xff); // dummy
		SPI_transmit(0xff); // dummy, because we are not using extended CAN or CAN FD bit fields
		
		P_message0_RX_D0 -> length = ( uint8_t ) SPI_Receive() & 0x0f; // received length (DLC)
		
		P_message0_RX_D0 -> data[0] = ( uint8_t ) SPI_Receive(); // received Data byte 0
		P_message0_RX_D0 -> data[1] = ( uint8_t ) SPI_Receive(); // received Data byte 1
		P_message0_RX_D0 -> data[2] = ( uint8_t ) SPI_Receive();
		P_message0_RX_D0 -> data[3] = ( uint8_t ) SPI_Receive();
		P_message0_RX_D0 -> data[4] = ( uint8_t ) SPI_Receive();
		P_message0_RX_D0 -> data[5] = ( uint8_t ) SPI_Receive();
		P_message0_RX_D0 -> data[6] = ( uint8_t ) SPI_Receive();
		P_message0_RX_D0 -> data[7] = ( uint8_t ) SPI_Receive();
		
		PORTB |= 1<<PINB4;
		
		// to check if RTR is sent by other CAN transmitter
		if ( bit_is_set ( status, 3 ) )
		{
			P_message0_RX_D0 -> rtr = 1 ;
		}
		else
		{
			P_message0_RX_D0 -> rtr = 0 ;
		}
		
		// Clear interrupt flags
		if (bit_is_set (status, 6))
		{
			SPI_CAN_Bit_Modify(ADDRESS_CANINTF, 0x01, 0, 0); //
		}
		else
		{
			SPI_CAN_Bit_Modify(ADDRESS_CANINTF, 0x02, 0, 0); //
		}
	}
	return ( status & 0x07 );
}