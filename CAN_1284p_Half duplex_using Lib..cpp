#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "CAN_Lib_Headers.h"

// can speed fixed to 125kbps for simplicity


int main(void)
{
	DDRC = 0xff; // used for output of data received
	
	Init_SPI(); // initialize spi
	
	// enables SPI with its CS pin and initializes CAN.
	CAN_Init0();

	CANMessage message0_TX_D0;

	message0_TX_D0.id = 0x01;        //  In struct you should always declare variable inside a function otherwise it wont work
	message0_TX_D0.length = 0x08;
	message0_TX_D0.rtr = 0;				// some random values
	message0_TX_D0.data [0] = 0x01;
	message0_TX_D0.data [1] = 0x02;
	message0_TX_D0.data [2] = 0x04;
	message0_TX_D0.data [3] = 0x08;
	message0_TX_D0.data [4] = 16;
	message0_TX_D0.data [5] = 32;
	message0_TX_D0.data [6] = 64;
	message0_TX_D0.data [7] = 128;
	
	CANMessage message0_RX_D0;

	message0_RX_D0.id = 0x00;        //  In struct you should always declare variable inside a function otherwise it wont work
	message0_RX_D0.length = 0x00;
	message0_RX_D0.rtr = 0;
	message0_RX_D0.data [0] = 0x00;
	message0_RX_D0.data [1] = 0x00;
	message0_RX_D0.data [2] = 0x00;
	message0_RX_D0.data [3] = 0x00;
	message0_RX_D0.data [4] = 0x00;
	message0_RX_D0.data [5] = 0x00;
	message0_RX_D0.data [6] = 0x00;
	message0_RX_D0.data [7] = 0x00;	
	
	
    while (1) 
    {
		asm volatile ("NOP"); // blank time
	
	// CAN transmitt data
		CAN_TX_Load_D0(&message0_TX_D0); 
		
	//	_delay_ms(1);  // dont put this delay between TX and RX
		
	// CAN receive the data if available, and put it in RX struct
		CAN_RX_Unload_D0(&message0_RX_D0);
	 
	// display the data in the port
		PORTC = message0_RX_D0.data[x];
		
    }
	
}
