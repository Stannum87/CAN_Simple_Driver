- Simple CAN driver that can be implemented easily.
- CAN ic: MCP2515 along with tja1050 for driving can bus.
- 8 bytes of data can be sent at a time with id of 11 bits(standard mode)
- use 120 ohm parallel termination between CANH and CANL for optimal operation
- can speed fixed to 125kbps for simplicity
- resoures that helped me develop this driver: 
1. http://www.kreatives-chaos.com/artikel/ansteuerung-eines-mcp2515
2. https://kvaser.com/support/calculators/bit-timing-calculator/
