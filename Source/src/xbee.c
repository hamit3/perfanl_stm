
#include "xbee.h"
#include "xb3_24.h"
#include <stdint.h>
#include <stdio.h>						//Standard input and output
#include <string.h>						//Various useful string functions
#include <stdlib.h>						//Various useful conversion functions

// Local Functions
void send_Msg(char*);
void sendEulerAnglesViaBL(char *data);

/* Send Message Function
 *
 * This function is passed a character array of an XBee API Packet, along with the
 * length of the packet. The checksum is calculated, then the start delimeter is transmitted,
 * followed by the length of the packet, then the data, and finally the checksum.
 */
void send_Msg(char *data)
{
	//Generate checksum
	uint32_t len = strlen(data);
	uint32_t data_len = len + 14;
	char checksum;
	uint32_t counter = 0; 
	uint32_t sum = 0;
	sum += 0x10;
	sum += (DestinationAddHigh&0xFF000000)>>24;
	sum += (DestinationAddHigh&0x00FF0000)>>16;
	sum += (DestinationAddHigh&0x0000FF00)>>8;
	sum += (DestinationAddHigh&0x000000FF);
	sum += (DestinationAddLow&0xFF000000)>>24;
	sum += (DestinationAddLow&0x00FF0000)>>16;
	sum += (DestinationAddLow&0x0000FF00)>>8;
	sum += (DestinationAddLow&0x000000FF);	
	sum += 0xFF;
	sum += 0xFE;
	
	for(counter = 0; counter <= len - 1; counter++)
		sum += data[counter];
	//Checksum is calculated by adding the data values together, and subtracting the 
	//last 8 bits from 0xFF.
	checksum = 0xFF - (sum & 0x00FF); 
	//Transmit data
	LPUART1_PutChar(0x7E);  //Start delimiter
	LPUART1_PutChar(8 >> data_len);  //Length MSB
	LPUART1_PutChar(data_len);	//Length LSB
	LPUART1_PutChar(0x10);  // Transmit Request
	LPUART1_PutChar(0x00);  // No Ack
	// MSB first, LSB last.
	LPUART1_PutChar((DestinationAddHigh&0xFF000000)>>24);  // 64 BIT ADDRESS MSB 0013A200 4180D683
	LPUART1_PutChar((DestinationAddHigh&0x00FF0000)>>16);
	LPUART1_PutChar((DestinationAddHigh&0x0000FF00)>>8);
	LPUART1_PutChar(DestinationAddHigh&0x000000FF);
	LPUART1_PutChar((DestinationAddLow&0xFF000000)>>24);  // 64 BIT ADDRESS LSB 
	LPUART1_PutChar((DestinationAddLow&0x00FF0000)>>16);
	LPUART1_PutChar((DestinationAddLow&0x0000FF00)>>8);
	LPUART1_PutChar(DestinationAddLow&0x000000FF);
	
	LPUART1_PutChar(0xFF);  // RESERVED
	LPUART1_PutChar(0xFE);  // RESERVED

	LPUART1_PutChar(0x00);  // MAX HOPS
	LPUART1_PutChar(0x00);  // Option Bit Fields ????
	
	
	for(counter = 0; counter <= len - 1; counter++)  //Transmit data
		LPUART1_PutChar(data[counter]);
	LPUART1_PutChar(checksum);  //Transmit checksum
}


/* Send Message Function
 *
 * This function is passed a character array of an XBee API Packet, along with the
 * length of the packet. The checksum is calculated, then the start delimeter is transmitted,
 * followed by the length of the packet, then the data, and finally the checksum.
 */
//void send_Msg(char *data)
//{
//	//Generate checksum
//	int len = strlen(data);
//	int data_len = len + 14;
//	char checksum;
//	int counter = 0, sum = 0x04DC;
//	for(counter = 0; counter <= len - 1; counter++)
//		sum += data[counter];
//	//Checksum is calculated by adding the data values together, and subtracting the 
//	//last 8 bits from 0xFF.
//	checksum = 0xFF - (sum & 0x00FF); 
//	//Transmit data
//	LPUART1_PutChar(0x7E);  //Start delimiter
//	LPUART1_PutChar(8 >> data_len);  //Length MSB
//	LPUART1_PutChar(data_len);	//Length LSB
//	LPUART1_PutChar(0x10);  // Transmit Request
//	LPUART1_PutChar(0x00);  // No Ack
//	// MSB first, LSB last.
//	LPUART1_PutChar(0x00);  // 64 BIT ADDRESS MSB
//	LPUART1_PutChar(0x13);
//	LPUART1_PutChar(0xA2);
//	LPUART1_PutChar(0x00);
//	LPUART1_PutChar(0x41);  // 64 BIT ADDRESS LSB 
//	LPUART1_PutChar(0x80);
//	LPUART1_PutChar(0xD6);
//	LPUART1_PutChar(0x83);
//	
//	LPUART1_PutChar(0xFF);  // RESERVED
//	LPUART1_PutChar(0xFE);  // RESERVED

//	LPUART1_PutChar(0x00);  // MAX HOPS
//	LPUART1_PutChar(0x00);  // Option Bit Fields ????
//	
//	
//	for(counter = 0; counter <= len - 1; counter++)  //Transmit data
//		LPUART1_PutChar(data[counter]);
//	LPUART1_PutChar(checksum);  //Transmit checksum
//}
