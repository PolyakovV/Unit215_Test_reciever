/************************************************************************************/
/*    Manufacture: OLIMEX                              	                            */
/*	  COPYRIGHT (C) 2011							    */
/*    Designed by:  Penko Todorov Bozhkov	        		            */
/*    Module Name:  USART		            		                    */
/*    File   Name:  usart.h					                    */
/*    Revision Identifier:	Rev.A						    */
/*		-> Initial:	Writed RAM related functions.                       */
/*		-> Rev.A:	Added ROM related functions.(10.03.2011)            */
/************************************************************************************/
#ifndef __USART_H
#define __USART_H

//#define RAM_Functions_Enabled		0x0F
#define ROM_Functions_Enabled		0xF0

/***** 1.All functions prototypes *****/
void USART_Init(void);
void USART_Send_Data(unsigned char Data);



// RAM related functions:
void USART_Send_RAM_String(char *String_pointer);
void USART_Send_RAM_Menu_Begin(void);
void USART_Send_RAM_Menu_Restore(void);
// ROM related functions
void USART_Send_ROM_String(const char *String_pointer);
void USART_Send_ROM_Menu_Begin(void);
void USART_Send_ROM_Menu_Restore(void);


/***** 2.All definitions *****/
#define USART_BUSY              ( UCA0STAT & UCBUSY )
//#define USART_BUSY              (!(UCA0IFG&UCTXIFG))
#define USART_TxData_Reg        UCA0TXBUF
#define USART_RxData_Reg        UCA0RXBUF
#define fUSART_Receive_Set      USART_Flags_Register |= 0x01;
#define fUSART_Receive_Clear    USART_Flags_Register &= ~0x01;
#define fUSART_Receive_Check    (USART_Flags_Register & 0x01)

//Other external definitions
extern volatile unsigned char USART_Received_Data;
#endif //__USART_H
/***** 3.All variables *****/

extern volatile unsigned char USART_Flags_Register;


extern char USART_Received_Data_Buff[6];
extern unsigned char Need_check_UART_Buff;
/////////////////////////////////////////////////////////////////////