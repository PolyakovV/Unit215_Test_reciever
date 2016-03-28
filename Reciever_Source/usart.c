/************************************************************************************/
/*    Manufacture: OLIMEX                                           	            */
/*	  COPYRIGHT (C) 2011							    */
/*    Designed by:  Penko Todorov Bozhkov	        		            */
/*    Module Name:  USART		            		                    */
/*    File   Name:  usart.c					                    */
/*    Revision Identifier:	Rev.A						    */
/*	  Revision History:							    */
/*		-> Initial:	Writed RAM related functions.                       */
/*		-> Rev.A:	Added ROM related functions.(10.03.2011)            */
/************************************************************************************/
#include <cc430x513x.h>
#include  <intrinsics.h>
#include "usart.h"

//Global Variables initialization:
unsigned char USART_Received_Data;
 
unsigned char pos=0;
char USART_Received_Data_Buff[6];
unsigned char Need_check_UART_Buff;
/****************************************************************************/
/*  Function name: USART_Init                                               */
/*  	Parameters                                                          */
/*          Input   :  No                                                   */
/*          Output  :  No	                                            */
/*	Action: Initialize USART operation mode and speed.		    */
/****************************************************************************/
// desired baud rate: 9600
void USART_Init(void)
{ 
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 6;                              // 1MHz 9600 (see User's Guide)
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
 
  P1DIR |= BIT6;                            // Set P1.6 as TX output
  P1SEL |= BIT5 + BIT6;                     // Select P1.5 & P1.6 to UART function
  
  UCA0CTL1 &= (~UCSWRST);                   // **Initialize USCI state machine**
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  UCA0IFG &= (~UCRXIFG);                    // Clear RX interrupt falg
  
  ///////////Re init for hight speed///////////////////
 UCA0MCTL = UCBRS_0 + UCBRF_3 + UCOS16; 
 UCA0BR0 = 103;                              // 1MHz 9600 (see User's Guide)
 UCA0BR1 = 0;

  
}


/****************************************************************************/
/*  Function name: USART_Send_Data                                          */
/*  	Parameters                                                          */
/*          Input   :  Data                                                 */
/*          Output  :  No                                                   */
/*	Action: Send one data byte to HyperTerminal via USART.              */
/****************************************************************************/
void USART_Send_Data(unsigned char Data){
  while(USART_BUSY);	// Check USCI busy flag
  USART_TxData_Reg = Data;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//##### Functions related with strings and so on placed in ROM(Flash) memory #####
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if defined(ROM_Functions_Enabled)

#define Menu_TableNumberRows	6

// 1.Put the strings in Program Space
const char row_0[] = ("\f\r*****************************************************");
const char row_1[] = ("\n\r*        Welcome to MSP430-CCRF Demo program!       *");
const char row_2[] = ("\n\r*****************************************************");
const char row_3[] = ("\n\r -> LED is toggled via Timer1 CC0 interrupt! ");
const char row_4[] = ("\n\r -> UART returns echo when a character is received! ");
const char row_5[] = ("\n\r\0");

// 2.Put "Menue_Table" in Program Space, where "Menue_Table" is an array of pointers to  
//   characters (strings), where each pointer is a pointer to the Program Space.
const char * Menu_Table[] = 
{
  row_0,
  row_1,
  row_2,
  row_3,
  row_4,
  row_5
};


/**********************************************************************************/
/*  Function name: USART_Send_ROM_String                                          */
/*  	Parameters                                                                */
/*          Input   :  String_pointer                                             */
/*          Output  :  No	                                                  */
/*	Action: Send string stored in Program Space to HyperTerminal via USART.	  */
/**********************************************************************************/
void USART_Send_ROM_String(const char *String_pointer){
  while((*String_pointer)){
    USART_Send_Data((*String_pointer));
    String_pointer++;	
  }	
}


/**********************************************************************************/
/*  Function name: USART_Send_ROM_Menu_Begin                                      */
/*  	Parameters                                                                */
/*          Input   :  No			                                  */
/*          Output  :  No	                                                  */
/*	Action: Send Menu stored in Program Space to HyperTerminal via USART.	  */
/**********************************************************************************/
void USART_Send_ROM_Menu_Begin(void){
  volatile unsigned char i=0;

  // String placed in ROM
  for(i=0;i<Menu_TableNumberRows;i++){
      USART_Send_ROM_String( (Menu_Table[i]) );
  }
}

#endif	//ROM_Functions_Enabled


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#####    Functions related with strings and so on placed in RAM memory     #####
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if defined(RAM_Functions_Enabled)

/**********************************************************************************/
/*  Function name: USART_Send_RAM_String                                          */
/*  	Parameters                                                                */
/*          Input   :  *String_pointer                                            */
/*          Output  :  No	                                                  */
/*	Action: Send string stored in RAM to HyperTerminal via USART.  		  */
/**********************************************************************************/
void USART_Send_RAM_String(char *String_pointer){
  while(*String_pointer){
    USART_Send_Data(*String_pointer);
    String_pointer++;	
  }	
}


/**********************************************************************************/
/*  Function name: USART_Send_RAM_Menu_Begin                                      */
/*  	Parameters                                                                */
/*          Input   :  No			                                  */
/*          Output  :  No	                                                  */
/*	Action: First TEST Menu sending to HyperTerminal.        		  */
/**********************************************************************************/
void USART_Send_RAM_Menu_Begin(void){
  // Write TEST Menu to HyperTerminal:
  USART_Send_RAM_String("\f\r*****************************************************");
  USART_Send_RAM_String("\n\r*        Welcome to MSP430-CCRF Demo program!       *");
  USART_Send_RAM_String("\n\r*****************************************************");
  USART_Send_RAM_String("\n\r -> LED is toggled via Timer1 CC0 interrupt! ");
  USART_Send_RAM_String("\n\r -> UART returns echo when receive a character! ");
  USART_Send_RAM_String("\n\r"); 
}

#endif	//RAM_Functions_Enabled



/**********************************************************************************/
/*      USART, Rx Complete interrupt service routine.                             */
/**********************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void UART_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (USART_BUSY);                     // USCI_A0 TX buffer ready?
    //uart has received a character in UDR
    //USART_Received_Data = USART_RxData_Reg; // Store received data
    //fUSART_Receive_Set;                     // Set received flag                  
    // Return Echo directly
    //USART_TxData_Reg = USART_RxData_Reg;    // i comment
     USART_Received_Data_Buff[pos] = USART_RxData_Reg;
     pos++;
    if (USART_RxData_Reg == 0x0A) {
                                    Need_check_UART_Buff = 0xFF;
                                    pos=0;
                                    }
    
    if (pos>5){pos=0;}
    
    break;
    
    
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
	
  

}






