/****************************************************************************/
/* Reciver                                                                  */
/****************************************************************************/

// Target : CC430F5137
#include <cc430x513x.h>
#include  <intrinsics.h>
#include "HAL/RF1A.h"
#include "HAL/cc430x613x_PMM.h"
#include "HAL/HAL_FLASH.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "RF_Connection.h"
#include  "usart.h"
#include  "Valik_os.h"
//////////////////////////////////////////////////////////////////////////



void Init_RF(void); 
void delay(volatile unsigned long p);
void delay_1us(volatile unsigned long p);
void delay_ms(volatile unsigned long p);
void Delay50us(void);
void goToSleep(void);
//////////////////////////////////////
/****************************************************************************/
/*     LOCAL VARIABLES                                                      */
/****************************************************************************/
/////////////////EventID byte///////////////////////////////
#define TamperBit      BIT0
#define AlarmBit       BIT1
#define LowBatBit      BIT2
#define StatusBit      BIT3
#define RestoreBit     BIT4
#define SupervisionBit BIT5

void setFreq(double Freq);
void FreqCalc (char *arr, double Freq);
unsigned char rssi_dec;
int rssi_dBm;
unsigned char rssi_offset = 74;
  char * Flash_ptr;                         // Initialize Flash pointer
  char Flag_First_Time_Power_On;
  char FreqOffset[3];
  
  char sRX[5],sTmp[5];   // buffer for recognized data
  	unsigned char data_right;
	unsigned char j,k,l,m;
	int lCount,hCount,allCount;
        char already_send_flag=0;
/////////////////////////Trigger events (Preffix "TR_" )///////////////////////
#define TR_INIT                     1
#define TR_CHANGE_ANT               2
#define TR_TRANSMIT                 3
#define TR_UPD_FREQUENCY            4  
#define TR_TOOGLE_LED               5  
#define TR_UART_DATA_OUT            6
#define TR_CLEAR_FLAG               7
#define TR_SHOW_RSSI                8
#define TR_CHECK_RSSI               9
        
/////////////////////////Processes (Preffix "PR_" )////////////////////////////  
#define PR_SETUP_BY_UART            1
#define PR_READ_RSSI                2
#define PR_TESTING                  3
#define PR_RECOGN                   4
  
  
///////////////////////////////////////////////////////////////////////////////
#define FLASH_UNLOCK    FCTL3 = FWKEY; FCTL1 = FWKEY + WRT;
#define FLASH_LOCK      FCTL1 = FWKEY; FCTL3 = FWKEY + LOCK;
#define FLASH_ERASE     FCTL3 = FWKEY; FCTL1 = FWKEY + ERASE;

#define CPU_FREQ   26000000


//Definitions

#define LED_On			P3OUT |= BIT3;		P3DIR |= BIT3;
#define LED_Off		        P3OUT &= (~BIT3);	P3DIR |= BIT3;
#define LED_Togg		P3OUT ^= BIT3;		P3DIR |= BIT3;
#define LED_Chk		        (P3IN & BIT3)


#define RSSI_Hi			P1OUT |= BIT2;		P1DIR |= BIT2;
#define RSSI_Middle	        P1DIR &= (~BIT2);       P1OUT &= (~BIT2); P1REN &= (~BIT2); P1SEL  &= ~BIT2;
#define RSSI_Low	        P1OUT &= (~BIT2);	P1DIR |= BIT2;


#define CON_RX_1_Off		P2OUT |= BIT2;		P2DIR |= BIT2;
#define CON_RX_1_On	        P2OUT &= (~BIT2);	P2DIR |= BIT2;
#define CON_RX_1_Togg		P2OUT ^= BIT2;		P2DIR |= BIT2;
#define CON_RX_1_Chk            (P2IN & BIT2)


#define CON_RX_2_Off		P2OUT |= BIT3;		P2DIR |= BIT3;
#define CON_RX_2_On	        P2OUT &= (~BIT3);	P2DIR |= BIT3;
#define CON_RX_2_Togg		P2OUT ^= BIT3;		P2DIR |= BIT3;
#define CON_RX_2_Chk            (P2IN & BIT3)


#define CON_TX_1_Off		P2OUT |= BIT5;		P2DIR |= BIT5;
#define CON_TX_1_On	        P2OUT &= (~BIT5);	P2DIR |= BIT5;
#define CON_TX_1_Togg		P2OUT ^= BIT5;		P2DIR |= BIT5;
#define CON_TX_1_Chk            (P2IN & BIT5)


#define CON_TX_2_Off		P2OUT |= BIT4;		P2DIR |= BIT4; 
#define CON_TX_2_On	        P2OUT &= (~BIT4);	P2DIR |= BIT4;
#define CON_TX_2_Togg		P2OUT ^= BIT4;		P2DIR |= BIT4;
#define CON_TX_2_Chk            (P2IN & BIT4)



#define ANT_1_Off		P2OUT |= BIT0;		P2DIR |= BIT0;
#define ANT_1_On	        P2OUT &= (~BIT0);	P2DIR |= BIT0;
#define ANT_1_Togg		P2OUT ^= BIT0;		P2DIR |= BIT0;
#define ANT_1_Chk               (P2IN & BIT0)



#define ANT_2_Off		P2OUT |= BIT1;		P2DIR |= BIT1;
#define ANT_2_On	        P2OUT &= (~BIT1);	P2DIR |= BIT1;
#define ANT_2_Togg		P2OUT ^= BIT1;		P2DIR |= BIT1;
#define ANT_2_Chk               (P2IN & BIT1)



#define AMPLIFIRE_On		P1OUT |= BIT7;		P1DIR |= BIT7;
#define AMPLIFIRE_Off	        P1OUT &= (~BIT7);	P1DIR |= BIT7;
#define AMPLIFIRE_Togg		P1OUT ^= BIT7;		P1DIR |= BIT7;
#define AMPLIFIRE_Chk           (P1IN & BIT7)

#define Pin_RX                  (P3IN & BIT7)


#define Toogle_Antens         if (!ANT_1_Chk) {  ANT_1_Off;ANT_2_On; } else { ANT_2_Off;ANT_1_On; }
#define Switch_RX             CON_TX_1_Off; CON_TX_2_Off; CON_RX_1_On; CON_RX_2_On                  
#define Switch_TX             CON_RX_1_Off; CON_RX_2_Off; CON_TX_1_On; CON_TX_2_On                  

////////////FLAGS/////////////////////

#define MAX_RSSI_LEV    -6  
#define MID_RSSI_LEV    -35
#define LOW_RSSI_LEV    -52

//////////////////List of process////////////////
int process0();
int process1();
int process_switch_ANT();
void Init_RF_Test();

unsigned char TxBuffer[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
char vRSSI_1,vRSSI_2;
/****************************************************************************/
/*  Function name: CPU_clock_init                                           */
/*  	Parameters                                                          */
/*          Input   :  No	                                            */
/*          Output  :  No	                                            */
/*	Action: Initialize CPU main                  clock                  */
/****************************************************************************/
void CPU_clock_init(void){       
// ACLK = 32kHz, MCLK = SMCLK = 16MHz

P5SEL |= BIT0 + BIT1; // Select XT1
UCSCTL6 |= XCAP_3; // Internal load cap

__bis_SR_register(SCG0);                // Disable the FLL control loop
UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
UCSCTL1 = DCORSEL_7;                // Select DCO range 16MHz operation
UCSCTL2 = FLLD_1 + 487;              // Set DCO Multiplier for 8MHz
                                                              // (N + 1) * FLLRef = Fdco
                                                              // (249 + 1) * 32768 = 8MHz
                                                              // (365 + 1) * 32768 = 12MHz
                                                              // (487 + 1) * 32768 = 16MHz
                                                              // Set FLL Div = fDCOCLK/2

__bic_SR_register(SCG0); // Enable the FLL control loop

// Worst-case settling time for the DCO when the DCO range bits have been
// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
// UG for optimization.
// 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
__delay_cycles(375000);
 }


/////////////////Read saved values////////////
int read_Flash(void) {
  Flash_ptr = (char *) 0x1800;
  *Flash_ptr = 0;
  Flag_First_Time_Power_On = *Flash_ptr++; // Read first flag
  
  if (Flag_First_Time_Power_On==0x00){ // check if write before
      
          for(int i=0;i<3;i++){
                                FreqOffset[i] = *Flash_ptr++; // read from memory bank
                              }
                        
        }    else {
              FreqOffset[0]= 0x0C;
              FreqOffset[1]= 0x1D;
              FreqOffset[2]= 0x89;
            }
process_trigger  (TR_UPD_FREQUENCY);
return 0;
}


////////////////////SAVE SETTINGS/////////////////
int save_Flash(void) {

////////////// CLEAR FLASH SEGMENT ///////////////
//Workaround: Disable global interrupt while erasing.
//local variable to store GIE status
//char * Flash_ptr;
 uint16_t gieStatus;
//Store current SR register
gieStatus = __get_SR_register() & GIE;
//Disable global interrupt
__disable_interrupt();
FLASH_ERASE;
//Dummy write to erase Flash seg
 Flash_ptr = (char *) 0x1800;
*Flash_ptr = 0;
//test busy
while (FCTL3 & BUSY);
FLASH_LOCK;
//Restore SR register
__bis_SR_register(gieStatus);
/////////////PLACE, WHERE WE WRITE VARIABLES ////////////////////
FLASH_UNLOCK;
      *Flash_ptr++ = 0x00; //Flag_First_Time_Power_On
      *Flash_ptr++ = FreqOffset[0];
      *Flash_ptr++ = FreqOffset[1];
      *Flash_ptr++ = FreqOffset[2];
FLASH_LOCK;
return 0;
}
///////////////ANTENNA INIT///////////////////////////
int ant_init(void)
{
ANT_1_On;
ANT_2_Off;
Switch_RX;//Switch_RX;//
return 0;
}
////////////////AMPLIFIRE init ///////////////////////
int amplifire_init(void)
{
AMPLIFIRE_On;
return 0;

}
////////////////Toogle led///////////////////////////
int Toogle_Led(void)
{ 
LED_Togg;
trigger_Event_On_Timer  (TR_TOOGLE_LED,500,0,ONCE);
}
////////////////Toogle led///////////////////////////
int Clear_flag_by_time(void)
{ 
 already_send_flag=0;
}

////////////////TX settings//////////////////////////

int prepare_for_TX(void)
{ 
ReceiveOff();
Init_RF_Test();
Switch_TX;
return 0;
}
//////////Convert RSSI value to dBM//////
int  convRSSIto_dBM(char rssi){
  int rssi_dbm=0;
 if (rssi >= 128)
                                                rssi_dbm = (int)((int)( rssi - 256) / 2) - rssi_offset;
                                                 else
                                                rssi_dbm = (rssi / 2) - rssi_offset;
  return rssi_dbm;
}
////////////////Show RSSI//////////////////////////
int Show_rssi_by_time(){
  char tmp_str[40]="";
  sprintf(tmp_str, "RSSI = %hhd dBm\n\r" ,rssi_dBm);
  USART_Send_ROM_String(tmp_str);
  trigger_Event_On_Timer  (TR_SHOW_RSSI,1000,0,ONCE);
}
////////////////Set frequency values/////////////////
int setFreqOffset(){
                        WriteSingleReg(FREQ0,FreqOffset[2]);//0x89
                        WriteSingleReg(FREQ1,FreqOffset[1]);//0x1D
                        WriteSingleReg(FREQ2,FreqOffset[0]);//0x0C
return 0;
}
//////////Transmitting constant values for test///////
int ForRFTest(){

      Transmit( (unsigned char*)TxBuffer, sizeof TxBuffer);      
      //Wait for TX status to be reached before going back to low power mode
      while((Strobe(RF_SNOP) & 0x70) != 0x20);
return 0;
}

//////////Send recived data to UART//////
int Data_out(){
char tmp_str[40]="";  
                     
                         
                     if (sTmp[0] == sRX[0]){ 
                       if (sTmp[1] == sRX[1]){ 
                         if (sTmp[2] == sRX[2]){ 
                           if (sTmp[3] == sRX[3]){ 
                             if (sTmp[4] == sRX[4]){ 
                                if    (already_send_flag==0) {
                                  USART_Send_ROM_String("\n\r");
                                  USART_Send_ROM_String("Serial = ");
                                for (j=1;j<4;j++){      
                                                        sprintf(tmp_str,"%X",sRX[3-j]);            
                                                        USART_Send_ROM_String(tmp_str);		
                                                        already_send_flag=1;
                                                        trigger_Event_On_Timer  (TR_CLEAR_FLAG,4000,0,ONCE);
                                                  }
                                                  USART_Send_ROM_String("\n\r");
                                                  //sprintf(tmp_str,"RSSI_1 = %hhd dBm\n\r",convRSSIto_dBM(vRSSI_1));
                                                  //USART_Send_ROM_String(tmp_str);
                                                  sprintf(tmp_str, "RSSI = %hhd dBm\n\r" ,convRSSIto_dBM(vRSSI_2));
                                                  USART_Send_ROM_String(tmp_str);
                                                   
                                                    if (sRX[3] & LowBatBit      ){sprintf(tmp_str,"LowBatBit is set \n\r");     USART_Send_ROM_String(tmp_str);}
                                                    if (sRX[3] & TamperBit      ){sprintf(tmp_str,"TamperBit is set \n\r");     USART_Send_ROM_String(tmp_str);}
                                                    if (sRX[3] & AlarmBit       ){sprintf(tmp_str,"AlarmBit is set  \n\r");     USART_Send_ROM_String(tmp_str);}
                                                    if (sRX[3] & StatusBit      ){sprintf(tmp_str,"StatusBit is set \n\r");     USART_Send_ROM_String(tmp_str);}
                                                    if (sRX[3] & RestoreBit     ){sprintf(tmp_str,"RestoreBit is set \n\r");    USART_Send_ROM_String(tmp_str);}
                                                    if (sRX[3] & SupervisionBit ){sprintf(tmp_str,"SupervisionBit is set \n\r");USART_Send_ROM_String(tmp_str);}
                                                    USART_Send_ROM_String("\n\r");USART_Send_ROM_String("\n\r");
                                                    } 
                     
                                                  }
                                                }
                                              }
                                            }
                                           }
                     else {
                       if (already_send_flag==0) {
                       for (j=0;j<5;j++){               
                                                        sTmp[j] = sRX[j];
                                                        sRX[j] = 0;  
                                        }      
                                  }             
                     }
                     
                     
                    
						

}
/****************************************************************************/
/*  Function name: ports_init                                               */
/*  	Parameters                                                          */
/*          Input   :  No	                                            */
/*          Output  :  No	                                            */
/*	Action: Initialize all Port's directions and states                 */
/****************************************************************************/

int ports_init(void)
{
  P1OUT = 0x00;
  P1DIR = 0x00; 
  P1REN = 0x00;
  
  P2OUT = 0x00;
  P2DIR = 0x00;
  P2REN = 0x00;
  
  P3OUT = 0x00;
  P3DIR = 0x00;
  P3REN = 0x00;
  
   
  P2OUT &= ~BIT4;
  P2DIR |= BIT4;
  
  // Set up LEDs 
  P1OUT &= ~BIT0;
  P1DIR |= BIT0;
  
 // Set up LEDs 
  P3OUT &= ~BIT3;
  P3DIR |= BIT3;
  
  // Set Input 
  //P3OUT |= BIT7;  //Pullup
  //P3DIR &= ~BIT7;  //Input
  
  
  P1MAP0|=BIT0;
  P1DIR |=BIT0;
  P1SEL |=BIT0;
  return 0;
}
/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                       INTERRUPTS                                         */
/*                                                                          */
/*                                                                          */
/****************************************************************************/


/**********************************************************************************/
/*      PORT1, interrupt service routine.                                         */
/**********************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  switch(__even_in_range(P1IV, 4))
  {
    case  0: break;
    case  2: break;                         // P1.0 IFG
    case  4:
      P1IE = 0;                             // 
        
      break;                                // P1.1 IFG
    case  6: break;                         // P1.2 IFG
    case  8: break;                         // P1.3 IFG
    case 10: break;                         // P1.4 IFG
    case 12: break;                         // P1.5 IFG
    case 14: break;                         // P1.6 IFG
    case 16: break;                         // P1.7 IFG 
      
  }
}


/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                       MAIN                                               */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
////////////////////////Process 1 ////////////////////////////////////////////
int process0()
{        int res=0;
         // LED_On;
         //USART_Send_ROM_String("\n\rProcess 0");
         if (Need_check_UART_Buff) {
           
                //USART_Send_ROM_String (USART_Received_Data_Buff);
           
           switch (USART_Received_Data_Buff[0]) {
           case 0xFF: 
                        USART_Send_Data(USART_Received_Data_Buff[1]);
                        USART_Send_Data(USART_Received_Data_Buff[2]);
                        USART_Send_Data(USART_Received_Data_Buff[3]);
                        FreqOffset[0]=USART_Received_Data_Buff[1];
                        FreqOffset[1]=USART_Received_Data_Buff[2];
                        FreqOffset[2]=USART_Received_Data_Buff[3];
                        process_trigger  (TR_UPD_FREQUENCY);
                        USART_Send_Data(USART_Received_Data_Buff[0]);
             break;
             
           case 0xFE: 
                        if (USART_Received_Data_Buff[1]) { 
                                                           process_trigger  (TR_INIT);
                                                           process_trigger  (TR_TRANSMIT);
                                                           process_detach_by_name(PR_READ_RSSI);
                                                           process_attach(PR_TESTING,   50, (int *)ForRFTest); // Transmit all the time
                                                          }
                                                    else {
                                                            process_trigger  (TR_INIT);
                                                            process_detach_by_name(PR_TESTING);
                                                            process_attach(PR_READ_RSSI,   50, (int *)process1); // Read RSSI, change antenns
                                                           
                                                        }
                        USART_Send_Data(USART_Received_Data_Buff[0]);
             break;
           
             case 0xFD: 
                        process_trigger  (TR_CHANGE_ANT);
                        USART_Send_Data(USART_Received_Data_Buff[0]);
                       
               break;  
               
              case 0xFC: 
                        
                        if( USART_Received_Data_Buff[1]&BIT0 ) { AMPLIFIRE_On;} else { AMPLIFIRE_Off; }//AMPLIFIRE EN/DIS
                        if( USART_Received_Data_Buff[1]&BIT1 ) { ANT_1_On; ANT_2_Off;    } else { ANT_2_On; ANT_1_Off;     }//ANT 1
                        if( USART_Received_Data_Buff[1]&BIT2 ) { CON_RX_1_On; CON_TX_1_Off; } else { CON_TX_1_On; CON_RX_1_Off;  }//CON_RX
                        if( USART_Received_Data_Buff[1]&BIT3 ) { CON_RX_2_On; CON_TX_2_Off; } else { CON_TX_2_On; CON_RX_2_Off;  }//CON_TX
                       // if( USART_Received_Data_Buff[1]&BIT4 ) { CON_RX_2_On; } else { CON_RX_2_Off;  }//CON_RX_2
                       // if( USART_Received_Data_Buff[1]&BIT5 ) { CON_TX_1_On; } else { CON_TX_1_Off;  }//CON_TX_1
                       // if( USART_Received_Data_Buff[1]&BIT6 ) { CON_TX_2_On; } else { CON_TX_2_Off;  }//CON_TX_2
                        if( USART_Received_Data_Buff[1]&BIT7 ) { LED_On;      } else { LED_Off;       }//LED
                        USART_Send_Data(USART_Received_Data_Buff[0]);
                       
               break;
               
           default:
             break;
             
           }
           
                  
           Need_check_UART_Buff  = 0x00;
  }
 // setFreq(315000);
        return res;
}
////////////////////////Process 2 //////////////////////////////////////////// 
int process1()
{       
        //USART_Send_ROM_String("\n\rProcess 1");
        
        rssi_dec = ReadSingleReg(RSSI);
        rssi_dBm = convRSSIto_dBM(rssi_dec);
       
              
           if (rssi_dBm  < LOW_RSSI_LEV)      {
                                               RSSI_Low;
                                               process_trigger  (TR_CHANGE_ANT);
                                              }
           else if (rssi_dBm <= MID_RSSI_LEV) {
                                               RSSI_Middle;
                                              }
           else if (rssi_dBm <= MAX_RSSI_LEV) {
                                               RSSI_Hi; 
                                              } 
                                               else  
                                              {
                                               RSSI_Low;
                                              }
        //delay_ms(10);
           trigger_Event_On_Timer  (TR_CHECK_RSSI,10,0,ONCE); 
        return 0;
}
////////////////////////Process 3 //////////////////////////////////////////// 
int process_switch_ANT()
{ 
Toogle_Antens;    
return 0;
}
////////////////////////Process 4 //////////////////////////////////////////// 
int Recogn (){
  
  
		if (Pin_RX >0 ){		
			lCount = 0;
		 	while (Pin_RX) {									  
				lCount++;
				Delay50us();
			      }
			if ((lCount >= 6) && (lCount <=10)){					
				sRX[0] = 0;
				sRX[1] = 0;
				sRX[2] = 0;
				sRX[3] = 0;
				sRX[4] = 0;
				data_right = 1;	
				vRSSI_1 = ReadSingleReg(RSSI);
			   	  				
				for (j=0;j<36;j++){
					lCount = 0;
					hCount = 0;	  					
				   	while (!Pin_RX) {									  
						lCount++;
						if (lCount > 20) {
							lCount = 100;
							break;
						}
						Delay50us();
					}
					while (Pin_RX) {									  
						hCount++;
						if (hCount > 20) {
							hCount = 100;
							break;
						}
						Delay50us();
					}
					allCount = lCount + hCount;
					if ((allCount >= 4) && (allCount <= 45)){			  
						if (lCount > hCount){
							sRX[j / 8] = sRX[j / 8] | (1 << (j % 8));
							
						}else{
							
						}
					}else{
						data_right = 0;
						break;
					}
				}
				
				Delay50us();
				if (data_right == 1){
					
					vRSSI_2 = ReadSingleReg(RSSI);
					k = 0;							
					for (j=0; j<4; j++){
						l = sRX[j] & 0x0f;
						k ^= l; 
						m = sRX[j] >> 4;
						k ^= m;				
					} 
					if (k == sRX[4]){
                                                          process_trigger  (TR_UART_DATA_OUT);
						
					
					}
					
								 
				} 			
			}
		}
  
  
}
//////////////////////////////////////////////////////////////////////////////
//  Main 
// 
int main(void){

  //1. Stop errant interrupts until set up
 WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
_BIC_SR(GIE); // Disable interrupts during initialization process
///////////////////// Init of mediator //////////////////////////////////////
 process_subscribe(TR_INIT            , "ports_init"       , (int *) ports_init);
 process_subscribe(TR_INIT            , "rf_init"          , (int *) Init_RF);
 process_subscribe(TR_INIT            , "cpu_clock_init"   , (int *) CPU_clock_init);
 process_subscribe(TR_INIT            , "uart_init"        , (int *) USART_Init);
 process_subscribe(TR_INIT            , "ant_init"         , (int *) ant_init);
 process_subscribe(TR_INIT            , "amplifire_init"   , (int *) amplifire_init);
 process_subscribe(TR_INIT            , "flash_init"       , (int *) read_Flash);
 process_subscribe(TR_CHANGE_ANT      , "change_ant"       , (int *) process_switch_ANT);
 process_subscribe(TR_TRANSMIT        , "TX_init"          , (int *) prepare_for_TX);
 process_subscribe(TR_TOOGLE_LED      , "Toogle_led"       , (int *) Toogle_Led);
 process_subscribe(TR_UART_DATA_OUT   , "Data_Ready"       , (int *) Data_out);
 process_subscribe(TR_CLEAR_FLAG      , "Clear_flag"       , (int *) Clear_flag_by_time);
 process_subscribe(TR_SHOW_RSSI       , "Show_rssi"        , (int *) Show_rssi_by_time);
 process_subscribe(TR_CHECK_RSSI      , "Check_rssi"       , (int *) process1);
 
 process_subscribe(TR_UPD_FREQUENCY   , "Save_Freq"        , (int *) save_Flash);
 process_subscribe(TR_UPD_FREQUENCY   , "upd_freq"         , (int *) setFreqOffset);

 process_trigger  (TR_INIT);

  TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
 
  TA1CCR0 = 50000;
  TA1CTL = TASSEL_2 + MC_2 + TACLR;  // SMCLK, contmode, clear TAR
                                           
trigger_Event_On_Timer  (TR_TOOGLE_LED,500,0,ONCE);
trigger_Event_On_Timer  (TR_SHOW_RSSI,1000,0,ONCE);
trigger_Event_On_Timer  (TR_CHECK_RSSI,10,0,ONCE); 
 
 // enable interrupt


//  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
 //process_unsubscribe_by_event(TR_INIT);
 //process_trigger  (TR_TRANSMIT);
 
 _BIS_SR(GIE); // Global Interrupt enabled. Do this at the END of the initialization!!!!!!!!

 delay_ms(200);

//////////////////////////////////////////////////////////////////////////////////
//                         M A I N   C Y C L E                                  //
//////////////////////////////////////////////////////////////////////////////////

 ///////// Add processes to process list /////////////////////
  //process_attach(PR_SETUP_BY_UART,   50, (int *)process0); // Set freq by UART
    //process_attach(PR_READ_RSSI    ,   50, (int *)process1); // Read RSSI, change antenns
  //process_attach(PR_TESTING,   50, (int *)ForRFTest); // Read RSSI, change antenns
    process_attach(PR_RECOGN,   50, (int *)Recogn); // Read RSSI, change antenns

  scheduler(); // run scheduler, no way back from here
  
 while(1){
  
    Recogn();
 }
  
}

/*****************************************************************************************/
/*                                                                                       */
/*  	 HEADERs LIBRARIES FUNCTIONS                                                     */
/*  	                                                                                 */
/*****************************************************************************************/


/**********************************************************************************/
/*  Function name   : Init_RF               	                                  */
/*  	Parameters                                                                */
/*          Input   :  No       			                          */
/*          Output  :  No	                                                  */
/*	Action: Initialize RF radio core.                               	  */
/**********************************************************************************/
void Init_RF(void){
  
  // Increase PMMCOREV level to 2 in order to avoid low voltage error 
  // when the RF core is enabled
  SetVCore(2);
  ResetRadioCore();  
  
WriteSingleReg(  IOCFG0   , 0x0C );
WriteSingleReg(  IOCFG2   , 0x33 );
WriteSingleReg(  MDMCFG0  , 0xF8 );
WriteSingleReg(  MDMCFG1  , 0x22 );
WriteSingleReg(  FREQ0    , 0x89 );
WriteSingleReg(  FREQ1    , 0x1D );
WriteSingleReg(  FREQ2    , 0x0C );
WriteSingleReg(  MDMCFG2  , 0x30 );
WriteSingleReg(  MDMCFG3  , 0xE4 );
WriteSingleReg(  MDMCFG4  , 0x56 );   // was f5
WriteSingleReg(  MCSM0    , 0x18 );
WriteSingleReg(  PKTCTRL0 , 0x35 );
WriteSingleReg(  AGCCTRL0 , 0x90 );
WriteSingleReg(  AGCCTRL2 , 0x07 );
WriteSingleReg(  FREND0   , 0x11 );
WriteSingleReg(  FREND1   , 0x66 );
WriteSingleReg(  FSCAL0   , 0x02 );
WriteSingleReg(  AGCCTRL1 , 0x40 );  // was 7
WriteSingleReg(  0x7E     , 0x00 );
WriteSingleReg(  0xCC     , 0x00 );
WriteSingleReg(  IOCFG2   , 0x00 );
WriteSingleReg(  IOCFG2   , 0x00 );
WriteSingleReg(  IOCFG0   , 0x0D );
 
  WritePATable();
  ReceiveOn();  
  //Wait for RX status to be reached
  while((Strobe(RF_SNOP) & 0x70) != 0x10);
  
}

void Init_RF_Test(void){
  // Increase PMMCOREV level to 2 in order to avoid low voltage error 
  // when the RF core is enabled
  SetVCore(2);
  ResetRadioCore(); 
WriteSingleReg(		IOCFG0	,	0x0C	);
WriteSingleReg(		IOCFG2 	,	0x33	);
WriteSingleReg( FIFOTHR,        0x47 );
WriteSingleReg( SYNC1,          0xD3 );
WriteSingleReg( SYNC0,          0x91 );
WriteSingleReg( PKTLEN,         0xFF );
WriteSingleReg( PKTCTRL1,       0x04 );
WriteSingleReg( PKTCTRL0,       0x12 );
WriteSingleReg( ADDR,           0x00 );
WriteSingleReg( CHANNR,         0x00 );
WriteSingleReg( FSCTRL1,        0x06 );
WriteSingleReg( FSCTRL0,        0x00 );
WriteSingleReg( FREQ2,          0x0C );
WriteSingleReg( FREQ1,          0x1D );
WriteSingleReg( FREQ0,          0x89 );
WriteSingleReg( MDMCFG4,        0xF5 );
WriteSingleReg( MDMCFG3,        0x83 );
WriteSingleReg( MDMCFG2,        0x30 );
WriteSingleReg( MDMCFG1,        0x22 );
WriteSingleReg( MDMCFG0,        0xF8 );
WriteSingleReg( DEVIATN,        0x15 );
WriteSingleReg( MCSM2,          0x07 );
WriteSingleReg( MCSM1,          0x30 );
WriteSingleReg( MCSM0,          0x10 );
WriteSingleReg( FOCCFG,         0x16 );
WriteSingleReg( BSCFG,          0x6C );
WriteSingleReg( AGCCTRL2,       0x03 );
WriteSingleReg( AGCCTRL1,       0x40 );
WriteSingleReg( AGCCTRL0,       0x91 );
WriteSingleReg( WOREVT1,        0x87 );
WriteSingleReg( WOREVT0,        0x6B );
WriteSingleReg( WORCTRL,        0xFB );
WriteSingleReg( FREND1,         0x56 );
WriteSingleReg( FREND0,         0x11 );
WriteSingleReg( FSCAL3,         0xE9 );
WriteSingleReg( FSCAL2,         0x2A );
WriteSingleReg( FSCAL1,         0x00 );
WriteSingleReg( FSCAL0,         0x1F );
WriteSingleReg( FSTEST,         0x59 );
WriteSingleReg( PTEST,          0x7F );
WriteSingleReg( AGCTEST,        0x3F );
WriteSingleReg( TEST2,          0x81 );
WriteSingleReg( TEST1,          0x35 );
WriteSingleReg( TEST0,          0x0B );
WriteSingleReg( PARTNUM,        0x00 );
WriteSingleReg( VERSION,        0x06 );
WriteSingleReg( FREQEST,        0x00 );
WriteSingleReg( LQI,            0x7F );
WriteSingleReg( RSSI,           0x80 );
WriteSingleReg( MARCSTATE,      0x00 );
WriteSingleReg( WORTIME1,       0x00 );
WriteSingleReg( WORTIME0,       0x00 );
WriteSingleReg( PKTSTATUS,      0x00 );
WriteSingleReg( VCO_VC_DAC,     0x94 );
WriteSingleReg( TXBYTES,        0x00 );
WriteSingleReg( RXBYTES,        0x00 );
WriteSingleReg( RF1AIFCTL0,     0x00 );
WriteSingleReg( RF1AIFCTL1,     0x70 );
WriteSingleReg( RF1AIFCTL2,     0x00 );
WriteSingleReg( RF1AIFERR,      0x00 );
WriteSingleReg( RF1AIFERRV,     0x00 );
WriteSingleReg( RF1AIFIV,       0x00 );

  WritePATable();

}
/****************************************************************************/
/*  Function name: delay                                                    */
/*  	Parameters                                                          */
/*          Input   :  p	                                            */
/*          Output  :  No	                                            */
/*	Action: Simple delay                                                */
/****************************************************************************/
void delay(volatile unsigned long p){
	while(p){p--;}
}

/****************************************************************************/
/*  Function name: delay_us                                                 */
/*  	Parameters                                                          */
/*          Input   :  p	                                            */
/*          Output  :  No	                                            */
/*	Action: Simple delay                                                */
/****************************************************************************/
void delay_1us(volatile unsigned long p){}

void delay_ms(volatile unsigned long p){
unsigned long l=p*20;
while (l){ l--;Delay50us();}
}

void Delay50us(void){
unsigned long l=43;//l=56;
  while (l){ l--;delay_1us(1);}
}

/****************************************************************************/
/*  Function name: goToSleep                                                */
/*  	Parameters                                                          */
/*          Input   :  No                                                   */
/*          Output  :  No	                                            */
/*	Action: Going sleep                                                 */
/****************************************************************************/
void goToSleep(void){

// Set up BUTTON 
 // P1OUT &= ~BIT1;  //Pullup
 // P1DIR |=  BIT1;  //Input  
  //P1MAP0 &= ~BIT0;
  P1MAP0=0X00;
     
  P1OUT = 0x00;
  P1DIR = 0xFF; 
  P1REN = 0xFF;
  P2OUT = 0x00;
  P2DIR = 0xFF;
  P2REN = 0x00;
  P3OUT = 0x00;
  P3DIR = 0xff;
  P3REN = 0x00;
 // P1SEL =0XFF;
  //P2SEL =0XFF;
  //P3SEL =0XFF;
  
 // P1DIR &= ~ BIT6;                            // Set P1.6 as TX output
 // P1SEL &= ~ (BIT5 + BIT6);
 WriteSingleReg (MCSM0,0x00);
 WriteSingleReg (IOCFG0,0x00);
 WriteSingleReg (IOCFG1,0x00);
 WriteSingleReg (IOCFG2,0x00);
  
  
//ReceiveOff();
   Strobe( RF_SIDLE );
   Strobe( RF_SFRX);
   Strobe( RF_SPWD);
  
__low_power_mode_4();

}


/****************************************************************************/
/*   Function name  : void setFreq(double Freq)                             */
/*  	Parameters                                                          */
/*           Input  : Freq - frequency value in kHz                         */
/*          Output  : No	                                            */
/*	    Action  : Set frequency of CC1101 core                          */
/****************************************************************************/
void setFreq(double Freq){
  char SetFreq[3];
  FreqCalc (SetFreq, Freq);
  WriteSingleReg(FREQ0,SetFreq[0]);//0x89 
  WriteSingleReg(FREQ1,SetFreq[1]);//0x1d 
  WriteSingleReg(FREQ2,SetFreq[2]);//0x0c
}


/****************************************************************************/
/*   Function name  : void FreqCalc (char *SFreq, double Freq)              */
/*  	Parameters                                                          */
/*           Input  : Freq - frequency value in kHz                         */
/*          Output  : SFreq - pointer of arr[3]                             */
/*	    Action  : Calculate values for FREQ 0-2 rigisters               */
/****************************************************************************/
void FreqCalc (char *SFreq, double Freq){
 long int  Koef;
 Koef = (long int )( Freq*1000/(396.728515625)); //      Freq(MHz)/(26000000/2^16)
*SFreq++ = (Koef&0xff);
*SFreq++ = (Koef>>8)&0xff;
*SFreq++ = (Koef>>16)&0xff;
}


