/******************************************************************************
* Simple RF Link to Toggle Receiver's LED by pressing Transmitter's Button    *
* Warning: This RF code example is setup to operate at 868Mhz frequency, 
* which might be out of allowable range of operation in certain countries. Please
* refer to the appropriate legal sources before performing tests with this code 
* example. 
* 
* This code example can be loaded to 2 MSP430-CCRF devices. Each device will transmit 
* a small packet upon a button pressed. Each device will also toggle its LED upon
* receiving said packet. 
* 
* The RF packet engine settings specify variable-length-mode with CRC check enabled
* The RX packet also appends 2 status bytes regarding CRC check, RSSI and LQI info.
* For specific register settings please refer to the comments for each register in
* RF1A_REGISTER_CONFIG[] or the CC430x613x User's Guide and the SmartRF Studio
* 
* All required changes, which enable this code to be portable for MSP430-CCRF,
* were made by Penko T. Bozhkov -> Olimex LTD
******************************************************************************/
#include "cc430x513x.h"
#include "HAL/RF1A.h"
#include "HAL/cc430x613x_PMM.h"
#include "RF_Connection.h"
#include "smartrf_CC430_315MHz.h"
#include  "usart.h"
//#ifdef MHZ_915
//  #include "HAL/RF_config_Olimex/smartrf_CC430F5137_915MHz_38k4Baud.h"
//#elif defined MHZ_868
//  #include "HAL/RF_config_Olimex/smartrf_CC430F5137_868MHz_38k4Baud.h"
//#endif


#define LED_Togg    P1OUT ^= 0x01;  P1DIR |= 0x01;

/* Deviation = 5.157471 */
/* Base frequency = 867.999939 */
/* Carrier frequency = 867.999939 */
/* Channel number = 0 */
/* Carrier frequency = 867.999939 */
/* Modulated = true */
/* Modulation format = 2-GFSK */
/* Manchester enable = false */
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Preamble count = 4 */
/* Channel spacing = 199.951172 */
/* Carrier frequency = 867.999939 */
/* Data rate = 1.19948 */
/* RX filter BW = 58.035714 */
/* Data format = Normal mode */
/* CRC enable = true */
/* Whitening = false */
/* Device address = 0 */
/* Address config = No address check */
/* CRC autoflush = false */
/* PA ramping = false */
/* TX power = 0 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/
#define SMARTRF_SETTING_IOCFG2     0x7A//0x29
#define SMARTRF_SETTING_IOCFG1     0xBA//0x2E
#define SMARTRF_SETTING_IOCFG0     0x32//0x06
#define SMARTRF_SETTING_FIFOTHR    0x47
#define SMARTRF_SETTING_SYNC1      0xD3
#define SMARTRF_SETTING_SYNC0      0x91
#define SMARTRF_SETTING_PKTLEN     0x04//0x0A  // Changed to 0x0A. Def = 0xFF
#define SMARTRF_SETTING_PKTCTRL1   0x04
#define SMARTRF_SETTING_PKTCTRL0   0x01//0x05
#define SMARTRF_SETTING_ADDR       0x00
#define SMARTRF_SETTING_CHANNR     0x00
#define SMARTRF_SETTING_FSCTRL1    0x06
#define SMARTRF_SETTING_FSCTRL0    0x00

#define SMARTRF_SETTING_FREQ2      0x0C
#define SMARTRF_SETTING_FREQ1      0x1D
#define SMARTRF_SETTING_FREQ0      0x89
//#define SMARTRF_SETTING_FREQ2      0x21
//#define SMARTRF_SETTING_FREQ1      0x62
//#define SMARTRF_SETTING_FREQ0      0x76

#define SMARTRF_SETTING_MDMCFG4    0xf6//0xF6//0xF5
#define SMARTRF_SETTING_MDMCFG3    0x83

//#define SMARTRF_SETTING_MDMCFG2    0x13
#define SMARTRF_SETTING_MDMCFG2    0xbc//0x38

#define SMARTRF_SETTING_MDMCFG1    0x22
#define SMARTRF_SETTING_MDMCFG0    0xF8
#define SMARTRF_SETTING_DEVIATN    0x15
#define SMARTRF_SETTING_MCSM2      0x07
#define SMARTRF_SETTING_MCSM1      0x3C  // Changed to 0x3C. When a packet has been received: Stay in RX. Def = 0x30
#define SMARTRF_SETTING_MCSM0      0x10
#define SMARTRF_SETTING_FOCCFG     0x16
#define SMARTRF_SETTING_BSCFG      0x6C
#define SMARTRF_SETTING_AGCCTRL2   0x03
#define SMARTRF_SETTING_AGCCTRL1   0x40
#define SMARTRF_SETTING_AGCCTRL0   0x91
#define SMARTRF_SETTING_WOREVT1    0x87
#define SMARTRF_SETTING_WOREVT0    0x6B
#define SMARTRF_SETTING_WORCTRL    0xb9//0xFB
#define SMARTRF_SETTING_FREND1     0x56

#define SMARTRF_SETTING_FREND0     0x11 //ASK
//#define SMARTRF_SETTING_FREND0     0x10

#define SMARTRF_SETTING_FSCAL3     0xE9
#define SMARTRF_SETTING_FSCAL2     0x2A
#define SMARTRF_SETTING_FSCAL1     0x00
#define SMARTRF_SETTING_FSCAL0     0x1F
#define SMARTRF_SETTING_FSTEST     0x59
#define SMARTRF_SETTING_PTEST      0x7F
#define SMARTRF_SETTING_AGCTEST    0x3F
#define SMARTRF_SETTING_TEST2      0x81
#define SMARTRF_SETTING_TEST1      0x35
#define SMARTRF_SETTING_TEST0      0x09

const char st[]="Start transmit\n";
const unsigned char RF1A_REGISTER_CONFIG[CONF_REG_SIZE]=
{
  SMARTRF_SETTING_IOCFG2  ,  // IOCFG2: GDO2 signals on RF_RDYn     
  SMARTRF_SETTING_IOCFG1  ,  // IOCFG1: GDO1 signals on RSSI_VALID     
  SMARTRF_SETTING_IOCFG0  ,  // IOCFG0: GDO0 signals on PA power down signal to control RX/TX switch         
  SMARTRF_SETTING_FIFOTHR , // FIFOTHR: RX/TX FIFO Threshold: 33 bytes in TX, 32 bytes in RX    
  SMARTRF_SETTING_SYNC1   , // SYNC1: high byte of Sync Word
  SMARTRF_SETTING_SYNC0   , // SYNC0: low byte of Sync Word
  SMARTRF_SETTING_PKTLEN  , // PKTLEN: Packet Length in fixed mode, Maximum Length in variable-length mode      
  SMARTRF_SETTING_PKTCTRL1, // PKTCTRL1: No status bytes appended to the packet    
  SMARTRF_SETTING_PKTCTRL0, // PKTCTRL0: Fixed-Length Mode, No CRC       
  SMARTRF_SETTING_ADDR    , // ADDR: Address for packet filtration       
  SMARTRF_SETTING_CHANNR  , // CHANNR: 8-bit channel number, freq = base freq + CHANNR * channel spacing          
  SMARTRF_SETTING_FSCTRL1 , // FSCTRL1: Frequency Synthesizer Control (refer to User's Guide/SmartRF Studio) 
  SMARTRF_SETTING_FSCTRL0 , // FSCTRL0: Frequency Synthesizer Control (refer to User's Guide/SmartRF Studio) 
  SMARTRF_SETTING_FREQ2   , // FREQ2: base frequency, high byte      
  SMARTRF_SETTING_FREQ1   , // FREQ1: base frequency, middle byte      
  SMARTRF_SETTING_FREQ0   , // FREQ0: base frequency, low byte      
  SMARTRF_SETTING_MDMCFG4 , // MDMCFG4: modem configuration (refer to User's Guide/SmartRF Studio)     
  SMARTRF_SETTING_MDMCFG3 , // MDMCFG3:                "                      "    
  SMARTRF_SETTING_MDMCFG2 , // MDMCFG2:                "                      "        
  SMARTRF_SETTING_MDMCFG1 , // MDMCFG1:                "                      "        
  SMARTRF_SETTING_MDMCFG0 , // MDMCFG0:                "                      "        
  SMARTRF_SETTING_DEVIATN , // DEVIATN: modem deviation setting (refer to User's Guide/SmartRF Studio)         
  SMARTRF_SETTING_MCSM2   , // MCSM2: Main Radio Control State Machine Conf. : timeout for sync word search disabled      
  SMARTRF_SETTING_MCSM1   , // MCSM1: CCA signals when RSSI below threshold, stay in RX after packet has been received      
  SMARTRF_SETTING_MCSM0   , // MCSM0: Auto-calibrate when going from IDLE to RX or TX (or FSTXON )      
  SMARTRF_SETTING_FOCCFG  , // FOCCFG: Frequency Offset Compensation Conf.     
  SMARTRF_SETTING_BSCFG   , // BSCFG: Bit Synchronization Conf.       
  SMARTRF_SETTING_AGCCTRL2, // AGCCTRL2: AGC Control   
  SMARTRF_SETTING_AGCCTRL1, // AGCCTRL1:     "   
  SMARTRF_SETTING_AGCCTRL0, // AGCCTRL0:     "   
  SMARTRF_SETTING_WOREVT1 , // WOREVT1: High Byte Event0 Timeout    
  SMARTRF_SETTING_WOREVT0 , // WOREVT0: High Byte Event0 Timeout
  SMARTRF_SETTING_WORCTRL , // WORCTL: Wave On Radio Control ****Feature unavailable in PG0.6****
  SMARTRF_SETTING_FREND1  , // FREND1: Front End RX Conf.    
  SMARTRF_SETTING_FREND0  , // FREND0: Front End TX Conf.               
  SMARTRF_SETTING_FSCAL3  , // FSCAL3: Frequency Synthesizer Calibration (refer to User's Guide/SmartRF Studio)    
  SMARTRF_SETTING_FSCAL2  , // FSCAL2:              "      
  SMARTRF_SETTING_FSCAL1  , // FSCAL1:              "     
  SMARTRF_SETTING_FSCAL0  , // FSCAL0:              "     
  0x00                    , // Reserved *read as 0*
  0x00                    , // Reserved *read as 0*
  SMARTRF_SETTING_FSTEST  , // FSTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_PTEST   , // PTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_AGCTEST , // AGCTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_TEST2   , // TEST2  : For test only, irrelevant for normal use case    
  SMARTRF_SETTING_TEST1   , // TEST1  : For test only, irrelevant for normal use case
  SMARTRF_SETTING_TEST0     // TEST0  : For test only, irrelevant for normal use case       
};


extern unsigned char packetReceived;
extern unsigned char packetTransmit; 

unsigned char RxBuffer[255], RxBufferLength = 0;
//const unsigned char TxBuffer[]= {0x9E,0x92,0x6b,0x32,0x00};
const unsigned char TxBuffer[]= {0xCB,0x34,0xC6,0x32,0x0b};



unsigned char buttonPressed = 0;
unsigned int i = 0; 


void convertToManchester(char *a, char *b){
  for (int i = 0; i< sizeof a;i++){
  
  }
  
}







