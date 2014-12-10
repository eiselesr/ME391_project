/***********************************************************************************


DONGLE FILE --- LOAD ONTO CC2530 RFB 525

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_joystick.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "util_lcd.h"
#include "basic_rf.h"


/***********************************************************************************
* CONSTANTS
*/
// Application parameters
#define RF_CHANNEL                25      // 2.4 GHz RF channel

// BasicRF address definitions
#define PAN_ID                0x2007

#define SWITCH_ADDR           0x2520
#define LIGHT_ADDR            0xBEEF

//#define DONGLE_ADDR            0xBABE
//#define ROBOT_ADDR              0xFEED

#define APP_PAYLOAD_LENGTH        105
#define LIGHT_TOGGLE_CMD          0

#define INIT_COMM_CMD 1
#define INIT_COEF_CMD 2
#define INIT_CONTDATA_CMD 3


// Application states
#define IDLE                      0
#define SEND_CMD                  1

#define DONGLE_ADDR   LIGHT_ADDR
#define ROBOT_ADDR    SWITCH_ADDR
#define UP_ARROW    72
#define LEFT_ARROW  75
#define DOWN_ARROW  80
#define RIGHT_ARROW 77
enum{
  INITIAL,
  SENDING,
  WAITING,
  READY
}states;
/***********************************************************************************
* LOCAL VARIABLES
*/

static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];

static basicRfCfg_t basicRfConfig;
int start = 0;
int initFlag=1;
int sendInitFlag=0;
int turnOnMotorFlag=0;
uint8 readCoefficients = 0;
uint8 bob;
uint8 status = 0;//boolean
int receiveData=0;
int state = 0;
int ACK = 0;
int16* myRSSI;
int8 getRSSI;

#ifdef SECURITY_CCM
// Security key
static uint8 key[]= {
  0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
  0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};
#endif

/***********************************************************************************
* LOCAL FUNCTIONS
*/
//static void appLight();
//static void appSwitch();

static void basicRfSetUp();
//static int initRobotComm();
//static int receiveCoefficients();
//static void receiveContinuousData();
void uartStartRxForIsr();
void configureUSART0forUART_ALT1();

_Pragma("vector=0x13") __near_func __interrupt void UART0_RX_ISR(void);





/***********************************************************************************
* @fn          main
*/
void main(void)
{
  // Initalise board peripherals
  halBoardInit();
  
  basicRfSetUp();
  
  // Initalise hal_rf
  if(halRfInit()==FAILED)
  {
    HAL_ASSERT(FALSE);
  }
  
  // Indicate that device is powered
  halLedSet(1);
  halMcuWaitMs(350);
  
  configureUSART0forUART_ALT1();
  uartStartRxForIsr();
  
  
  while(TRUE)
  {
    //----------------------
    // INITIALIZE
    //----------------------
    if(initFlag)
    {
      while(!start); //waiting for 'a' key from PC 
      //respond to PC -- going to try to start up WRS 
      start=0;
      pTxData[0] = INIT_COMM_CMD;
      basicRfReceiveOff();
      if(basicRfSendPacket(ROBOT_ADDR, pTxData, APP_PAYLOAD_LENGTH)==SUCCESS)
      {
        state=1;
      }
      basicRfReceiveOn();
      //wait for ACK from WRS
      
      pTxData[0] = INIT_COEF_CMD;
      basicRfReceiveOff();
      if(basicRfSendPacket(ROBOT_ADDR, pTxData, APP_PAYLOAD_LENGTH)==SUCCESS)
      {
        basicRfReceiveOn();
        
        //WAIT FOR COEFFICIENTS FROM WRS
        while(!basicRfPacketIsReady());//wait to receive acknowledgement
        
        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) 
        {
          if(pRxData[0] == 'C') 
          {  
            //Pass to PC
            for (unsigned int uartTxIndex = 0; uartTxIndex<105; uartTxIndex++)
            {
              U0CSR &= ~0x02; //SET U0TX_BYTE to 0
              U0DBUF = pRxData[uartTxIndex];      
              while (!(U0CSR&0x02));
            }
          }    
        }
      }
      
      //finished sending coefficients to PC 
      
      basicRfReceiveOn();
      initFlag=0;
    }
    
    
    
    if(turnOnMotorFlag){
      if(basicRfSendPacket(ROBOT_ADDR, pTxData, APP_PAYLOAD_LENGTH)==SUCCESS)//send command to WRS
      {
        turnOnMotorFlag=0;
      }
    }
    
    if(sendInitFlag){
      if(basicRfSendPacket(ROBOT_ADDR, pTxData, APP_PAYLOAD_LENGTH)==SUCCESS)//send command to WRS
      {
        initFlag=1;
        sendInitFlag=0;
        //Pass to PC
        for (unsigned int uartTxIndex = 0; uartTxIndex<105; uartTxIndex++)
        {
          pRxData[0] == 'R';
          U0CSR &= ~0x02; //SET U0TX_BYTE to 0
          U0DBUF = pRxData[uartTxIndex];      
          while (!(U0CSR&0x02));
        }
      }
    }
    //Receive package from WRS
    if(basicRfPacketIsReady())
    { 
      
      if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, myRSSI)>0) {
        getRSSI = basicRfGetRssi();
        pRxData[104]=getRSSI;
        if(pRxData[0] == 'D')//||(pRxData[0] == 'I'))
        {          
          //SEND DATA TO PC 
          for (unsigned int uartTxIndex = 0; uartTxIndex<105; uartTxIndex++)
          {
            U0CSR &= ~0x02; //SET U0TX_BYTE to 0
            U0DBUF = pRxData[uartTxIndex];      
            while (!(U0CSR&0x02));
          }
        }
      }
      
    }    
  }
}
//END OF MAIN WHILE LOOP



//-------------------------------------------
// RX interrupt service routine
//-------------------------------------------

_Pragma("vector=0x13") __near_func __interrupt void UART0_RX_ISR(void)
{	
  IEN0 &= ~0x80;//DISABLE INTERRUPTS
  URX0IF = 0; //Interrupt not pending
  
  unsigned int keyVal = U0DBUF;
  
  switch(keyVal)    
  {
  case 97:// 'a' key
    start = 1;//Start communication with WRS
    break;
    //  case 98: //'b' key
    //    turnOnMotorFlag=1;
    //    pTxData[0] = keyVal;
    //    break;
    //  case 107:
    //    readCoefficients=1;//start reading coeffs from pressure sensor
    //    break;
    //  case 'Z': //an idea for getting ack from PC that it is done reading in all data -- since this does take some time..
    //    ACK = 1;
    //    break;
  case UP_ARROW:
    pTxData[0] = keyVal;
    turnOnMotorFlag = 1; 
    break;
  case DOWN_ARROW:
    pTxData[0] = keyVal;
    sendInitFlag = 1; 
    break;
    //  case LEFT_ARROW:
    //    pTxData[0] = keyVal;
    //    changePWMflag = 1; 
    //    break;
    //  case RIGHT_ARROW:
    //    pTxData[0] = keyVal;
    //    changePWMflag = 1; 
    //    break;
  }
  IEN0 |= 0x80; //ENABLE INTERRUPTS GENERALLY
}


//-------------------------------------------
//      Set Up Basic RF
//-------------------------------------------
static void basicRfSetUp()
{
  // Config basicRF
  basicRfConfig.panId = PAN_ID;
  basicRfConfig.channel = RF_CHANNEL;
  basicRfConfig.ackRequest = TRUE;
  
  
#ifdef SECURITY_CCM
  basicRfConfig.securityKey = key;
#endif
  
  basicRfConfig.myAddr = DONGLE_ADDR;
  
  // Initialize BasicRF
  
  if(basicRfInit(&basicRfConfig)==FAILED) {
    HAL_ASSERT(FALSE);
  }
}


//-------------------------------------------
//      CONFIGURE USART
//-------------------------------------------

void configureUSART0forUART_ALT1(){   
  PERCFG &= ~0X01; //SET USART0 TO ALT LOCATION 1 - FAMILY PG. 85
  P0SEL  |=  0x0C; //SET RX(bit2) and TX(bit3) to PERIPHERAL FUNCTION [---- 11--] - FAMILY PG. 85
  P0DIR  |=  0x08; //SET TX(bit3) TO OUTPUT(1) [---- 1---] - FAMILY PG.86
  P0DIR  &= ~0X04; //SET RX(bit2) TO INPUT(0) [---- -0--] - FAMILY PG.86
  
  U0CSR  |=  0x80; //SET USART0 TO UART MODE -FAMILY PG.160
  
  // set stop/start bit levels parity, number of stop bits etc...
  U0UCR |=  0x06;//Flow control disabled, 8bit transfer, Parity diabled, 2 stop bits, high stop bit, low start bit[-0-0 0110]
  U0UCR &= ~0x59;
  
  // Chose 28800 baud rate... because it seemed like a good number
  U0BAUD =  0xD8; //SET BAUD_M = 216(0xD8) - Family pdf pg159 
  U0GCR |=  0x09; //SET U0GCR.BAUD_E = 9 => Set 0 and 3 bit to 1 [---0 1001]
  U0GCR &= ~0x16; //                        and bits 1,2, and 4 to 0.    
}


//-------------------------------------------
//      Turn on RX
//-------------------------------------------
void uartStartRxForIsr()
{
  // uartRxIndex = 0;
  URX0IF = 0;
  U0CSR |= 0x40;//Enables UART receiver
  IEN0 |= 0x04; //Enable USART0 RX interrupt => Set bit 2 to 1 (family pg. 45)
  IEN0 |= 0x80; //ENABLE INTERRUPTS GENERALLY
}


//--------------------------------------------------------------
//
//
//      END OF WORKING CODE 
//
//--------------------------------------------------------------

