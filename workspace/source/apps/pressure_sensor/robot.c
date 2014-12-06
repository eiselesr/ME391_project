/***********************************************************************************

WIRELESS ROBOTIC SYSTEM FILE --- LOAD ONTO CC2530 RFB 743

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
#define RF_CHANNEL                25      // 2.4 GHz RF channel Can choose 11-26
// Hakan: 17

// BasicRF address definitions
#define PAN_ID                0x2007
#define SWITCH_ADDR           0x2520
#define LIGHT_ADDR            0xBEEF

//#define ROBOT_ADDR             0xFEED
//#define DONGLE_ADDR             0xBABE

#define ROBOT_ADDR      SWITCH_ADDR
#define DONGLE_ADDR     LIGHT_ADDR



#define APP_PAYLOAD_LENGTH        105
#define LIGHT_TOGGLE_CMD          0

#define INIT_COMM_CMD 1
#define INIT_CONTDATA_CMD 2
#define ACK 23

// Application states
#define IDLE                      0
#define SEND_CMD                  1

// ADC MASKS
#define ADC_AIN5            0x05     // single ended P0_5
#define ADC_AIN6            0x06     // single ended P0_6
#define ADC_AIN7            0x07     // single ended P0_7
#define ADC_AIN4            0x04     // single ended P0_4
#define ADC_AIN1            0x01     // single ended P0_1
#define ADC_AIN0            0x00     // single ended P0_0
#define ADC_VDD_3           0x0F     // (vdd/3)
#define ADC_10_BIT          0x20     // 256 decimation rate 
#define ADC_7_BIT          0x00     // 64 decimation rate 



//STATES AND OTHER DATA DEFINES

#define ADC 0
#define SPI 1

#define CS_1 P1_4 // CHIP SELECT FOR PRESSURE CENSOR
#define CS_2 P1_4 // CHIP SELECT FOR PRESSURE CENSOR
#define CS_3 P1_4 // CHIP SELECT FOR PRESSURE CENSOR
#define CS_4 P1_4 // CHIP SELECT FOR PRESSURE CENSOR
#define CS_A P1_4 // CHIP SELECT FOR ACCELEROMETER
#define CS_G P1_4 // CHIP SELECT FOR GYROSCOPE

#define a0_MSB 0x88 //Coefficient addresses (Read built in)
#define a0_LSB 0x8A
#define b1_MSB 0x8C
#define b1_LSB 0x8E
#define b2_MSB 0x90
#define b2_LSB 0x92
#define c12_MSB 0x94
#define c12_LSB 0x96

#define Padc_MSB 0x80  //Sensor addresses (Read built in)
#define Padc_LSB 0x82
#define Tadc_MSB 0x84
#define Tadc_LSB 0x86

#define START 0x24

//Accelerometer
#define WHO_AM_I_G 0x0F
#define READ 0x80
#define WRITE 0x00
#define JUNK 0xAA
#define CTRL_REG5_A 0x20
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

#define UP_ARROW    72
#define LEFT_ARROW  75
#define DOWN_ARROW  80
#define RIGHT_ARROW 77

#define delta 25
#define OFFSET 73


#define numPressureSensors 4

/***********************************************************************************
* LOCAL VARIABLES
*/
unsigned char spiTxBuffer[APP_PAYLOAD_LENGTH];//10
unsigned char spiRxBuffer_IMU[APP_PAYLOAD_LENGTH];  //20
unsigned char spiRxBuffer_Pressure[APP_PAYLOAD_LENGTH];  //20

//unsigned char CSBuff[4];


static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];
static uint8 coeffBuf[APP_PAYLOAD_LENGTH];
static uint8 biasBuf[APP_PAYLOAD_LENGTH];

uint8 status;

static basicRfCfg_t basicRfConfig;

//Pressure Sensor Biasing
int a0;
int b1;
int b2;
int c12;
double a0dec;
double b1dec;
double b2dec;
double c12dec;

//IMU Biasing
unsigned int bias;
int x_bias;
int y_bias;
int z_bias;
unsigned short xL;
unsigned short yL;
unsigned short zL;
unsigned short xH;
unsigned short yH;
unsigned short zH;
short x;
short y;
short z;

int value; // Value in which ADC conversion is stored.
int counter = 0;
int dataCounter = 0;
int spiPkg;
int dongleActive = 0;
int state = 0;
int sendPressureFlag = 0;
uint8 rssiPow;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

static void basicRfSetUp();
static void configurePressure();
static void readPressure();
//static void collectData(); //says variable dataBuf (gl0bal) and used in this is set but never referenced
uint8 sendPressure();
void configureStuff();
static void turnOnCS();
static void turnOffCS();
static void configureIMU();
static void readIMU();
uint8 sendIMU();
static void turnOnMotor();
static void turnOffMotor();

_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void);
///////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////
void main(void)
{
  configureStuff();
  basicRfSetUp();
  if(halRfInit()==FAILED) {
    HAL_ASSERT(FALSE);
  }
  halMcuWaitMs(350);
  configurePressure();
  
  basicRfReceiveOn();
  
  while(!basicRfPacketIsReady()); //WAITING FOR  INIT COMM CMD
  
  if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) 
  {
    if(pRxData[0] == INIT_COMM_CMD) 
    {        
      IEN0 |= 0x80; //[1--- ----] Allow interrupts
      IEN1 |= 0x02; //[---- --1-] Enable Timer 1 interrupt
    }
  }
  
  while(!basicRfPacketIsReady()); 
  
  if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) //WAITING FOR CMD TO SEND COEFFS
  {
    if(pRxData[0] == INIT_CONTDATA_CMD)
    {
      basicRfReceiveOff();
      basicRfSendPacket(DONGLE_ADDR, coeffBuf, 105);
      
      basicRfReceiveOn();
      //IEN1 |= 0x02; //[---- --1-] Enable Timer 1 interrupt <- Probably won't use
    }
  }
  
  
  //ENTERS WITH BASIC RF RECEIVE ON 
  
  //MAIN LOOP -- SENDS PRESSURE and ADC data and WAIT FOR CMDS to change pulse widths
  while(TRUE)
  {   
    //Get RSSI
    if(RSSISTAT & 0x01){
      rssiPow = RSSI - OFFSET;
    }
    //send pressure data
    if(sendPressureFlag == 1 && !basicRfPacketIsReady())
    {
      //basicRfReceiveOff();      
      readPressure();
      sendPressure();//does not turn on/off RFReceive
      sendPressureFlag = 0;
      //basicRfReceiveOn();
    }  
  }
}

//-----------------------------------------
// CONFIGURE AND INITIALIZE BASIC RF 
//
//--------------------------------------------

static void basicRfSetUp()
{
  // Config basicRF
  basicRfConfig.panId = PAN_ID;
  basicRfConfig.channel = RF_CHANNEL;
  basicRfConfig.ackRequest = TRUE;
  
  
#ifdef SECURITY_CCM
  basicRfConfig.securityKey = key;
#endif
  
  basicRfConfig.myAddr = ROBOT_ADDR;
  
  // Initialize BasicRF
  
  if(basicRfInit(&basicRfConfig)==FAILED) {
    HAL_ASSERT(FALSE);
  }
}

//-----------------------------------------
// CONFIGURE PRESSURE SENSOR AND OBTAIN COEFFICIENTS
//-----------------------------------------
static void configurePressure()
{
  CS_1=1;
  CS_2=1;
  CS_3=1;
  CS_4=1;
  CS_A=1;
  CS_G=1;
  
  //Set VDD high, GND low, SDN low
  P1_1=1; //VDD
  P1_2=0; //GND
  P1_3=1; //SDN
  
  //Commands
  spiTxBuffer[0] = a0_MSB;
  spiTxBuffer[1] = a0_LSB;
  spiTxBuffer[2] = b1_MSB;
  spiTxBuffer[3] = b1_LSB;
  spiTxBuffer[4] = b2_MSB;
  spiTxBuffer[5] = b2_LSB;
  spiTxBuffer[6] = c12_MSB;
  spiTxBuffer[7] = c12_LSB;
  spiTxBuffer[8] = 0x00;
  
  
  coeffBuf[0] = 'C'; //Coefficient header
  coeffBuf[1] = 0; //Packet Number
  coeffBuf[2] = 1; //Packet Number
  
  //int *CS;
  
  for(int sensorNum=0; sensorNum<numPressureSensors;sensorNum++){
    // CS = CS_1  -- make CS point to the current sensors chip select so in loop correct pin is set high/low
    //CS = CSBuff[i];=
    //CS = &CS_1;
    //   *CS = 1;
    
    
    //Read coefficients
    // *CS = 0;
    turnOffCS(sensorNum);
    //I (Eric) have this set up to send one coeffBuf containing info for all 4 sensors
    for(int i=0; i<9; i++)
    {
      U1TX_BYTE = 0;
      U1DBUF = spiTxBuffer[i];
      while(!U1TX_BYTE);    
      
      U1TX_BYTE = 0; 
      U1DBUF = 0x00;
      while (!U1TX_BYTE);    
      coeffBuf[i+3+9*sensorNum] = U1DBUF;    
    } 
    turnOnCS(sensorNum);
    //*CS = 1;
    
  }
}
//-----------------------------------------
// READ PRESSURE AND TEMPERATURE
//-----------------------------------------
static void readPressure()
{
  CS_1=1;
  CS_2=1;
  CS_3=1;
  CS_4=1;
  CS_A=1;
  CS_G=1;
  
  //Set VDD high, GND low, SDN low
  P1_1=1; //VDD
  P1_2=0; //GND
  P1_3=1; //SDN
  
  for(int sensorNum=0; sensorNum<numPressureSensors;sensorNum++){
    
    //Pressure Sensor #1
    //CS = *CS_1
    //CS = 1;
    
    
    turnOffCS(sensorNum);
    //CS=0;
    //Start Measuring Pressure
    U1TX_BYTE = 0;
    U1DBUF = START;
    while(!U1TX_BYTE);    
    
    U1TX_BYTE = 0; 
    U1DBUF = 0x00;
    while (!U1TX_BYTE);    
    turnOnCS(sensorNum);
    //CS=1;
    
    halMcuWaitMs(3);//Wait for conversion
    
    //Read coefficients
    spiTxBuffer[0] = Padc_MSB;
    spiTxBuffer[1] = Padc_LSB;
    spiTxBuffer[2] = Tadc_MSB;
    spiTxBuffer[3] = Tadc_LSB;
    spiTxBuffer[4] = 0x00;
    
    //CS = 0;
    turnOffCS(sensorNum);
    for(int i=0; i<5; i++)
    {
      
      U1TX_BYTE = 0;
      U1DBUF = spiTxBuffer[i];
      while(!U1TX_BYTE);    
      
      U1TX_BYTE = 0; 
      U1DBUF = 0x00;
      while (!U1TX_BYTE);    
      spiRxBuffer_Pressure[i+5*sensorNum] = U1DBUF;
      
    }
    turnOnCS(sensorNum);
    //CS = 1;
    
  }//end iterating loop
}

//-----------------------------------------
// Send PRESSURE AND TEMPERATURE
//-----------------------------------------
uint8 sendPressure()
{  
  uint8 status = FAILED;// FAILED = 1
  //int i = 0;
  // while(i<5)
  {
    pTxData[0] = 'P'; //P => Pressure data
    pTxData[1] = spiPkg/256;
    pTxData[2] = spiPkg%256;
    for(int sensorNum=0; sensorNum<numPressureSensors; sensorNum++){
      
      //Only 4 values are sent for each sensor, but we stored 5 (one extra for junk)
      //This is why sensorNum is multiplied by different numbers.
      pTxData[3+sensorNum*4] = spiRxBuffer_Pressure[0+sensorNum*5];
      pTxData[4+sensorNum*4] = spiRxBuffer_Pressure[1+sensorNum*5];
      pTxData[5+sensorNum*4] = spiRxBuffer_Pressure[2+sensorNum*5];
      pTxData[6+sensorNum*4] = spiRxBuffer_Pressure[3+sensorNum*5];
    }
    pTxData[102] = rssiPow;
    
    status = basicRfSendPacket(DONGLE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
    spiPkg++;
    // i++;
    if(status == SUCCESS){
      //  break;
    }
  }
  return status;  
}

//-------------------------------------------
//      BIAS IMU SENSOR
//-------------------------------------------
static void configureIMU()
{
  CS_1=1;
  CS_2=1;
  CS_3=1;
  CS_4=1;
  CS_A=1;
  CS_G=1;
  
  //Set VDD high, GND low, SDN low
  P1_1=1; //VDD
  P1_2=0; //GND
  P1_3=1; //SDN
  
  biasBuf[0] = 'C'; //Coefficient header
  biasBuf[1] = 0; //Packet Number
  biasBuf[2] = 1; //Packet Number
  
  for(int i=0; i<7; i++)//READ EXTRA BIT. LAST ONE IS JUNK TO FLUSH OUT NEEDED BITS
  {   
    CS_A = 0;
    U1TX_BYTE = 0;
    U1DBUF = (OUT_X_L_A + i) | READ;
    while(!U1TX_BYTE);    
    U1TX_BYTE = 0; //NOT SURE IF THIS IS NEEDED OR NOT
    U1DBUF = JUNK;
    while (!U1TX_BYTE);    
    CS_A =1;
    biasBuf[i] = U1DBUF;// START FROM 0 with a junk bit or 1 if bit is good.     
  }	

}

//--------------------------------------------------------------------------
// READ ACCELEROMETER/ Assemble package
//-------------------------------------------------------------------------- 
static void readIMU() {
  for(int i=0; i<7; i++)//READ EXTRA BIT. LAST ONE IS JUNK TO FLUSH OUT NEEDED BITS
  {   
    CS_A = 0;
    U1TX_BYTE = 0;
    U1DBUF = (OUT_X_L_A + i) | READ;
    while(!U1TX_BYTE);    
    U1TX_BYTE = 0; //NOT SURE IF THIS IS NEEDED OR NOT
    U1DBUF = JUNK;
    while (!U1TX_BYTE);    
    CS_A =1;
    spiRxBuffer_IMU[i] = U1DBUF;// START FROM 0 with a junk bit or 1 if bit is good.     
  }	 
  
}

//-----------------------------------------
// Send IMU
//-----------------------------------------
uint8 sendIMU()
{  
  uint8 status = FAILED;// FAILED = 1
  //int i = 0;
  // while(i<5)
  
  pTxData[0] = 'I'; //I => IMU
  pTxData[1] = spiPkg/256;
  pTxData[2] = spiPkg%256;
  
  for (int i=0;i<6;i++) {
    pTxData[i+3] = spiRxBuffer_IMU[i];
  }
  
  pTxData[102] = rssiPow;
  
  status = basicRfSendPacket(DONGLE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
  spiPkg++;
  // i++;
  if(status == SUCCESS){
    //  break;
  }

return status;  
}

//-----------------------------------------
// TURN ON MOTOR
//-----------------------------------------
static void turnOnMotor() {
  //T1CC3H = PWM_1/256;
  //T1CC3L = PWM_1%256;
  
  //set motor counter to 0
}


//-----------------------------------------
// TIMER INTERRUPT
//-----------------------------------------
_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void)
{
  //P1_0 = (P1_0 == 0);
  counter = counter+1;
  //if motor counter < 1000, motorcounter=motorcounter+1;
  //if motorcounter=1000, set flag
  
  if(counter>=160){//Every 200 ms
    P0_0 = !P0_0; //LED1
    counter=0;
    sendPressureFlag = 1;
    dataCounter++;
  }
}

static void turnOnCS(int sensorNum){
  if(sensorNum==0)
  {
    CS_1=1;
  }
  if(sensorNum==1)
  {
    CS_2=1;
  }
  if(sensorNum==2)
  {
    CS_3=1;
  }
  if(sensorNum==3)
  {
    CS_4=1;
  } 
}

static void turnOffCS(int sensorNum){
  if(sensorNum==0)
  {
    CS_1=0;
  }
  if(sensorNum==1)
  {
    CS_2=0;
  }
  if(sensorNum==2)
  {
    CS_3=0;
  }
  if(sensorNum==3)
  {
    CS_4=0;
  } 
}

void configureStuff()
{
  CLKCONCMD &= 0x00;//Set system clock to 32MHZ => set bit 6 to 1 => [-1-- ----]
  while(CLKCONSTA&0x40);//waiting for clock to become stable
  //-------------------------------------------------------------------------
  // TEST
  //-------------------------------------------------------------------------
  P1SEL &= ~ 0x01;  
  P1DIR |= 0x01;
  P1_0=1;
  //-------------------------------------------------------------------------
  // CONFIGURE ADC 5, 6, and 7
  //-------------------------------------------------------------------------
  P0SEL |=  0x13;//SET P0_7, 6, 5 to peripheral [---1 --11]
  P0DIR &= ~0x13;//SET P0_7, 6, 5 to input [---0  --00]
  APCFG |= 0x13;// Analog Peripheral I/O enable [---1 --11] pg.85, 134  
  
  //--------------------------------------------------------------------------
  // CONFIGURE SPI (USART 1 ALT 2)
  //--------------------------------------------------------------------------
  PERCFG |= 0x02; //SET USART 1 I/O location TO ALTERNATIVE 2 => set bit 1 to 1: [---- --1-] - Family pg. 85
  U1CSR &= ~0xA0; //Set USART 1 TO SPI MODE and Set USART 1 SPI TO MASTER MODE[0-0- ----]
  
  //-----------------------------------------------------
  // CONFIGURE SPI PERIPHERALS
  //-----------------------------------------------------  
  P1SEL |=0xE0; //set P1_5, P1_6, P1_7 are peripherals [111- ----]
  P1SEL &= ~0x1F;  //P1_4(CS),3(SDN),2(GROUND),1(VDD) are GP I/O (SSN) [---0 0000]
  P1DIR = 0x7F; // SET MO, C, CS, VDD, GND, SDN to output [0111 1111]
  //P1DIR &=~0x80; //SET MI to input[0--- ----]
  //-----------------------------------------------------
  // CONFIGURE SPI BAUD RATE
  //-----------------------------------------------------   
  U1BAUD = 0x3B;//BAUD_M= 59  //0x00;// BAUD_M = 0
  U1GCR |= 0x06;//BAUD_E = 6  //0x11;// BAUD_E = 17
  //-----------------------------------------------------
  // CONFIGURE SPI POLARITY, DATA TRANSFER, AND BIT ORDER
  //-----------------------------------------------------   
  //CPHA = 0 means:
  //Data is output on MOSI when SCK goes from CPOL inverted to CPOL, and data input
  //is sampled on MISO when SCK goes from CPOL to CPOL inverted.
  //CPOL = 0 => Clock polarity is negative
  U1GCR &= ~0xC0; //U1GCR.CPOL = U1GCR.CPHA = 0 [00-- ----] - familiy pg. 163
  U1GCR |=0x20;// U1GCR.ORDER = 1=> MSB first  [--1- ----]
  
  //-----------------------------------------------------
  // CONFIGURE TIMER 1
  //-----------------------------------------------------
  PERCFG |= 0x40; //[-1-- ----]  (TIMER 1 ALT 2)
  //PERCFG |= 0x03;
  P0SEL |= 0xC0; //[11-- ----] (CH 1 and CH 2 to peripheral)
  P0DIR |= 0xC0; //[11-- ----] (CH1 and CH2 to output)
  //------------------------
  // CONFIGURE 800Hz Timer
  //------------------------
  //Set divisor to 32 => clock speed is 1 MHz, Sets timer to up-down mode (Pg 114) (Family)
  T1CTL |= 0x0B; //[---- 1-11]
  T1CTL &= ~0x04; //[---- -0--]
  //T1CC0 = 625. Need 800hz trigger frq => (1MHZ timer/count)=800 => count = 1250.
  //In up-down mode, the timer counts up to a number, then counts down to zero to trigger interrupt.
  //We want the high number to be 1250/2 => 650. 
  T1CC0L = 0x71;
  T1CC0H = 0x02; 
  //------------------------
  // Set-up PWM
  //------------------------
  P2DIR |= 0xC0;
  //Test PWM.  CH1 MAX, CH2 MIN
  T1CC3H = 0x02;
  T1CC3L = 0x70;
  T1CC4H = 0x02;
  T1CC4L = 0x70;
  //Set each channel to set-up, clear-down and compare mode
  T1CCTL3 |= 0x1C;  //[---1 11--]
  T1CCTL3 &= ~0xE3;  //[-00- --00]
  T1CCTL4 |= 0x1C;  //[---1 11--]
  T1CCTL4 &= ~0xE3;  //[-00- --00]
  
  //  //To Test Timer (5Hz)
  P0SEL &= ~0x01;
  P0DIR |= 0x01;  
  
}