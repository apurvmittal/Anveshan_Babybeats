/**
 *****************************************************************************
   @example  BB.c 
   @brief    Library .c file for BabyBeats system

   @version  V0.1
   @author   TheTeam.Apurv
   @date     22/1/2015 


**/
#include	"babybeats.h"
#include <stdio.h>
#include <string.h>
#include <ADuCM360.h>

#include <..\common\ClkLib.h>
#include <..\common\IntLib.h>
#include <..\common\DioLib.h>
#include <..\common\WdtLib.h>
#include <..\common\DioLib.h>
#include <..\common\I2cLib.h>
#include <..\common\UrtLib.h>
#include <..\common\AdcLib.h>

// Variables valid for I2C
unsigned int uiMasterRxIndex = 0;
unsigned int uiMasterTxIndex = 0;
unsigned int uiRxLength = 0;
unsigned int uiTxLength = 0;
unsigned char ucMasterTxDat[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
unsigned char ucMasterRxDat[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //Maximum number of bytes which one can receive is 2 for ADT
unsigned char ucComplete = 0;  //Flag to indicate if uC is out of ISR
unsigned int sl_addr; //slave address variable for ADT, ADXL
unsigned int repeat_start=0;  //flag to indicate if repeat-start is required
int temp;		//global variable to get temperature

// Variables for UART 
volatile unsigned char ucComRx = 0;       //Data received from UART
unsigned char ucTxBufferEmpty  = 0;				// Used to indicate that the UART Tx buffer is empty
// UART-based external variables
unsigned char UART_data[64] = " Fuck the World";         // Used to store string before printing to UART
unsigned char ucPacketRcvd = 0;                      // Flag to indicate Received packet

// Variables for ADC1
unsigned char* get_adcvoltval(void);
volatile unsigned char ADC_Ready = 0;	  // Flag used to indicate ADC0 resutl ready 	
unsigned char ADC_data[64] = "";					// Used to store ADC0 result 
volatile  long ulADC1Result = 0;		        // Variable that ADC1DAT is read into in ADC1 IRQ
volatile unsigned char ucADC0ERR = 0;
float  fVoltage = 0.0;   			            // ADC value converted to voltage
float fVolts = 0.0;

// Simple Delay routine
void delay (long int length)
{
   while (length >0)
      length--;
}

void LEDpin_config(void)
{
   DioOen(pADI_GP1,0x08);    // Set P1.3 as an output to toggle the LED
   DioOen(pADI_GP1,0x05);    // Set P1.0, P1.2 as an indicator of successfull temp read
	 // The remaining pins of P1 are configured as input pins - External interrupt pins 
 	 
}

//Generic function to be called for configuring UART

void UART_config(void)
{
	 //Select IO pins for UART.
   pADI_GP0->GPCON |= 0x3C;                     // Configure P0.1/P0.2 for UART
   UrtCfg(pADI_UART,B9600,COMLCR_WLS_8BITS,0);  // setup baud rate for 9600, 8-bits
   UrtMod(pADI_UART,COMMCR_DTR,0);              // Setup modem bits
   UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI|COMIEN_ELSI|COMIEN_EDSSI|COMIEN_EDMAT|COMIEN_EDMAR);  // Setup UART IRQ sources
   NVIC_EnableIRQ(UART_IRQn);  //Enable Uart Interrupts
}

void ADC1INIT(void)
{
   AdcMski(pADI_ADC1,ADCMSKI_RDY,1);              // Enable ADC ready interrupt source		
   AdcFlt(pADI_ADC1,124,14,FLT_NORMAL|ADCFLT_NOTCH2|ADCFLT_CHOP); // ADC filter set for 3.75Hz update rate with chop on enabled
   AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G4,ADCCON_ADCCODE_INT); // Internal reference selected, Gain of 4, Signed integer output
   // Turn off input buffers to ADC and external reference	
   AdcBuf(pADI_ADC1,ADCCFG_EXTBUF_OFF,ADCCON_BUFBYPN|ADCCON_BUFBYPP|ADCCON_BUFPOWP|ADCCON_BUFPOWN); 
   AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN0,ADCCON_ADCCP_AIN1); // Select AIN1 as postive input and AIN0 as negative input
}

void ADC_config()
{
	 AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE);			// Place ADC1 in Idle mode
   ADC1INIT();															// Setup ADC1
   AdcGo(pADI_ADC1,ADCMDE_ADCMD_CONT);			// Start ADC1 for continuous conversions
   NVIC_EnableIRQ(ADC1_IRQn);								// Enable ADC1 and UART interrupt sources		
}

void EXTINT_config(void)
{
 	DioOen(pADI_GP0, 0x00);     // Set P0 as an input port for ADXL interrupt pin
	DioPul(pADI_GP0, 0xF9);     // ADXL, External pull ups required externally at P0.6
	DioPul(pADI_GP1, 0xFE);     // ADT, External pull ups required externally at P1.1
  
	//Enable IRQ4 to detect a falling edge at INT pin of ADT7420, high temp alert
	EiCfg(EXTINT4,INT_EN,INT_FALL);   
	//Enable IRQ2, falling edge detection, for ADXL, free fall, inactivity alerts
	EiCfg(EXTINT2,INT_EN,INT_FALL);  
}

void I2C_config(void)
{
	
  DioCfg(pADI_GP2, 0x05);     // Configure P2.0/P2.1 as I2C pins; refer UG-367, Digital Port Multiplex
  DioPul(pADI_GP2, 0xFC);     // External pull ups required externally
  
	// Enable I2C Master mode, baud rate and Interrupt Sources
  I2cMCfg(I2CMCON_TXDMA_DIS|I2CMCON_RXDMA_DIS, I2CMCON_IENCMP|I2CMCON_IENRX|I2CMCON_IENTX|I2CMCON_IENNACK, I2CMCON_MAS_EN); 
  I2cBaud(0x4E,0x4F); // 100kHz clock
  NVIC_EnableIRQ(I2CM_IRQn);
	
}

void LED_Blink(int Blink_Rate)
{
	   int j=0;
	   while(j<5) {
 			// if not in final state = 3 then blink 
      
	    LED1_OFF(); 					// LED at P1.3 off
      delay(Blink_Rate);      // Delay routine
      LED1_ON(); 						// LED at P1.3 on
      delay(Blink_Rate);    // Delay routine
      j++;			
		 }
  
}

void Sys_Conf(void)
{
  WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS);  // Disable Watchdog timer resets
	
  //Disable clock to unused peripherals
	//I2C, UART and ADC are being used
  ClkDis(CLKDIS_DISSPI0CLK|CLKDIS_DISSPI1CLK|
	CLKDIS_DISPWMCLK|CLKDIS_DIST0CLK|CLKDIS_DIST1CLK|
	CLKDIS_DISDACCLK|CLKDIS_DISDMACLK);
  
	ClkCfg(CLK_CD0,CLK_HF,CLKSYSDIV_DIV2EN_DIS,CLK_UCLKCG);            // Select CD0 for CPU clock - 16Mhz clock standard

  // configure the clocks for I2C, UART, ADC
	// Add 
  ClkSel(CLK_CD0,CLK_CD0,CLK_CD0,CLK_CD0);  //SPI, I2C. UART, PWM
	
	LEDpin_config();
	I2C_config();
	UART_config();
	ADC_config();
	EXTINT_config();
}

//************************* I2C read, write functions ****************************
//This function implements a write-repeat_start-read sequence
//Returns the pointer to the array data in case of successfull read
//Appropriate flags are set in case of failed transaction

int I2C_readrs( unsigned int sl_address, unsigned int reg_address, unsigned int num_bytes) 
// Always the data will be returned in ucMasterRxDat
{
 	unsigned int i = MASTER;  //always MASTER mode
    sl_addr = sl_address;     // Pass slave address as a global variable to ISR
	  uiRxLength = num_bytes;   //Inform ISR number of bytes to read
	  repeat_start=1;
	  ucComplete = 0;           //Flag for ISR to set when work is completed
    uiTxLength = 0;
	
		I2cTx(i,reg_address);   			// point to reg to be read. 
		I2cMWrCfg(sl_address);			// Select the I2C slave device, 7 bit + R/W=0.
    while(!ucComplete){};   // run a timer inside the while
	
    if(ucComplete>1)
		return 0;         // Failure; see error code in ucComplete value
	else
		return 1;			   // Success
}

int I2C_write(unsigned int sl_address, unsigned int reg_address, unsigned int num_bytes)
// Always the data will be accepted in ucMasterTxDat
{
 	unsigned int i = MASTER;  //always MASTER mode
  sl_addr = sl_address;     // Pass slave address as a global variable to ISR
	uiTxLength = num_bytes;   //Inform ISR number of bytes to write
  uiRxLength = 0;                  	
	repeat_start=0;
	ucComplete = 0;           //Flag for ISR to set when work is completed

	I2cTx(i, reg_address);   			// point to reg to be read. 
	I2cMWrCfg(sl_address);			// Select the I2C slave device, 7 bit + R/W=0.
  while(!ucComplete){};   // run a timer inside the while
	
	if(ucComplete>1)
		return 0;         // Failure; see error code in ucComplete value
	else
		return 1;			   // Success
	
}

//************************* I2C - End ****************************



//*************************** UART Tx, Rx functions ***********************************
//Generic function to be called for data transmission and reception. Can directly read  the received array
// Returns 1 for success and true for failure, txrx=0 for transmit,1 for receive,
//to be transmitted data to be loaded into tx_data, and received data will be in rx_data
int UART_Tx( unsigned char tx_data[])
{
	   while (1)
   {
      
				int nLen,i = 0;
         DioTgl(pADI_GP1,0x8);            // Toggle P1.3
         nLen = strlen((char*)tx_data);      // Call function to calcualte the length of scanned string
         if (nLen <64) //Keeping the size limit as 64
         {
            for (i = 0 ; i < nLen ; i++ )	// loop to send ADC1 result	to UART
            {
               ucTxBufferEmpty = 0;	   // Clear flag
               UrtTx(pADI_UART,UART_data[i]);// Load UART Tx register.
               while (ucTxBufferEmpty == 0)// Wait for UART Tx interrupt
               {
               }
            }
         }
      }
//			if( ucPacketRcvd == 1) 
//			{
//				if(j==34){j=0;}//Reset j
//				ucPackedRcvd = 0;
//				for(j=0;j<34;j++)
//				{
//									rx_data[j] = ucComRx;

//				}
//			}
		

 }

 //Function to be checked for finding weather any data is available to be read by the controler from UART
 //If return value is 1, read the data from the UART. If 0 no data to read
//Define this as extern function available to all users 
unsigned char uart_datainbuffer()
 {
	 if(ucPacketRcvd == 1)
	 {

		   return 1;
	 }
	 else
	 {
		 return 0;
	 }
 }
 
 //Function to reset the flag after reading the data from UART
 //Function to be external
 
unsigned char uart_dataread()
 {
	 ucPacketRcvd = 0;
	 return ucComRx;
 }

//************************* UART - End ****************************


 //************************* ADC Functions ****************************
 long get_adcval(void)
{

	ADC_Ready = 0;
	return ulADC1Result;

}

//Function to convert ADC value to a voltage. Get it to a string. Both Global functions.
unsigned char* get_adcvoltval(void)
{
	   unsigned char nLen = 0;

	      if (ADC_Ready == 1) 				// ADC result ready to be sent to UART
      {
         DioTgl(pADI_GP1,0x8);            // Toggle P1.3
         ADC_Ready = 0;            // Clear flag
         fVolts = fVoltage;
         fVolts   = (1.2 / 268435456);      // Internal reference, calculate lsb size in volts
         fVoltage = (ulADC1Result * fVolts);   // Calculate ADC result in volts
         sprintf ( (char*)ADC_data, "%f\n\0",fVoltage );// Scan string with the Temperature Result                           
      }
			else
				sprintf ( (char*)ADC_data, "\0");
			return ADC_data;
}

//Function to check ADC working before a read. Should be global

unsigned int adc_check()
{
	if(ADC_Ready ==1 )
{
	return 1;
}
else
{
	return 0;
}
}
//************************* ADC - End ****************************

// Configure ADT in appropriate mode 

int ADT_init(void) 
{
  ucMasterTxDat[0] = 0xA0; //Refer ADT_getTemp for detail
	if(I2C_write(ADT_WR_CMD, ADT_CONF_REG, 1))
	{  
		ucMasterTxDat[0] = 0xA6; //configure the T_crit
				if(I2C_write(ADT_WR_CMD, ADT_tCRIT_REG, 1))
				{
					 ucMasterTxDat[0] = 0xA6; //configure the T_high
							if(I2C_write(ADT_WR_CMD, ADT_tHIGH_REG, 1))
							{
								 ucMasterTxDat[0] = 0xA6; //configure the T_low
								 if(I2C_write(ADT_WR_CMD, ADT_tLOW_REG, 1))
									 return 1;
				       }  	 
        }		
	}
		else
			return 0;
	
	//decide what to do if write failed	
  return 0;	
} // End of ADT_Init


//COnvert the 2 bytes into an int and indicate successful read of temp
//If MS bit is 1 (temp read is negative), return 0

int ADT_convTemp()
{
	unsigned char temp_byte = (ucMasterRxDat[0] << 8) + ucMasterRxDat[1]; //MSB is [0]
  int i,temp_int=0,t0;
  if(temp_byte & (0x01 << 15))
	{
    for(i=0;i<15;i++)
	 	{
			if(temp_byte & (0x01 << i))
			{
				t0 = 0.0078*(2^i);
			}
			else
			{
				t0 = 0;
			}
			  temp_int = temp_int + t0;
		 }
		 return temp_int;
  }
	else
		return 0;
}
	


int ADT_id()
{
  int t = 0;
	t = I2C_readrs(ADT_WR_CMD, ADT_ID_REG, 1);
	if(t)
	 {  
		 if(ucMasterRxDat[0]==0xCB)  //check if data is CB
     {
			 LED_Blink(150000);
			 delay(500000);
			 return 1;			
		 }
		 else
			 {
				 LED_Blink(1500000);
				 delay(500000);
			   return 0;
       }
	 } 
	else
	 {
			LED_Blink(300000);
		  delay(500000);
			return 0;
	 }
	
}

int ADT_confreg()
{
 	int t = 0;
	t = I2C_readrs(ADT_WR_CMD, ADT_CONF_REG, 1);   //reads configuration register located at 0x03	
	if(t)
	 {  
		 if(ucMasterRxDat[0]==0x00)  //check if data is 00
     {
			 LED_Blink(150000);
			 delay(500000);
			 return 1;			
		 }
		 else
			 {
				 LED_Blink(1500000);
				 delay(500000);
			   return 0;
       }
	 } 
	else
	 {
			LED_Blink(300000);
		  delay(500000);
			return 0;
	 }
	
}

// This function checks the uC - ADT7420 interface
int ADT_check()
{
	  int id=0;
	  int conf=0;
	  id = ADT_id();
	 	conf = ADT_confreg();
	  return (id && conf);
}

//Configuraion register detail 
//[1:0]   00
//2       0 --- CT pin active low
//3       0 --- INT pin active low
//4       0 --- Interrupt mode
//[6:5]   01 --- one shot
// 				11 --- shutdown mode
// 				00 --- continuous conversion
// 				10 --- 1 sps
//7				1  --- 16 bit resolution

//For ADT configured in one shot mode, 
//everytime a new temperature reading is required
//configuration register needs to be written with 0xA0 

int ADT_getTemp(unsigned int mode)  
{
	int st = 0;
	ucMasterTxDat[0] = mode; 
	if(I2C_write(ADT_WR_CMD, ADT_CONF_REG, 1))
	{
		st = I2C_readrs(ADT_WR_CMD, ADT_TEMP_REG, 2);
		if(!st)
		{
			LED_Blink(12000);
			return 0;
		}
		else
		{
			temp = ADT_convTemp();
		  return 1;
	  }
  }
	
	return 0; //decide what to do if not able to read 
}

void ADT_reset()
{
	if(I2C_write(ADT_WR_CMD, 0x2F, 1))
	  delay(10000);  //should wait for 200 us for ADT to reset itself
  else
		LED_Blink(20000);
}


//ISRs for External Interrupt, UART, I2C, ADC1

void WakeUp_Int_Handler(void)
{
  
}

//For ADT7420, temp alert
void Ext_Int4_Handler ()
{           
  ADT_getTemp(0xA0);
	//Send 'temp' to UART
} 

//For  ADXL, free fall, inactivity alert
void Ext_Int2_Handler ()
{         
  //Send 'Freefall, inactivity, wakeup' to UART
}

void ADC0_Int_Handler()
{
   
}

//Neeraj
void ADC1_Int_Handler ()
{
   volatile unsigned int uiADCSTA = 0;
   
   uiADCSTA = pADI_ADC1->STA;               // read ADC status register
   ulADC1Result = AdcRd(pADI_ADC1);         // read ADC result register
   ADC_Ready = 1;                    // Set flag to indicate ready to send result to UART
}

//Neeraj
void UART_Int_Handler ()
{
   volatile unsigned char ucCOMSTA0 = 0;
   volatile unsigned char ucCOMIID0 = 0;
   volatile unsigned int uiUartCapTime = 0;
   
   ucCOMSTA0 = UrtLinSta(pADI_UART);         // Read Line Status register
   ucCOMIID0 = UrtIntSta(pADI_UART);         // Read UART Interrupt ID register         
   if ((ucCOMIID0 & 0x2) == 0x2)             // Transmit buffer empty
   {
      ucTxBufferEmpty = 1;
   }
   if ((ucCOMIID0 & 0x4) == 0x4)             // Receive byte
   {
      ucComRx   = UrtRx(pADI_UART);
			ucPacketRcvd = 1;                     // Flag to indicate packet received

   }
} 

///////////////////////////////////////////////////////////////////////////
// I2C0 master handler 
///////////////////////////////////////////////////////////////////////////
void I2C0_Master_Int_Handler(void) 
{
  unsigned int uiStatus;
  uiStatus = I2cSta(MASTER);

	if((uiStatus & I2CMSTA_NACKADDR) == I2CMSTA_NACKADDR) //Address NACK IRQ
	{
		ucComplete = 2;
	}
	
	if(((uiStatus & I2CMSTA_NACKDATA) == I2CMSTA_NACKDATA)) //Data NACK IRQ
	{
		ucComplete = 3;
	}		
	
  if((uiStatus & I2CMSTA_RXREQ) == I2CMSTA_RXREQ)	// Master Recieve IRQ
  {
// 	if (uiRxLength)
// 		{
			ucMasterRxDat[uiMasterRxIndex++] = I2cRx(MASTER); //    
			//uiRxLength--;

    if(uiMasterRxIndex > (sizeof(ucMasterRxDat)-1) ) 			 // Resetting value of i if it has been incremented by RX // buffer rollover
      uiMasterRxIndex = 0;
		//}
		
  }
  
  if((uiStatus & I2CMSTA_TXREQ) == I2CMSTA_TXREQ) //Master Transmit IRQ	
  {
   
		if(repeat_start)
		{
			I2cMRdCfg(sl_addr, uiRxLength , DISABLE); //Uncomment this line inorder to generate a repeat start
			repeat_start=0;
 		}
 		else
		{
	  if (uiTxLength)
		{
			 I2cTx(MASTER,ucMasterTxDat[uiMasterTxIndex]);
			 uiMasterTxIndex++;
			 uiTxLength--;
		}
		else
  		I2cMCfg(I2CMCON_TXDMA_DIS|I2CMCON_RXDMA_DIS, I2CMCON_IENCMP|I2CMCON_IENRX|I2CMCON_IENTX_DIS, I2CMCON_MAS_EN);    // TXREQ disabled to avoid multiple unecessary interrupts   
    }	
  }
	
  if((uiStatus & I2CMSTA_TCOMP_SET) == I2CMSTA_TCOMP_SET) // communication complete	
  {
    I2cMCfg(I2CMCON_TXDMA_DIS|I2CMCON_RXDMA_DIS, I2CMCON_IENCMP|I2CMCON_IENRX|I2CMCON_IENTX_EN, I2CMCON_MAS_EN);   // TXREQ enabled for future master transmissions    
    ucComplete = 1;
	  
		
  }

}
