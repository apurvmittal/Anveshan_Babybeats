

/**
 *****************************************************************************
   @example  babybeats.h 
   @brief    Header file for BabyBeats system

   @version  V0.1
   @author   TheTeam.Apurv
   @date     24/1/2015 


**/
#include <ADuCM360.h>

extern void LEDpin_config(void);
extern void UART_config(void);
extern void ADC1INIT(void);
extern void ADC_config(void);
extern void EXTINT_config(void);
extern void I2C_config(void);
extern void LED_Blink(int Blink_Rate);
extern void Sys_Conf(void);
extern int I2C_readrs(unsigned int sl_address, unsigned int reg_address, unsigned int num_bytes) ;
extern int I2C_write(unsigned int sl_address, unsigned int reg_address, unsigned int num_bytes);
extern int UART_Tx(unsigned char tx_data[]);
extern unsigned char uart_datainbuffer(void);
unsigned char uart_dataread(void);
extern long get_adcval(void);
extern unsigned char* get_adcvoltval(void);
extern unsigned int adc_check(void);
extern int ADT_init(void);
extern int ADT_convTemp(void);
extern int ADT_id(void);
extern int ADT_confreg(void);
extern int ADT_check(void);
extern int ADT_getTemp(unsigned int mode); 
extern void ADT_reset(void);
extern void delay(long int);

#define LED1_ON()   DioSet(pADI_GP1,0x8);
#define LED1_OFF()  DioClr(pADI_GP1,0x8);
#define I2C_error(A) LED_Blink(A)

#define MAX_COUNT 8
#define ADXL	0x50
#define ADT		0x48
#define ADT_WR_CMD (ADT<<1)
#define ADT_RD_CMD (ADT_WR_CMD +1)

#define ADT_ID_REG 0x0B
#define ADT_CONF_REG 0x03
#define ADT_TEMP_REG 0x00 
#define ADT_tCRIT_REG 0x08
#define ADT_tHIGH_REG 0x04
#define ADT_tLOW_REG 0x06


