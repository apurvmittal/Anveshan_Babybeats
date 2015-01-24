
/**
 *****************************************************************************
   @example  BB_system.c 
   @brief    Main .c file for BabyBeats system

   @version  V0.1
   @author   TheTeam.Apurv
   @date     24/1/2015 


**/

//#include "BB_Lib.c"
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
#include <..\common\UrtLib.h>
#include <..\common\AdcLib.h>


//******************** Main ***************************

int main (void)
{
  
	Sys_Conf();
	LED_Blink(300000);
	ADT_check();
	delay(500000);
	ADT_init(); 
	ADT_getTemp(0xA0);  //Configure ADT in mode 0xA0 and get temperature
	while(1);
	
} // end of main
