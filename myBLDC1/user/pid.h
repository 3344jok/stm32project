/**
  ******************************************************************************
  * @file    stm3210c_eval_lcd.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file contains all the functions prototypes for the lcd firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdio.h>
#include <string.h>

extern int PWM_u;
extern double u;
extern double r;
extern int PWM_r;
extern double e;
extern double p;
extern double i;
extern double d;
extern float speed;
extern double y;
extern uint32_t ADC_ConvertedValue;

extern void cal_PWM_u(void);
extern void cal_PWM_r(void);
extern void cal_u(void);
extern void cal_r(void);
extern void cal_e(void);
extern void cal_p(void);
extern void cal_i(void);
extern void cal_d(void);
extern void cal_y(void);

extern 

#endif 
/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
