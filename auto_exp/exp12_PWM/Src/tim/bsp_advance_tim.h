#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f4xx.h"

/* ��ʱ�� */
#define ADVANCE_TIM           				TIM1
#define ADVANCE_TIM_CLK_ENABLE()  			__TIM1_CLK_ENABLE()

/* TIM1ͨ��1������� */
#define ADVANCE_OCPWM_PIN           		GPIO_PIN_9              
#define ADVANCE_OCPWM_GPIO_PORT     		GPIOE                      
#define ADVANCE_OCPWM_GPIO_CLK_ENABLE() 	__GPIOC_CLK_ENABLE()
#define ADVANCE_OCPWM_AF					GPIO_AF1_TIM1 

/* TIM8ͨ��1����������� */
#define ADVANCE_OCNPWM_PIN            		GPIO_PIN_5              
#define ADVANCE_OCNPWM_GPIO_PORT      		GPIOA                      
#define ADVANCE_OCNPWM_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define ADVANCE_OCNPWM_AF					GPIO_AF1_TIM1 

/* TIM8��·�������� */
#define ADVANCE_BKIN_PIN              		GPIO_PIN_6              
#define ADVANCE_BKIN_GPIO_PORT        		GPIOA                      
#define ADVANCE_BKIN_GPIO_CLK_ENABLE()  	__GPIOA_CLK_ENABLE()
#define ADVANCE_BKIN_AF						GPIO_AF1_TIM1 


extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);

#endif /* __ADVANCE_TIM_H */

