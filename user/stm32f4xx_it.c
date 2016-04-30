/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "delay.h"
#include "buzzer.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
	Millis++;
	
	if(buzzerTime < 0)
		beep_off;
	
	buzzerTime--;
	
	systick();	
	
}



/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/




void EXTI9_5_IRQHandler(void) {
	
	
	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
		
		// Disregard short button presses
		delay_ms(10);
		
		if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
			//debounce the button
			delay_ms(300);
			button0_interrupt();
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	
	
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		
		// Disregard short button presses
		delay_ms(10);
		
		if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
			//debounce the button
			delay_ms(300);	
			button3_interrupt();
			EXTI_ClearITPendingBit(EXTI_Line6);
		}
	}
	
}


void EXTI15_10_IRQHandler(void) {
	
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET) 
  {
		
		// Disregard short button presses
		delay_ms(10);
		
		if(EXTI_GetITStatus(EXTI_Line14) != RESET)  {
			//debounce the button
			delay_ms(300);
			button1_interrupt();
			EXTI_ClearITPendingBit(EXTI_Line14);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) 
  {

		// Disregard short button presses
		delay_ms(10);

		if(EXTI_GetITStatus(EXTI_Line15) != RESET)  {
			//debounce the button
			delay_ms(300);
			button2_interrupt();
			EXTI_ClearITPendingBit(EXTI_Line15);
		}
	}
	
}



/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/