/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f1xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/**
  * @brief  This function handles NMI Interrupt.
  * @param  errorMsg c-string that is being written during error
  * @retval None
  */
void DefErrorHandler(const char* errorMsg)
{
	printf(errorMsg);
	#ifdef DEBUG
	__BKPT(0);
	#endif
}

/**
  * @brief  This function handles a NMI interrupt.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	DefErrorHandler("Nmi interrupt was triggered.");
}

/**
  * @brief  This function handles a HardFault interrupt.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	DefErrorHandler("HardFault interrupt was triggered.");
}

/**
  * @brief  This function handles a MemManage interrupt.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	DefErrorHandler("MemManage interrupt was triggered.");
}

/**
  * @brief  This function handles a BusFault interrupt.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	DefErrorHandler("Bus Fault interrupt was triggered.");
}

/**
  * @brief  This function handles a UsageFault interrupt.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	DefErrorHandler("Usage fault interrupt was triggered");
}
