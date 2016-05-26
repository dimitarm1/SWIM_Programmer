/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO/STM32
**                STMicroelectronics STM32F10x Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "swim.h"

int main(void)
{
	uint8_t data[4];
	uint32_t ret1, ret2, ret3, ret4;

	if((ret1 = swim_init()) == SWIM_ERR_OK)	//init
	{
		data[0] = SWIM_FLASH_PUKR_KEY1;
		if((ret2 = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data)) == SWIM_ERR_OK)	//write pukr1
		{
			data[0] = SWIM_FLASH_PUKR_KEY2;
			if((ret3 = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data)) == SWIM_ERR_OK) //write pukr2
			{
				data[0] = 0x4;
				ret4 = swim_write_on_the_fly(1, SWIM_PROGRAM + 0x200, data);	//write data (one byte to the right address)
			}
		}
	}
	else
	{
		ret1 = ret1;
	}

	RESET_ASSERT();
	uint8_t i = 0;
	while(--i);
	RESET_DEASSERT();

	while(1);

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval sEE_FAIL.
  */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Return with error code */
  return sEE_FAIL;
}
#endif
#endif /* USE_SEE */

