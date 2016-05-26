/*
 * swim.c
 *
 *  Created on: Mar 8, 2013
 *      Author: Tomas Jakubik
 */

#include "swim.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"

uint32_t swim_init()
{
	uint32_t i;
	uint8_t data;
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	TIM_ICInitTypeDef TIMIC_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
		RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);

	swim_out_pin = SWIM_OUT_PIN;
	swim_out_port = SWIM_OUT_PORT;
	reset_pin = RESET_PIN;
	reset_port = RESET_PORT;
	tim = TIM3;

	//SWIM output
	SWIM_SET();
	GPIO_InitStructure.GPIO_Pin = swim_out_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(swim_out_port, &GPIO_InitStructure);
	//Reset pin
	RESET_DEASSERT();
	GPIO_InitStructure.GPIO_Pin = reset_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(reset_port, &GPIO_InitStructure);

	//tim = TIM3 - SWIM activation delay - set for SystemCoreClock = SYSCLK_FREQ_24MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Period = 0xffff;
	TIM_InitStruct.TIM_Prescaler = 0;	//24MHz
	TIM_TimeBaseInit(tim, &TIM_InitStruct);
	TIM_Cmd(tim, ENABLE);

//timer
	//TIM2 - set for SystemCoreClock = SYSCLK_FREQ_24MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Period = 0xffff;
	TIM_InitStruct.TIM_Prescaler = 0;	//24MHz
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);

	//PA0 - TIM2_CH1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PA1 - TIM2_CH2 TODO: remove
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	tim2 = TIM2;	//TODO: remove
	dma = DMA1_Channel7;

	//Channel 1 - counter reset
	TIMIC_InitStruct.TIM_Channel = TIM_Channel_1;
	TIMIC_InitStruct.TIM_ICFilter = 0;
	TIMIC_InitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIMIC_InitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIMIC_InitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &TIMIC_InitStruct);

	//Channel 2 - DMA input capture
	TIMIC_InitStruct.TIM_Channel = TIM_Channel_2;
	TIMIC_InitStruct.TIM_ICFilter = 0;
	TIMIC_InitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIMIC_InitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIMIC_InitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInit(TIM2, &TIMIC_InitStruct);
	TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);

	TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

	//DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStruct.DMA_BufferSize = 0;	//do not listen yet
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(&(swim_data.low));
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM2->CCR2));
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_Init(DMA1_Channel7, &DMA_InitStruct);
	TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

	//swim_data
	swim_data.to_listen = 0;
	swim_data.done = 1;

	TIM_Cmd(TIM2, ENABLE);

//Starting...
	DMA_SetCurrDataCounter(DMA1_Channel7, SWIM_BUF_SIZE);	//flush DMA
	DMA_Cmd(DMA1_Channel7, ENABLE);	//enable DMA

	tim->CNT = 0;
	while(tim->CNT < 24000);	//1ms

	DMA_Cmd(DMA1_Channel7, DISABLE);	//disable DMA
	DMA_SetCurrDataCounter(DMA1_Channel7, 9);	//get 9 items
	//TIM_ClearFlag(TIM2, TIM_FLAG_CC2);	//clear timer flag
	DMA_Cmd(DMA1_Channel7, ENABLE);	//enable DMA

	RESET_ASSERT();
	tim->CNT = 0;
	while(tim->CNT < 12000);	//0.5ms

//1 - LOW for 16us, I wait a bit more
	SWIM_RESET();
	tim->CNT = 0;
	while(tim->CNT < 4000);

//2 - 4 pulses 1kHz and 4 pulses 2kHz
	for(i = 0; i < 4; i++)
	{
		SWIM_SET();
		tim->CNT = 0;
		while(tim->CNT < 12000);
		SWIM_RESET();
		tim->CNT = 0;
		while(tim->CNT < 12000);
	}

	for(i = 0; i < 4; i++)
	{
		SWIM_SET();
		tim->CNT = 0;
		while(tim->CNT < 6000);
		SWIM_RESET();
		tim->CNT = 0;
		while(tim->CNT < 6000);
	}

	SWIM_SET();
	tim->CNT = 0;
	while(tim->CNT < 6000);

//3 - HSI is turned on

//4 - synchronization pulse
	if(DMA_GetCurrDataCounter(DMA1_Channel7))
	{
		RESET_DEASSERT();
		return SWIM_ERR_SYNC;	//target didn't responded
	}
	if((swim_data.low[1] < 1000) && (swim_data.low[3] < 1000))	//target responded to every low pulse with communication reset and sent sync pulse
	{
		if(swim_data.low[1] < 50)	//sync pulse is weird
		{
			RESET_DEASSERT();
			return SWIM_ERR_SYNC;
		}
		swim_data.bit_sync = swim_data.low[1];	//second pulse had to be sync
	}
	else	//target had to response to
	{
		if((swim_data.low[9] > 1000) || (swim_data.low[9] < 50))	//sync pulse is weird
		{
			RESET_DEASSERT();
			return SWIM_ERR_SYNC;
		}
		swim_data.bit_sync = swim_data.low[9];	//second pulse had to be sync
	}
	swim_data.bit_sync /= 32;	// /64 should be exact short pulse length, so /32 is limit for '1'

	DMA_Cmd(DMA1_Channel7, DISABLE);	//disable DMA channel
	DMA_ClearFlag(DMA1_FLAG_TC7);	//clear interrupt flag
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);	//enable interrupt for future use

//5 - 300ns delay, I wait a bit more
	tim->CNT = 0;
	while(tim->CNT < 12000);

//6 - write 0xa0 to SWIM_CSR
	data = 0xa0;
	if(swim_write_on_the_fly(1, SWIM_CSR, &data))
	{
		RESET_DEASSERT();
		return SWIM_ERR_FIRST_WRITE;
	}

//7 - release RESET and wait 1 ms
	RESET_DEASSERT();
	tim->CNT = 0;
	while(tim->CNT < 25000);

//8 - low speed active, SWIM clock = 8 MHz

	//Could check NO_ACCESS bit, but according to datasheet that could mean anything.

	return SWIM_ERR_OK;
}

/* void swim_listen_to(uint32_t number)
 *   sets DMA for measuring pulse lengths
 *   interrupt routine decodes them into bits and stores to swim_data.result
 * @param number number of pulses to measure
 */
void swim_listen_to(uint32_t number)
{
	DMA_SetCurrDataCounter(DMA1_Channel7, SWIM_BUF_SIZE);	//flush DMA
	DMA_Cmd(DMA1_Channel7, ENABLE);	//enable DMA

	if(number > SWIM_BUF_SIZE) number = SWIM_BUF_SIZE;	//check buffer size
	swim_data.done = 0;	//reset done flag
	swim_data.to_listen = number;	//set number for interrupt routine
	__NOP();
	__NOP();
	//hope DMA is flushed now

	DMA_Cmd(DMA1_Channel7, DISABLE);	//disable DMA channel to change CNDTR
	DMA_SetCurrDataCounter(DMA1_Channel7, number);	//set number for DMA
	DMA_ClearFlag(DMA1_FLAG_TC7);	//clear interrupt flag
	//TIM_ClearFlag(TIM2, TIM_FLAG_CC2);	//clear timer flag
	DMA_Cmd(DMA1_Channel7, ENABLE);	//enable DMA channel
}

void DMA1_Channel7_IRQHandler(void)
{
	uint32_t i;

	DMA_ClearFlag(DMA1_FLAG_TC7);	//clear interrupt flag

	if(swim_data.done)	//nothing to do
	{
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Channel7, 0);	//reset DMA address
	}
	else
	{
		swim_data.result = 0;	//clean
		for(i = 0; i < swim_data.to_listen; i++)
		{
			swim_data.result <<= 1;	//rotate and make space for new bit
			if(swim_data.low[i] < swim_data.bit_sync)	//decide if bit is 1
			{
				swim_data.result |= 0x1;
			}
		}
		swim_data.done = 1;	//all done
		DMA_Cmd(DMA1_Channel7, DISABLE);
	}
}

uint32_t swim_send_header(uint8_t header)
{
	uint32_t i;

	//listen for everything with ACK in the end
	tim->CNT = 0;
	swim_listen_to(6);

	//startbit
	SWIM_RESET();
	SWIM_LONG_DELAY();
	SWIM_SET();

	//3 bits and parity
	for(i = 0; i < 4; i++)
	{
		if(header & 0x08)
		{
			SWIM_RESET();
			SWIM_SET();
			SWIM_LONG_DELAY();
		}
		else
		{
			SWIM_RESET();
			SWIM_LONG_DELAY();
			SWIM_SET();
		}
		header <<= 1;
	}

	while(!swim_data.done)	//wait for ACK to be received
		if(tim->CNT > SWIM_DELAY_MAX)	//too long
		{
			return SWIM_ERR_HEADER;
		}

	if(!(swim_data.result & 0x1)) return SWIM_ERR_HEADER_NACK;	//NACK

	return SWIM_ERR_OK;
}

uint32_t swim_send_byte(uint8_t byte)
{
	uint32_t parity = 0;
	uint32_t temp;
	uint32_t i;

	//count parity
	temp = byte;
	for(i = 0; i < 8; i++)
	{
		parity ^= temp & 0x1;
		temp >>= 1;
	}

	//listen for everything with ACK in the end
	tim->CNT = 0;
	swim_listen_to(11);

	//startbit
	SWIM_RESET();
	SWIM_LONG_DELAY();
	SWIM_SET();

	//data bits
	for(i = 0; i < 8; i++)
	{
		if(byte & 0x80)
		{
			SWIM_RESET();
			SWIM_SET();
			SWIM_LONG_DELAY();
		}
		else
		{
			SWIM_RESET();
			SWIM_LONG_DELAY();
			SWIM_SET();
		}
		byte <<= 1;
	}

	if(parity)
	{
		SWIM_RESET();
		SWIM_SET();
		SWIM_LONG_DELAY();
	}
	else
	{
		SWIM_RESET();
		SWIM_LONG_DELAY();
		SWIM_SET();
	}

	while(!swim_data.done)	//wait for ACK to be received
		if(tim->CNT > SWIM_DELAY_MAX)	//too long
		{
			return SWIM_ERR_BWRITE;
		}

	if(!(swim_data.result & 0x1)) return SWIM_ERR_BWRITE_NACK;	//NACK

	return SWIM_ERR_OK;
}

uint32_t swim_system_reset()
{
	return swim_send_header(SWIM_SRST);
}

uint32_t swim_write_on_the_fly(uint8_t n, uint32_t address, uint8_t* data)
{
	uint32_t i;
	uint32_t ret;
	uint32_t nacked = 0;

	//header
	if(swim_send_header(SWIM_WOTF) != SWIM_ERR_OK) return SWIM_ERR_WRITE;

	//send n
	if(swim_send_byte(n) != SWIM_ERR_OK) return SWIM_ERR_WRITE;

	//send addres byte 3, 2 and 1
	if(swim_send_byte((address & 0xff0000) >> 16) != SWIM_ERR_OK) return SWIM_ERR_WRITE;
	if(swim_send_byte((address & 0x00ff00) >> 8) != SWIM_ERR_OK) return SWIM_ERR_WRITE;
	if(swim_send_byte(address & 0x0000ff) != SWIM_ERR_OK) return SWIM_ERR_WRITE;

	//send data
	for(i = 0; i < n; i++)
	{
		if((ret = swim_send_byte(data[i])) != SWIM_ERR_OK)
		{
			if(ret == SWIM_ERR_BWRITE_NACK)
			{
				i--;	//send again
				if(nacked++ > SWIM_NACK_MAX) return SWIM_ERR_WRITE;
			}
			else return SWIM_ERR_WRITE;
		}
	}

	return SWIM_ERR_OK;
}

uint32_t swim_read_on_the_fly(uint8_t n, uint32_t address, uint8_t* data)
{
	uint32_t i, j;
	uint32_t parity, temp;
	uint32_t nacked = 0;

//header
	if(swim_send_header(SWIM_ROTF) != SWIM_ERR_OK) return SWIM_ERR_READ;

//send n
	if(swim_send_byte(n) != SWIM_ERR_OK) return SWIM_ERR_READ;

//send addres byte 3 and 2
	if(swim_send_byte((address & 0xff0000) >> 16) != SWIM_ERR_OK) return SWIM_ERR_READ;
	if(swim_send_byte((address & 0x00ff00) >> 8) != SWIM_ERR_OK) return SWIM_ERR_READ;

	swim_listen_to(21);	//get 21 bits

//send last address byte
	//count parity
	parity = 0;
	temp = (address & 0x0000ff);
	for(i = 0; i < 8; i++)
	{
		parity ^= temp & 0x1;
		temp >>= 1;
	}

	//startbit
	SWIM_RESET();
	SWIM_LONG_DELAY();
	SWIM_SET();

	//data bits
	for(i = 0; i < 8; i++)
	{
		if(address & 0x80)
		{
			SWIM_RESET();
			SWIM_SET();
			SWIM_LONG_DELAY();
		}
		else
		{
			SWIM_RESET();
			SWIM_LONG_DELAY();
			SWIM_SET();
		}
		address <<= 1;
	}

	//sendparity
	if(parity)
	{
		SWIM_RESET();
		SWIM_SET();
		SWIM_LONG_DELAY();
	}
	else
	{
		SWIM_RESET();
		SWIM_LONG_DELAY();
		SWIM_SET();
	}

//catch all data
	for(i = 0; i < n; i++)
	{
		tim->CNT = 0;
		while(!swim_data.done)	//wait for data
			if(tim->CNT > SWIM_DELAY_MAX)	//too long
			{
				return SWIM_ERR_READ;
			}

		//count parity
		temp = swim_data.result >> 1;
		data[i] = temp;
		parity = 0;
		for(j = 0; j < 8; j++)
		{
			parity ^= temp & 0x1;
			temp >>= 1;
		}

		//check address ack
		if(i == 0)
		{
			if(!(swim_data.result & 0x400))	//NACK in last byte of the address
				return SWIM_ERR_READ;
		}

		if(!(swim_data.result & 0x200) || (parity != (swim_data.result & 0x1)))	//startbit or parity is wrong
		{
			//NACK
			swim_listen_to(11);	//get 11 bits of next byte
			SWIM_RESET();
			SWIM_LONG_DELAY();
			SWIM_SET();
			i--;
			if(nacked++ > SWIM_NACK_MAX) return SWIM_ERR_WRITE;
		}
		else
		{
			//ACK
			if(i != (n-1)) swim_listen_to(11);	//get 11 bits of next byte
			SWIM_RESET();
			SWIM_SET();
		}
	}

	return SWIM_ERR_OK;
}






