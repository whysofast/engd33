/*
 * serial_uart.c
 *
 *  Created on: Nov 27, 2021
 *      Author: user
 */


#include <stdint.h>

#include "main.h"
#include "serial_uart.h"



#define UART_RX_BUFFER_SIZE                                       64// O valor que for necessário


UART_HandleTypeDef *hUART; //UART configurada no arquivo .ioc

static uint8_t UART_RX_DATA;

//Variáveis usadas para implementar um buffer circular
//Ver https://en.wikipedia.org/wiki/Circular_buffer
//Ver https://www.embarcados.com.br/buffer-circular-para-sistemas-embarcados/
volatile uint8_t UART_RX_BUFFER[UART_RX_BUFFER_SIZE]; //Buffer para armazenar os dados recebidos
volatile uint16_t UART_RX_READ_POINTER = 0; //Indica a posição no buffer que deve ser retornado por Serial_Read
volatile uint16_t UART_RX_HEAD_POINTER = 0; //Indica a posição no buffer onde um novo byte recebido deve ser copiado
volatile uint8_t  UART_RX_ERR_FLAG = 0;


void Serial_begin(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef status;
	status = HAL_UART_Receive_IT(huart, &UART_RX_DATA, 1);//Inicia nova recepção de apena um byte por interrupção
	if(status != HAL_OK)
	{
		//Possível tratamento de erro
	}
	else
	{
		hUART = huart;
	}
}


void Serial_end(void)
{
	HAL_StatusTypeDef status;
	__disable_irq(); //Disabling interruptions
	status = HAL_UART_AbortReceive_IT(hUART);//Para interrupção pra recepção da UART
	__enable_irq(); //Enabling interruptions
	if(status != HAL_OK)
	{
		//Possível tratamento de erro
	}
	else
	{
		hUART = NULL;
	}
}


uint16_t Serial_Available(void)
{
	uint16_t available;
	__disable_irq(); //Disabling interruptions
	if(UART_RX_HEAD_POINTER >= UART_RX_READ_POINTER)
	{
		available = UART_RX_HEAD_POINTER - UART_RX_READ_POINTER;
	}
	else
	{
		available = (UART_RX_BUFFER_SIZE - UART_RX_READ_POINTER) + UART_RX_HEAD_POINTER;
	}
	__enable_irq(); //Enabling interruptions
	return available;
}


uint8_t Serial_Read(void)
{
	uint8_t byte;

	__disable_irq(); //Disabling interruptions
	byte = UART_RX_BUFFER[UART_RX_READ_POINTER];
	if(UART_RX_READ_POINTER != UART_RX_HEAD_POINTER)
	{
		UART_RX_READ_POINTER++;
	}
	if(UART_RX_READ_POINTER >= UART_RX_BUFFER_SIZE)
	{
		UART_RX_READ_POINTER = 0;
	}
	__enable_irq(); //Enabling interruptions
  return byte;
}


void Serial_Write(uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(hUART, buf, len, 1);
}



/**
 * @brief Função de callback para recepção da UART por interrupção completa.
 * É chamada de dentro da função de interrupção da UART assim que o número configurado de
 * bytes é recebido.
 * Nesta função não podem ser usados loops infinitos, funções de delay nem de temporização.
 * O protótipo desta função pode ser encontrado em stm32f4xx_hal_uart.h
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef status;
	if(huart->Instance == USART2)
	{
		UART_RX_BUFFER[UART_RX_HEAD_POINTER] = UART_RX_DATA;  //Salva dado novo na cabeça do buffer
		UART_RX_HEAD_POINTER ++;                              //Incrementa indicador do local da cabeça
		if(UART_RX_HEAD_POINTER >= UART_RX_BUFFER_SIZE)       //Zera local da cabeça caso tenha atingido fim do buffer
		{
			UART_RX_HEAD_POINTER = 0;                         //
		}
		status = HAL_UART_Receive_IT(huart, &UART_RX_DATA, 1);//Inicia nova recepção de apena um byte por interrupção
		if(status != HAL_OK)
		{
                                                              //Possível tratamento de erro
		}

	}

}
