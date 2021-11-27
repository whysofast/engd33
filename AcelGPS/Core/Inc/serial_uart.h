/*
 * serial_uart.h
 *
 *  Created on: Nov 27, 2021
 *      Author: user
 *
 *      Código de exemplo:
 *
 *      int main(void)
 *      {
 *      	Serial_begin(&huart2);  //Configurando para transmitir e recebervia uart
 *
 *      	while(1)
 *      	{
 *      		if(Serial_Available > 0)  //Verificando se tem byter recebidos
 *      		{
 *      			uint8_t byte = Serial_Read(); //Lendo o byte
 *      			Serial_Write(&byte, 1);  //Enviando o byte recebido, como se fosse um eco
 *      		}
 *      	}
 *      }
 *
 */

#ifndef SERIAL_UART_H
#define SERIAL_UART_H


#include <stddef.h>

/**
 * @brief Inicia a recepção por interrupção
 */
void Serial_begin(UART_HandleTypeDef *huart);

/**
 * @brief Aborta recepção por interrupção
 */
void Serial_end(void);

/**
 * @brief Visa copiar o comportamento da função Serial.available do Arduino
 * Retorna o número de bytes disponíveis para leitura no buffer.
 */
uint16_t Serial_Available(void);

/**
 * @brief Visa copiar o comportamento da função Serial.read do Arduino
 * Retorna um byte do buffer de recepção
 * Ao contrário da função original, não retorna -1 caso o buffer esteja vazio.
 * Logo, deve ser chamado apenas após Serial_available ser testado.
 */
uint8_t Serial_Read(void);

/**
 * Função visa copiar o comportamento da função Serial.write do Arduino
 * Ao contrário da função original, não retorna o número de bytes enviados
 * Também não possui comportamento não bloqueante, ainda que a função original
 * fique bloqueada caso o buffer interno fique cheio.
 */
void Serial_Write(uint8_t *buf, uint16_t len);

#endif /* SERIAL_UART_H */
