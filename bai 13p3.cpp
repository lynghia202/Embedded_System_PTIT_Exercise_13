#include "stm32f10x.h"                  
#include "FreeRTOS.h"                   
#include "task.h"                       
#include "semphr.h"                     
#include "stm32f10x_usart.h"            
#include "stm32f10x_rcc.h"              
#include "stm32f10x_gpio.h"             

SemaphoreHandle_t xUartMutex;   // tao bien mutex de bao ve UART

void uart_init(uint32_t baudrate);
void uart_send_char(char c);
void taskA(void *pvParameters);
void taskB(void *pvParameters);

int main(void)
{
	SystemInit();
	uart_init(9600); // khoi tao UART1

	xUartMutex = xSemaphoreCreateMutex(); // tao mutex UART

	const char *startMsg = "--- UART Mutex Demo Start ---\r\n";
	while(*startMsg) uart_send_char(*startMsg++); // gui thong bao khoi dong

	if (xUartMutex != NULL)
	{
		// tao 2 task co cung do uu tien
		xTaskCreate(taskA, "TASKA", 128, NULL, 1, NULL);
		xTaskCreate(taskB, "TASKB", 128, NULL, 1, NULL);

		vTaskStartScheduler(); // bat dau scheduler
	}
	else
	{
		while(1); // neu tao mutex that bai thi dung lai
	}

	while(1);
}

void uart_init(uint32_t baudrate)
{
	// cau hinh GPIO cho UART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_9; // TX
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_10; // RX
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	// cau hinh UART1
	USART_InitTypeDef uart;
	uart.USART_BaudRate = baudrate;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &uart);
	USART_Cmd(USART1, ENABLE);
}

void uart_send_char(char c)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, (uint16_t)c);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

//================ TASK A ==================
void taskA(void *pvParameters)
{
	const char *s1 = "AAAAAAAAAA\r\n";
	while(1)
	{
		// xin quyen dung UART (neu dang bi task khac giu thi cho)
		if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) 
		{
			// gui chuoi A qua UART
			for(const char *i=s1; *i; i++)
				uart_send_char(*i);

			// tra lai quyen UART
			xSemaphoreGive(xUartMutex); 
		}

		vTaskDelay(pdMS_TO_TICKS(10)); // tam nghi 10ms roi lap lai
	}
}

//================ TASK B ==================
void taskB(void *pvParameters)
{
	const char *s2 = "BBBBBBBBBB\r\n";
	while(1)
	{
		// xin quyen dung UART
		if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) 
		{
			// gui chuoi B qua UART
			for(const char *i=s2; *i; i++)
				uart_send_char(*i);

			// tra lai quyen UART
			xSemaphoreGive(xUartMutex); 
		}

		vTaskDelay(pdMS_TO_TICKS(12)); // tam nghi 12ms roi lap lai
	}
}
