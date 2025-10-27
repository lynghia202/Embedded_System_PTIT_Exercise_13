#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define BUTTON_PIN        GPIO_Pin_13 
#define BLINK_LED_PIN     GPIO_Pin_0  
#define ALERT_LED_PIN     GPIO_Pin_1  

SemaphoreHandle_t xInterruptSemaphore; 
TaskHandle_t xAlertTaskHandle = NULL;

static void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BLINK_LED_PIN | ALERT_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_ResetBits(GPIOA, BLINK_LED_PIN | ALERT_LED_PIN); 
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
}

static void EXTI_Config(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13); 
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

static void NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void vBlinkLedTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(500);
    for (;;) {
        GPIOA->ODR ^= BLINK_LED_PIN; 
        vTaskDelay(xDelay);
    }
}

void vAlertTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xInterruptSemaphore, portMAX_DELAY) == pdTRUE) { 
            GPIO_SetBits(GPIOA, ALERT_LED_PIN); 
            vTaskDelay(pdMS_TO_TICKS(2000));
            GPIO_ResetBits(GPIOA, ALERT_LED_PIN); 
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    // Kiem tra xem co phai ngat den tu chan EXTI line 13 (vi du nut nhan) hay khong
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) { 
        
        // Bien dung de danh dau xem co task uu tien cao hon can duoc chay sau ISR khong
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Gui (give) semaphore tu trong ham ngat
        // Dieu nay thong bao cho task rang su kien ngoai (ngat) da xay ra
        xSemaphoreGiveFromISR(xInterruptSemaphore, &xHigherPriorityTaskWoken);

        // Xoa co ngat cua line 13 de san sang cho lan ngat tiep theo
        EXTI_ClearITPendingBit(EXTI_Line13); 

        // Neu viec gui semaphore lam task uu tien cao hon san sang chay,
        // thi yeu cau he thong chuyen ngu canh (context switch) ngay sau khi thoat ISR
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int main(void) {
    GPIO_Config();
    EXTI_Config();
    NVIC_Config();
    xInterruptSemaphore = xSemaphoreCreateBinary(); // Create a binary semaphore
    if (xInterruptSemaphore != NULL) {
        xTaskCreate(vBlinkLedTask, "BlinkLED",128,NULL,1,NULL);
        xTaskCreate(vAlertTask,"AlertTask",128,NULL,2,&xAlertTaskHandle);
        vTaskStartScheduler();
    }

    while (1) {
			
		}
}

