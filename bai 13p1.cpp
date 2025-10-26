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

static void prvSetupHardware(void);
static void GPIO_Config(void);
static void EXTI_Config(void);
static void NVIC_Config(void); 
void vBlinkLedTask(void *pvParameters);
void vAlertTask(void *pvParameters);


int main(void) {
    prvSetupHardware();

    xInterruptSemaphore = xSemaphoreCreateBinary();

    if (xInterruptSemaphore != NULL) {
        xTaskCreate(vBlinkLedTask,
                    "BlinkLED",
                    128,
                    NULL,
                    1,
                    NULL);

        xTaskCreate(vAlertTask,
                    "AlertTask",
                    128,
                    NULL,
                    2,
                    &xAlertTaskHandle);

        vTaskStartScheduler();
    }

    while (1) {}
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


static void prvSetupHardware(void) {
    GPIO_Config();
    EXTI_Config();
    NVIC_Config();
}



void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) { 
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR(xInterruptSemaphore, &xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line13); 
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
