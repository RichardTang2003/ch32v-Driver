#if defined(CH32V00X)
#include <ch32v00x.h>
#elif defined(CH32V10X)
#include <ch32v10x.h>
#elif defined(CH32V20X)
#include <ch32v20x.h>
#elif defined(CH32V30X)
#include <ch32v30x.h>
#endif
#include <debug.h>
#include "my-GPIO.hpp"
#include "my-USART.hpp"

using namespace myCH32v;

extern "C" void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Global define */

/* Global Variable */
static uint8_t states = 1;
static GPIO a15{GPIOA, GPIO_Pin_15, RCC_APB2Periph_GPIOA, GPIO_Mode_Out_PP};
static GPIO b4{GPIOB, GPIO_Pin_4, RCC_APB2Periph_GPIOB, GPIO_Mode_Out_PP};
static GPIO c0{GPIOC, GPIO_Pin_0, RCC_APB2Periph_GPIOC, GPIO_Mode_Out_OD};
static GPIO c1{GPIOC, GPIO_Pin_1, RCC_APB2Periph_GPIOC, GPIO_Mode_Out_OD};

static LED led1{a15};
static LED led2{b4};
static LED led3{c0};
static LED led4{c1};

static void ledOffHelper()
{
    led1.off();
    led2.off();
    led3.off();
    led4.off();
}

static USART pcSerial{USART2};

/*********************************************************************
 * @fn      EXTI0_INT_INIT
 *
 * @brief   Initializes EXTI0 collection.
 *
 * @return  none
 */
void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* GPIOA ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    EXTI0_INT_INIT();
    USARTx_CFG();

    while(1)
    {
        switch(states)
        {
            case 0:
                ledOffHelper();
                led1.on();
                led2.on();
                led3.on();
                led4.on();
                pcSerial.sendStr("Mode1.");
                Delay_Ms(2000);
                break;
            case 1:
                ledOffHelper();
                led1.blink(500);
                led2.blink(500);
                led3.blink(500);
                led4.blink(500);
                pcSerial.sendStr("Mode2.");
                break;
            case 2:
                ledOffHelper();
                led1.on();
                led2.on();
                led3.off();
                led4.off();
                pcSerial.sendStr("Mode3.");
                Delay_Ms(2000);
                break;
        }
    }
}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
    while (1)
    {
    }
}
