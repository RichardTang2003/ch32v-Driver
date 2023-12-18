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
#include <cassert>

namespace myCH32v
{
    enum GPIO_Usage
    {
        notUsed,
        forLED,
        forButton
    };

    class GPIO
    {
    private:
        GPIO_TypeDef *m_GPIOx;
        GPIO_Usage m_usage;
        GPIOSpeed_TypeDef m_speed;
        GPIOMode_TypeDef m_mode;
        uint32_t m_rccClock;
        uint16_t m_GPIOPin;

    public:
        GPIO() = delete;
        GPIO(GPIO &rhs) = delete;
        GPIO(GPIO &&rhs) = delete;
        GPIO &operator=(GPIO &rhs) = delete;
        GPIO &operator=(GPIO &&rhs) = delete;

        GPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, uint32_t RCC_APB2Periph_GPIOx, GPIOSpeed_TypeDef GPIO_Speed_x, GPIOMode_TypeDef GPIO_Mode_x)
            : m_GPIOx(GPIOx), m_usage(notUsed), m_speed(GPIO_Speed_x), m_mode(GPIO_Mode_x), m_rccClock(RCC_APB2Periph_GPIOx), m_GPIOPin(GPIO_Pin_x)
        {
            initHelper();
        }

        GPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, uint32_t RCC_APB2Periph_GPIOx, GPIOMode_TypeDef GPIO_Mode_x)
            : m_GPIOx(GPIOx), m_usage(notUsed),m_speed(GPIO_Speed_50MHz), m_mode(GPIO_Mode_x), m_rccClock(RCC_APB2Periph_GPIOx), m_GPIOPin(GPIO_Pin_x)
        {
            initHelper();
        }


        [[nodiscard]] bool getState() const
        {
            uint8_t result = GPIO_ReadInputDataBit(m_GPIOx, m_GPIOPin);
            return result ? true : false;
        }

        [[nodiscard]] GPIOMode_TypeDef getMode() const
        {
            return m_mode;
        }

        [[nodiscard]] GPIO_Usage getUsage() const
        {
            return m_usage;
        }

        void setState(bool state)
        {
            if (m_mode == GPIO_Mode_Out_OD || m_mode == GPIO_Mode_Out_PP || m_mode == GPIO_Mode_AF_OD || m_mode == GPIO_Mode_AF_PP)
            {
                BitAction bit = state ? Bit_SET : Bit_RESET;
                GPIO_WriteBit(m_GPIOx, m_GPIOPin, bit);
            }
        }

        void setUsage(GPIO_Usage usage)
        {
            m_usage = usage;
        }


    private:
        void initHelper()
        {
            RCC_APB2PeriphClockCmd(m_rccClock, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure{m_GPIOPin, m_speed, m_mode};
            GPIO_Init(m_GPIOx, &GPIO_InitStructure);
        }
    };

    class LED
    {
    private:
        GPIO* m_GPIO_p;


    public:
        // Creator Function
        LED(GPIO &GPIO_obj)
            : m_GPIO_p(&GPIO_obj)
        {
            assert(m_GPIO_p->getUsage() == notUsed);
            switch (m_GPIO_p->getMode())
            {
                case GPIO_Mode_Out_OD:
                    m_GPIO_p->setState(1);
                    m_GPIO_p->setUsage(forLED);
                    break;
                case GPIO_Mode_Out_PP:
                    m_GPIO_p->setState(0);
                    m_GPIO_p->setUsage(forLED);
                    break;
                default:
                    m_GPIO_p = nullptr;
            }
            
        }



        LED() = delete;
        LED(LED &rhs) = delete;
        LED(LED &&rhs) = delete;
        LED &operator=(LED &rhs) = delete;
        LED &operator=(LED &&rhs) = delete;
        ~LED()
        {
            m_GPIO_p->setState(notUsed);
        }

        void blink(int time_ms) const
        {
            on();
            Delay_Ms(time_ms / 2);
            off();
            Delay_Ms(time_ms / 2);
        }

        void on() const
        {
            switch (m_GPIO_p->getMode())
            {
                case GPIO_Mode_Out_OD:
                    m_GPIO_p->setState(0);
                    m_GPIO_p->setUsage(forLED);
                    break;
                case GPIO_Mode_Out_PP:
                    m_GPIO_p->setState(1);
                    m_GPIO_p->setUsage(forLED);
                    break;
                default:
                    break;
            }
        }

        void off() const
        {
            switch (m_GPIO_p->getMode())
            {
                case GPIO_Mode_Out_OD:
                    m_GPIO_p->setState(1);
                    m_GPIO_p->setUsage(forLED);
                    break;
                case GPIO_Mode_Out_PP:
                    m_GPIO_p->setState(0);
                    m_GPIO_p->setUsage(forLED);
                    break;
                default:
                    break;
            }
        }
    };


}
