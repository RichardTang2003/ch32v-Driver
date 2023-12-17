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

namespace myCH32v
{
    class LED
    {
    private:
        GPIO_TypeDef *m_GPIOPort;
        uint16_t m_GPIOPin;
        uint32_t m_rccClock;
        BitAction m_states;

    public:
        // Creator Function
        LED(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t rcc_clock)
            : m_GPIOPort(gpio_port), m_GPIOPin(gpio_pin), m_rccClock(rcc_clock)
        {
            RCC_APB2PeriphClockCmd(m_rccClock, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure{m_GPIOPin, GPIO_Speed_50MHz, GPIO_Mode_Out_PP};
            GPIO_Init(m_GPIOPort, &GPIO_InitStructure);
        }
        LED(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t rcc_clock, BitAction states)
            : m_GPIOPort(gpio_port), m_GPIOPin(gpio_pin), m_rccClock(rcc_clock), m_states(states)
        {
            RCC_APB2PeriphClockCmd(m_rccClock, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure{m_GPIOPin, GPIO_Speed_50MHz, GPIO_Mode_Out_PP};
            GPIO_Init(m_GPIOPort, &GPIO_InitStructure);
            writeBit();
        }

        LED() = delete;
        LED(LED &rhs) = default;
        LED(LED &&rhs) = default;
        LED &operator=(LED &rhs) = delete;
        LED &operator=(LED &&rhs) = delete;

        void blink(int time_ms)
        {
            on();
            Delay_Ms(time_ms / 2);
            off();
            Delay_Ms(time_ms / 2);
        }
        void rBlink(int time_ms)
        {
            off();
            Delay_Ms(time_ms / 2);
            on();
            Delay_Ms(time_ms / 2);
        }
        void on()
        {
            m_states = BitAction::Bit_SET;
            writeBit();
        }
        void off()
        {
            m_states = BitAction::Bit_RESET;
            writeBit();
        }

    private:
        void writeBit() const { GPIO_WriteBit(m_GPIOPort, m_GPIOPin, m_states); }
    };

    class Button
    {
    private:
        GPIO_TypeDef *m_GPIOPort;
        uint32_t m_rccClock;
        uint16_t m_GPIOPin;

    public:
        // Creator Function
        Button(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t rcc_clock)
            : m_GPIOPort(gpio_port), m_rccClock(rcc_clock), m_GPIOPin(gpio_pin)
        {
            RCC_APB2PeriphClockCmd(m_rccClock, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure{m_GPIOPin, GPIO_Speed_50MHz, GPIO_Mode_IPU};
            GPIO_Init(m_GPIOPort, &GPIO_InitStructure);
        }

        // Principle of Five
        Button() = delete;
        Button(Button &rhs) = default;
        Button(Button &&rhs) = default;
        Button &operator=(Button &rhs) = delete;
        Button &operator=(Button &&rhs) = delete;

        // Member Function
        uint8_t getBit() const
        {
            uint8_t bit = GPIO_ReadInputDataBit(m_GPIOPort, m_GPIOPin);
            return bit;
        }
    };

    class Interrupt
    {
    };
}
