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
#include <string>
#include <vector>

extern "C" void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


void USARTx_CFG(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    USART_DeInit(USART2);
    /* USART2 TX-->A.2   RX-->A.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{

}

namespace myCH32v
{
    enum bufferState
    {
        Ready,
        Receiving,
        Done,
        Broken
    };

    class USART
    {
    private:
        std::vector<char> m_buffer;
        USART_TypeDef *m_USARTx_p;
        bufferState m_receivingState = bufferState::Ready;
        bool m_fillOnce = false;

    public:
        USART() = delete;
        USART(USART &rhs) = delete;
        USART(USART &&rhs) = delete;
        USART operator=(USART &rhs) = delete;
        USART operator=(USART &&rhs) = delete;

        USART(USART_TypeDef *USARTx_p)
            : m_USARTx_p(USARTx_p){};
        USART(USART_TypeDef *USARTx_p, bool fillOnce)
            : m_USARTx_p(USARTx_p), m_fillOnce(fillOnce){};

        USART &sendByte(uint8_t ch)
        {
            USART_SendData(m_USARTx_p, ch);
            while (USART_GetFlagStatus(m_USARTx_p, USART_FLAG_TXE) == RESET)
                ;

            return *this;
        }

        USART &sendStr(char const *str)
        {
            int i = 0;
            while (*(str + i) != '\0')
            {
                sendByte(*(str + i));
                ++i;
            }

            while (USART_GetFlagStatus(m_USARTx_p, USART_FLAG_TC) == RESET)
                ;

            return *this;
        }

        USART &sendStr(const std::string &str)
        {
            for (uint8_t ch : str)
            {
                sendByte(ch);
            }
            while (USART_GetFlagStatus(m_USARTx_p, USART_FLAG_TC) == RESET)
                ;

            return *this;
        }

        USART &operator<<(const std::string &str)
        {
            return sendStr(str);
        }

        void
        receiveDataPack()
        {
            uint8_t ch = USART_ReceiveData(m_USARTx_p);
            switch (m_receivingState)
            {
            case bufferState::Ready:
            {
                if (ch == '[')
                {
                    m_receivingState = bufferState::Receiving;
                }
                break;
            }

            case bufferState::Receiving:
            {
                if (ch == ']')
                {
                    m_receivingState = bufferState::Done;
                }
                m_buffer.emplace_back(ch);
                break;
            }
            }
            USART_ClearITPendingBit(m_USARTx_p, USART_IT_CTS);
        }

        std::string getData()
        {
            std::string stringView;
            for (char ch : m_buffer)
            {
                stringView += ch;
            }

            return std::move(stringView);
        }
    };

};
