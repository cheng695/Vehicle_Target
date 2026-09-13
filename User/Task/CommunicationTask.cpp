#include "CommunicationTask.hpp"
#include "RobotContext.hpp"
#include "SerialTask.hpp"
#include "uart_device.hpp"
#include <cstdint>


uint8_t send_str2[sizeof(float) * 11];  // 鍙戦€乿ofa


void BoardCommunicationInit()
{
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9)
{
    const uint8_t sendSize = sizeof(float); 


    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;
    *((float*)&send_str2[sendSize * 6]) = x7;
    *((float*)&send_str2[sendSize * 7]) = x8;
    *((float*)&send_str2[sendSize * 8]) = x9;


    *((uint32_t*)&send_str2[sizeof(float) * 9]) = 0x7F800000; 
}

void UART_TX()
{
    // vofa
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    HAL::UART::Data uart3_tx_buffer{send_str2, sizeof(send_str2)};
    uart3.transmit_dma(uart3_tx_buffer);
    vofa_send(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}


extern "C" {
void CommunicationTask(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        UART_TX();
        osDelay(5);
    }
}

}
