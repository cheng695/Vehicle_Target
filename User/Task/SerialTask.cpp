#include "SerialTask.hpp"
#include "RobotContext.hpp"

uint8_t DT7Rx_buffer[18];

void SerivalInit()
{
    DT7.SetDeadzone(20.0f);

    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    HAL::UART::Data remote_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};

    uart1.receive_dma_idle(remote_rx_buffer);
    uart1.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(DT7Rx_buffer))
        {
            DT7.parseData(data.buffer);
        }
    });
}

void UpdateMomentaryInput(OperatorInput &input)
{
    input.left_switch = DT7.get_s1();
    input.right_switch = DT7.get_s2();
    input.keyboard_mode = false;
    input.spin = false;
    input.follow_enabled = false;

    input.move_x = DT7.get_left_y();
    input.move_y = DT7.get_left_x();
    input.rotate = DT7.get_right_x();
}

OperatorInput BuildOperatorInput()
{
    OperatorInput input = operator_input;
    UpdateMomentaryInput(input);
    return input;
}

void KeyProcess()
{
    UpdateMomentaryInput(operator_input);
}

extern "C" {
void SerivalTask(void const * argument)
{
    SerivalInit();
    for (;;)
    {
        KeyProcess();
        osDelay(1);
    }
}
}
