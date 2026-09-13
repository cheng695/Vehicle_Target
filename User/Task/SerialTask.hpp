#ifndef SERIALTASK_H
#define SERIALTASK_H

#include "FreeRTOS.h"
#include "OperatorInput.hpp"
#include "cmsis_os.h"
#include "uart_hal.hpp"

#include <cstring>

extern uint8_t DT7Rx_buffer[18];

OperatorInput BuildOperatorInput();

extern "C" {
void SerivalTask(void const* argument);
}

#endif
