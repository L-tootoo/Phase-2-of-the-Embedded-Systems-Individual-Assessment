#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "task.h"

sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

extern "C" void uart_task(void const * argument)
{
  remote.request();
  pm02.request();

  while (true) {
    taskENTER_CRITICAL();
    pm02.update_radar_data_timeout(HAL_GetTick());
    taskEXIT_CRITICAL();
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote.update(Size, osKernelSysTick());
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.request();
  }
}
