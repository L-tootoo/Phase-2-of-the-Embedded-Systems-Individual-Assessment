#include "cmsis_os.h"
#include "referee/pm02/pm02.hpp"

// C板
sp::PM02 pm02(&huart6);

// 达妙
// sp::PM02 pm02(&huart1, false);

extern "C" void uart_task()
{
  pm02.request();

  while (true) {
    // 即使裁判系统 UART 完全断流，也要周期维护雷达 0x0301 数据的 1.5 s 超时。
    // PM02 可能在 UART 中断中更新，因此任务侧调用时应放在临界区内。
    taskENTER_CRITICAL();
    pm02.update_radar_data_timeout(HAL_GetTick());
    taskEXIT_CRITICAL();

    // 使用调试(f5)查看pm02内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
}