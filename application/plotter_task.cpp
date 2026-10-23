#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "task.h"

// motor_task.cpp 中已有的接口。
void motor_get_power_compare(float data[3]);
void motor_get_power_debug(float data[5]);

void motor_get_power_fit_snapshot(float data[11], uint32_t & stamp_ms);

// can_task.cpp 中已有的接口。
bool power_meter_get_fit_snapshot(float data[3], uint32_t & stamp_ms, uint32_t & sequence);

sp::Plotter plotter(&huart1);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  float compare_data[3] = {};
  float debug_data[5] = {};
  float terms_data[11] = {};
  float meter_data[3] = {};

  uint32_t motor_stamp_ms = 0;
  uint32_t meter_stamp_ms = 0;
  uint32_t meter_sequence = 0;

  while (true) {
    bool meter_alive;
    uint32_t now_ms;

    // 只在读取数据时进入临界区。
    // 串口发送必须放在临界区外。
    taskENTER_CRITICAL();

    motor_get_power_compare(compare_data);
    motor_get_power_debug(debug_data);

    motor_get_power_fit_snapshot(terms_data, motor_stamp_ms);

    meter_alive = power_meter_get_fit_snapshot(meter_data, meter_stamp_ms, meter_sequence);

    now_ms = osKernelSysTick();

    taskEXIT_CRITICAL();

    // 当前时刻距离快照/报文接收的时间。
    // 功率计离线时显示 -1，避免误读成有效数据。
    const float meter_age_ms = meter_alive ? static_cast<float>(now_ms - meter_stamp_ms) : -1.0f;

    const float motor_age_ms =
      compare_data[2] > 0.5f ? static_cast<float>(now_ms - motor_stamp_ms) : -1.0f;

    plotter.plot(
      compare_data[0],                      // 1：指令预测，即 power_after，W
      compare_data[1],                      // 2：反馈预测，W
      meter_data[2],                        // 3：功率计实测功率，W
      debug_data[0],                        // 4：限制前指令预测功率，W
      debug_data[2],                        // 5：转矩缩放比例，0～1
      debug_data[3],                        // 6：控制功率上限，W
      debug_data[4],                        // 7：限制状态，0正常/1限制/2无解
      compare_data[2],                      // 8：比较有效，1有效/0无效
      meter_alive ? 1.0f : 0.0f,            // 9：功率计在线状态
      static_cast<float>(now_ms) * 0.001f,  // 10：系统时间，s
      meter_age_ms,                         // 11：功率计报文年龄，ms
      motor_age_ms,                         // 12：电机快照年龄，ms
      static_cast<float>(meter_sequence)    // 13：功率计报文序号
    );

    osDelay(10);
  }
}